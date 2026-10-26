#include "smp_ota.h"

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "mbedtls/sha256.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "os/os_mbuf.h"

static const char *TAG = "smp_ota";

/* SMP BLE UUIDs (same as nordic MCUmgr / ota_updater.py), LSB-first. */
static const ble_uuid128_t smp_svc_uuid =
    BLE_UUID128_INIT(0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
                     0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d);
static const ble_uuid128_t smp_chr_uuid =
    BLE_UUID128_INIT(0x48, 0x7c, 0x99, 0x74, 0x11, 0x26, 0x9e, 0xae,
                     0x01, 0x4e, 0xce, 0xfb, 0x28, 0x78, 0x2e, 0xda);

#define SMP_HEADER_SIZE 8
#define OP_READ 0
#define OP_READ_RSP 1
#define OP_WRITE 2
#define OP_WRITE_RSP 3
#define GRP_OS 0
#define GRP_IMG 1
#define IMG_STATE 0
#define IMG_UPLOAD 1
#define OS_RESET 5

#define NVS_NS "smp_ota"
#define NVS_KEY_SLOT1_HASH "s1hash"
#define NVS_KEY_SLOT1_VER "s1ver"
#define NVS_KEY_SLOT0_HASH "s0hash"

static uint16_t smp_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t smp_chr_val_handle;
static smp_ota_busy_cb_t s_busy_cb;

static esp_ota_handle_t s_ota_handle;
static const esp_partition_t *s_update_part;
static bool s_ota_active;
static uint32_t s_ota_off;
static uint32_t s_ota_len;
static uint8_t s_ota_sha[32];
static bool s_ota_sha_set;
static mbedtls_sha256_context s_sha_ctx;
static bool s_sha_ctx_init;
static uint8_t s_slot1_hash[32];
static bool s_slot1_hash_valid;
static char s_slot1_version[32];
static uint8_t s_slot0_hash[32];
static bool s_slot0_hash_valid;
static bool s_confirmed = true;

/* ---------- minimal CBOR helpers (subset used by ota_updater.py) ---------- */

static int cbor_skip(const uint8_t *p, size_t len, size_t *idx);

static int cbor_read_type(const uint8_t *p, size_t len, size_t *idx,
                          uint8_t *major, uint64_t *val)
{
    if (*idx >= len) {
        return -1;
    }
    uint8_t b = p[(*idx)++];
    *major = b >> 5;
    uint8_t ai = b & 0x1f;
    if (ai < 24) {
        *val = ai;
        return 0;
    }
    if (ai == 24) {
        if (*idx + 1 > len) {
            return -1;
        }
        *val = p[(*idx)++];
        return 0;
    }
    if (ai == 25) {
        if (*idx + 2 > len) {
            return -1;
        }
        *val = ((uint64_t)p[*idx] << 8) | p[*idx + 1];
        *idx += 2;
        return 0;
    }
    if (ai == 26) {
        if (*idx + 4 > len) {
            return -1;
        }
        *val = ((uint64_t)p[*idx] << 24) | ((uint64_t)p[*idx + 1] << 16) |
               ((uint64_t)p[*idx + 2] << 8) | p[*idx + 3];
        *idx += 4;
        return 0;
    }
    return -1;
}

static int cbor_skip(const uint8_t *p, size_t len, size_t *idx)
{
    uint8_t major;
    uint64_t val;
    if (cbor_read_type(p, len, idx, &major, &val) != 0) {
        return -1;
    }
    if (major == 2 || major == 3) { /* bstr / tstr */
        if (*idx + val > len) {
            return -1;
        }
        *idx += (size_t)val;
        return 0;
    }
    if (major == 4) { /* array */
        for (uint64_t i = 0; i < val; i++) {
            if (cbor_skip(p, len, idx) != 0) {
                return -1;
            }
        }
        return 0;
    }
    if (major == 5) { /* map */
        for (uint64_t i = 0; i < val; i++) {
            if (cbor_skip(p, len, idx) != 0 || cbor_skip(p, len, idx) != 0) {
                return -1;
            }
        }
        return 0;
    }
    if (major == 7 && val == 20) { /* false */
        return 0;
    }
    if (major == 7 && val == 21) { /* true */
        return 0;
    }
    /* uint / nint / simple */
    return 0;
}

typedef struct {
    const uint8_t *data;
    size_t data_len;
    uint32_t off;
    bool has_off;
    uint32_t len;
    bool has_len;
    const uint8_t *sha;
    size_t sha_len;
    int image;
    bool has_image;
    const uint8_t *hash;
    size_t hash_len;
    bool confirm;
    bool has_confirm;
} smp_map_t;

static int parse_smp_map(const uint8_t *p, size_t len, smp_map_t *out)
{
    memset(out, 0, sizeof(*out));
    size_t idx = 0;
    uint8_t major;
    uint64_t nkeys;
    if (cbor_read_type(p, len, &idx, &major, &nkeys) != 0 || major != 5) {
        return -1;
    }
    for (uint64_t k = 0; k < nkeys; k++) {
        uint64_t klen;
        if (cbor_read_type(p, len, &idx, &major, &klen) != 0 || major != 3) {
            return -1;
        }
        if (idx + klen > len) {
            return -1;
        }
        const char *key = (const char *)&p[idx];
        idx += (size_t)klen;

        if (klen == 4 && memcmp(key, "data", 4) == 0) {
            uint64_t dlen;
            if (cbor_read_type(p, len, &idx, &major, &dlen) != 0 || major != 2) {
                return -1;
            }
            if (idx + dlen > len) {
                return -1;
            }
            out->data = &p[idx];
            out->data_len = (size_t)dlen;
            idx += (size_t)dlen;
        } else if (klen == 3 && memcmp(key, "off", 3) == 0) {
            uint64_t v;
            if (cbor_read_type(p, len, &idx, &major, &v) != 0 || major != 0) {
                return -1;
            }
            out->off = (uint32_t)v;
            out->has_off = true;
        } else if (klen == 3 && memcmp(key, "len", 3) == 0) {
            uint64_t v;
            if (cbor_read_type(p, len, &idx, &major, &v) != 0 || major != 0) {
                return -1;
            }
            out->len = (uint32_t)v;
            out->has_len = true;
        } else if (klen == 3 && memcmp(key, "sha", 3) == 0) {
            uint64_t dlen;
            if (cbor_read_type(p, len, &idx, &major, &dlen) != 0 || major != 2) {
                return -1;
            }
            if (idx + dlen > len) {
                return -1;
            }
            out->sha = &p[idx];
            out->sha_len = (size_t)dlen;
            idx += (size_t)dlen;
        } else if (klen == 5 && memcmp(key, "image", 5) == 0) {
            uint64_t v;
            if (cbor_read_type(p, len, &idx, &major, &v) != 0 || major != 0) {
                return -1;
            }
            out->image = (int)v;
            out->has_image = true;
        } else if (klen == 4 && memcmp(key, "hash", 4) == 0) {
            uint64_t dlen;
            if (cbor_read_type(p, len, &idx, &major, &dlen) != 0 || major != 2) {
                return -1;
            }
            if (idx + dlen > len) {
                return -1;
            }
            out->hash = &p[idx];
            out->hash_len = (size_t)dlen;
            idx += (size_t)dlen;
        } else if (klen == 7 && memcmp(key, "confirm", 7) == 0) {
            uint64_t v;
            if (cbor_read_type(p, len, &idx, &major, &v) != 0 || major != 7) {
                return -1;
            }
            out->confirm = (v == 21);
            out->has_confirm = true;
        } else {
            if (cbor_skip(p, len, &idx) != 0) {
                return -1;
            }
        }
    }
    return 0;
}

static size_t cbor_put_uint(uint8_t *out, uint64_t v)
{
    if (v < 24) {
        out[0] = (uint8_t)v;
        return 1;
    }
    if (v < 256) {
        out[0] = 24;
        out[1] = (uint8_t)v;
        return 2;
    }
    if (v < 65536) {
        out[0] = 25;
        out[1] = (uint8_t)(v >> 8);
        out[2] = (uint8_t)v;
        return 3;
    }
    out[0] = 26;
    out[1] = (uint8_t)(v >> 24);
    out[2] = (uint8_t)(v >> 16);
    out[3] = (uint8_t)(v >> 8);
    out[4] = (uint8_t)v;
    return 5;
}

static size_t cbor_put_tstr(uint8_t *out, const char *s)
{
    size_t n = strlen(s);
    size_t o = 0;
    if (n < 24) {
        out[o++] = (uint8_t)(0x60 | n);
    } else {
        out[o++] = 0x78;
        out[o++] = (uint8_t)n;
    }
    memcpy(&out[o], s, n);
    return o + n;
}

static size_t cbor_put_bstr(uint8_t *out, const uint8_t *b, size_t n)
{
    size_t o = 0;
    if (n < 24) {
        out[o++] = (uint8_t)(0x40 | n);
    } else if (n < 256) {
        out[o++] = 0x58;
        out[o++] = (uint8_t)n;
    } else {
        out[o++] = 0x59;
        out[o++] = (uint8_t)(n >> 8);
        out[o++] = (uint8_t)n;
    }
    memcpy(&out[o], b, n);
    return o + n;
}

static size_t cbor_put_bool(uint8_t *out, bool v)
{
    out[0] = v ? 0xf5 : 0xf4;
    return 1;
}

/* Encode one image map entry used by img state. */
static size_t encode_image_entry(uint8_t *out, int slot, const char *version,
                                 const uint8_t *hash, bool hash_valid,
                                 bool active, bool confirmed, bool pending)
{
    /* 8 keys */
    size_t o = 0;
    out[o++] = 0xa8;

    o += cbor_put_tstr(&out[o], "slot");
    o += cbor_put_uint(&out[o], (uint64_t)slot);

    o += cbor_put_tstr(&out[o], "version");
    o += cbor_put_tstr(&out[o], version ? version : "");

    o += cbor_put_tstr(&out[o], "hash");
    static const uint8_t zhash[32] = {0};
    o += cbor_put_bstr(&out[o], hash_valid ? hash : zhash, 32);

    o += cbor_put_tstr(&out[o], "bootable");
    o += cbor_put_bool(&out[o], true);

    o += cbor_put_tstr(&out[o], "pending");
    o += cbor_put_bool(&out[o], pending);

    o += cbor_put_tstr(&out[o], "confirmed");
    o += cbor_put_bool(&out[o], confirmed);

    o += cbor_put_tstr(&out[o], "active");
    o += cbor_put_bool(&out[o], active);

    o += cbor_put_tstr(&out[o], "permanent");
    o += cbor_put_bool(&out[o], confirmed && active);

    return o;
}

static void nvs_save_slot1(const uint8_t hash[32], const char *ver)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_blob(h, NVS_KEY_SLOT1_HASH, hash, 32);
    if (ver) {
        nvs_set_str(h, NVS_KEY_SLOT1_VER, ver);
    }
    nvs_commit(h);
    nvs_close(h);
}

static void nvs_load_slot1(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    size_t len = 32;
    if (nvs_get_blob(h, NVS_KEY_SLOT1_HASH, s_slot1_hash, &len) == ESP_OK && len == 32) {
        s_slot1_hash_valid = true;
    }
    len = sizeof(s_slot1_version);
    if (nvs_get_str(h, NVS_KEY_SLOT1_VER, s_slot1_version, &len) != ESP_OK) {
        s_slot1_version[0] = '\0';
    }
    nvs_close(h);
}

static void nvs_save_slot0_hash(const uint8_t hash[32])
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_blob(h, NVS_KEY_SLOT0_HASH, hash, 32);
    nvs_commit(h);
    nvs_close(h);
    memcpy(s_slot0_hash, hash, 32);
    s_slot0_hash_valid = true;
}

static void nvs_load_slot0_hash(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    size_t len = 32;
    if (nvs_get_blob(h, NVS_KEY_SLOT0_HASH, s_slot0_hash, &len) == ESP_OK && len == 32) {
        s_slot0_hash_valid = true;
    }
    nvs_close(h);
}

static const char *running_version(void)
{
    const esp_app_desc_t *d = esp_app_get_description();
    return d ? d->version : "0.0.0";
}

static void abort_upload(void)
{
    if (s_ota_active) {
        esp_ota_abort(s_ota_handle);
        s_ota_active = false;
    }
    if (s_sha_ctx_init) {
        mbedtls_sha256_free(&s_sha_ctx);
        s_sha_ctx_init = false;
    }
    if (s_busy_cb) {
        s_busy_cb(false);
    }
}

static int handle_img_upload(const smp_map_t *m, uint8_t *rsp, size_t rsp_cap, size_t *rsp_len)
{
    if (!m->has_off || !m->data) {
        return 3; /* EINVAL */
    }

    if (m->off == 0) {
        abort_upload();
        if (!m->has_len || m->len == 0) {
            return 3;
        }
        s_update_part = esp_ota_get_next_update_partition(NULL);
        if (!s_update_part) {
            ESP_LOGE(TAG, "No OTA partition");
            return 11;
        }
        if (m->len > s_update_part->size) {
            ESP_LOGE(TAG, "Image too large %u > %u", (unsigned)m->len,
                     (unsigned)s_update_part->size);
            return 3;
        }
        esp_err_t err = esp_ota_begin(s_update_part, m->len, &s_ota_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
            return 11;
        }
        s_ota_active = true;
        s_ota_off = 0;
        s_ota_len = m->len;
        s_ota_sha_set = false;
        if (m->sha && m->sha_len == 32) {
            memcpy(s_ota_sha, m->sha, 32);
            s_ota_sha_set = true;
        }
        mbedtls_sha256_init(&s_sha_ctx);
        mbedtls_sha256_starts(&s_sha_ctx, 0);
        s_sha_ctx_init = true;
        if (s_busy_cb) {
            s_busy_cb(true);
        }
        ESP_LOGI(TAG, "OTA begin len=%u part=%s", (unsigned)m->len, s_update_part->label);
    }

    if (!s_ota_active) {
        return 5; /* EBUSY / bad state */
    }
    if (m->off != s_ota_off) {
        ESP_LOGW(TAG, "offset mismatch host=%u dev=%u", (unsigned)m->off, (unsigned)s_ota_off);
        /* echo expected offset */
        size_t o = 0;
        rsp[o++] = 0xa2;
        o += cbor_put_tstr(&rsp[o], "rc");
        o += cbor_put_uint(&rsp[o], 0);
        o += cbor_put_tstr(&rsp[o], "off");
        o += cbor_put_uint(&rsp[o], s_ota_off);
        *rsp_len = o;
        return 0;
    }

    esp_err_t err = esp_ota_write(s_ota_handle, m->data, m->data_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
        abort_upload();
        return 11;
    }
    mbedtls_sha256_update(&s_sha_ctx, m->data, m->data_len);
    s_ota_off += m->data_len;

    if (s_ota_off >= s_ota_len) {
        uint8_t calc[32];
        mbedtls_sha256_finish(&s_sha_ctx, calc);
        mbedtls_sha256_free(&s_sha_ctx);
        s_sha_ctx_init = false;

        if (s_ota_sha_set && memcmp(calc, s_ota_sha, 32) != 0) {
            ESP_LOGE(TAG, "SHA256 mismatch");
            abort_upload();
            return 13;
        }
        err = esp_ota_end(s_ota_handle);
        s_ota_active = false;
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
            if (s_busy_cb) {
                s_busy_cb(false);
            }
            return 11;
        }
        memcpy(s_slot1_hash, calc, 32);
        s_slot1_hash_valid = true;
        /* Try read version from image header in OTA partition. */
        esp_app_desc_t desc;
        if (esp_ota_get_partition_description(s_update_part, &desc) == ESP_OK) {
            strncpy(s_slot1_version, desc.version, sizeof(s_slot1_version) - 1);
            s_slot1_version[sizeof(s_slot1_version) - 1] = '\0';
        } else {
            strncpy(s_slot1_version, "unknown", sizeof(s_slot1_version) - 1);
        }
        nvs_save_slot1(s_slot1_hash, s_slot1_version);
        ESP_LOGI(TAG, "OTA image written ver=%s part=%s", s_slot1_version,
                 s_update_part ? s_update_part->label : "?");
        if (s_busy_cb) {
            s_busy_cb(false);
        }
    }

    size_t o = 0;
    rsp[o++] = 0xa1;
    o += cbor_put_tstr(&rsp[o], "off");
    o += cbor_put_uint(&rsp[o], s_ota_off);
    *rsp_len = o;
    return 0;
}

static int handle_img_state_read(uint8_t *rsp, size_t rsp_cap, size_t *rsp_len)
{
    (void)rsp_cap;
    const char *ver0 = running_version();
    bool pending = false;
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (boot && run && boot != run) {
        pending = true;
    }

    /* {"images":[ img0, img1 ]} */
    size_t o = 0;
    rsp[o++] = 0xa1;
    o += cbor_put_tstr(&rsp[o], "images");
    rsp[o++] = 0x82; /* array 2 */

    o += encode_image_entry(&rsp[o], 0, ver0, s_slot0_hash, s_slot0_hash_valid,
                            true, s_confirmed, false);
    o += encode_image_entry(&rsp[o], 1,
                            s_slot1_hash_valid ? s_slot1_version : "",
                            s_slot1_hash, s_slot1_hash_valid,
                            false, false, pending);
    *rsp_len = o;
    return 0;
}

static int handle_img_state_write(const smp_map_t *m)
{
    if (!m->hash || m->hash_len != 32) {
        return 3;
    }
    if (!s_slot1_hash_valid || memcmp(m->hash, s_slot1_hash, 32) != 0) {
        ESP_LOGW(TAG, "state write: unknown hash");
        return 5;
    }

    if (m->has_confirm && m->confirm) {
        /* Confirm currently running image */
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "confirm failed: %s", esp_err_to_name(err));
            return 11;
        }
        s_confirmed = true;
        nvs_save_slot0_hash(s_slot0_hash_valid ? s_slot0_hash : s_slot1_hash);
        ESP_LOGI(TAG, "Image confirmed");
        return 0;
    }

    /* Boot the partition that received the upload (or the inactive OTA slot). */
    const esp_partition_t *part = s_update_part;
    if (!part) {
        part = esp_ota_get_next_update_partition(NULL);
    }
    if (!part) {
        ESP_LOGE(TAG, "no update partition for test boot");
        return 11;
    }
    esp_err_t err = esp_ota_set_boot_partition(part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_boot_partition(%s): %s", part->label, esp_err_to_name(err));
        return 11;
    }
    ESP_LOGI(TAG, "Boot partition set to %s @0x%08lx (test/rollback)",
             part->label, (unsigned long)part->address);
    return 0;
}

static void smp_notify(uint16_t conn, uint8_t op_rsp, uint16_t group, uint8_t seq,
                       uint8_t id, const uint8_t *cbor, size_t cbor_len)
{
    uint8_t hdr[SMP_HEADER_SIZE];
    hdr[0] = op_rsp;
    hdr[1] = 0;
    hdr[2] = (uint8_t)(cbor_len >> 8);
    hdr[3] = (uint8_t)(cbor_len & 0xff);
    hdr[4] = (uint8_t)(group >> 8);
    hdr[5] = (uint8_t)(group & 0xff);
    hdr[6] = seq;
    hdr[7] = id;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(hdr, SMP_HEADER_SIZE);
    if (!om) {
        return;
    }
    if (cbor_len && os_mbuf_append(om, cbor, cbor_len) != 0) {
        os_mbuf_free_chain(om);
        return;
    }
    ble_gatts_notify_custom(conn, smp_chr_val_handle, om);
}

static int process_smp_frame(uint16_t conn, const uint8_t *frame, size_t frame_len)
{
    if (frame_len < SMP_HEADER_SIZE) {
        return 0;
    }
    uint8_t op = frame[0];
    uint16_t plen = ((uint16_t)frame[2] << 8) | frame[3];
    uint16_t group = ((uint16_t)frame[4] << 8) | frame[5];
    uint8_t seq = frame[6];
    uint8_t id = frame[7];
    if (frame_len < SMP_HEADER_SIZE + plen) {
        return 0;
    }
    const uint8_t *cbor = frame + SMP_HEADER_SIZE;

    uint8_t rsp[512];
    size_t rsp_len = 0;
    int rc = 0;
    uint8_t op_rsp = (op == OP_READ) ? OP_READ_RSP : OP_WRITE_RSP;

    smp_map_t map;
    memset(&map, 0, sizeof(map));
    if (plen > 0) {
        if (parse_smp_map(cbor, plen, &map) != 0) {
            /* empty map ok for some cmds */
            if (!(plen == 1 && cbor[0] == 0xa0)) {
                ESP_LOGW(TAG, "CBOR parse failed plen=%u", plen);
            }
        }
    }

    if (group == GRP_IMG && id == IMG_UPLOAD && op == OP_WRITE) {
        rc = handle_img_upload(&map, rsp, sizeof(rsp), &rsp_len);
        if (rc != 0) {
            rsp_len = 0;
            rsp[rsp_len++] = 0xa1;
            rsp_len += cbor_put_tstr(&rsp[rsp_len], "rc");
            rsp_len += cbor_put_uint(&rsp[rsp_len], (uint64_t)rc);
        }
    } else if (group == GRP_IMG && id == IMG_STATE && op == OP_READ) {
        rc = handle_img_state_read(rsp, sizeof(rsp), &rsp_len);
        if (rc != 0) {
            rsp_len = 0;
            rsp[rsp_len++] = 0xa1;
            rsp_len += cbor_put_tstr(&rsp[rsp_len], "rc");
            rsp_len += cbor_put_uint(&rsp[rsp_len], (uint64_t)rc);
        }
    } else if (group == GRP_IMG && id == IMG_STATE && op == OP_WRITE) {
        rc = handle_img_state_write(&map);
        rsp_len = 0;
        rsp[rsp_len++] = 0xa1;
        rsp_len += cbor_put_tstr(&rsp[rsp_len], "rc");
        rsp_len += cbor_put_uint(&rsp[rsp_len], (uint64_t)rc);
    } else if (group == GRP_OS && id == OS_RESET && op == OP_WRITE) {
        rsp_len = 0;
        rsp[rsp_len++] = 0xa1;
        rsp_len += cbor_put_tstr(&rsp[rsp_len], "rc");
        rsp_len += cbor_put_uint(&rsp[rsp_len], 0);
        smp_notify(conn, op_rsp, group, seq, id, rsp, rsp_len);
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
        return 0;
    } else {
        rsp_len = 0;
        rsp[rsp_len++] = 0xa1;
        rsp_len += cbor_put_tstr(&rsp[rsp_len], "rc");
        rsp_len += cbor_put_uint(&rsp[rsp_len], 8); /* ENOTSUP */
    }

    smp_notify(conn, op_rsp, group, seq, id, rsp, rsp_len);
    return 0;
}

/*
 * Defer heavy OTA/CBOR work out of the GATT access callback.  Doing
 * esp_ota_write + notify inside the NimBLE host callback often drops the
 * response (host sees 30s TimeoutError on the first upload chunk).
 */
#define SMP_RX_MAX 2048
typedef struct {
    uint16_t conn;
    uint16_t len;
    uint8_t data[SMP_RX_MAX];
} smp_rx_msg_t;

static QueueHandle_t s_smp_q;

static void smp_worker_task(void *arg)
{
    (void)arg;
    smp_rx_msg_t msg;
    for (;;) {
        if (xQueueReceive(s_smp_q, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (msg.len >= SMP_HEADER_SIZE) {
            uint16_t plen = ((uint16_t)msg.data[2] << 8) | msg.data[3];
            if (msg.len >= SMP_HEADER_SIZE + plen) {
                process_smp_frame(msg.conn, msg.data, msg.len);
            }
        }
    }
}

static int smp_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)attr_handle;
    (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len == 0 || len > SMP_RX_MAX || s_smp_q == NULL) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        smp_rx_msg_t msg;
        msg.conn = conn_handle;
        msg.len = len;
        if (os_mbuf_copydata(ctxt->om, 0, len, msg.data) != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        smp_conn_handle = conn_handle;
        if (xQueueSend(s_smp_q, &msg, 0) != pdTRUE) {
            ESP_LOGW(TAG, "SMP queue full, drop %u bytes", len);
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def smp_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &smp_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &smp_chr_uuid.u,
                .access_cb = smp_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &smp_chr_val_handle,
            },
            {
                0,
            },
        },
    },
    {
        0,
    },
};

const struct ble_gatt_svc_def *smp_ota_svc_defs(void)
{
    return smp_svcs;
}

void smp_ota_set_busy_callback(smp_ota_busy_cb_t cb)
{
    s_busy_cb = cb;
}

static void confirm_timer_cb(void *arg)
{
    (void)arg;
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        s_confirmed = true;
        ESP_LOGI(TAG, "OTA image auto-confirmed");
        /* After first boot of a newly swapped image, promote slot1 hash to slot0. */
        if (s_slot1_hash_valid) {
            nvs_save_slot0_hash(s_slot1_hash);
        } else if (!s_slot0_hash_valid) {
            /* Factory / first boot: derive a stable hash from app desc version string. */
            uint8_t h[32];
            const char *v = running_version();
            mbedtls_sha256((const uint8_t *)v, strlen(v), h, 0);
            nvs_save_slot0_hash(h);
        }
    } else {
        ESP_LOGW(TAG, "auto-confirm: %s", esp_err_to_name(err));
    }
}

void smp_ota_init(void)
{
    if (s_smp_q == NULL) {
        s_smp_q = xQueueCreate(4, sizeof(smp_rx_msg_t));
        xTaskCreate(smp_worker_task, "smp_ota", 8192, NULL, 5, NULL);
    }

    nvs_load_slot0_hash();
    nvs_load_slot1();

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t st = ESP_OTA_IMG_VALID;
    bool pending = false;
    if (running && esp_ota_get_state_partition(running, &st) == ESP_OK) {
        pending = (st == ESP_OTA_IMG_PENDING_VERIFY);
    }

    /*
     * After a successful OTA swap the running image's content hash is what the
     * host stored as "slot1" during upload.  Expose it as slot0 so
     * ota_updater.py can match hash + active + confirmed.
     */
    if (s_slot1_hash_valid &&
        (!s_slot0_hash_valid || pending ||
         memcmp(s_slot0_hash, s_slot1_hash, 32) != 0)) {
        /* If we just booted the uploaded image, slot1 hash describes us. */
        if (pending || !s_slot0_hash_valid) {
            memcpy(s_slot0_hash, s_slot1_hash, 32);
            s_slot0_hash_valid = true;
        }
    }

    if (pending) {
        s_confirmed = false;
        ESP_LOGI(TAG, "Pending verify — will auto-confirm in 3s");
        const esp_timer_create_args_t targs = {
            .callback = &confirm_timer_cb,
            .name = "ota_confirm",
        };
        esp_timer_handle_t th;
        if (esp_timer_create(&targs, &th) == ESP_OK) {
            esp_timer_start_once(th, 3000000);
        }
    } else {
        s_confirmed = true;
        if (!s_slot0_hash_valid) {
            uint8_t h[32];
            const char *v = running_version();
            mbedtls_sha256((const uint8_t *)v, strlen(v), h, 0);
            nvs_save_slot0_hash(h);
        }
    }

    ESP_LOGI(TAG, "SMP OTA ready running=%s ver=%s confirmed=%d",
             running ? running->label : "?", running_version(), (int)s_confirmed);
}
