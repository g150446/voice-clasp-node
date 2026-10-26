#include "axp192.h"

#include <stddef.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "hn_axp";

#define AXP_I2C_PORT  I2C_NUM_0
#define AXP_I2C_ADDR  0x34
#define AXP_SDA_GPIO  21
#define AXP_SCL_GPIO  22
#define AXP_I2C_HZ    400000
#define AXP_TIMEOUT_MS 50

static bool s_ready;

static esp_err_t axp_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(AXP_I2C_PORT, AXP_I2C_ADDR, buf, sizeof(buf),
                                      pdMS_TO_TICKS(AXP_TIMEOUT_MS));
}

static esp_err_t axp_read(uint8_t reg, uint8_t *val)
{
    return i2c_master_write_read_device(AXP_I2C_PORT, AXP_I2C_ADDR, &reg, 1, val, 1,
                                        pdMS_TO_TICKS(AXP_TIMEOUT_MS));
}

static esp_err_t axp_read_n(uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_write_read_device(AXP_I2C_PORT, AXP_I2C_ADDR, &reg, 1, buf, n,
                                        pdMS_TO_TICKS(AXP_TIMEOUT_MS));
}

static uint8_t axp_read8(uint8_t reg)
{
    uint8_t v = 0;
    (void)axp_read(reg, &v);
    return v;
}

static uint16_t axp_read12(uint8_t reg)
{
    uint8_t buf[2] = { 0, 0 };
    if (axp_read_n(reg, buf, 2) != ESP_OK) {
        return 0;
    }
    return (uint16_t)((buf[0] << 4) | (buf[1] & 0x0f));
}

esp_err_t axp192_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = AXP_SDA_GPIO,
        .scl_io_num = AXP_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AXP_I2C_HZ,
    };
    esp_err_t ret = i2c_param_config(AXP_I2C_PORT, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_driver_install(AXP_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install: %s", esp_err_to_name(ret));
        return ret;
    }

    /* M5StickC Plus / Plus SE factory sequence. */
    ESP_ERROR_CHECK(axp_write(0x28, 0xcc));                 /* LDO2/LDO3 = 3.0 V (BL / LCD) */
    ESP_ERROR_CHECK(axp_write(0x82, 0xff));                 /* ADC all enable */
    ESP_ERROR_CHECK(axp_write(0x33, 0xc0));                 /* charge 4.2 V / 100 mA */
    ESP_ERROR_CHECK(axp_write(0x12, (uint8_t)(axp_read8(0x12) | 0x4d))); /* EXTEN, LDO2, LDO3, DCDC1 */
    ESP_ERROR_CHECK(axp_write(0x36, 0x0c));                 /* 128 ms on / 4 s off */
    ESP_ERROR_CHECK(axp_write(0x91, 0xf0));                 /* GPIO0 LDO 3.3 V (mic) */
    ESP_ERROR_CHECK(axp_write(0x90, 0x02));                 /* GPIO0 = LDO */
    ESP_ERROR_CHECK(axp_write(0x30, 0x80));                 /* disable VBUS hold limit */
    ESP_ERROR_CHECK(axp_write(0x39, 0xfc));                 /* temp protection */
    ESP_ERROR_CHECK(axp_write(0x35, 0xa2));                 /* RTC backup charge */
    ESP_ERROR_CHECK(axp_write(0x32, 0x46));                 /* battery detection */

    s_ready = true;
    int mv = 0;
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        mv = axp192_battery_millivolt();
        if (mv > 0) {
            break;
        }
    }
    ESP_LOGI(TAG, "AXP192 ready bat=%d mV vbus=%d chg=%d",
             mv, axp192_vbus_present(), axp192_is_charging());
    return ESP_OK;
}

void axp192_set_ldo2(bool on)
{
    if (!s_ready) {
        return;
    }
    uint8_t v = axp_read8(0x12);
    if (on) {
        v = (uint8_t)(v | (1u << 2));
    } else {
        v = (uint8_t)(v & ~(1u << 2));
    }
    (void)axp_write(0x12, v);
}

void axp192_set_mic_ldo(bool on)
{
    if (!s_ready) {
        return;
    }
    /* GPIO0: 0x02 = LDO output, 0x07 = floating. */
    (void)axp_write(0x90, on ? 0x02 : 0x07);
}

void axp192_prepare_sleep(void)
{
    if (!s_ready) {
        return;
    }
    (void)axp_write(0x31, (uint8_t)(axp_read8(0x31) | (1u << 3)));
    (void)axp_write(0x90, 0x00);
    (void)axp_write(0x12, (uint8_t)(axp_read8(0x12) & 0xa1));
}

void axp192_wake_rails(void)
{
    if (!s_ready) {
        return;
    }
    (void)axp_write(0x12, (uint8_t)(axp_read8(0x12) | 0x4d));
    (void)axp_write(0x91, 0xf0);
    (void)axp_write(0x90, 0x02);
}

int axp192_battery_millivolt(void)
{
    if (!s_ready) {
        return 0;
    }
    /* 1.1 mV / LSB, 12-bit at 0x78. */
    return (int)((axp_read12(0x78) * 11 + 5) / 10);
}

bool axp192_is_charging(void)
{
    if (!s_ready) {
        return false;
    }
    return (axp_read8(0x01) & 0x40) != 0;
}

bool axp192_vbus_present(void)
{
    if (!s_ready) {
        return false;
    }
    return (axp_read8(0x00) & 0x20) != 0;
}
