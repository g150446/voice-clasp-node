/*
 * nordic-main: XIAO nRF52840 Sense → Handy BLE firmware
 *
 * Audio streaming via BLE with Handy-compatible protocol.
 * IMU gesture detection triggers BLE event packets to control Handy recording.
 * RGB LED indicates device state.
 *
 * BLE Protocol (Handy-compatible):
 *   Audio Service:  00000001-0000-1000-8000-00805f9b34fb
 *     TX 0x0002 (NOTIFY): [seq][0xAA][PCM...]  — audio stream
 *                         [0x00][0x55][event]  — event packets
 *     RX 0x0003 (WRITE):  0x01=start, 0x00=stop
 *
 * LED states (active-low, Zephyr GPIO_ACTIVE_LOW flag handles inversion):
 *   Boot          White  1s then advertising or idle
 *   Advertising   Blue   200ms on / 1800ms off (no connections)
 *   Idle          Off    connected idle
 *   Recording     Red    solid
 *   Error         Red    200ms fast blink
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>

#include <hal/nrf_power.h>
#include <math.h>

#include "audio_capture.h"
#include "app_diagnostics.h"
#include "tap_detector.h"
#include "tap_processing.h"
#include "debug_session.h"

LOG_MODULE_REGISTER(nordic_main, LOG_LEVEL_INF);

/* Atomic mirror for diagnostics in the audio / Bluetooth threads. Policy and
 * owner lifetime are managed solely by the main loop. Arguments are not
 * evaluated when diagnostics are off. Deferred Zephyr logging is retained. */
static atomic_t debug_active;
bool app_debug_enabled(void) { return atomic_get(&debug_active) != 0; }

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define BLE_DEVICE_NAME         "HarnessNode"
#define PCM_PACKET_SIZE         200
#define AUDIO_FRAME_SAMPLES     320   /* 20 ms @ 16 kHz */
#define AUDIO_FRAME_BYTES       (AUDIO_FRAME_SAMPLES * sizeof(int16_t))
/* Software queue decouples DMIC capture from BLE Notify backpressure. */
#define AUDIO_FRAME_QUEUE_LEN   16    /* 320 ms of PCM */
#define AUDIO_CAPTURE_MAX_CONSEC_FAIL 20
#define AUDIO_NOTIFY_IN_FLIGHT  6
#define AUDIO_NOTIFY_RETRY_MS   2
#define AUDIO_NOTIFY_DRAIN_TIMEOUT_MS 1000
#define AUDIO_QUEUE_DRAIN_TIMEOUT_MS 500
#define WDT_NODE                DT_NODELABEL(wdt0)
/* Must stay LONGER than the BLE supervision timeout (8 s, see conn_param_work).
 * With 5 s the watchdog always won: a peer that stopped ACKing parked the main
 * loop in bt_gatt_notify() and the SoC reset before the controller could drop
 * the link and free the buffers.  0.0.98/0.0.99 survived only because their
 * supervision timeout was 4 s, i.e. below the old 5 s dog. */
#define WDT_TIMEOUT_MS          12000
#define MAIN_LOOP_INTERVAL_MS   25
#define ERROR_BLINK_MS          200
#define ADV_LED_ON_MS           200
#define ADV_LED_OFF_MS          1800
/* Unconnected this long → SYSTEMOFF.  Two tiers: a peer that was connected and
 * went away is probably coming back (Doze, out of range, app restart), so the
 * post-drop fuse keeps a reconnect buffer.  It was 30 min, but the software tap
 * classifier polls the IMU every MAIN_LOOP_INTERVAL_MS while awake, so that long
 * window dominated standby drain after every connect.  5 min keeps a transient-
 * drop buffer yet caps the idle cost; a drop past it is resumed by a tap. */
#define ADV_NO_CONN_SLEEP_MS     (3 * 60 * 1000)   /* cold boot / after tap wake */
#define ADV_AFTER_DROP_SLEEP_MS  (5 * 60 * 1000)   /* after losing a peer */
#define ADV_LOW_BATT_PCT         15   /* below this, sleep on the short fuse */
#define HEARTBEAT_INTERVAL_MS    30000
#define MAIN_LOOP_STALL_WARN_MS  500
/* Fast advertising is kept this long after the connection count changes so a
 * second client (Mac) is still found quickly; after that it drops to slow. */
#define ADV_FAST_BURST_MS        30000
#define BATTERY_POLL_INTERVAL_MS 60000  /* 1 minute */
#define BATTERY_CAPACITY_MAH        40   /* LiPo capacity in mAh */
#define BATTERY_CHARGE_CURRENT_MA  100   /* BQ25120 charge current on XIAO nRF52840 */
#define BATTERY_CHARGE_MAX_EST_PCT 100   /* linear to 100% (slightly early, CV phase ignored) */
#define SLEEP_IDLE_TIMEOUT_MS   10000  /* idle this long with no motion → light sleep */
#define SLEEP_POLL_INTERVAL_MS  50

#define LED_RED_NODE    DT_ALIAS(led0)
#define LED_GREEN_NODE  DT_ALIAS(led1)
#define LED_BLUE_NODE   DT_ALIAS(led2)

/* IMU / motion detection (accel always on; gyro on-demand) */
#define IMU_NODE                    DT_ALIAS(imu0)
#define ACCEL_ODR_HZ                416   /* must match FIFO / software detector */
BUILD_ASSERT(ACCEL_ODR_HZ == TAP_SAMPLE_HZ);
#define GYRO_ODR_HZ                 104
#define GYRO_FULL_SCALE_DPS         500
#define MOTION_SAMPLE_INTERVAL_MS   25
#define CALIBRATION_SAMPLES         25
#define CALIBRATION_ACCEL_SPAN_MAX_MS2 0.8f
#define ACTIVITY_WINDOW_SAMPLES     4
#define MOTION_ENTRY_ACTIVITY_MS2   8.0
#define MOTION_ENTRY_PEAK_MS2       2.4
#define MOTION_CONTINUE_ACTIVITY_MS2 4.0
#define MOTION_CONTINUE_PEAK_MS2    1.4
#define MOTION_SETTLE_ACTIVITY_MS2  4.0
#define MOTION_SETTLE_PEAK_MS2      1.4
#define MOTION_START_WINDOWS        2
#define MOTION_SETTLE_WINDOWS       2
#define BASELINE_ALPHA              0.03
#define REPORT_COOLDOWN_MS          700
#define MOTION_DURATION_SAMPLES     2

/* Runtime single/double tap classification uses 416 Hz acceleration FIFO.
 * Embedded taps are enabled ONLY while the CPU is in SYSTEMOFF, for wake. */
#define EVT_DOUBLE_TAP                 0x12
#define EVT_SINGLE_TAP                 0x14
#define LSM6DS3TR_C_REG_FIFO_CTRL1    0x06
#define LSM6DS3TR_C_REG_FIFO_CTRL5    0x0A
#define LSM6DS3TR_C_REG_FIFO_STATUS1  0x3A
#define LSM6DS3TR_C_REG_FIFO_DATA     0x3E
#define TAP_FIFO_RUNNING             0x36 /* ODR=416 Hz, continuous mode */
#define TAP_FIFO_MAX_SAMPLES         64   /* bound main-loop work / stale data */
#define LSM6DS3TR_C_REG_INT1_CTRL      0x0D
#define LSM6DS3TR_C_REG_CTRL1_XL       0x10
#define LSM6DS3TR_C_REG_CTRL6_C        0x15
#define LSM6DS3TR_C_REG_TAP_SRC        0x1C
#define LSM6DS3TR_C_REG_TAP_CFG        0x58
#define LSM6DS3TR_C_REG_TAP_THS_6D     0x59
#define LSM6DS3TR_C_REG_INT_DUR2       0x5A
#define LSM6DS3TR_C_REG_WAKE_UP_THS    0x5B
#define LSM6DS3TR_C_REG_MD1_CFG        0x5E
#define LSM6DS3TR_C_CTRL6_XL_HM_MODE   BIT(4)
#define LSM6DS3TR_C_TAP_CFG_MASK       (BIT(7) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define LSM6DS3TR_C_TAP_CFG_Z_LATCHED  (BIT(7) | BIT(1) | BIT(0))
#define LSM6DS3TR_C_TAP_THS_MASK       0x1F
#define LSM6DS3TR_C_TAP_THS_BALANCED   0x08 /* ~1 g wake tap at +/-4 g, wake only */
#define LSM6DS3TR_C_INT_DUR_BALANCED   0x4A
#define LSM6DS3TR_C_DOUBLE_TAP_ENABLE  BIT(7)
#define LSM6DS3TR_C_MD1_TAP_MASK       (BIT(6) | BIT(3))
#define LSM6DS3TR_C_MD1_SINGLE_TAP     BIT(6)
#define LSM6DS3TR_C_MD1_DOUBLE_TAP     BIT(3)
/* Legacy D0 register snapshot layout is retained: ret now reports FIFO reads
 * and count reports software classifications. D1 (raw TAP_SRC) is retired.
 * D2 acknowledges register diagnostics; runtime ODR/range/routing are locked. */
#define EVT_TAP_DIAG                   0xD0
#define EVT_TAP_REG_ACK                0xD2

/*
 * Boot report, sent once per connection with the greeting.
 *
 * A watchdog reset and a plain radio-side link loss look identical from the
 * peer: both surface as HCI 0x08 / Android status=8. Telling them apart used
 * to need the USB serial log, which is exactly what is unavailable when the
 * device is being worn. The Android app already prints every unrecognised TX
 * event as hex, so publishing the reset cause and the crash box here makes the
 * previous boot readable straight out of logcat with no app change.
 *
 *   [00][55][D3][cause u32][crash_valid][stage][prev_up_s u16][hb u16][ndrop u16]
 *
 * prev_up_s / hb / ndrop describe the run that died; they are zero unless
 * crash_valid is 1. A reconnect whose report repeats the previous cause and
 * carries crash_valid=0 means the firmware never restarted -- the link alone
 * dropped.
 */
#define EVT_BOOT_REPORT                0xD3

/* Ack for CMD_REBOOT_BOOTLOADER: [00][55][D4][mode][accepted].
 * mode 1 = Adafruit UF2 bootloader. accepted 0 means the request was refused
 * (currently: a recording is in progress). Sent before the reboot work runs, so
 * the host knows whether to start waiting for the mass-storage volume. */
#define EVT_REBOOT_ACK                 0xD4

/* Host-driven IMU register poke, so tap tuning does not need an OTA per try.
 * Writes are restricted to the tap/ODR registers listed in tap_reg_writable(). */
#define CMD_IMU_REG_WRITE              0x50
#define CMD_IMU_REG_READ               0x51

/* Reboot into the Adafruit UF2 bootloader: [0x60][0xB0][0x07].
 * MCUboot lives at 0x27000 and is not reachable over MCUmgr -- only the app's
 * secondary slot is -- so changing it needs a UF2 write, and the only documented
 * way in was a double-tap of the reset button (the 1200-baud trigger is not
 * implemented by Zephyr's CDC ACM, and serial DFU needs the nRF5 SDK protocol).
 * The two magic bytes are deliberate: audio_rx_write() has no unknown-command
 * branch and 0x00-0x07 are all no-argument commands, so a bare byte would be far
 * too easy to hit by accident for something this disruptive. */
#define CMD_REBOOT_BOOTLOADER          0x60
#define REBOOT_MAGIC_1                 0xB0
#define REBOOT_MAGIC_2                 0x07
/* Adafruit bootloader DFU_MAGIC_UF2_RESET, read out of GPREGRET at boot. */
#define BOOTLOADER_MAGIC_UF2           0x57


/*
 * Dorsal-side gesture classifier (accel + on-demand gyro_y).
 *
 * Board on the back of the wrist, component-side against skin.  Axes: X across
 * the forearm, Y along the forearm (pronation axis = gyro_y), Z normal to the
 * board (component-out). Sequence: palm-up horizontal dwell → lift →
 * rotate from the palm-up pose and hold still 0.5 s.
 * A horizontal, still pose held for 0.5 seconds is treated as the palm-up
 * candidate. Gyro enables when that dwell is accepted, stays on through
 * recording, and powers down when
 * recording ends (or the sequence resets without a match).
 * A match additionally requires a strong lift and a near-complete palm flip.
 * Recording stops on the reverse-of-lift hand-lower pulse.
 */
#define GESTURE_GRAVITY_MS2                  9.80665f
#define GESTURE_RAD_TO_DEG                   57.29577951308232
#define GESTURE_START_ARM_MS                2500
/* Board-flat gate: |z / |a||. Hold palm-down is a flip from the palm-up pose. */
#define GESTURE_START_PALM_UP_Z_MIN_RATIO       0.75f
#define GESTURE_START_GRAVITY_MIN_MS2           8.5f
#define GESTURE_START_GRAVITY_MAX_MS2          11.5f
#define GESTURE_PALM_UP_DWELL_MS                 500
#define GESTURE_PALM_UP_DWELL_TILT_MAX_DEG      20.0f
#define GESTURE_START_QUIET_ACCEL_MS2             4.0f
/* Hold flip: gravity rotation plus a coupled lift/pronation motion. */
#define GESTURE_PRONATION_MIN_DEG               15.0f
#define GESTURE_PRONATION_Z_RATIO_DONE           0.40f
#define GESTURE_PRONATION_Z_SIGN_MIN_MS2         2.0f
#define GESTURE_PRONATION_GRAVITY_MIN_MS2        7.5f
#define GESTURE_PRONATION_GRAVITY_MAX_MS2       12.5f
/* Legacy palm-up stop constants kept for diag field layout only (unused). */
#define GESTURE_OUTBOUND_MIN_DEG                 20.0f
#define GESTURE_OUTBOUND_Z_RATIO_DONE            0.50f
#define GESTURE_OUTBOUND_TILT_MIN_DEG            30.0f
/* gyro_y is forearm pronation; gyro_x supplies the arm-lift coupling. */
#define GESTURE_GYRO_SETTLE_MS                    50
#define GESTURE_GYRO_BIAS_CAPTURE_MAX_DPS        15.0f
#define GESTURE_HOLD_GYRO_INTEGRATE_RATE_DPS     10.0f
/* 0.0.70: natural palm-down had ∫~35°; 50° rejected valid starts. */
#define GESTURE_HOLD_GYRO_ANGLE_MIN_DEG         30.0f
#define GESTURE_HOLD_GYRO_XY_PEAK_RATIO_MIN      0.42f
/* XY waiver only if lift fired before pronation AND impulse is strong. */
#define GESTURE_LIFT_PREFLIP_MAX_DEG            50.0f
#define GESTURE_LIFT_XY_WAIVER_IMPULSE_MIN_MS    0.30f
/* Recording-stop (0.0.69): reverse of lift-axis linear pulse, then settle.
 * Palm-up / gyro_y stop paths removed. Pulse window rejects long vehicle G. */
/* 0.0.71: Android real-use (リンゴ log) stop barely passed opp=0.189/peak=0.57
 * after ~13s; Mac deliberate lower was opp~1.4. Loosen for phone-held lowers. */
/* 0.0.73: soft-lower (time-decay thresholds, pulse gap hysteresis, resilient
 * settle) + stop_near_miss diag. 2026-08-28 リンゴ delay≈24s opp=0.181. */
#define GESTURE_STOP_OPP_ACCEL_MIN_MS2           0.25f
#define GESTURE_STOP_OPP_ACCEL_SOFT_MS2          0.18f
#define GESTURE_STOP_OPP_ACCEL_SOFT2_MS2         0.15f
#define GESTURE_STOP_OPP_CONSECUTIVE_SAMPLES        2
#define GESTURE_STOP_OPP_IMPULSE_MIN_MS           0.10f
#define GESTURE_STOP_OPP_IMPULSE_SOFT_MS          0.08f
#define GESTURE_STOP_OPP_IMPULSE_SOFT2_MS         0.07f
#define GESTURE_STOP_OPP_IMPULSE_LIFT_RATIO       0.20f
#define GESTURE_STOP_OPP_IMPULSE_LIFT_CAP_MS      0.35f
#define GESTURE_STOP_OPP_PULSE_MIN_MS              60
#define GESTURE_STOP_OPP_PULSE_SLOW_MS            180
#define GESTURE_STOP_OPP_PULSE_MAX_MS             2000
#define GESTURE_STOP_OPP_TAIL_MS2                0.03f
#define GESTURE_STOP_PULSE_GAP_MS                   50
#define GESTURE_STOP_SETTLE_MS                     80
#define GESTURE_STOP_SETTLE_SOFT_MS                50
#define GESTURE_STOP_SETTLE_SOFT2_MS               40
#define GESTURE_STOP_SETTLE_LINEAR_MS2            4.0f
#define GESTURE_STOP_SETTLE_LINEAR_SOFT_MS2       5.0f
#define GESTURE_STOP_SETTLE_SPIKE_SAMPLES           2
#define GESTURE_STOP_SOFTEN_AFTER_MS             5000
#define GESTURE_STOP_SOFTEN2_AFTER_MS           10000
#define GESTURE_STOP_NEAR_MISS_MIN_INTERVAL_MS    400
#define GESTURE_STOP_NEAR_MISS_MIN_PULSE_MS        40
/* Legacy gyro stop constants (unused after 0.0.69). */
#define GESTURE_STOP_GYRO_INTEGRATE_RATE_DPS     20.0f
#define GESTURE_STOP_GYRO_ANGLE_MIN_DEG          45.0f
#define GESTURE_STOP_GYRO_ANGLE_PEAK_MIN_DPS     30.0f
#define GESTURE_STOP_GYRO_PEAK_DPS              50.0f
#define GESTURE_STOP_HOLD_MS                     500
/* Gyro quiet only gates *entering* hold; residual wrist rate during hold is looser. */
#define GESTURE_FINAL_QUIET_RATE_DPS            90.0f
/* Final: upward acceleration pulse, braking pulse, then a quiet hold. */
#define GESTURE_LIFT_ACCEL_MIN_MS2               0.40f
#define GESTURE_LIFT_BRAKE_MIN_MS2               0.15f
/* 0.0.68: daily FP had +imp ~0.10–0.26; require a clearer arm lift. */
#define GESTURE_LIFT_POS_IMPULSE_MIN_MS           0.30f
/* 0.0.72 final match gate: reject daily motion after the permissive early
 * palm-down latch. Android false positives were below this combined gate. */
#define GESTURE_MATCH_POS_IMPULSE_MIN_MS          0.65f
#define GESTURE_MATCH_PRONATION_MIN_DEG          140.0f
#define GESTURE_LIFT_NEG_IMPULSE_MIN_MS           0.015f
#define GESTURE_LIFT_BRAKE_RATIO_MIN              0.05f
#define GESTURE_LIFT_PULSE_MIN_MS                  150
#define GESTURE_LIFT_CONSECUTIVE_SAMPLES             2
#define GESTURE_LIFT_FINAL_TILT_MAX_DEG           15.0f
#define GESTURE_FINAL_HOLD_MS                     500
/* 0.0.74: dwell後の迷い猶予（Mac trial final_accel_missing ~5s）。 */
#define GESTURE_LIFT_START_TIMEOUT_MS            8000
#define GESTURE_MOTION_COMPLETE_MAX_MS           4500
#define GESTURE_FINAL_STILL_RMS_MS2              3.0f
/* 0.0.74: Android final_hold_interrupted 多発 → やや緩和。 */
#define GESTURE_FINAL_HOLD_RMS_EXIT_MS2          4.0f
#define GESTURE_FINAL_HOLD_RMS_EXIT_SAMPLES         3
#define GESTURE_FINAL_RMS_WINDOW_SAMPLES             4
#define GESTURE_LIFT_NEAR_MISS_MIN_INTERVAL_MS    400
#define GESTURE_LIFT_NEAR_MISS_MIN_IMPULSE_MS    0.05f
#define GESTURE_RETRIGGER_BLOCK_MS               3000
#define GESTURE_GRAVITY_LP_TAU_S                 0.30f
#define GESTURE_QUIET_ACCEL_MS2                  3.0f
/* Compiles in the gesture history and 6-axis trajectory dump.  On since
 * 0.0.91: the dump is now switched at runtime by CMD_SET_GESTURE_CAPTURE
 * (off at boot), so carrying it costs ~21 KB of RAM and nothing on the air
 * until the host asks.  Set to 0 for a lean build; the capture ack then
 * always reports off. */
#ifndef GESTURE_DEBUG_HISTORY
#define GESTURE_DEBUG_HISTORY                     1
#endif
#define GESTURE_HISTORY_CAP                         96
#define GESTURE_HISTORY_FLUSH_GAP_MS                 5
#define GESTURE_TRAJECTORY_CAP                      384
#define GESTURE_TRAJECTORY_CHUNK_SAMPLES              8
#define GESTURE_TRAJECTORY_VERSION                    1
#define GESTURE_HOST_COLLECTION_MS                  6000

/* BLE gesture diagnostics: event 0x30, stage/reason + three float values. */
#define GESTURE_DIAG_OUTBOUND_START           0x01
#define GESTURE_DIAG_OUTBOUND_READY           0x02
#define GESTURE_DIAG_FINAL_HOLD_START         0x07
#define GESTURE_DIAG_FINAL_READY              0x08
#define GESTURE_DIAG_MATCH                    0x09
/* Match metrics: v1=xy_ratio, v2=pos_imp_at_lift, v3=roll_at_lift.
 * reason bit0=lift_before_flip, bit1=xy_waived. */
#define GESTURE_DIAG_MATCH_DETAIL             0x0A
/* Stop near-miss: v1=opp_imp, v2=peak, v3=need_imp (or pulse_ms). */
#define GESTURE_DIAG_STOP_NEAR_MISS           0x0B
#define GESTURE_DIAG_STOP_PALM_UP             0x0C  /* legacy name; hand-lower */
#define GESTURE_DIAG_GYRO_ENABLED             0x0D
#define GESTURE_DIAG_GYRO_DISABLED            0x0E
#define GESTURE_DIAG_OUTBOUND_GYRO            0x0F  /* roll_deg, peak_dps, sign */
/* Stop hand-lower: v1=neg_impulse, v2=peak_a_opp, v3=pulse_ms */
#define GESTURE_DIAG_STOP_HAND_LOWER          0x0C
#define GESTURE_DIAG_HOLD_SAMPLE              0x22  /* rms, tilt_deg, |gy|_dps */
#define GESTURE_DIAG_MOTION_COMPLETE           0x23  /* elapsed_ms, peak_y, roll */
#define GESTURE_DIAG_PALM_DOWN_GATE            0x24  /* gate-specific metrics */
/* Lift near-miss: v1=peak_a_up, v2=pos_imp, v3=elapsed_ms (0.0.74). */
#define GESTURE_DIAG_LIFT_NEAR_MISS           0x25
#define GESTURE_DIAG_FINAL_SAMPLE             0x21
#define GESTURE_DIAG_WAIT_REJECT               0x10
#define GESTURE_DIAG_RESET                     0x80
#define GESTURE_DEBUG_FINAL_PERIOD_MS           100

/* Operation mode controlled by the Android companion.  NORMAL allows gesture
 * start/stop only when gesture_detect is also on; DRIVING disables gesture
 * start/stop.  Single tap toggles recording in both modes; double tap is
 * notify-only.  Gesture detect itself is a separate runtime switch (0x07),
 * default off, because false triggers dominate everyday use. */
#define CMD_SET_OPERATION_MODE                  0x05
#define CMD_SET_GESTURE_CAPTURE                 0x06
#define CMD_SET_GESTURE_DETECT                  0x07
#define OPERATION_MODE_NORMAL                   0x00
#define OPERATION_MODE_DRIVING                 0x01
#define OPERATION_MODE_PENDING_NONE             0xff
#define EVT_OPERATION_MODE                     0x40
#define EVT_GESTURE_CAPTURE                    0x39
#define EVT_GESTURE_DETECT                     0x3A

/* History batch events (only when GESTURE_DEBUG_HISTORY=1). */
#define EVT_GESTURE_HISTORY_BEGIN             0x33
#define EVT_GESTURE_HISTORY_ENTRY             0x34
#define EVT_GESTURE_HISTORY_END               0x35
#define EVT_GESTURE_TRAJECTORY_BEGIN          0x36
#define EVT_GESTURE_TRAJECTORY_CHUNK          0x37
#define EVT_GESTURE_TRAJECTORY_END            0x38

#define GESTURE_DIAG_REASON_NONE               0x00
#define GESTURE_DIAG_REASON_QUIET_NOT_READY    0x01
#define GESTURE_DIAG_REASON_START_NOT_PALM_UP  0x02
#define GESTURE_DIAG_REASON_OUTBOUND_RATE_LOW  0x03
#define GESTURE_DIAG_REASON_OUTBOUND_TIMEOUT   0x11
#define GESTURE_DIAG_REASON_INCOMPLETE_OUTBOUND 0x12
#define GESTURE_DIAG_REASON_FINAL_HOLD_INTERRUPTED 0x1a
#define GESTURE_DIAG_REASON_FINAL_HOLD_TIMEOUT 0x1b
#define GESTURE_DIAG_REASON_SEQUENCE_TIMEOUT   0x1c
#define GESTURE_DIAG_REASON_FINAL_ACCEL_MISSING 0x1d
#define GESTURE_DIAG_REASON_FINAL_BRAKE_MISSING 0x1e
#define GESTURE_DIAG_REASON_FINAL_BRAKE_RATIO_LOW 0x1f
#define GESTURE_DIAG_REASON_FINAL_TILT_UNSTABLE 0x20
#define GESTURE_DIAG_REASON_FINAL_PULSE_DURATION_INVALID 0x21
#define GESTURE_DIAG_REASON_SHAKE_NOT_OSCILLATORY 0x22
#define GESTURE_DIAG_REASON_LIFT_PALM_STILL_UP     0x23
#define GESTURE_DIAG_REASON_MOTION_TOO_SLOW         0x24
#define GESTURE_DIAG_REASON_PALM_DOWN_GRAVITY_LOW   0x25
#define GESTURE_DIAG_REASON_PALM_DOWN_GYRO_ANGLE_LOW 0x26
#define GESTURE_DIAG_REASON_PALM_DOWN_XY_RATIO_LOW  0x27
#define GESTURE_DIAG_REASON_PALM_DOWN_GATE_FAILED   0x28
#define GESTURE_DIAG_REASON_MATCH_LIFT_IMPULSE_LOW  0x29
#define GESTURE_DIAG_REASON_MATCH_PRONATION_LOW     0x2a
#define GESTURE_DIAG_REASON_MATCH_GATE_FAILED       0x2b
#define GESTURE_DIAG_REASON_STOP_IMPULSE_LOW        0x2c
#define GESTURE_DIAG_REASON_STOP_PEAK_LOW           0x2d
#define GESTURE_DIAG_REASON_STOP_PULSE_SHORT        0x2e
#define GESTURE_DIAG_REASON_STOP_PULSE_LONG         0x2f
#define GESTURE_DIAG_REASON_LIFT_PULSE_WEAK         0x30
#define GESTURE_DIAG_REASON_LIFT_START_TIMEOUT      0x31

/* ============================================================================
 * BLE UUIDs (Handy-compatible)
 * ============================================================================ */

/* Audio Service: 00000001-0000-1000-8000-00805f9b34fb */
#define AUDIO_UUID_SERVICE \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00

/* Audio TX: 00000002-... (Notify) */
#define AUDIO_UUID_TX_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00

/* Audio RX: 00000003-... (Write) */
#define AUDIO_UUID_RX_CHAR \
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
    0x00, 0x10, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00

/* ============================================================================
 * Global Variables
 * ============================================================================ */

/* Advertising data */
static const uint8_t adv_flags_data[] = { BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR };
static const uint8_t adv_uuid_data[] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
};
static const struct bt_data adv_data[] = {
    BT_DATA(BT_DATA_FLAGS, adv_flags_data, sizeof(adv_flags_data)),
    BT_DATA(BT_DATA_UUID128_ALL, adv_uuid_data, sizeof(adv_uuid_data)),
};
static const struct bt_data scan_rsp[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, sizeof(BLE_DEVICE_NAME) - 1),
};

#define MAX_CONNS 2
static struct bt_conn *connections[MAX_CONNS];
static int primary_idx = -1;

static struct bt_conn *get_primary_conn(void) {
    return (primary_idx >= 0) ? connections[primary_idx] : NULL;
}

static int conn_index(struct bt_conn *conn) {
    for (int i = 0; i < MAX_CONNS; i++)
        if (connections[i] == conn) return i;
    return -1;
}

static int free_slot(void) {
    for (int i = 0; i < MAX_CONNS; i++)
        if (connections[i] == NULL) return i;
    return -1;
}

static int active_count(void) {
    int n = 0;
    for (int i = 0; i < MAX_CONNS; i++) if (connections[i]) n++;
    return n;
}

/* Audio state */
static uint8_t tx_packet[512];
static uint8_t seq_num;
static volatile bool is_recording;
static bool ble_primary_want_fast;
static int64_t unconnected_since_ms;
/* true once any peer has connected since boot / since the last SYSTEMOFF wake:
 * selects the long SYSTEMOFF fuse so a transient drop is not fatal. */
static bool had_conn_this_cycle;
static int64_t last_recording_stop_ms;
/* Per-slot diagnosis state, printed on disconnect. */
static int64_t conn_started_ms[MAX_CONNS];
static uint16_t conn_param_interval[MAX_CONNS];
static uint16_t conn_param_latency[MAX_CONNS];
static uint16_t conn_param_timeout[MAX_CONNS];
static bool conn_param_seen[MAX_CONNS];
static volatile bool recording_requested;
static volatile bool stop_requested;
static volatile bool capture_enabled;
static volatile bool capture_fatal;
static volatile uint8_t operation_mode = OPERATION_MODE_NORMAL;
static volatile uint8_t operation_mode_pending = OPERATION_MODE_PENDING_NONE;
K_SEM_DEFINE(audio_control_wake, 0, 1);
K_SEM_DEFINE(audio_capture_wake, 0, 1);
K_MSGQ_DEFINE(serial_rx_q, sizeof(uint8_t), 64, 1);
K_SEM_DEFINE(audio_notify_slots, AUDIO_NOTIFY_IN_FLIGHT, AUDIO_NOTIFY_IN_FLIGHT);

struct audio_pcm_frame {
    uint16_t nbytes;
    uint8_t pcm[AUDIO_FRAME_BYTES];
};

K_MSGQ_DEFINE(audio_frame_q, sizeof(struct audio_pcm_frame),
              AUDIO_FRAME_QUEUE_LEN, 4);

struct audio_session_stats {
    uint32_t frames_captured;
    uint32_t frames_sent;
    uint32_t bytes_sent;
    uint32_t notifies_sent;
    uint32_t notify_wait_count;
    uint32_t notify_wait_max_ms;
    uint32_t notify_retry_count;
    uint32_t dmic_read_errors;
    uint32_t last_dmic_errno;
    uint32_t dmic_overruns;
    uint32_t queue_high_watermark;
    int64_t session_start_ms;
    int64_t dmic_stop_ms;
};

static struct audio_session_stats audio_stats;

/* Audio control/sender thread + dedicated DMIC capture thread */
static K_THREAD_STACK_DEFINE(audio_stack, 4096);
static K_THREAD_STACK_DEFINE(audio_capture_stack, 2048);
static struct k_thread audio_thread_data;
static struct k_thread audio_capture_thread_data;

/* Serial command thread */
static K_THREAD_STACK_DEFINE(serial_stack, 1024);
static struct k_thread serial_thread_data;

/* Deferred advertising restart — calling bt_le_adv_start() directly from
 * within a BT callback can fail silently on Nordic NCS because the BT stack
 * lock may already be held.  Schedule via a work item instead. */
static struct k_work_delayable adv_work;

/* Deferred BLE connection parameter update.
 * After primary/secondary role changes we schedule this work item so that
 * bt_conn_le_param_update() is called from the system workqueue (not from
 * inside a BT/GATT callback where the BT lock is already held). */
static struct k_work_delayable conn_param_work;

/* Deferred reboot into the Adafruit UF2 bootloader (RX 0x60 / serial 'b').
 * Deferred for two reasons: sys_reboot() must not run on the BT RX thread, and
 * notify_one() only queues the ack -- the packet is not on air until
 * ctrl_notify_done() fires, so resetting synchronously would swallow it. */
static struct k_work_delayable reboot_work;

static bool primary_conn_should_be_fast(void)
{
    return is_recording;
}

static void sync_primary_conn_speed(void)
{
    bool want_fast = primary_conn_should_be_fast();

    if (want_fast == ble_primary_want_fast) {
        return;
    }
    ble_primary_want_fast = want_fast;
    k_work_schedule(&conn_param_work, K_MSEC(200));
}

static void conn_param_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    /* Fast params for primary while recording. */
    static const struct bt_le_conn_param fast_param = {
        .interval_min = 6,    /* 6 × 1.25 ms = 7.5 ms */
        .interval_max = 12,   /* 12 × 1.25 ms = 15 ms  */
        .latency      = 0,
        .timeout      = 800,  /* 8 s — keep timeout identical across sets */
    };
    /* Idle primary: 30–50 ms (tap latency vs radio cost). */
    static const struct bt_le_conn_param idle_param = {
        .interval_min = 24,   /* 30 ms */
        .interval_max = 40,   /* 50 ms */
        .latency      = 0,
        .timeout      = 800,
    };
    /* Secondary: 100–150 ms. Wider 200–500 ms + 4 s timeout dropped Android. */
    static const struct bt_le_conn_param slow_param = {
        .interval_min = 80,   /* 100 ms */
        .interval_max = 120,  /* 150 ms */
        .latency      = 0,
        .timeout      = 800,
    };
    bool primary_fast = primary_conn_should_be_fast();

    ble_primary_want_fast = primary_fast;

    for (int i = 0; i < MAX_CONNS; i++) {
        if (!connections[i]) continue;
        const struct bt_le_conn_param *p;
        const char *label;

        if (i == primary_idx) {
            if (primary_fast) {
                p = &fast_param;
                label = "fast(7.5ms)";
            } else {
                p = &idle_param;
                label = "idle(30ms)";
            }
        } else {
            p = &slow_param;
            label = "slow(100ms)";
        }
        /* On a peripheral this only records the request: the host sends it
         * from conn->deferred_work ~5 s after connect. Nothing is on air yet,
         * and le_param_updated() is the only proof it was accepted. */
        int ret = bt_conn_le_param_update(connections[i], p);
        if (ret) {
            LOG_WRN(">>> conn_param[%d] request failed: %d\n", i, ret);
        } else {
            APP_TRACE(">>> conn_param[%d]: %s requested (host sends at T+5s)\n",
                   i, label);
        }
    }
}

/* IMU device */
static const struct device *const imu     = DEVICE_DT_GET(IMU_NODE);
static const struct i2c_dt_spec imu_i2c = I2C_DT_SPEC_GET(IMU_NODE);
/* INT1 is used only to wake from SYSTEMOFF. */
static const struct gpio_dt_spec imu_int1 = GPIO_DT_SPEC_GET(IMU_NODE, irq_gpios);
static const struct sensor_trigger tap_trigger = {
    .type = SENSOR_TRIG_DATA_READY,
    .chan = SENSOR_CHAN_ACCEL_XYZ,
};
static struct tap_detector software_tap;
static struct tap_config tap_live_config = TAP_CONFIG_DEFAULT;
static struct tap_config tap_capture_config = TAP_CONFIG_DEFAULT;
/* Explicitly armed RAM capture, at most eight seconds. Pull only after the
 * acquisition stops, so BLE transfer does not perturb the measured tap. */
#define TAP_CAPTURE_MAX (8 * TAP_SAMPLE_HZ)
struct tap_capture_sample { int16_t raw[3]; uint16_t flags; };
static struct tap_capture_sample tap_capture[TAP_CAPTURE_MAX];
static uint16_t tap_capture_id, tap_capture_target, tap_capture_count;
static uint32_t tap_capture_gaps;
static int64_t tap_capture_deadline_ms;
static bool tap_capture_active, tap_capture_gap;
struct tap_debug_command {
    struct bt_conn *conn; /* queue owns a reference until processed */
    uint8_t command, op;
    uint16_t arg;
    struct tap_config config;
};
K_MSGQ_DEFINE(tap_debug_queue, sizeof(struct tap_debug_command), 4, 4);
static struct debug_session debug_session;
static struct bt_conn *debug_owner; /* extra reference while enabled */
static bool tap_fifo_ready;
/* On-demand diagnostics for missed taps; no periodic BLE traffic. Main-loop
 * ownership keeps snapshots/reset coherent with the sample processing. */
static atomic_t tap_stats_request;
static struct tap_metrics tap_stats;
static int tap_last_read_ret;
static uint16_t tap_nonzero_count;
/* Per-connection greeting (mode status + one tap-register snapshot).
 *
 * It cannot be sent from ble_connected(): that runs about a second before the
 * client writes the TX CCCD, so there is no subscriber yet and the notification
 * is dropped.  BT_GATT_CCC's cfg_changed is not a usable hook either -- Zephyr
 * only calls it when the aggregate value across all clients changes, so a second
 * client subscribing while the first is already subscribed never triggers it.
 *
 * So mark the connection here and let the main loop poll bt_gatt_is_subscribed()
 * for that specific connection.  The main loop also owns send_tap_diag(), which
 * blocks on eight I2C reads and must not run on the BT callback thread. */
static bool conn_needs_greeting[MAX_CONNS];
static bool tap_diag_once_pending;

/* Gesture state */
typedef enum {
    GESTURE_WAITING,
    GESTURE_HOLDING_FINAL,
} gesture_phase_t;

typedef enum {
    GESTURE_LIFT_WAIT_ACCEL,
    GESTURE_LIFT_WAIT_BRAKE,
    GESTURE_LIFT_WAIT_HOLD,
} gesture_lift_stage_t;

static int64_t motion_active_start_ms;
static gesture_phase_t gesture_phase;
static int64_t gesture_sequence_start_ms;
static int64_t gesture_phase_start_ms;
static int64_t gesture_final_since_ms;
static int64_t gesture_last_sample_ms;
static int64_t gesture_quiet_since_ms;
static int64_t gesture_armed_until_ms;
static int64_t gesture_block_until_ms;
static int64_t gesture_diag_last_report_ms;
static int64_t gesture_palm_candidate_since_ms;
static float gesture_palm_candidate_gx;
static float gesture_palm_candidate_gy;
static float gesture_palm_candidate_gz;
static bool gesture_rearm_required;
static float gesture_quiet_accel_x, gesture_quiet_accel_y, gesture_quiet_accel_z;
static bool gesture_quiet_accel_valid;
static float gesture_start_accel_x, gesture_start_accel_y, gesture_start_accel_z;
static float gesture_z_window_min, gesture_z_window_max;
static int64_t gesture_z_window_start_ms;
static float gesture_pronation_phi_deg;
static float gesture_pronation_peak_deg;
static float gesture_pronation_peak_tilt_deg;
static float gesture_pronation_ref_phi_deg;
static float gesture_armed_az;
static float gesture_armed_z_ratio;
static float gesture_armed_gx, gesture_armed_gy, gesture_armed_gz;
static float gesture_gravity_lp_x, gesture_gravity_lp_y, gesture_gravity_lp_z;
static float gesture_linear_world_x, gesture_linear_world_y;
static float gesture_linear_world_z;
static float gesture_linear_accel_norm_ms2;
static gesture_lift_stage_t gesture_lift_stage;
static float gesture_lift_hold_axis_x;
static float gesture_lift_hold_axis_y;
static float gesture_lift_hold_axis_z;
static float gesture_lift_pos_impulse_ms;
static float gesture_lift_neg_impulse_ms;
static float gesture_lift_net_impulse_ms;
static float gesture_lift_final_tilt_deg;
static float gesture_lift_max_hold_tilt_deg;
static float gesture_lift_peak_a_up_ms2;
static bool gesture_lift_hold_axis_valid;
static bool gesture_lift_pose_failed;
static bool gesture_lift_before_flip;
static float gesture_lift_pos_impulse_at_entry_ms;
static float gesture_lift_roll_at_entry_deg;
static bool gesture_palm_down_latched;
static int64_t gesture_lift_event_start_ms;
static int64_t gesture_lift_near_miss_last_ms;
static uint8_t gesture_lift_accel_samples;
static uint8_t gesture_lift_brake_samples;
static uint8_t gesture_hold_rms_exit_samples;
static int64_t gesture_debug_final_last_ms;
static float gesture_accel_history_ms2[GESTURE_FINAL_RMS_WINDOW_SAMPLES];
static uint8_t gesture_accel_history_index;
static uint8_t gesture_accel_history_count;
/* Lift-axis unit vector frozen at MATCH; stop on opposite linear pulse. */
static bool recording_stop_axis_valid;
static bool recording_stop_armed;
static float recording_stop_axis_x, recording_stop_axis_y, recording_stop_axis_z;
static float recording_stop_lift_impulse_ms;
static float recording_stop_opp_impulse_ms;
static float recording_stop_opp_peak_ms2;
static float recording_stop_pulse_ms_latched;
static float gesture_lift_dir_acc_x, gesture_lift_dir_acc_y, gesture_lift_dir_acc_z;
static int64_t recording_stop_match_ms;
static int64_t recording_stop_pulse_since_ms;
static int64_t recording_stop_pulse_gap_since_ms;
static int64_t recording_stop_settle_since_ms;
static int64_t recording_stop_near_miss_last_ms;
static uint8_t recording_stop_opp_samples;
static uint8_t recording_stop_settle_spike_samples;
static bool recording_stop_pulse_latched;
/* Legacy fields retained so older reset call sites compile cleanly. */
static bool recording_stop_ref_valid;
static bool recording_stop_ref_locked;

/* On-demand gyro (powered while gesture sequence or recording is active). */
static bool gyro_enabled;
static int64_t gyro_enabled_since_ms;
static bool gyro_sample_valid;
static float gyro_bias_y_dps;
static bool gyro_bias_valid;
static float gesture_gyro_roll_deg;
static float gesture_gyro_peak_abs_dps;
static float gesture_gyro_x_peak_abs_dps;
static float gesture_outbound_gyro_sign;

/* Runtime switch for wrist-gesture start/stop. Off at boot so everyday wear
 * is tap-only until the companion opts in. RAM only; the phone re-asserts on
 * every connect (same pattern as gesture_capture). Driving mode still wins. */
static bool gesture_detect_enabled;

/* Runtime switch for the IMU trajectory dump (0x36/0x37/0x38) and the
 * milestone batch (0x33/0x34/0x35). Off at boot: one gesture attempt costs
 * about 10 KB of notifications, so it is only worth paying while collecting
 * training data. The phone re-sends it on every connect, like the operation
 * mode, because this lives in RAM and a reset clears it. */
static bool gesture_capture_enabled;

#if GESTURE_DEBUG_HISTORY
typedef struct {
	uint16_t t_ms;
	uint8_t stage;
	uint8_t reason;
	float v1;
	float v2;
	float v3;
} gesture_history_entry_t;

static gesture_history_entry_t gesture_history[GESTURE_HISTORY_CAP];
static uint8_t gesture_history_count;
static uint8_t gesture_history_session;
static int64_t gesture_history_t0_ms;
static bool gesture_history_flush_pending;

typedef struct {
    uint16_t t_ms;
    uint8_t flags;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} gesture_trajectory_sample_t;

static gesture_trajectory_sample_t gesture_trajectory[GESTURE_TRAJECTORY_CAP];
static uint16_t gesture_trajectory_count;
static uint8_t gesture_trajectory_session;
static int64_t gesture_trajectory_t0_ms;
static uint8_t gesture_trajectory_result;
static uint8_t gesture_trajectory_reason;
static bool gesture_trajectory_active;
static bool gesture_trajectory_committed;
static bool gesture_trajectory_overflow;
static bool gesture_trajectory_flush_pending;

/* Host-triggered collection is independent of classifier-owned history. */
static gesture_trajectory_sample_t gesture_host_collection[GESTURE_TRAJECTORY_CAP];
static uint16_t gesture_host_collection_count;
static int64_t gesture_host_collection_t0_ms;
static bool gesture_host_collection_active;
static bool gesture_host_collection_overflow;
static bool gesture_host_collection_flush_pending;
#endif

/* Motion detection state */
static atomic_t detected_motion_count;
static uint32_t last_motion_time_ms;
static bool baseline_valid;
static double baseline_x, baseline_y, baseline_z;
static uint8_t calibration_count, active_high_count, settle_count;
static bool motion_active;
static bool inhibit_next_settle;   /* 録音中断直後のsettleをスキップ */
static bool light_sleep_active;    /* true = IMUポーリングを低速化して省電力 */
static int64_t last_activity_ms;   /* motion_active/settled/recording_stop の最終時刻 */
static int64_t last_report_ms;
static bool previous_sample_valid;
static double previous_accel_x, previous_accel_y, previous_accel_z;
static double step_window[ACTIVITY_WINDOW_SAMPLES];
static uint8_t step_window_index, step_window_count;
static double step_window_sum;
static double z_excursion_peak;

/* Extended motion metrics */
static float motion_vel_x, motion_vel_y, motion_vel_z;
static float motion_distance;
static float motion_peak_speed;
static float motion_speed_sum;
static uint32_t motion_speed_samples;

/* Battery ADC (P0.31 = AIN7) + Enable GPIO (P0.14) */
static const struct adc_dt_spec adc_bat =
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct gpio_dt_spec bat_en =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), bat_read_enable_gpios);
static int16_t bat_sample_buf;
static struct adc_sequence bat_sequence = {
    .buffer      = &bat_sample_buf,
    .buffer_size = sizeof(bat_sample_buf),
};
static bool    bat_adc_ready;
static uint8_t bat_last_pct      = 50;  /* last ADC-measured % (initial: 50% fallback) */
static uint32_t bat_charge_start_ms;    /* k_uptime when USB was connected */
static uint8_t  bat_charge_start_pct;   /* battery % when USB was connected */
static bool     bat_vbus_prev;          /* previous VBUS state for edge detection */

/* Watchdog */
static const struct device *const wdt = DEVICE_DT_GET(WDT_NODE);
static int wdt_channel_id = -1;

/* RGB LED GPIO specs */
static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(LED_RED_NODE,   gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(LED_BLUE_NODE,  gpios);

/* LED state machine */
typedef enum {
    LED_BOOT,
    LED_IDLE,
    LED_ADVERTISING,
    LED_RECORDING,
    LED_ERROR,
} led_state_t;

static led_state_t current_led_state = LED_BOOT;
static int64_t led_state_enter_ms;
static bool blink_on;
static int64_t blink_next_ms;

/* Microphone power enable (P1.10, active-high) */
#define MIC_PWR_NODE DT_NODELABEL(msm261d3526hicpm_c_en)
static const struct gpio_dt_spec mic_pwr =
    GPIO_DT_SPEC_GET(MIC_PWR_NODE, enable_gpios);

/* ============================================================================
 * RGB LED
 * ============================================================================ */

static int configure_leds(void)
{
    if (!gpio_is_ready_dt(&led_red) ||
        !gpio_is_ready_dt(&led_green) ||
        !gpio_is_ready_dt(&led_blue)) {
        LOG_WRN("RGB LED GPIOs not ready");
        return -ENODEV;
    }
    int ret = 0;

    ret |= gpio_pin_configure_dt(&led_red,   GPIO_OUTPUT_INACTIVE);
    ret |= gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
    ret |= gpio_pin_configure_dt(&led_blue,  GPIO_OUTPUT_INACTIVE);
    return ret;
}

/* Set logical RGB values. Zephyr's GPIO_ACTIVE_LOW flag handles active-low inversion. */
static void rgb_set(bool r, bool g, bool b)
{
    gpio_pin_set_dt(&led_red,   r ? 1 : 0);
    gpio_pin_set_dt(&led_green, g ? 1 : 0);
    gpio_pin_set_dt(&led_blue,  b ? 1 : 0);
}

static void led_set_state(led_state_t state)
{
    current_led_state = state;
    led_state_enter_ms = k_uptime_get();

    switch (state) {
    case LED_BOOT:
        rgb_set(true, true, true);   /* White */
        break;
    case LED_IDLE:
        rgb_set(false, false, false); /* Off */
        break;
    case LED_ADVERTISING:
        rgb_set(false, false, true); /* Blue ON to start */
        blink_on = true;
        blink_next_ms = k_uptime_get() + ADV_LED_ON_MS;
        break;
    case LED_RECORDING:
        rgb_set(true, false, false); /* Red solid */
        break;
    case LED_ERROR:
        rgb_set(true, false, false); /* Red ON to start */
        blink_on = true;
        blink_next_ms = k_uptime_get() + ERROR_BLINK_MS;
        break;
    }
}

static void led_sync_runtime_state(void)
{
    led_state_t desired = LED_IDLE;

    if (current_led_state == LED_BOOT || current_led_state == LED_ERROR) {
        return;
    }

    if (is_recording) {
        desired = LED_RECORDING;
    } else if (active_count() == 0) {
        desired = LED_ADVERTISING;
    }

    if (current_led_state != desired) {
        led_set_state(desired);
    }
}

/* Called from main loop every MAIN_LOOP_INTERVAL_MS to handle blink patterns. */
static void led_tick(void)
{
    int64_t now = k_uptime_get();

    if (current_led_state == LED_ERROR) {
        if (now >= blink_next_ms) {
            blink_on = !blink_on;
            rgb_set(blink_on, false, false);
            blink_next_ms = now + ERROR_BLINK_MS;
        }
    } else if (current_led_state == LED_ADVERTISING) {
        if (now >= blink_next_ms) {
            blink_on = !blink_on;
            rgb_set(false, false, blink_on);
            blink_next_ms = now + (blink_on ? ADV_LED_ON_MS : ADV_LED_OFF_MS);
        }
    } else if (current_led_state == LED_BOOT) {
        /* Turn off white after 1 second */
        if ((now - led_state_enter_ms) >= 1000) {
            rgb_set(false, false, false);
        }
    }
}

/* ============================================================================
 * Reset Cause & Watchdog
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * Crash box — survives a watchdog / soft reset so the next boot can say where
 * the main loop was when it stopped feeding the dog.  Lives in .noinit, so it
 * is garbage after a power-on reset and after a SYSTEMOFF wake; the magic word
 * filters both (SYSTEMOFF wake is already reported as RESET_LOW_POWER_WAKE).
 * --------------------------------------------------------------------------- */
#define CRASHBOX_MAGIC 0x48424f58u  /* "HBOX" */

typedef enum {
    STAGE_IDLE = 0,
    STAGE_MOTION,
    STAGE_TAP,
    STAGE_GREET,
    STAGE_TAPDIAG,
    STAGE_SLEEPNFY,
    STAGE_BATTERY,
} loop_stage_t;

static const char *stage_str(uint8_t stage)
{
    switch (stage) {
    case STAGE_IDLE:     return "idle";
    case STAGE_MOTION:   return "motion";
    case STAGE_TAP:      return "tap";
    case STAGE_GREET:    return "greet";
    case STAGE_TAPDIAG:  return "tapdiag";
    case STAGE_SLEEPNFY: return "sleepnfy";
    case STAGE_BATTERY:  return "battery";
    default:             return "?";
    }
}

struct crashbox {
    uint32_t magic;
    uint32_t up_s;
    uint32_t hb_seq;
    uint32_t ndrop;
    uint8_t  stage;
    uint8_t  conns;
    uint8_t  recording;
};

static __noinit struct crashbox cb;

/* Snapshot taken at boot, before log_reset_cause() clears the hardware cause
 * and re-arms the crash box. Published to every client as EVT_BOOT_REPORT. */
static uint32_t boot_reset_cause;
static bool     boot_crash_valid;
static uint8_t  boot_crash_stage;
static uint16_t boot_crash_up_s;
static uint16_t boot_crash_hb;
static uint16_t boot_crash_ndrop;

/* Mark where the main loop is before anything that can block. */
static inline void loop_stage(loop_stage_t stage)
{
    cb.stage = (uint8_t)stage;
}

static void log_reset_cause(void)
{
    uint32_t cause = 0;

    hwinfo_get_reset_cause(&cause);
    hwinfo_clear_reset_cause();

    APP_TRACE("Reset cause: 0x%08x", cause);
    if (cause & RESET_WATCHDOG)   APP_TRACE(" [WATCHDOG — firmware hang]");
    if (cause & RESET_BROWNOUT)   APP_TRACE(" [BROWNOUT — low voltage/battery]");
    if (cause & RESET_PIN)        APP_TRACE(" [PIN — external reset]");
    if (cause & RESET_SOFTWARE)   APP_TRACE(" [SOFTWARE]");
    if (cause & RESET_CPU_LOCKUP) APP_TRACE(" [CPU_LOCKUP]");
    if (cause & RESET_POR)        APP_TRACE(" [POR — power-on]");
    if (cause & RESET_LOW_POWER_WAKE) APP_TRACE(" [LOW_POWER_WAKE — SYSTEMOFF]");
    if (cause == 0)               APP_TRACE(" [POR — power-on / battery removed]");
    APP_TRACE("\n");

    boot_reset_cause = cause;

    if (cb.magic == CRASHBOX_MAGIC && !(cause & RESET_LOW_POWER_WAKE)) {
        boot_crash_valid = true;
        boot_crash_stage = cb.stage;
        boot_crash_up_s  = (uint16_t)MIN(cb.up_s, 0xFFFFU);
        boot_crash_hb    = (uint16_t)MIN(cb.hb_seq, 0xFFFFU);
        boot_crash_ndrop = (uint16_t)MIN(cb.ndrop, 0xFFFFU);
        APP_TRACE(">>> CRASHBOX stage=%u(%s) up=%us hb=%u conns=%u rec=%u ndrop=%u\n",
               cb.stage, stage_str(cb.stage), cb.up_s, cb.hb_seq,
               cb.conns, cb.recording, cb.ndrop);
    } else {
        APP_TRACE(">>> CRASHBOX: none (cold boot or SYSTEMOFF wake)\n");
    }
    memset(&cb, 0, sizeof(cb));
    cb.magic = CRASHBOX_MAGIC;
}

static void configure_watchdog(void)
{
    if (!device_is_ready(wdt)) {
        LOG_WRN("WDT not ready — firmware hang detection disabled");
        return;
    }
    struct wdt_timeout_cfg cfg = {
        .flags = WDT_FLAG_RESET_SOC,
        .window = { .min = 0U, .max = WDT_TIMEOUT_MS },
    };
    wdt_channel_id = wdt_install_timeout(wdt, &cfg);
    if (wdt_channel_id < 0) {
        LOG_ERR("WDT install failed: %d", wdt_channel_id);
        return;
    }
    if (wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG) < 0) {
        LOG_ERR("WDT setup failed");
        return;
    }
    APP_INFO("WDT armed: %d ms timeout", WDT_TIMEOUT_MS);
    APP_TRACE("Watchdog armed (%d ms)\n", WDT_TIMEOUT_MS);
}

/* ============================================================================
 * Battery Monitor
 * ============================================================================ */

static int battery_init(void)
{
    if (!adc_is_ready_dt(&adc_bat)) {
        LOG_WRN("Battery ADC not ready");
        return -ENODEV;
    }
    int err = adc_channel_setup_dt(&adc_bat);
    if (err < 0) {
        LOG_ERR("Battery ADC channel setup failed: %d", err);
        return err;
    }
    /* Configure P0.14 as output (battery measurement enable) */
    if (gpio_is_ready_dt(&bat_en)) {
        gpio_pin_configure_dt(&bat_en, GPIO_OUTPUT_INACTIVE);
    } else {
        LOG_WRN("Battery enable GPIO not ready");
    }
    bat_adc_ready = true;
    APP_INFO("Battery ADC ready (AIN7/P0.31, enable P0.14)");
    return 0;
}

static int battery_get_millivolt(void)
{
    int err;
    int32_t val_mv;

    /* P0.14 must remain LOW — HIGH injects 3.3V into P0.31 and saturates ADC */

    err = adc_sequence_init_dt(&adc_bat, &bat_sequence);
    if (err < 0) { return err; }

    err = adc_read(adc_bat.dev, &bat_sequence);
    if (err < 0) { return err; }

    val_mv = bat_sample_buf;
    err = adc_raw_to_millivolts_dt(&adc_bat, &val_mv);
    if (err < 0) { return err; }

    /*
     * Correction factor ×3: XIAO nRF52840 divider is 2MΩ:1MΩ (not 1:1).
     * Verified empirically: pin_mv=1398mV × 3 = 4194mV ≈ 4200mV (USB charge).
     */
    return (int)(val_mv * 3);
}

static uint8_t battery_millivolt_to_percent(int mv)
{
    /*
     * OCV (Open Circuit Voltage) based LUT for BQ25101 on XIAO nRF52840 Sense.
     * The charger terminates at 4.2V CV / 5mA (ITERM=10% of 50mA default).
     * After termination and USB removal, the battery OCV settles at ~4.08-4.15V
     * (not 4.2V, which is only the charger-applied terminal voltage).
     * This LUT uses resting OCV values so that a fully charged battery reads ~100%.
     */
    static const struct { int mv; int pct; } lut[] = {
        { 4150, 100 }, { 4050, 90 }, { 3950, 80 },
        { 3850,  70 }, { 3750, 60 }, { 3650, 50 },
        { 3550,  40 }, { 3450, 30 }, { 3350, 20 },
        { 3250,  10 }, { 3000,  0 },
    };
    if (mv >= lut[0].mv) { return 100; }
    for (int i = 0; i < (int)ARRAY_SIZE(lut) - 1; i++) {
        if (mv >= lut[i + 1].mv) {
            int mv_range  = lut[i].mv  - lut[i + 1].mv;
            int pct_range = lut[i].pct - lut[i + 1].pct;
            return (uint8_t)(lut[i + 1].pct +
                   (mv - lut[i + 1].mv) * pct_range / mv_range);
        }
    }
    return 0;
}

static void battery_update(void)
{
    if (!bat_adc_ready) { return; }

    bool vbus = nrf_power_usbregstatus_vbusdet_get(NRF_POWER);

    /* USBが新たに接続された → 充電開始を記録 */
    if (vbus && !bat_vbus_prev) {
        bat_charge_start_ms  = k_uptime_get_32();
        bat_charge_start_pct = bat_last_pct;
        APP_INFO("Battery: USB connected, charging from %d%%", bat_charge_start_pct);
    }
    bat_vbus_prev = vbus;

    if (vbus) {
        /* 充電中：経過時間から残量を線形推定 */
        uint32_t elapsed_min = (k_uptime_get_32() - bat_charge_start_ms) / 60000U;
        /* 充電レート [%/min] = charge_current_mA * 100 / (capacity_mAh * 60) */
        uint32_t added_pct = (elapsed_min * BATTERY_CHARGE_CURRENT_MA * 100U)
                             / (BATTERY_CAPACITY_MAH * 60U);
        uint8_t est_pct = (uint8_t)MIN(bat_charge_start_pct + added_pct,
                                       BATTERY_CHARGE_MAX_EST_PCT);
        APP_INFO("Battery: charging ~%d%% (est, %u min)", est_pct, elapsed_min);
        APP_TRACE("Battery: charging ~%d%% (est, %u min)\n", est_pct, elapsed_min);
        bt_bas_set_battery_level(est_pct);
        return;
    }

    /* USB切断中：ADCで実測 */
    int mv = battery_get_millivolt();
    if (mv <= 0) {
        LOG_WRN("Battery read failed: %d", mv);
        return;
    }
    uint8_t pct = battery_millivolt_to_percent(mv);
    bat_last_pct = pct;
    APP_INFO("Battery: %d mV (%d%%)", mv, pct);
    APP_TRACE("Battery: %d mV (%d%%)\n", mv, pct);
    bt_bas_set_battery_level(pct);
}

/* ============================================================================
 * Mic Power
 * ============================================================================ */

static int configure_mic_power(void)
{
    if (!gpio_is_ready_dt(&mic_pwr)) {
        LOG_ERR("Mic power GPIO not ready");
        return -ENODEV;
    }
    return gpio_pin_configure_dt(&mic_pwr, GPIO_OUTPUT_INACTIVE);
}

static void mic_power_on(void)
{
    gpio_pin_set_dt(&mic_pwr, 1);
    k_msleep(50);   /* wait for MEMS bias to stabilise */
    APP_TRACE("Mic power ON\n");
}

static void mic_power_off(void)
{
    gpio_pin_set_dt(&mic_pwr, 0);
    APP_TRACE("Mic power OFF\n");
}

/* Forward declarations */
static void send_event_packet(uint8_t event_code);
static void send_event_packet_xyz(uint8_t event_code, float x, float y, float z);
static void send_event_packet_settle(float x, float y, float z, uint32_t elapsed_ms,
                                     float avg_speed, float peak_speed, float distance);
static void send_gesture_diag(uint8_t stage, uint8_t reason,
                              float value1, float value2, float value3);
static float deg_diff(float a, float b);
static void gyro_set_enabled(bool enable);
static void gyro_reset_phase_metrics(void);
static void flush_gesture_history(void);
static void flush_gesture_trajectory(void);
static void flush_host_collection(void);
static void process_tap_event(int64_t now);
static void send_tap_diag(void);
static void send_tap_reg_ack(uint8_t reg, uint8_t value, int ret);

static void apply_pending_operation_mode(void);
static void send_operation_mode_status(void);
static bool gesture_detect_effective(void);
static void send_gesture_detect_status(void);
static bool gesture_capture_effective(void);
static void send_gesture_capture_status(void);
static void notify_all_conns(const void *data, uint16_t len);
static int notify_one(struct bt_conn *conn, const void *data, uint16_t len);

/* ============================================================================
 * Motion Detection
 * ============================================================================ */

static double abs_double(double v) { return v < 0.0 ? -v : v; }

static void set_baseline(double x, double y, double z)
{
    baseline_x = x; baseline_y = y; baseline_z = z;
    baseline_valid = true;
}

static void update_baseline(double x, double y, double z)
{
    if (!baseline_valid) { set_baseline(x, y, z); return; }
    /* Suppress baseline drift when the board is vibrating (e.g., in a car).
     * Only update when the total acceleration change is very small,
     * indicating the device is truly still. */
    double step = abs_double(x - baseline_x) + abs_double(y - baseline_y) +
                  abs_double(z - baseline_z);
    if (step < 0.5) {
        baseline_x += (x - baseline_x) * BASELINE_ALPHA;
        baseline_y += (y - baseline_y) * BASELINE_ALPHA;
        baseline_z += (z - baseline_z) * BASELINE_ALPHA;
    }
}

static double motion_delta(double x, double y, double z)
{
    return abs_double(x - baseline_x) + abs_double(y - baseline_y) +
           abs_double(z - baseline_z);
}

static double sample_delta(double x, double y, double z)
{
    if (!previous_sample_valid) {
        previous_accel_x = x; previous_accel_y = y; previous_accel_z = z;
        previous_sample_valid = true;
        return 0.0;
    }
    /* Use z-axis only for step calculation.  The arm-lift gesture is
     * predominantly a change in the gravity (z) direction, while
     * horizontal vibrations (e.g., from a car) are ignored. */
    double d = abs_double(z - previous_accel_z);
    previous_accel_x = x; previous_accel_y = y; previous_accel_z = z;
    return d;
}

static void reset_step_window(void)
{
    for (size_t i = 0; i < ACTIVITY_WINDOW_SAMPLES; ++i) step_window[i] = 0.0;
    step_window_index = step_window_count = 0;
    step_window_sum = 0.0;
}

static void update_step_window(double s)
{
    if (step_window_count == ACTIVITY_WINDOW_SAMPLES) {
        step_window_sum -= step_window[step_window_index];
    } else {
        step_window_count++;
    }
    step_window[step_window_index] = s;
    step_window_sum += s;
    step_window_index = (step_window_index + 1U) % ACTIVITY_WINDOW_SAMPLES;
}

static double step_window_peak(void)
{
    double p = 0.0;
    for (size_t i = 0; i < step_window_count; ++i) {
        if (step_window[i] > p) p = step_window[i];
    }
    return p;
}

static void accumulate_calibration(double x, double y, double z)
{
    if (calibration_count > 0) {
        double accel_dx = x - baseline_x;
        double accel_dy = y - baseline_y;
        double accel_dz = z - baseline_z;
        double accel_span = sqrt(accel_dx * accel_dx +
                                 accel_dy * accel_dy +
                                 accel_dz * accel_dz);

        /* Restart the window until all calibration samples describe one stable pose. */
        if (accel_span > (double)CALIBRATION_ACCEL_SPAN_MAX_MS2) {
            calibration_count = 0;
        }
    }

    if (calibration_count == 0) {
        set_baseline(x, y, z);
    } else {
        baseline_x = ((baseline_x * calibration_count) + x) / (calibration_count + 1);
        baseline_y = ((baseline_y * calibration_count) + y) / (calibration_count + 1);
        baseline_z = ((baseline_z * calibration_count) + z) / (calibration_count + 1);
    }
    calibration_count++;

    if (calibration_count == CALIBRATION_SAMPLES) {
        APP_INFO("Baseline ready: accel=(%.3f, %.3f, %.3f) m/s^2",
                baseline_x, baseline_y, baseline_z);
        previous_accel_x = baseline_x;
        previous_accel_y = baseline_y;
        previous_accel_z = baseline_z;
        previous_sample_valid = true;
        gesture_quiet_accel_x = (float)baseline_x;
        gesture_quiet_accel_y = (float)baseline_y;
        gesture_quiet_accel_z = (float)baseline_z;
        gesture_quiet_accel_valid = true;
        gesture_quiet_since_ms = k_uptime_get();
        gesture_last_sample_ms = gesture_quiet_since_ms;
        reset_step_window();
        last_report_ms = k_uptime_get();
        last_motion_time_ms = (uint32_t)k_uptime_get();
    }
}

static int configure_motion_detection(void)
{
    struct sensor_value accel_odr = {ACCEL_ODR_HZ, 0};
    struct sensor_value gyro_odr = {0, 0};
    struct sensor_value threshold = {1, 500000};
    struct sensor_value duration = {MOTION_DURATION_SAMPLES, 0};
    int ret;

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, &accel_odr);
    if (ret < 0) { LOG_ERR("ODR set failed: %d", ret); return ret; }

    /* Power-down gyro until a palm-up dwell or recording needs it. */
    ret = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, &gyro_odr);
    if (ret < 0) {
        LOG_WRN("Gyro power-down failed: %d (continuing accel-only)", ret);
    }

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &threshold);
    if (ret == -ENOTSUP) LOG_WRN("SLOPE_TH not supported");
    else if (ret < 0) { LOG_ERR("threshold set failed: %d", ret); return ret; }

    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &duration);
    if (ret == -ENOTSUP) LOG_WRN("SLOPE_DUR not supported");
    else if (ret < 0) { LOG_ERR("duration set failed: %d", ret); return ret; }

    gyro_enabled = false;
    gyro_sample_valid = false;
    APP_INFO("Motion detection ready: accel=%d Hz gyro=on-demand@%d Hz, sample=%d ms",
            ACCEL_ODR_HZ, GYRO_ODR_HZ, MOTION_SAMPLE_INTERVAL_MS);
    APP_INFO("Calibrating for %.1f s; keep the board still",
            (double)(CALIBRATION_SAMPLES * MOTION_SAMPLE_INTERVAL_MS) / 1000.0);
    return 0;
}

/* Legacy embedded-tap diagnostics only. ODR/range/routing stay locked to
 * protect the software classifier's sample rate and acceleration scale. */
static bool tap_reg_writable(uint8_t reg)
{
    switch (reg) {
    case LSM6DS3TR_C_REG_TAP_CFG:
    case LSM6DS3TR_C_REG_TAP_THS_6D:
    case LSM6DS3TR_C_REG_INT_DUR2:
    case LSM6DS3TR_C_REG_WAKE_UP_THS:
        return true;
    default:
        return false;
    }
}

/* Flush FIFO and classifier together on any loss of sample continuity. */
static int reset_tap_fifo(void)
{
    tap_detector_reset(&software_tap);
    int ret = i2c_reg_write_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_FIFO_CTRL5, 0);
    if (ret == 0) {
        ret = i2c_reg_write_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_FIFO_CTRL5,
                                  TAP_FIFO_RUNNING);
    }
    tap_fifo_ready = ret == 0;
    return ret;
}

static int configure_tap_detection(void)
{
    tap_fifo_ready = false;
    if (!i2c_is_ready_dt(&imu_i2c)) return -ENODEV;
    int ret = sensor_trigger_set(imu, &tap_trigger, NULL);
    if (ret < 0) return ret;
    ret = gpio_pin_interrupt_configure_dt(&imu_int1, GPIO_INT_DISABLE);
    if (ret < 0) return ret;
    ret = i2c_reg_write_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_INT1_CTRL, 0);
    if (ret < 0) return ret;
    ret = i2c_reg_update_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_MD1_CFG,
                               LSM6DS3TR_C_MD1_TAP_MASK, 0);
    if (ret < 0) return ret;
    ret = i2c_reg_update_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_TAP_CFG,
                               LSM6DS3TR_C_TAP_CFG_MASK, 0);
    if (ret < 0) return ret;
    ret = i2c_reg_update_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_CTRL6_C,
                               LSM6DS3TR_C_CTRL6_XL_HM_MODE, 0);
    if (ret < 0) return ret;
    struct sensor_value odr = {ACCEL_ODR_HZ, 0};
    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    if (ret < 0) return ret;
    struct sensor_value range;
    /* +/-4 g is required, not just sufficient: the SAME accelerometer feeds the
     * gesture path whose thresholds (e.g. GESTURE_LIFT_ACCEL_MIN_MS2 0.40 m/s^2
     * = 0.041 g) need resolution finer than a +/-8 g LSB (0.244 g). Widening to
     * 8 g to separate hard taps from maximal wrist jerks would break gesture
     * detection, and a maximal jerk is physically a hard transient anyway, so it
     * is not separable by range or peak gate without losing real taps. */
    sensor_g_to_ms2(4, &range);
    ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
                          SENSOR_ATTR_FULL_SCALE, &range);
    if (ret < 0) return ret;
    /* FIFO_CTRL1..5: no timestamp/temp/gyro/external data; XYZ every sample.
     * Enter bypass first so old mixed patterns cannot enter our classifier. */
    const uint8_t fifo_config[] = {0, 0, 1, 0, 0};
    ret = i2c_reg_write_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_FIFO_CTRL5, 0);
    if (ret < 0) return ret;
    ret = i2c_burst_write_dt(&imu_i2c, LSM6DS3TR_C_REG_FIFO_CTRL1,
                             fifo_config, sizeof(fifo_config));
    if (ret < 0) return ret;
    ret = reset_tap_fifo();
    if (ret == 0) APP_INFO("Software tap ready: acceleration FIFO 416 Hz +/-4g");
    return ret;
}

/* CPU cannot run the software classifier in SYSTEMOFF. Embedded tap is
 * configured just before poweroff, and removed again if poweroff is aborted. */
static int configure_tap_wake(void)
{
    int ret = i2c_reg_write_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_FIFO_CTRL5, 0);
    if (ret < 0) return ret;
    ret = i2c_reg_update_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_TAP_CFG,
                               LSM6DS3TR_C_TAP_CFG_MASK,
                               LSM6DS3TR_C_TAP_CFG_Z_LATCHED);
    if (ret < 0) return ret;
    ret = i2c_reg_update_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_TAP_THS_6D,
                               LSM6DS3TR_C_TAP_THS_MASK,
                               LSM6DS3TR_C_TAP_THS_BALANCED);
    if (ret < 0) return ret;
    ret = i2c_reg_write_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_INT_DUR2,
                               LSM6DS3TR_C_INT_DUR_BALANCED);
    if (ret < 0) return ret;
    ret = i2c_reg_update_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_WAKE_UP_THS,
                               LSM6DS3TR_C_DOUBLE_TAP_ENABLE,
                               LSM6DS3TR_C_DOUBLE_TAP_ENABLE);
    if (ret < 0) return ret;
    return i2c_reg_update_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_MD1_CFG,
                                 LSM6DS3TR_C_MD1_TAP_MASK,
                                 LSM6DS3TR_C_MD1_SINGLE_TAP |
                                 LSM6DS3TR_C_MD1_DOUBLE_TAP);
}

static void gyro_reset_phase_metrics(void)
{
    gesture_gyro_roll_deg = 0.0f;
    gesture_gyro_peak_abs_dps = 0.0f;
    gesture_gyro_x_peak_abs_dps = 0.0f;
}

static void gyro_set_enabled(bool enable)
{
    struct sensor_value odr = {enable ? GYRO_ODR_HZ : 0, 0};
    struct sensor_value full_scale;
    int ret;

#if GESTURE_DEBUG_HISTORY
    if (!enable && gesture_host_collection_active) {
        return;
    }
#endif
    if (enable == gyro_enabled) {
        return;
    }

    if (enable) {
        sensor_degrees_to_rad(GYRO_FULL_SCALE_DPS, &full_scale);
        ret = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
                              SENSOR_ATTR_FULL_SCALE, &full_scale);
        if (ret < 0) {
            LOG_ERR("Gyro full-scale set failed: %d", ret);
            return;
        }
    }

    ret = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
                          SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
    if (ret < 0) {
        LOG_ERR("Gyro ODR set failed: %d", ret);
        return;
    }

    gyro_enabled = enable;
    if (enable) {
        gyro_enabled_since_ms = k_uptime_get();
        gyro_sample_valid = false;
        gyro_reset_phase_metrics();
        APP_TRACE(">>> Gyro ON (%d Hz)\n", GYRO_ODR_HZ);
        send_gesture_diag(GESTURE_DIAG_GYRO_ENABLED, GESTURE_DIAG_REASON_NONE,
                          (float)GYRO_ODR_HZ, gyro_bias_y_dps,
                          gyro_bias_valid ? 1.0f : 0.0f);
    } else {
        gyro_sample_valid = false;
        APP_TRACE(">>> Gyro OFF\n");
        send_gesture_diag(GESTURE_DIAG_GYRO_DISABLED, GESTURE_DIAG_REASON_NONE,
                          gesture_gyro_roll_deg, gesture_gyro_peak_abs_dps,
                          0.0f);
    }
}

static bool gyro_is_settled(int64_t now)
{
    return gyro_enabled &&
           (now - gyro_enabled_since_ms) >= GESTURE_GYRO_SETTLE_MS;
}

static float gyro_y_corrected_dps(float gy_raw_dps)
{
    if (!gyro_bias_valid) {
        return gy_raw_dps;
    }
    return gy_raw_dps - gyro_bias_y_dps;
}

static void accumulate_gyro_roll(float gy_dps, float dt_s, float min_rate_dps,
                                 float *roll_deg, float *peak_abs_dps)
{
    float abs_rate = fabsf(gy_dps);

    if (abs_rate > *peak_abs_dps) {
        *peak_abs_dps = abs_rate;
    }
    if (abs_rate >= min_rate_dps) {
        *roll_deg += gy_dps * dt_s;
    }
}

static bool gesture_gyro_hold_flip_ok(void)
{
    float signed_roll = fabsf(gesture_gyro_roll_deg);

    if (gesture_outbound_gyro_sign == 0.0f) {
        signed_roll = fabsf(gesture_gyro_roll_deg);
    } else {
        /* Opposite sign from the outbound palm-up rotation. */
        signed_roll = gesture_gyro_roll_deg * (-gesture_outbound_gyro_sign);
    }

    bool angle_ok = signed_roll >= GESTURE_HOLD_GYRO_ANGLE_MIN_DEG;
    bool lift_coupled =
        gesture_gyro_x_peak_abs_dps >=
            gesture_gyro_peak_abs_dps *
                GESTURE_HOLD_GYRO_XY_PEAK_RATIO_MIN;
    /* Accel lift waives xy only when it is early (pre-flip) AND strong enough
     * to look like an intentional arm raise (rejects daily micro-lifts). */
    bool lift_stage_ok =
        gesture_lift_stage != GESTURE_LIFT_WAIT_ACCEL &&
        gesture_lift_before_flip &&
        gesture_lift_pos_impulse_at_entry_ms >=
            GESTURE_LIFT_XY_WAIVER_IMPULSE_MIN_MS;

    return angle_ok && (lift_coupled || lift_stage_ok);
}

static float gesture_gyro_hold_signed_roll_deg(void)
{
    if (gesture_outbound_gyro_sign == 0.0f) {
        return fabsf(gesture_gyro_roll_deg);
    }
    return gesture_gyro_roll_deg * (-gesture_outbound_gyro_sign);
}

static float gesture_gyro_hold_xy_peak_ratio(void)
{
    return gesture_gyro_peak_abs_dps > 0.1f
        ? gesture_gyro_x_peak_abs_dps / gesture_gyro_peak_abs_dps
        : 0.0f;
}


#if GESTURE_DEBUG_HISTORY
static void gesture_history_clear(void)
{
    gesture_history_count = 0;
    gesture_history_t0_ms = k_uptime_get();
    gesture_history_flush_pending = false;
}

static void gesture_history_push(uint8_t stage, uint8_t reason,
                                 float v1, float v2, float v3)
{
    gesture_history_entry_t *e;

    if (!atomic_get(&debug_active) || !gesture_capture_enabled) {
        return;
    }
    if (gesture_history_count == 0) {
        gesture_history_t0_ms = k_uptime_get();
    }
    if (gesture_history_count >= GESTURE_HISTORY_CAP) {
        /* Drop oldest; keep recent trajectory. */
        memmove(&gesture_history[0], &gesture_history[1],
                sizeof(gesture_history[0]) * (GESTURE_HISTORY_CAP - 1));
        gesture_history_count = GESTURE_HISTORY_CAP - 1;
    }
    e = &gesture_history[gesture_history_count++];
    {
        int64_t dt = k_uptime_get() - gesture_history_t0_ms;
        if (dt < 0) {
            dt = 0;
        }
        if (dt > 65535) {
            dt = 65535;
        }
        e->t_ms = (uint16_t)dt;
    }
    e->stage = stage;
    e->reason = reason;
    e->v1 = v1;
    e->v2 = v2;
    e->v3 = v3;
}

static void gesture_trajectory_clear(void)
{
    gesture_trajectory_count = 0;
    gesture_trajectory_t0_ms = k_uptime_get();
    gesture_trajectory_result = 0;
    gesture_trajectory_reason = GESTURE_DIAG_REASON_NONE;
    /* Staying inactive makes every push, finish and flush a no-op, so the
     * capture switch costs nothing but this branch while it is off. */
    gesture_trajectory_active = atomic_get(&debug_active) && gesture_capture_enabled;
    gesture_trajectory_committed = false;
    gesture_trajectory_overflow = false;
    gesture_trajectory_flush_pending = false;
}

static void gesture_trajectory_discard(void)
{
    gesture_trajectory_count = 0;
    gesture_trajectory_active = false;
    gesture_trajectory_committed = false;
    gesture_trajectory_overflow = false;
    gesture_trajectory_flush_pending = false;
}

static void gesture_trajectory_push(int64_t now, uint8_t flags,
                                    float ax, float ay, float az,
                                    float gx, float gy, float gz)
{
    gesture_trajectory_sample_t *sample;
    int64_t dt;

    if (!atomic_get(&debug_active) || !gesture_trajectory_active) {
        return;
    }
    if (gesture_trajectory_count >= GESTURE_TRAJECTORY_CAP) {
        gesture_trajectory_overflow = true;
        return;
    }
    sample = &gesture_trajectory[gesture_trajectory_count++];
    dt = now - gesture_trajectory_t0_ms;
    sample->t_ms = (uint16_t)CLAMP(dt, 0, UINT16_MAX);
    sample->flags = flags;
    sample->ax = ax;
    sample->ay = ay;
    sample->az = az;
    sample->gx = gx;
    sample->gy = gy;
    sample->gz = gz;
}

static void gesture_trajectory_finish(uint8_t result, uint8_t reason)
{
    if (!atomic_get(&debug_active) || !gesture_trajectory_active || !gesture_trajectory_committed) {
        return;
    }
    gesture_trajectory_active = false;
    gesture_trajectory_result = result;
    gesture_trajectory_reason = reason;
    gesture_trajectory_flush_pending = true;
}

static bool gesture_host_collection_start(void)
{
    if (gesture_host_collection_active ||
        gesture_host_collection_flush_pending || is_recording ||
        recording_requested) {
        return false;
    }
    gesture_host_collection_count = 0;
    gesture_host_collection_t0_ms = k_uptime_get();
    gesture_host_collection_overflow = false;
    gesture_host_collection_active = true;
    gyro_set_enabled(true);
    APP_TRACE(">>> Host IMU collection started: %d ms\n",
           GESTURE_HOST_COLLECTION_MS);
    return true;
}

static void gesture_host_collection_push(int64_t now, uint8_t flags,
                                         float ax, float ay, float az,
                                         float gx, float gy, float gz)
{
    gesture_trajectory_sample_t *sample;
    int64_t dt;

    if (!gesture_host_collection_active) {
        return;
    }
    if (gesture_host_collection_count >= GESTURE_TRAJECTORY_CAP) {
        gesture_host_collection_overflow = true;
        gesture_host_collection_active = false;
        gesture_host_collection_flush_pending = true;
        return;
    }
    sample = &gesture_host_collection[gesture_host_collection_count++];
    dt = now - gesture_host_collection_t0_ms;
    sample->t_ms = (uint16_t)CLAMP(dt, 0, UINT16_MAX);
    sample->flags = flags;
    sample->ax = ax;
    sample->ay = ay;
    sample->az = az;
    sample->gx = gx;
    sample->gy = gy;
    sample->gz = gz;
    if (dt >= GESTURE_HOST_COLLECTION_MS) {
        gesture_host_collection_active = false;
        gesture_host_collection_flush_pending = true;
        APP_TRACE(">>> Host IMU collection complete: %u samples\n",
               gesture_host_collection_count);
    }
}
#endif

/* ============================================================================
 * Gesture Detection
 * ============================================================================ */

static float vector_norm3(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

static void reset_gesture_motion(void)
{
    gesture_gravity_lp_x = gesture_start_accel_x;
    gesture_gravity_lp_y = gesture_start_accel_y;
    gesture_gravity_lp_z = gesture_start_accel_z;
    gesture_linear_accel_norm_ms2 = 0.0f;
    gesture_linear_world_x = 0.0f;
    gesture_linear_world_y = 0.0f;
    gesture_linear_world_z = 0.0f;
    gesture_lift_stage = GESTURE_LIFT_WAIT_ACCEL;
    gesture_lift_hold_axis_x = 0.0f;
    gesture_lift_hold_axis_y = 0.0f;
    gesture_lift_hold_axis_z = 0.0f;
    gesture_lift_pos_impulse_ms = 0.0f;
    gesture_lift_neg_impulse_ms = 0.0f;
    gesture_lift_net_impulse_ms = 0.0f;
    gesture_lift_final_tilt_deg = 0.0f;
    gesture_lift_max_hold_tilt_deg = 0.0f;
    gesture_lift_peak_a_up_ms2 = 0.0f;
    gesture_lift_hold_axis_valid = false;
    gesture_lift_pose_failed = false;
    gesture_lift_before_flip = false;
    gesture_lift_pos_impulse_at_entry_ms = 0.0f;
    gesture_lift_roll_at_entry_deg = 0.0f;
    gesture_lift_dir_acc_x = 0.0f;
    gesture_lift_dir_acc_y = 0.0f;
    gesture_lift_dir_acc_z = 0.0f;
    gesture_palm_down_latched = false;
    gesture_lift_event_start_ms = 0;
    gesture_lift_near_miss_last_ms = 0;
    gesture_lift_accel_samples = 0;
    gesture_lift_brake_samples = 0;
    gesture_hold_rms_exit_samples = 0;
    gesture_debug_final_last_ms = 0;
    memset(gesture_accel_history_ms2, 0, sizeof(gesture_accel_history_ms2));
    gesture_accel_history_index = 0;
    gesture_accel_history_count = 0;
}

/* Low-pass raw acceleration to estimate gravity (in the body frame).  With the
 * gyro disabled this is the only orientation information available.  The LP
 * removes vehicle accelerations that are slow compared to the lift motion. */
static void update_gravity_lp(float ax, float ay, float az, float dt_s)
{
    float alpha = dt_s / (GESTURE_GRAVITY_LP_TAU_S + dt_s);
    gesture_gravity_lp_x += (ax - gesture_gravity_lp_x) * alpha;
    gesture_gravity_lp_y += (ay - gesture_gravity_lp_y) * alpha;
    gesture_gravity_lp_z += (az - gesture_gravity_lp_z) * alpha;

    float linear_x = ax - gesture_gravity_lp_x;
    float linear_y = ay - gesture_gravity_lp_y;
    float linear_z = az - gesture_gravity_lp_z;
    gesture_linear_world_x = linear_x;
    gesture_linear_world_y = linear_y;
    gesture_linear_world_z = linear_z;
    gesture_linear_accel_norm_ms2 = vector_norm3(linear_x, linear_y, linear_z);
}

static bool gesture_board_flat(float z_gravity_ratio)
{
    return fabsf(z_gravity_ratio) >= GESTURE_START_PALM_UP_Z_MIN_RATIO;
}

static void reset_recording_stop_pulse(void)
{
    recording_stop_opp_impulse_ms = 0.0f;
    recording_stop_opp_peak_ms2 = 0.0f;
    recording_stop_pulse_ms_latched = 0.0f;
    recording_stop_pulse_since_ms = 0;
    recording_stop_pulse_gap_since_ms = 0;
    recording_stop_settle_since_ms = 0;
    recording_stop_opp_samples = 0;
    recording_stop_settle_spike_samples = 0;
    recording_stop_pulse_latched = false;
}

static void reset_recording_stop_state(void)
{
    recording_stop_axis_valid = false;
    recording_stop_armed = false;
    recording_stop_axis_x = 0.0f;
    recording_stop_axis_y = 0.0f;
    recording_stop_axis_z = 0.0f;
    recording_stop_lift_impulse_ms = 0.0f;
    recording_stop_match_ms = 0;
    recording_stop_near_miss_last_ms = 0;
    recording_stop_ref_valid = false;
    recording_stop_ref_locked = false;
    reset_recording_stop_pulse();
}

static void capture_recording_stop_axis_from_lift(void)
{
    float n = vector_norm3(gesture_lift_dir_acc_x,
                           gesture_lift_dir_acc_y,
                           gesture_lift_dir_acc_z);
    float g_norm = vector_norm3(gesture_gravity_lp_x,
                                gesture_gravity_lp_y,
                                gesture_gravity_lp_z);

    if (n > 0.05f) {
        recording_stop_axis_x = gesture_lift_dir_acc_x / n;
        recording_stop_axis_y = gesture_lift_dir_acc_y / n;
        recording_stop_axis_z = gesture_lift_dir_acc_z / n;
        recording_stop_axis_valid = true;
    } else if (g_norm > 0.1f) {
        /* a_up uses gravity; fallback keeps reverse-of-lift semantics. */
        recording_stop_axis_x = gesture_gravity_lp_x / g_norm;
        recording_stop_axis_y = gesture_gravity_lp_y / g_norm;
        recording_stop_axis_z = gesture_gravity_lp_z / g_norm;
        recording_stop_axis_valid = true;
    } else {
        recording_stop_axis_valid = false;
    }

    recording_stop_lift_impulse_ms =
        gesture_lift_pos_impulse_at_entry_ms > 0.05f
            ? gesture_lift_pos_impulse_at_entry_ms
            : gesture_lift_pos_impulse_ms;
    recording_stop_match_ms = k_uptime_get();
    recording_stop_near_miss_last_ms = 0;
    recording_stop_armed = recording_stop_axis_valid;
    recording_stop_ref_valid = recording_stop_axis_valid;
    recording_stop_ref_locked = false;
    reset_recording_stop_pulse();
}

static float recording_stop_impulse_threshold_base(void)
{
    float from_lift = recording_stop_lift_impulse_ms *
                      GESTURE_STOP_OPP_IMPULSE_LIFT_RATIO;
    if (from_lift > GESTURE_STOP_OPP_IMPULSE_LIFT_CAP_MS) {
        from_lift = GESTURE_STOP_OPP_IMPULSE_LIFT_CAP_MS;
    }
    return from_lift > GESTURE_STOP_OPP_IMPULSE_MIN_MS
               ? from_lift
               : GESTURE_STOP_OPP_IMPULSE_MIN_MS;
}

static void recording_stop_active_thresholds(int64_t now, float *need_imp,
                                            float *peak_min,
                                            float *settle_linear_max,
                                            int64_t *settle_ms)
{
    int64_t elapsed =
        (recording_stop_match_ms > 0) ? (now - recording_stop_match_ms) : 0;
    float need = recording_stop_impulse_threshold_base();
    float peak = GESTURE_STOP_OPP_ACCEL_MIN_MS2;
    float linear_max = GESTURE_STOP_SETTLE_LINEAR_MS2;
    int64_t settle = GESTURE_STOP_SETTLE_MS;

    if (elapsed >= GESTURE_STOP_SOFTEN_AFTER_MS) {
        if (need > GESTURE_STOP_OPP_IMPULSE_SOFT_MS) {
            need = GESTURE_STOP_OPP_IMPULSE_SOFT_MS;
        }
        peak = GESTURE_STOP_OPP_ACCEL_SOFT_MS2;
        linear_max = GESTURE_STOP_SETTLE_LINEAR_SOFT_MS2;
        settle = GESTURE_STOP_SETTLE_SOFT_MS;
    }
    if (elapsed >= GESTURE_STOP_SOFTEN2_AFTER_MS) {
        if (need > GESTURE_STOP_OPP_IMPULSE_SOFT2_MS) {
            need = GESTURE_STOP_OPP_IMPULSE_SOFT2_MS;
        }
        peak = GESTURE_STOP_OPP_ACCEL_SOFT2_MS2;
        settle = GESTURE_STOP_SETTLE_SOFT2_MS;
    }

    *need_imp = need;
    *peak_min = peak;
    *settle_linear_max = linear_max;
    *settle_ms = settle;
}

static void emit_recording_stop_near_miss(int64_t now, uint8_t reason,
                                         float need_imp, float pulse_ms)
{
    if (pulse_ms < (float)GESTURE_STOP_NEAR_MISS_MIN_PULSE_MS &&
        recording_stop_opp_peak_ms2 < GESTURE_STOP_OPP_ACCEL_SOFT2_MS2) {
        return;
    }
    if (recording_stop_near_miss_last_ms > 0 &&
        (now - recording_stop_near_miss_last_ms) <
            GESTURE_STOP_NEAR_MISS_MIN_INTERVAL_MS) {
        return;
    }
    recording_stop_near_miss_last_ms = now;
    APP_TRACE(">>> Stop near-miss reason=0x%02x opp=%.3f peak=%.2f "
           "pulse=%.0f need=%.3f\n",
           reason,
           (double)recording_stop_opp_impulse_ms,
           (double)recording_stop_opp_peak_ms2,
           (double)pulse_ms,
           (double)need_imp);
    send_gesture_diag(GESTURE_DIAG_STOP_NEAR_MISS, reason,
                      recording_stop_opp_impulse_ms,
                      recording_stop_opp_peak_ms2,
                      (reason == GESTURE_DIAG_REASON_STOP_PULSE_SHORT ||
                       reason == GESTURE_DIAG_REASON_STOP_PULSE_LONG)
                          ? pulse_ms
                          : need_imp);
}

static void reject_recording_stop_pulse(int64_t now, float need_imp,
                                       float peak_min, float pulse_ms)
{
    uint8_t reason = GESTURE_DIAG_REASON_STOP_IMPULSE_LOW;

    if (pulse_ms > (float)GESTURE_STOP_OPP_PULSE_MAX_MS) {
        reason = GESTURE_DIAG_REASON_STOP_PULSE_LONG;
    } else if (pulse_ms < (float)GESTURE_STOP_OPP_PULSE_MIN_MS) {
        reason = GESTURE_DIAG_REASON_STOP_PULSE_SHORT;
    } else if (recording_stop_opp_peak_ms2 < peak_min) {
        reason = GESTURE_DIAG_REASON_STOP_PEAK_LOW;
    } else if (recording_stop_opp_impulse_ms < need_imp) {
        reason = GESTURE_DIAG_REASON_STOP_IMPULSE_LOW;
    }
    emit_recording_stop_near_miss(now, reason, need_imp, pulse_ms);
    reset_recording_stop_pulse();
}

static bool recording_stop_pulse_eligible(float need_imp, float peak_min,
                                         float pulse_ms)
{
    if (pulse_ms < (float)GESTURE_STOP_OPP_PULSE_MIN_MS ||
        pulse_ms > (float)GESTURE_STOP_OPP_PULSE_MAX_MS) {
        return false;
    }
    if (recording_stop_opp_impulse_ms < need_imp) {
        return false;
    }
    /* Fast path: peak meets active min. */
    if (recording_stop_opp_peak_ms2 >= peak_min) {
        return true;
    }
    /* Slow path: longer soft pulse may pass with a slightly lower peak. */
    if (pulse_ms >= (float)GESTURE_STOP_OPP_PULSE_SLOW_MS &&
        recording_stop_opp_peak_ms2 >= GESTURE_STOP_OPP_ACCEL_SOFT2_MS2 &&
        recording_stop_opp_impulse_ms >= need_imp) {
        return true;
    }
    return false;
}

static void latch_recording_stop_pulse(float pulse_ms)
{
    recording_stop_pulse_latched = true;
    recording_stop_pulse_ms_latched = pulse_ms;
    recording_stop_settle_since_ms = 0;
    recording_stop_settle_spike_samples = 0;
    recording_stop_pulse_gap_since_ms = 0;
}

static void request_recording_stop_hand_lower(int64_t now)
{
    float pulse_ms = recording_stop_pulse_ms_latched;

    (void)now;
    APP_TRACE(">>> Hand-lower while recording → stop opp_imp=%.3f peak=%.2f "
           "pulse=%.0fms lift_imp=%.3f\n",
           (double)recording_stop_opp_impulse_ms,
           (double)recording_stop_opp_peak_ms2,
           (double)pulse_ms,
           (double)recording_stop_lift_impulse_ms);
    send_gesture_diag(GESTURE_DIAG_STOP_HAND_LOWER, GESTURE_DIAG_REASON_NONE,
                      recording_stop_opp_impulse_ms,
                      recording_stop_opp_peak_ms2,
                      pulse_ms);
    stop_requested = true;
    k_sem_give(&audio_control_wake);
    inhibit_next_settle = true;
    gesture_block_until_ms = now + GESTURE_RETRIGGER_BLOCK_MS;
    reset_recording_stop_state();
}

static void process_recording_stop_sample(float ax, float ay, float az,
                                         float gy_dps, bool gyro_ok,
                                         float dt_s, int64_t now)
{
    float accel_norm = vector_norm3(ax, ay, az);
    float a_along;
    float a_opp;
    float need_imp;
    float peak_min;
    float settle_linear_max;
    int64_t settle_need_ms;
    float pulse_ms_f;
    int64_t pulse_ms;
    bool quiet;

    (void)gy_dps;
    (void)gyro_ok;

    if (stop_requested || !recording_stop_axis_valid) {
        return;
    }

    /* Post-start inhibit: ignore opposite pulses while pose settles. */
    if (now < gesture_block_until_ms) {
        reset_recording_stop_pulse();
        recording_stop_armed = true;
        return;
    }
    recording_stop_armed = true;

    a_along = gesture_linear_world_x * recording_stop_axis_x +
              gesture_linear_world_y * recording_stop_axis_y +
              gesture_linear_world_z * recording_stop_axis_z;
    a_opp = -a_along;
    recording_stop_active_thresholds(now, &need_imp, &peak_min,
                                     &settle_linear_max, &settle_need_ms);
    quiet = accel_norm >= GESTURE_PRONATION_GRAVITY_MIN_MS2 &&
            accel_norm <= GESTURE_PRONATION_GRAVITY_MAX_MS2 &&
            gesture_linear_accel_norm_ms2 <= settle_linear_max;

    if (!recording_stop_pulse_latched) {
        if (a_opp >= peak_min) {
            if (recording_stop_pulse_since_ms == 0) {
                recording_stop_pulse_since_ms = now;
                recording_stop_opp_impulse_ms = 0.0f;
                recording_stop_opp_peak_ms2 = 0.0f;
                recording_stop_opp_samples = 0;
            }
            recording_stop_pulse_gap_since_ms = 0;
            if (recording_stop_opp_samples < 255) {
                recording_stop_opp_samples++;
            }
            recording_stop_opp_impulse_ms += a_opp * dt_s;
            if (a_opp > recording_stop_opp_peak_ms2) {
                recording_stop_opp_peak_ms2 = a_opp;
            }
        } else if (recording_stop_pulse_since_ms > 0 &&
                   a_opp > GESTURE_STOP_OPP_TAIL_MS2) {
            recording_stop_pulse_gap_since_ms = 0;
            recording_stop_opp_impulse_ms += a_opp * dt_s;
            if (a_opp > recording_stop_opp_peak_ms2) {
                recording_stop_opp_peak_ms2 = a_opp;
            }
            recording_stop_opp_samples = 0;
        } else if (recording_stop_pulse_since_ms > 0) {
            if (recording_stop_pulse_gap_since_ms == 0) {
                recording_stop_pulse_gap_since_ms = now;
            }
            if ((now - recording_stop_pulse_gap_since_ms) <
                GESTURE_STOP_PULSE_GAP_MS) {
                /* Brief dip: keep the pulse open (hysteresis). */
                return;
            }
            pulse_ms = now - recording_stop_pulse_since_ms;
            pulse_ms_f = (float)pulse_ms;
            if (recording_stop_pulse_eligible(need_imp, peak_min,
                                              pulse_ms_f)) {
                latch_recording_stop_pulse(pulse_ms_f);
            } else {
                reject_recording_stop_pulse(now, need_imp, peak_min,
                                            pulse_ms_f);
            }
        }

        if (recording_stop_pulse_since_ms > 0 &&
            !recording_stop_pulse_latched) {
            pulse_ms = now - recording_stop_pulse_since_ms;
            pulse_ms_f = (float)pulse_ms;
            if (pulse_ms > GESTURE_STOP_OPP_PULSE_MAX_MS) {
                reject_recording_stop_pulse(now, need_imp, peak_min,
                                            pulse_ms_f);
            } else if (recording_stop_opp_samples >=
                           GESTURE_STOP_OPP_CONSECUTIVE_SAMPLES &&
                       a_opp < peak_min &&
                       recording_stop_pulse_eligible(need_imp, peak_min,
                                                     pulse_ms_f)) {
                latch_recording_stop_pulse(pulse_ms_f);
            }
        }
        return;
    }

    /* After a valid opposite pulse, require a short quiet settle.
     * Single-sample a_opp spikes no longer immediately unlatch (0.0.73). */
    if (a_opp >= peak_min) {
        if (recording_stop_settle_spike_samples < 255) {
            recording_stop_settle_spike_samples++;
        }
        if (recording_stop_settle_spike_samples >=
            GESTURE_STOP_SETTLE_SPIKE_SAMPLES) {
            recording_stop_pulse_latched = false;
            recording_stop_pulse_since_ms = now;
            recording_stop_pulse_gap_since_ms = 0;
            recording_stop_opp_impulse_ms = a_opp * dt_s;
            recording_stop_opp_peak_ms2 = a_opp;
            recording_stop_opp_samples = 1;
            recording_stop_settle_since_ms = 0;
            recording_stop_settle_spike_samples = 0;
        }
        return;
    }
    recording_stop_settle_spike_samples = 0;
    if (!quiet) {
        recording_stop_settle_since_ms = 0;
        return;
    }
    if (recording_stop_settle_since_ms == 0) {
        recording_stop_settle_since_ms = now;
        return;
    }
    if ((now - recording_stop_settle_since_ms) < settle_need_ms) {
        return;
    }
    request_recording_stop_hand_lower(now);
}

static float deg_diff(float a, float b)
{
    float d = a - b;
    while (d > 180.0f) {
        d -= 360.0f;
    }
    while (d < -180.0f) {
        d += 360.0f;
    }
    return d;
}

/* RMS of a fully populated short linear-acceleration window. */
static float gesture_accel_rms(void)
{
    uint8_t n = gesture_accel_history_count;
    if (n < GESTURE_FINAL_RMS_WINDOW_SAMPLES) {
        return INFINITY;
    }
    float sq = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float a = gesture_accel_history_ms2[i];
        sq += a * a;
    }
    return sqrtf(sq / (float)n);
}

static void push_accel_history(float a)
{
    gesture_accel_history_ms2[gesture_accel_history_index] = a;
    gesture_accel_history_index =
        (gesture_accel_history_index + 1U) %
        GESTURE_FINAL_RMS_WINDOW_SAMPLES;
    if (gesture_accel_history_count < GESTURE_FINAL_RMS_WINDOW_SAMPLES) {
        gesture_accel_history_count++;
    }
}

static void clear_accel_history(void)
{
    memset(gesture_accel_history_ms2, 0, sizeof(gesture_accel_history_ms2));
    gesture_accel_history_index = 0;
    gesture_accel_history_count = 0;
}

static void retry_lift_pulse(void)
{
    gesture_lift_stage = GESTURE_LIFT_WAIT_ACCEL;
    gesture_lift_pos_impulse_ms = 0.0f;
    gesture_lift_neg_impulse_ms = 0.0f;
    gesture_lift_net_impulse_ms = 0.0f;
    gesture_lift_final_tilt_deg = 0.0f;
    gesture_lift_peak_a_up_ms2 = 0.0f;
    gesture_lift_hold_axis_valid = false;
    gesture_lift_pose_failed = false;
    gesture_lift_before_flip = false;
    gesture_lift_pos_impulse_at_entry_ms = 0.0f;
    gesture_lift_roll_at_entry_deg = 0.0f;
    gesture_lift_dir_acc_x = 0.0f;
    gesture_lift_dir_acc_y = 0.0f;
    gesture_lift_dir_acc_z = 0.0f;
    gesture_lift_event_start_ms = 0;
    gesture_lift_accel_samples = 0;
    gesture_lift_brake_samples = 0;
    gesture_hold_rms_exit_samples = 0;
    gesture_final_since_ms = 0;
    clear_accel_history();
}

static void emit_lift_near_miss(int64_t now, uint8_t reason, float elapsed_ms)
{
    if (gesture_lift_peak_a_up_ms2 < GESTURE_LIFT_ACCEL_MIN_MS2 &&
        gesture_lift_pos_impulse_ms < GESTURE_LIFT_NEAR_MISS_MIN_IMPULSE_MS) {
        return;
    }
    if (gesture_lift_near_miss_last_ms > 0 &&
        (now - gesture_lift_near_miss_last_ms) <
            GESTURE_LIFT_NEAR_MISS_MIN_INTERVAL_MS) {
        return;
    }
    gesture_lift_near_miss_last_ms = now;
    APP_TRACE(">>> Lift near-miss reason=0x%02x peak_a_up=%.2f imp=%.3f "
           "elapsed=%.0fms\n",
           reason,
           (double)gesture_lift_peak_a_up_ms2,
           (double)gesture_lift_pos_impulse_ms,
           (double)elapsed_ms);
    send_gesture_diag(GESTURE_DIAG_LIFT_NEAR_MISS, reason,
                      gesture_lift_peak_a_up_ms2,
                      gesture_lift_pos_impulse_ms,
                      elapsed_ms);
}

static void begin_gesture_final_hold(void)
{
    gesture_lift_stage = GESTURE_LIFT_WAIT_HOLD;
    gesture_lift_hold_axis_valid = false;
    gesture_lift_pose_failed = false;
    gesture_hold_rms_exit_samples = 0;
    clear_accel_history();
}

static float gesture_tilt_from_hold_axis_deg(void)
{
    if (!gesture_lift_hold_axis_valid) {
        return 0.0f;
    }

    float g_norm = vector_norm3(gesture_gravity_lp_x,
                                gesture_gravity_lp_y,
                                gesture_gravity_lp_z);
    if (g_norm <= 0.1f) {
        return 180.0f;
    }

    float cos_tilt = (gesture_gravity_lp_x * gesture_lift_hold_axis_x +
                      gesture_gravity_lp_y * gesture_lift_hold_axis_y +
                      gesture_gravity_lp_z * gesture_lift_hold_axis_z) /
                     g_norm;
    cos_tilt = CLAMP(cos_tilt, -1.0f, 1.0f);
    return acosf(cos_tilt) * (float)GESTURE_RAD_TO_DEG;
}

static void reset_gesture_sequence(void)
{
    gesture_phase = GESTURE_WAITING;
    gesture_palm_candidate_since_ms = 0;
    gesture_palm_candidate_gx = 0.0f;
    gesture_palm_candidate_gy = 0.0f;
    gesture_palm_candidate_gz = 0.0f;
    gesture_sequence_start_ms = 0;
    gesture_phase_start_ms = 0;
    gesture_final_since_ms = 0;
    gesture_z_window_min = 0.0f;
    gesture_z_window_max = 0.0f;
    gesture_z_window_start_ms = 0;
    gesture_pronation_phi_deg = 0.0f;
    gesture_pronation_peak_deg = 0.0f;
    gesture_pronation_peak_tilt_deg = 0.0f;
    gesture_pronation_ref_phi_deg = 0.0f;
    gesture_armed_az = 0.0f;
    gesture_armed_z_ratio = 0.0f;
    gesture_armed_gx = 0.0f;
    gesture_armed_gy = 0.0f;
    gesture_armed_gz = 0.0f;
    gesture_outbound_gyro_sign = 0.0f;
    gyro_reset_phase_metrics();
    reset_gesture_motion();
    gesture_armed_until_ms = 0;
    if (!is_recording && !recording_requested) {
        gyro_set_enabled(false);
#if GESTURE_DEBUG_HISTORY
        if (gesture_history_count > 0) {
            gesture_history_flush_pending = true;
        }
#endif
    }
}

static void emit_tap_event(int64_t now, bool is_double)
{
    /* Single tap is notify-only.  The host decides whether to start/stop
     * recording via RX 0x01 / 0x00 (passthrough mode uses single tap for G2
     * page advance instead).  Double tap is also notify-only. */
    if (!is_double) {
        last_activity_ms = now;
        if (light_sleep_active) {
            light_sleep_active = false;
            send_event_packet(0x21);
            APP_TRACE(">>> Light sleep wake (single tap)\n");
        }
        APP_TRACE(">>> Single tap detected (software acceleration)\n");
        send_event_packet(EVT_SINGLE_TAP);
        return;
    }

    /* Double tap: discard any in-progress start-gesture dwell so a tap does
     * not become palm-up still.  No recording toggle. */
    if (gesture_detect_effective() &&
        operation_mode != OPERATION_MODE_DRIVING &&
        !is_recording && !recording_requested) {
#if GESTURE_DEBUG_HISTORY
        if (gesture_trajectory_committed) {
            gesture_trajectory_finish(2, GESTURE_DIAG_REASON_SEQUENCE_TIMEOUT);
        } else if (gesture_trajectory_active) {
            gesture_trajectory_discard();
        }
#endif
        reset_gesture_sequence();
        gesture_rearm_required = true;
        gesture_block_until_ms = now + GESTURE_RETRIGGER_BLOCK_MS;
    }

    if (light_sleep_active) {
        light_sleep_active = false;
        send_event_packet(0x21);
        APP_TRACE(">>> Light sleep wake (double tap)\n");
    }
    last_activity_ms = now;
    APP_TRACE(">>> Double tap detected (software acceleration)\n");
    send_event_packet(EVT_DOUBLE_TAP);
}

static void process_tap_event(int64_t now)
{
    if (!tap_fifo_ready) {
        /* Retry failed FIFO recovery without using stale classifier state. */
        static int64_t retry_after_ms;
        if (device_is_ready(imu) && now >= retry_after_ms) {
            retry_after_ms = now + 1000;
            tap_last_read_ret = configure_tap_detection();
        }
        return;
    }
    uint8_t status[4];
    int ret = i2c_burst_read_dt(&imu_i2c, LSM6DS3TR_C_REG_FIFO_STATUS1,
                               status, sizeof(status));
    tap_last_read_ret = ret;
    if (ret < 0) goto lost_samples;
    unsigned words = status[0] | ((status[1] & 0x07) << 8);
    unsigned pattern = status[2] | ((status[3] & 0x03) << 8);
    unsigned samples = words / 3;
    if ((status[1] & (BIT(6) | BIT(5))) || pattern != 0 ||
        samples > TAP_FIFO_MAX_SAMPLES) {
        LOG_WRN("Tap FIFO discontinuity: words=%u pattern=%u", words, pattern);
        goto lost_samples;
    }
    for (unsigned n = 0; n < samples; n++) {
        int16_t capture_raw[3];
        /* FIFO output is a 16-bit port, NOT the normal XYZ register bank.
         * Read one word per transaction; do not assume 6-byte auto-wrap. */
        for (unsigned axis = 0; axis < 3; axis++) {
            uint8_t raw[2];
            ret = i2c_burst_read_dt(&imu_i2c, LSM6DS3TR_C_REG_FIFO_DATA,
                                   raw, sizeof(raw));
            if (ret < 0) goto lost_samples;
            capture_raw[axis] = (int16_t)sys_get_le16(raw);
        }
        uint16_t flags;
        enum tap_result result = tap_process_sample(&software_tap, &tap_live_config,
            capture_raw, atomic_get(&debug_active) ? &tap_stats : NULL,
            tap_capture_active ? &flags : NULL);
        if (tap_capture_active) {
            struct tap_capture_sample *v = &tap_capture[tap_capture_count++];
            memcpy(v->raw, capture_raw, sizeof(v->raw));
            v->flags = flags | (tap_capture_gap ? 64 : 0);
            tap_capture_gap = false;
            if (tap_capture_count >= tap_capture_target) tap_capture_active = false;
        }
        if (result != TAP_NONE) {
            tap_nonzero_count++; /* Software classifications, retained D0 layout. */
            emit_tap_event(now, result == TAP_DOUBLE);
        }
    }
    return;

lost_samples:
    if (tap_capture_active) { tap_capture_gap = true; tap_capture_gaps++; }
    if (atomic_get(&debug_active)) tap_stats.discontinuities++;
    tap_last_read_ret = ret < 0 ? ret : -EOVERFLOW;
    if (ret < 0) LOG_WRN("Tap FIFO read failed: %d", ret);
    (void)reset_tap_fifo();
}

static void process_tap_stats_request(void)
{
    int request = atomic_set(&tap_stats_request, 0);
    if (!request || !atomic_get(&debug_active)) return;
    if (request == 2) memset(&tap_stats, 0, sizeof(tap_stats));
    uint8_t pkt[43] = {0x00, 0x55, 0xD5};
    const uint32_t counters[] = {tap_stats.samples, tap_stats.discontinuities,
        tap_stats.candidates, tap_stats.strikes, tap_stats.broad, tap_stats.lateral};
    for (unsigned i = 0; i < ARRAY_SIZE(counters); i++) {
        sys_put_le32(counters[i], &pkt[3 + 4 * i]);
    }
    /* Explicit IEEE754 little-endian serialization, matching existing events. */
    const float values[] = {tap_stats.peak[0], tap_stats.peak[1],
                            tap_stats.peak[2], tap_stats.edge};
    for (unsigned i = 0; i < ARRAY_SIZE(values); i++) {
        uint32_t bits;
        memcpy(&bits, &values[i], sizeof(bits));
        sys_put_le32(bits, &pkt[27 + 4 * i]);
    }
    notify_all_conns(pkt, sizeof(pkt));
}

static void tap_config_wire(struct tap_config *c, uint8_t *wire, bool decode)
{
    float *fields[] = {&c->quiet_g, &c->peak_g, &c->edge_g, &c->dominance,
                       &c->gravity_alpha, &c->noise_multiplier};
    uint16_t *times[] = {&c->pre_quiet_ms, &c->max_rise_ms, &c->post_quiet_ms,
        &c->max_pulse_ms, &c->min_gap_ms, &c->max_gap_ms, &c->cooldown_ms, &c->suppress_ms};
    for (unsigned i = 0; i < ARRAY_SIZE(fields); i++) {
        uint32_t bits;
        if (decode) { bits = sys_get_le32(wire + 4*i); memcpy(fields[i], &bits, 4); }
        else { memcpy(&bits, fields[i], 4); sys_put_le32(bits, wire + 4*i); }
    }
    for (unsigned i = 0; i < ARRAY_SIZE(times); i++) {
        if (decode) *times[i] = sys_get_le16(wire + 24 + 2*i);
        else sys_put_le16(*times[i], wire + 24 + 2*i);
    }
}

static void send_debug_status(struct bt_conn *conn, uint8_t op, uint8_t status)
{
    uint8_t pkt[8] = {0, 0x55, 0xD8, op, status,
                     debug_session_enabled(&debug_session) ? 1 : 0};
    sys_put_le16(debug_session_remaining(&debug_session, k_uptime_get()), pkt + 6);
    notify_one(conn, pkt, sizeof(pkt));
}

static void debug_cleanup(void)
{
    atomic_clear(&debug_active);
    tap_capture_active = false;
    tap_capture_target = tap_capture_count = 0;
    atomic_clear(&tap_stats_request);
    memset(&tap_stats, 0, sizeof(tap_stats));
#if GESTURE_DEBUG_HISTORY
    bool was_host_collection = gesture_host_collection_active || gesture_host_collection_flush_pending;
    gesture_capture_enabled = false;
    gesture_trajectory_discard();
    gesture_history_clear();
    gesture_host_collection_active = false;
    gesture_host_collection_flush_pending = false;
    gesture_host_collection_count = 0;
    if (was_host_collection && gesture_phase == GESTURE_WAITING &&
        !is_recording && !recording_requested) gyro_set_enabled(false);
#endif
    if (debug_owner) { bt_conn_unref(debug_owner); debug_owner = NULL; }
}

static void handle_debug_command(const struct tap_debug_command *command)
{
    struct tap_debug_command cmd = *command;
    struct bt_conn_info info;
    /* A queued command from a disconnected peer must not activate diagnostics. */
    if (bt_conn_get_info(cmd.conn, &info) || info.state != BT_CONN_STATE_CONNECTED) return;
    if (cmd.command == 0x55) {
        uint8_t result = debug_session_request(&debug_session, cmd.op,
                                               (uintptr_t)cmd.conn, k_uptime_get());
        if (result == DEBUG_OK && cmd.op == 1) {
            if (!debug_owner) {
                debug_owner = bt_conn_ref(cmd.conn);
                memset(&tap_stats, 0, sizeof(tap_stats));
            }
            atomic_set(&debug_active, 1);
        } else if (result == DEBUG_OK && cmd.op == 2) {
            debug_cleanup();
        }
        send_debug_status(cmd.conn, cmd.op, result);
        return;
    }
    bool config_read = cmd.command == 0x54 && cmd.op == 0;
    if (!config_read && !debug_session_allowed(&debug_session, (uintptr_t)cmd.conn)) {
        send_debug_status(cmd.conn, cmd.command,
                          debug_session_enabled(&debug_session) ? DEBUG_BUSY : DEBUG_REQUIRED);
        return;
    }
    if (cmd.command == 0x52) {
        atomic_set(&tap_stats_request, cmd.op ? 2 : 1);
        return;
    }
    if (cmd.command == CMD_IMU_REG_READ || cmd.command == CMD_IMU_REG_WRITE) {
        uint8_t reg = cmd.op, value = 0;
        int ret = 0;
        if (cmd.command == CMD_IMU_REG_WRITE) {
            ret = tap_reg_writable(reg) ? i2c_reg_write_byte_dt(&imu_i2c, reg, cmd.arg) : -EPERM;
        }
        if (!ret) ret = i2c_reg_read_byte_dt(&imu_i2c, reg, &value);
        send_tap_reg_ack(reg, value, ret);
        return;
    }
#if GESTURE_DEBUG_HISTORY
    if (cmd.command == CMD_SET_GESTURE_CAPTURE) {
        gesture_capture_enabled = cmd.op != 0;
        if (!gesture_capture_enabled) { gesture_trajectory_discard(); gesture_history_clear(); }
        send_gesture_capture_status();
        return;
    }
    if (cmd.command == 0x04) {
        send_debug_status(cmd.conn, cmd.command,
                          gesture_host_collection_start() ? DEBUG_OK : DEBUG_BUSY);
        return;
    }
#endif
    uint8_t status = 0;
    if (cmd.command == 0x54) {
        if (cmd.op != 0) {
            if (tap_capture_active) status = 1;
            else if (cmd.op == 1 && !tap_config_valid(&cmd.config)) status = 2;
            else {
                tap_live_config = cmd.op == 2 ? tap_default_config : cmd.config;
                if (reset_tap_fifo() < 0) status = 3;
            }
        }
        uint8_t pkt[44] = {0, 0x55, 0xD7, status};
        tap_config_wire(&tap_live_config, pkt + 4, false);
        notify_all_conns(pkt, sizeof(pkt));
        return;
    }
    if (cmd.op == 1) {
        if (tap_capture_active || is_recording || recording_requested) status = 1;
        else if (cmd.arg < TAP_SAMPLE_HZ || cmd.arg > TAP_CAPTURE_MAX) status = 2;
        else if (reset_tap_fifo() < 0) status = 3;
        else {
            tap_capture_config = tap_live_config;
            tap_capture_id++;
            tap_capture_count = 0; tap_capture_target = cmd.arg;
            tap_capture_gaps = 0; tap_capture_gap = false;
            tap_capture_deadline_ms = k_uptime_get() +
                (1000 * cmd.arg + TAP_SAMPLE_HZ - 1) / TAP_SAMPLE_HZ + 2000;
            tap_capture_active = true;
        }
    }
    if (cmd.op == 2) {
        uint8_t pkt[138] = {0, 0x55, 0xD6, 2, 0};
        unsigned count = 0;
        if (tap_capture_active) pkt[4] = 1;
        else if (cmd.arg >= tap_capture_count) pkt[4] = 2;
        else count = MIN(16, tap_capture_count - cmd.arg);
        sys_put_le16(tap_capture_id, pkt + 5);
        sys_put_le16(cmd.arg, pkt + 7); pkt[9] = count;
        for (unsigned i = 0; i < count; i++) {
            const struct tap_capture_sample *v = &tap_capture[cmd.arg + i];
            for (unsigned axis = 0; axis < 3; axis++)
                sys_put_le16((uint16_t)v->raw[axis], pkt + 10 + 8*i + 2*axis);
            sys_put_le16(v->flags, pkt + 16 + 8*i);
        }
        notify_all_conns(pkt, 10 + count*8);
        return;
    }
    uint8_t pkt[58] = {0, 0x55, 0xD6, cmd.op, status};
    sys_put_le16(tap_capture_id, pkt + 5);
    sys_put_le16(tap_capture_target, pkt + 7);
    sys_put_le16(tap_capture_count, pkt + 9);
    sys_put_le16(TAP_SAMPLE_HZ, pkt + 11);
    sys_put_le32(tap_capture_gaps, pkt + 13);
    pkt[17] = tap_capture_active;
    tap_config_wire(&tap_capture_config, pkt + 18, false);
    notify_all_conns(pkt, sizeof(pkt));
}

static void process_tap_debug_command(void)
{
    struct bt_conn_info info;
    bool connected = debug_owner && !bt_conn_get_info(debug_owner, &info) &&
                     info.state == BT_CONN_STATE_CONNECTED;
    if (debug_session_expire(&debug_session, k_uptime_get(), connected)) debug_cleanup();
    if (tap_capture_active && k_uptime_get() > tap_capture_deadline_ms) {
        tap_capture_active = false;
        tap_capture_gaps++;
    }
    struct tap_debug_command cmd;
    if (!k_msgq_get(&tap_debug_queue, &cmd, K_NO_WAIT)) {
        handle_debug_command(&cmd);
        bt_conn_unref(cmd.conn);
    }
}

static void apply_pending_operation_mode(void)
{
    uint8_t requested = operation_mode_pending;

    if (requested == OPERATION_MODE_PENDING_NONE ||
        requested == operation_mode) {
        if (requested == operation_mode) {
            operation_mode_pending = OPERATION_MODE_PENDING_NONE;
            send_operation_mode_status();
        }
        return;
    }
    if (requested != OPERATION_MODE_NORMAL &&
        requested != OPERATION_MODE_DRIVING) {
        operation_mode_pending = OPERATION_MODE_PENDING_NONE;
        send_operation_mode_status();
        return;
    }
    if (is_recording || recording_requested) {
        return;
    }

    operation_mode = requested;
    operation_mode_pending = OPERATION_MODE_PENDING_NONE;
    reset_gesture_sequence();
    gesture_quiet_since_ms = 0;
    APP_TRACE(">>> Operation mode: %s\n",
           operation_mode == OPERATION_MODE_DRIVING ? "DRIVING" : "NORMAL");
    send_operation_mode_status();
}

static void update_quiet_accel_reference(float x, float y, float z)
{
    if (!gesture_quiet_accel_valid) {
        gesture_quiet_accel_x = x;
        gesture_quiet_accel_y = y;
        gesture_quiet_accel_z = z;
        gesture_quiet_accel_valid = true;
        return;
    }

    /* Low-pass only while accelerometrically quiet; vehicle vibration is averaged out. */
    gesture_quiet_accel_x += (x - gesture_quiet_accel_x) * 0.1f;
    gesture_quiet_accel_y += (y - gesture_quiet_accel_y) * 0.1f;
    gesture_quiet_accel_z += (z - gesture_quiet_accel_z) * 0.1f;
}

static void arm_pronation_reference(float phi, float ax, float ay, float az,
                                    float z_gravity_ratio, int64_t now)
{
    float n = vector_norm3(ax, ay, az);

    gesture_armed_until_ms = now + GESTURE_START_ARM_MS;
    gesture_pronation_ref_phi_deg = phi;
    gesture_armed_az = az;
    gesture_armed_z_ratio = z_gravity_ratio;
    if (n > 0.1f) {
        gesture_armed_gx = ax / n;
        gesture_armed_gy = ay / n;
        gesture_armed_gz = az / n;
    } else {
        gesture_armed_gx = 0.0f;
        gesture_armed_gy = 0.0f;
        gesture_armed_gz = 0.0f;
    }
    gesture_z_window_min = az;
    gesture_z_window_max = az;
    gesture_z_window_start_ms = now;
}

static float gesture_z_ratio_delta(float z_gravity_ratio)
{
    return fabsf(z_gravity_ratio - gesture_armed_z_ratio);
}

static bool gesture_z_sign_flipped(float az)
{
    return (az * gesture_armed_az < 0.0f) &&
           fabsf(az) >= GESTURE_PRONATION_Z_SIGN_MIN_MS2 &&
           fabsf(gesture_armed_az) >= GESTURE_PRONATION_Z_SIGN_MIN_MS2;
}

static float gesture_palm_candidate_tilt_deg(void)
{
    float n = vector_norm3(gesture_gravity_lp_x,
                           gesture_gravity_lp_y,
                           gesture_gravity_lp_z);
    float c;

    if (n <= 0.1f) {
        return 180.0f;
    }
    c = (gesture_gravity_lp_x * gesture_palm_candidate_gx +
         gesture_gravity_lp_y * gesture_palm_candidate_gy +
         gesture_gravity_lp_z * gesture_palm_candidate_gz) / n;
    return acosf(CLAMP(c, -1.0f, 1.0f)) * (float)GESTURE_RAD_TO_DEG;
}

static bool gesture_pronation_complete_detected(float phi_deg,
                                                float z_gravity_ratio,
                                                float az)
{
    bool gravity_ok =
        phi_deg >= GESTURE_PRONATION_MIN_DEG ||
        gesture_z_ratio_delta(z_gravity_ratio) >=
            GESTURE_PRONATION_Z_RATIO_DONE ||
        gesture_z_sign_flipped(az);

    return gravity_ok && gesture_gyro_hold_flip_ok();
}

static void report_palm_down_gate(float phi_deg, float z_gravity_ratio,
                                  float az)
{
    float z_delta = gesture_z_ratio_delta(z_gravity_ratio);
    bool gravity_ok =
        phi_deg >= GESTURE_PRONATION_MIN_DEG ||
        z_delta >= GESTURE_PRONATION_Z_RATIO_DONE ||
        gesture_z_sign_flipped(az);
    float signed_roll = gesture_gyro_hold_signed_roll_deg();
    float xy_ratio = gesture_gyro_hold_xy_peak_ratio();

    if (!gravity_ok) {
        send_gesture_diag(GESTURE_DIAG_PALM_DOWN_GATE,
                          GESTURE_DIAG_REASON_PALM_DOWN_GRAVITY_LOW,
                          phi_deg, z_delta,
                          gesture_z_sign_flipped(az) ? 1.0f : 0.0f);
    } else if (signed_roll < GESTURE_HOLD_GYRO_ANGLE_MIN_DEG) {
        send_gesture_diag(GESTURE_DIAG_PALM_DOWN_GATE,
                          GESTURE_DIAG_REASON_PALM_DOWN_GYRO_ANGLE_LOW,
                          signed_roll, GESTURE_HOLD_GYRO_ANGLE_MIN_DEG,
                          gesture_gyro_peak_abs_dps);
    } else if (xy_ratio < GESTURE_HOLD_GYRO_XY_PEAK_RATIO_MIN &&
               !(gesture_lift_stage != GESTURE_LIFT_WAIT_ACCEL &&
                 gesture_lift_before_flip &&
                 gesture_lift_pos_impulse_at_entry_ms >=
                     GESTURE_LIFT_XY_WAIVER_IMPULSE_MIN_MS)) {
        send_gesture_diag(GESTURE_DIAG_PALM_DOWN_GATE,
                          GESTURE_DIAG_REASON_PALM_DOWN_XY_RATIO_LOW,
                          xy_ratio, GESTURE_HOLD_GYRO_XY_PEAK_RATIO_MIN,
                          gesture_lift_pos_impulse_at_entry_ms);
    }
}

static void process_gesture_sample(float ax, float ay, float az,
                                   float gx_raw_dps, float gy_raw_dps,
                                   float gz_raw_dps, bool gyro_read_ok,
                                   float gy_dps, bool gyro_ok,
                                   int64_t now)
{

    /* Gesture detect off or driving: keep acceleration FIFO sampling alive for
     * software taps, but do not let ordinary motion samples arm or advance
     * the gesture state machine. Tap recording is host-authorized via RX. */
    if ((!gesture_detect_effective() ||
         operation_mode == OPERATION_MODE_DRIVING) && !is_recording) {
        return;
    }

    float accel_norm = vector_norm3(ax, ay, az);
    float z_gravity_ratio = accel_norm > 0.1f ? az / accel_norm : 0.0f;
    float palm_up_z_ratio = fabsf(z_gravity_ratio);
    float dt_s = (gesture_last_sample_ms > 0)
                 ? (float)(now - gesture_last_sample_ms) / 1000.0f
                 : (float)MOTION_SAMPLE_INTERVAL_MS / 1000.0f;

    gesture_last_sample_ms = now;
    if (dt_s <= 0.0f || dt_s > 0.1f) {
        dt_s = (float)MOTION_SAMPLE_INTERVAL_MS / 1000.0f;
    }

    if (is_recording) {
        update_gravity_lp(ax, ay, az, dt_s);
#if GESTURE_DEBUG_HISTORY
        if (gesture_trajectory_active) {
            uint8_t flags = (gyro_enabled ? BIT(0) : 0) |
                            (gyro_read_ok ? BIT(1) : 0) |
                            (gyro_ok ? BIT(2) : 0);
            gesture_trajectory_push(now, flags, ax, ay, az,
                                    gx_raw_dps, gy_raw_dps, gz_raw_dps);
        }
#endif
        if (gesture_detect_effective() &&
            operation_mode != OPERATION_MODE_DRIVING && !stop_requested) {
            process_recording_stop_sample(ax, ay, az, gy_dps, gyro_ok,
                                          dt_s, now);
        }
        if (gesture_linear_accel_norm_ms2 <= GESTURE_QUIET_ACCEL_MS2) {
            if (gesture_quiet_since_ms == 0) {
                gesture_quiet_since_ms = now;
            }
            update_quiet_accel_reference(ax, ay, az);
            if (gyro_ok && !gyro_bias_valid &&
                fabsf(gy_dps) < GESTURE_GYRO_BIAS_CAPTURE_MAX_DPS) {
                gyro_bias_y_dps = gy_dps;
                gyro_bias_valid = true;
            }
        } else {
            gesture_quiet_since_ms = 0;
        }
        return;
    }

    if (recording_requested) {
        update_gravity_lp(ax, ay, az, dt_s);
        /* Stop axis already frozen at MATCH; wait for DMIC start. */
        return;
    }

    if (now < gesture_block_until_ms) {
        reset_gesture_sequence();
        if (gesture_linear_accel_norm_ms2 <= GESTURE_QUIET_ACCEL_MS2) {
            if (gesture_quiet_since_ms == 0) {
                gesture_quiet_since_ms = now;
            }
            update_quiet_accel_reference(ax, ay, az);
        } else {
            gesture_quiet_since_ms = 0;
        }
        return;
    }

    update_gravity_lp(ax, ay, az, dt_s);

    if (gesture_phase == GESTURE_WAITING) {
        bool board_flat = gesture_board_flat(z_gravity_ratio);
        bool gravity_ok =
            accel_norm >= GESTURE_START_GRAVITY_MIN_MS2 &&
            accel_norm <= GESTURE_START_GRAVITY_MAX_MS2;
        bool accel_quiet =
            gesture_linear_accel_norm_ms2 <= GESTURE_START_QUIET_ACCEL_MS2;

        if (!board_flat || !gravity_ok || !accel_quiet) {
            gesture_palm_candidate_since_ms = 0;
            clear_accel_history();
#if GESTURE_DEBUG_HISTORY
            if (gesture_trajectory_active && !gesture_trajectory_committed) {
                gesture_trajectory_discard();
            }
#endif
            if (!board_flat || !gravity_ok || !accel_quiet) {
                gesture_rearm_required = false;
            }
            if ((now - gesture_diag_last_report_ms) >= 250) {
                send_gesture_diag(GESTURE_DIAG_WAIT_REJECT,
                                  board_flat
                                      ? GESTURE_DIAG_REASON_QUIET_NOT_READY
                                      : GESTURE_DIAG_REASON_START_NOT_PALM_UP,
                                  palm_up_z_ratio,
                                  GESTURE_START_PALM_UP_Z_MIN_RATIO,
                                  gesture_linear_accel_norm_ms2);
                gesture_diag_last_report_ms = now;
            }
            return;
        }

        if (gesture_rearm_required) {
            return;
        }

        if (gesture_palm_candidate_since_ms == 0) {
            float g_norm = vector_norm3(gesture_gravity_lp_x,
                                        gesture_gravity_lp_y,
                                        gesture_gravity_lp_z);
            if (g_norm <= 0.1f) {
                return;
            }
            gesture_palm_candidate_since_ms = now;
            gesture_palm_candidate_gx = gesture_gravity_lp_x / g_norm;
            gesture_palm_candidate_gy = gesture_gravity_lp_y / g_norm;
            gesture_palm_candidate_gz = gesture_gravity_lp_z / g_norm;
            clear_accel_history();
#if GESTURE_DEBUG_HISTORY
            gesture_history_clear();
            gesture_trajectory_clear();
#endif
            light_sleep_active = false;
            last_activity_ms = now;
            send_gesture_diag(GESTURE_DIAG_OUTBOUND_START,
                              GESTURE_DIAG_REASON_NONE,
                              palm_up_z_ratio, 0.0f,
                              gesture_linear_accel_norm_ms2);
        }

        push_accel_history(gesture_linear_accel_norm_ms2);
#if GESTURE_DEBUG_HISTORY
        gesture_trajectory_push(now, 0, ax, ay, az, 0.0f, 0.0f, 0.0f);
#endif
        if ((gesture_accel_history_count >= GESTURE_FINAL_RMS_WINDOW_SAMPLES &&
             gesture_accel_rms() > GESTURE_START_QUIET_ACCEL_MS2) ||
            gesture_palm_candidate_tilt_deg() >
                GESTURE_PALM_UP_DWELL_TILT_MAX_DEG) {
            gesture_palm_candidate_since_ms = 0;
            clear_accel_history();
#if GESTURE_DEBUG_HISTORY
            gesture_trajectory_discard();
#endif
            return;
        }

        if ((now - gesture_palm_candidate_since_ms) >=
            GESTURE_PALM_UP_DWELL_MS) {
            float phi = atan2f(-gesture_gravity_lp_x,
                               gesture_gravity_lp_z) *
                        (float)GESTURE_RAD_TO_DEG;
            float g_norm = vector_norm3(gesture_gravity_lp_x,
                                        gesture_gravity_lp_y,
                                        gesture_gravity_lp_z);
            float zr = gesture_gravity_lp_z / g_norm;

#if GESTURE_DEBUG_HISTORY
            gesture_trajectory_committed = true;
#endif
            gyro_set_enabled(true);
            arm_pronation_reference(phi, gesture_gravity_lp_x,
                                     gesture_gravity_lp_y,
                                     gesture_gravity_lp_z, zr, now);
            gesture_phase = GESTURE_HOLDING_FINAL;
            gesture_sequence_start_ms = gesture_palm_candidate_since_ms;
            gesture_phase_start_ms = now;
            gesture_pronation_phi_deg = 0.0f;
            gesture_pronation_peak_deg = 0.0f;
            gesture_pronation_peak_tilt_deg = 0.0f;
            gesture_outbound_gyro_sign = 0.0f;
            gyro_reset_phase_metrics();
            gesture_quiet_since_ms = 0;
            gesture_start_accel_x = gesture_gravity_lp_x;
            gesture_start_accel_y = gesture_gravity_lp_y;
            gesture_start_accel_z = gesture_gravity_lp_z;
            reset_gesture_motion();
            clear_accel_history();
            APP_TRACE(">>> Gesture palm-up dwell ready: %lld ms z=%.2f\n",
                   now - gesture_palm_candidate_since_ms,
                   (double)palm_up_z_ratio);
            send_gesture_diag(GESTURE_DIAG_OUTBOUND_READY,
                              GESTURE_DIAG_REASON_NONE,
                              (float)(now - gesture_palm_candidate_since_ms),
                              palm_up_z_ratio,
                              gesture_linear_accel_norm_ms2);
            return;
        }
        return;
    }

#if GESTURE_DEBUG_HISTORY
    if (gesture_trajectory_active) {
        uint8_t flags = (gyro_enabled ? BIT(0) : 0) |
                        (gyro_read_ok ? BIT(1) : 0) |
                        (gyro_ok ? BIT(2) : 0);
        gesture_trajectory_push(now, flags, ax, ay, az,
                                gx_raw_dps, gy_raw_dps, gz_raw_dps);
    }
#endif

    /*
     * HOLDING_FINAL: lift pulse (any palm), then rotate away from the
     * palm-up reference and hold still for 400 ms.
     *
     * Linear acceleration is projected onto the current low-pass gravity
     * direction so wrist rotation is not accumulated as translation.  The
     * final pose baseline is captured separately when the hold starts.
     * Integrating each pulse once is substantially less drift-prone than
     * estimating displacement with a double integral.
     */
    {
        float phi = atan2f(-ax, az) * (float)GESTURE_RAD_TO_DEG;
        float phi_change =
            fabsf(deg_diff(phi, gesture_pronation_ref_phi_deg));
        if (phi_change > gesture_pronation_phi_deg) {
            gesture_pronation_phi_deg = phi_change;
        }
        if (phi_change > gesture_pronation_peak_deg) {
            gesture_pronation_peak_deg = phi_change;
        }
        if (az < gesture_z_window_min) {
            gesture_z_window_min = az;
        }
        if (az > gesture_z_window_max) {
            gesture_z_window_max = az;
        }
        if (gyro_ok) {
            accumulate_gyro_roll(gy_dps, dt_s,
                                 GESTURE_HOLD_GYRO_INTEGRATE_RATE_DPS,
                                 &gesture_gyro_roll_deg,
                                 &gesture_gyro_peak_abs_dps);
            if (fabsf(gx_raw_dps) > gesture_gyro_x_peak_abs_dps) {
                gesture_gyro_x_peak_abs_dps = fabsf(gx_raw_dps);
            }
        }

        float g_norm = vector_norm3(gesture_gravity_lp_x,
                                    gesture_gravity_lp_y,
                                    gesture_gravity_lp_z);
        float a_up = 0.0f;
        if (g_norm > 0.1f) {
            a_up = (gesture_linear_world_x * gesture_gravity_lp_x +
                    gesture_linear_world_y * gesture_gravity_lp_y +
                    gesture_linear_world_z * gesture_gravity_lp_z) /
                   g_norm;
        }
        bool palm_down_now = gesture_pronation_complete_detected(
            gesture_pronation_phi_deg, z_gravity_ratio, az);
        if (!gesture_palm_down_latched && palm_down_now) {
            gesture_palm_down_latched = true;
            APP_TRACE(">>> Gesture palm-down latched: phi=%.1f roll=%.1f "
                   "xy_ratio=%.2f\n",
                   (double)gesture_pronation_phi_deg,
                   (double)gesture_gyro_roll_deg,
                   gesture_gyro_peak_abs_dps > 0.1f
                       ? (double)(gesture_gyro_x_peak_abs_dps /
                                  gesture_gyro_peak_abs_dps)
                       : 0.0);
        }
        bool palm_down_ok = gesture_palm_down_latched;
        float gy_abs = fabsf(gy_dps);
        bool gyro_quiet = !gyro_ok || (gy_abs <= GESTURE_FINAL_QUIET_RATE_DPS);

        if (gesture_lift_stage == GESTURE_LIFT_WAIT_ACCEL) {
            if (a_up > gesture_lift_peak_a_up_ms2) {
                gesture_lift_peak_a_up_ms2 = a_up;
            }
            if (a_up >= GESTURE_LIFT_ACCEL_MIN_MS2) {
                if (gesture_lift_event_start_ms == 0) {
                    gesture_lift_event_start_ms = now;
                    gesture_lift_dir_acc_x = 0.0f;
                    gesture_lift_dir_acc_y = 0.0f;
                    gesture_lift_dir_acc_z = 0.0f;
                }
                gesture_lift_accel_samples++;
                gesture_lift_pos_impulse_ms += a_up * dt_s;
                gesture_lift_net_impulse_ms += a_up * dt_s;
                gesture_lift_dir_acc_x += gesture_linear_world_x * dt_s;
                gesture_lift_dir_acc_y += gesture_linear_world_y * dt_s;
                gesture_lift_dir_acc_z += gesture_linear_world_z * dt_s;
            } else if (gesture_lift_event_start_ms > 0 && a_up > 0.0f) {
                gesture_lift_pos_impulse_ms += a_up * dt_s;
                gesture_lift_net_impulse_ms += a_up * dt_s;
                gesture_lift_dir_acc_x += gesture_linear_world_x * dt_s;
                gesture_lift_dir_acc_y += gesture_linear_world_y * dt_s;
                gesture_lift_dir_acc_z += gesture_linear_world_z * dt_s;
                gesture_lift_accel_samples = 0;
            } else if (a_up <= 0.0f) {
                if (gesture_lift_event_start_ms > 0) {
                    float pulse_ms =
                        (float)(now - gesture_lift_event_start_ms);
                    if (gesture_lift_pos_impulse_ms >=
                            GESTURE_LIFT_NEAR_MISS_MIN_IMPULSE_MS ||
                        gesture_lift_peak_a_up_ms2 >=
                            GESTURE_LIFT_ACCEL_MIN_MS2) {
                        emit_lift_near_miss(
                            now, GESTURE_DIAG_REASON_LIFT_PULSE_WEAK,
                            pulse_ms);
                    }
                }
                gesture_lift_event_start_ms = 0;
                gesture_lift_accel_samples = 0;
                gesture_lift_pos_impulse_ms = 0.0f;
                gesture_lift_net_impulse_ms = 0.0f;
                gesture_lift_dir_acc_x = 0.0f;
                gesture_lift_dir_acc_y = 0.0f;
                gesture_lift_dir_acc_z = 0.0f;
            } else {
                gesture_lift_accel_samples = 0;
            }

            if (gesture_lift_accel_samples >=
                    GESTURE_LIFT_CONSECUTIVE_SAMPLES &&
                gesture_lift_pos_impulse_ms >=
                    GESTURE_LIFT_POS_IMPULSE_MIN_MS) {
                float roll_at_lift = fabsf(gesture_gyro_roll_deg);

                gesture_lift_roll_at_entry_deg = roll_at_lift;
                gesture_lift_pos_impulse_at_entry_ms =
                    gesture_lift_pos_impulse_ms;
                gesture_lift_before_flip =
                    roll_at_lift < GESTURE_LIFT_PREFLIP_MAX_DEG;
                gesture_lift_stage = GESTURE_LIFT_WAIT_BRAKE;
                gesture_lift_brake_samples = 0;
                APP_TRACE(">>> Gesture lift pulse: roll=%.1f before_flip=%d "
                       "imp=%.3f\n",
                       (double)roll_at_lift,
                       gesture_lift_before_flip ? 1 : 0,
                       (double)gesture_lift_pos_impulse_at_entry_ms);
            }
        } else if (gesture_lift_stage == GESTURE_LIFT_WAIT_BRAKE) {
            gesture_lift_net_impulse_ms += a_up * dt_s;
            if (a_up > 0.0f) {
                gesture_lift_pos_impulse_ms += a_up * dt_s;
            } else {
                gesture_lift_neg_impulse_ms += -a_up * dt_s;
            }

            if (a_up <= -GESTURE_LIFT_BRAKE_MIN_MS2) {
                if (gesture_lift_brake_samples <
                    GESTURE_LIFT_CONSECUTIVE_SAMPLES) {
                    gesture_lift_brake_samples++;
                }
            } else if (gesture_lift_brake_samples <
                       GESTURE_LIFT_CONSECUTIVE_SAMPLES) {
                gesture_lift_brake_samples = 0;
            }

            int64_t pulse_ms = now - gesture_lift_event_start_ms;
            bool brake_ready =
                gesture_lift_brake_samples >=
                    GESTURE_LIFT_CONSECUTIVE_SAMPLES &&
                gesture_lift_neg_impulse_ms >=
                    GESTURE_LIFT_NEG_IMPULSE_MIN_MS;
            float brake_ratio =
                gesture_lift_pos_impulse_ms > 0.0f
                    ? gesture_lift_neg_impulse_ms /
                        gesture_lift_pos_impulse_ms
                    : 0.0f;
            bool settled_without_brake = false;

            if (pulse_ms >= GESTURE_LIFT_PULSE_MIN_MS &&
                palm_down_ok && gyro_quiet) {
                push_accel_history(gesture_linear_accel_norm_ms2);
                settled_without_brake =
                    gesture_accel_history_count >=
                        GESTURE_FINAL_RMS_WINDOW_SAMPLES &&
                    gesture_accel_rms() <= GESTURE_FINAL_STILL_RMS_MS2;
            } else {
                clear_accel_history();
            }

            if (brake_ready && pulse_ms < GESTURE_LIFT_PULSE_MIN_MS) {
                send_gesture_diag(
                    GESTURE_DIAG_WAIT_REJECT,
                    GESTURE_DIAG_REASON_FINAL_PULSE_DURATION_INVALID,
                    (float)pulse_ms,
                    gesture_lift_pos_impulse_ms,
                    gesture_lift_neg_impulse_ms);
                retry_lift_pulse();
                return;
            } else if (brake_ready &&
                       brake_ratio >= GESTURE_LIFT_BRAKE_RATIO_MIN) {
                begin_gesture_final_hold();
            } else if (settled_without_brake) {
                begin_gesture_final_hold();
            }
        }

        if (gesture_lift_stage != GESTURE_LIFT_WAIT_ACCEL &&
            gesture_lift_event_start_ms > 0 &&
            (now - gesture_lift_event_start_ms) >=
                GESTURE_MOTION_COMPLETE_MAX_MS) {
            int64_t motion_ms = now - gesture_lift_event_start_ms;
            uint8_t timeout_reason = gesture_palm_down_latched
                ? GESTURE_DIAG_REASON_MOTION_TOO_SLOW
                : GESTURE_DIAG_REASON_PALM_DOWN_GATE_FAILED;
            send_gesture_diag(GESTURE_DIAG_RESET,
                              timeout_reason,
                              (float)motion_ms,
                              gesture_palm_down_latched
                                  ? gesture_gyro_peak_abs_dps
                                  : gesture_gyro_hold_signed_roll_deg(),
                              gesture_palm_down_latched
                                  ? fabsf(gesture_gyro_roll_deg)
                                  : gesture_gyro_hold_xy_peak_ratio());
#if GESTURE_DEBUG_HISTORY
            gesture_trajectory_finish(2, timeout_reason);
#endif
            gesture_rearm_required = true;
            reset_gesture_sequence();
            gesture_quiet_since_ms = 0;
            return;
        }

        gesture_lift_final_tilt_deg = gesture_tilt_from_hold_axis_deg();
        float final_rms = INFINITY;
        bool final_still = false;
        if (gesture_lift_stage == GESTURE_LIFT_WAIT_HOLD && !palm_down_ok) {
            gesture_lift_hold_axis_valid = false;
            gesture_final_since_ms = 0;
            clear_accel_history();
        } else if (gesture_lift_stage == GESTURE_LIFT_WAIT_HOLD) {
            push_accel_history(gesture_linear_accel_norm_ms2);
            final_rms = gesture_accel_rms();
            final_still = final_rms <= GESTURE_FINAL_STILL_RMS_MS2;
            if (final_still && !gesture_lift_hold_axis_valid) {
                float hold_g_norm = vector_norm3(gesture_gravity_lp_x,
                                                 gesture_gravity_lp_y,
                                                 gesture_gravity_lp_z);
                if (hold_g_norm > 0.1f) {
                    gesture_lift_hold_axis_x =
                        gesture_gravity_lp_x / hold_g_norm;
                    gesture_lift_hold_axis_y =
                        gesture_gravity_lp_y / hold_g_norm;
                    gesture_lift_hold_axis_z =
                        gesture_gravity_lp_z / hold_g_norm;
                    gesture_lift_hold_axis_valid = true;
                    gesture_lift_final_tilt_deg = 0.0f;
                }
            } else if (gesture_lift_hold_axis_valid) {
                gesture_lift_final_tilt_deg =
                    gesture_tilt_from_hold_axis_deg();
                if (gesture_lift_final_tilt_deg >
                    gesture_lift_max_hold_tilt_deg) {
                    gesture_lift_max_hold_tilt_deg =
                        gesture_lift_final_tilt_deg;
                }
            }
        }
        bool final_pose_stable =
            gesture_lift_hold_axis_valid &&
            gesture_lift_final_tilt_deg <=
                GESTURE_LIFT_FINAL_TILT_MAX_DEG;
        bool hold_rms_ok = final_still;

        if (gesture_final_since_ms > 0) {
            if (final_rms <= GESTURE_FINAL_HOLD_RMS_EXIT_MS2) {
                gesture_hold_rms_exit_samples = 0;
            } else if (gesture_hold_rms_exit_samples <
                       GESTURE_FINAL_HOLD_RMS_EXIT_SAMPLES) {
                gesture_hold_rms_exit_samples++;
            }
            hold_rms_ok =
                gesture_hold_rms_exit_samples <
                    GESTURE_FINAL_HOLD_RMS_EXIT_SAMPLES;
        } else {
            gesture_hold_rms_exit_samples = 0;
        }

        if (gesture_debug_final_last_ms == 0 ||
            (now - gesture_debug_final_last_ms) >=
                GESTURE_DEBUG_FINAL_PERIOD_MS) {
            gesture_debug_final_last_ms = now;
            if (gesture_lift_stage == GESTURE_LIFT_WAIT_HOLD) {
                float reported_rms =
                    gesture_accel_history_count >=
                        GESTURE_FINAL_RMS_WINDOW_SAMPLES
                        ? final_rms
                        : gesture_linear_accel_norm_ms2;
                send_gesture_diag(GESTURE_DIAG_HOLD_SAMPLE,
                                  GESTURE_DIAG_REASON_NONE,
                                  reported_rms,
                                  gesture_lift_final_tilt_deg,
                                  gy_abs);
                if (!gesture_palm_down_latched) {
                    report_palm_down_gate(gesture_pronation_phi_deg,
                                          z_gravity_ratio, az);
                }
            } else {
                send_gesture_diag(GESTURE_DIAG_FINAL_SAMPLE,
                                  GESTURE_DIAG_REASON_NONE,
                                  (float)gesture_lift_stage,
                                  a_up, gesture_lift_net_impulse_ms);
            }
        }

        /*
         * Hold entry needs gyro quiet and RMS <= 3.0 once. Palm-down remains
         * latched for this sequence. While the timer is running, tolerate
         * minor RMS excursions; interrupt only after two samples > 3.5.
         */
        bool hold_body_ok = palm_down_ok && hold_rms_ok && final_pose_stable;
        bool hold_entry_ok = hold_body_ok &&
            (gesture_final_since_ms > 0 || gyro_quiet);

        if (gesture_lift_stage == GESTURE_LIFT_WAIT_HOLD && hold_entry_ok) {
            if (gesture_final_since_ms == 0) {
                gesture_final_since_ms = now;
                gesture_hold_rms_exit_samples = 0;
                send_gesture_diag(GESTURE_DIAG_FINAL_HOLD_START,
                                  GESTURE_DIAG_REASON_NONE,
                                  gesture_lift_pos_impulse_ms,
                                  gesture_lift_neg_impulse_ms,
                                  gesture_lift_final_tilt_deg);
            }

            int64_t hold_ms = now - gesture_final_since_ms;
            if (hold_ms >= GESTURE_FINAL_HOLD_MS) {
                bool match_lift_ok =
                    gesture_lift_pos_impulse_ms >=
                        GESTURE_MATCH_POS_IMPULSE_MIN_MS;
                bool match_pronation_ok =
                    gesture_pronation_phi_deg >=
                        GESTURE_MATCH_PRONATION_MIN_DEG;

                if (!match_lift_ok || !match_pronation_ok) {
                    uint8_t failure_bits =
                        (!match_lift_ok ? 0x01 : 0x00) |
                        (!match_pronation_ok ? 0x02 : 0x00);

                    APP_TRACE(">>> Gesture rejected by final gate: "
                           "impulse=%.3f/%.2f phi=%.1f/%.0f bits=0x%02x\n",
                           (double)gesture_lift_pos_impulse_ms,
                           (double)GESTURE_MATCH_POS_IMPULSE_MIN_MS,
                           (double)gesture_pronation_phi_deg,
                           (double)GESTURE_MATCH_PRONATION_MIN_DEG,
                           failure_bits);

                    if (!match_lift_ok) {
                        send_gesture_diag(
                            GESTURE_DIAG_WAIT_REJECT,
                            GESTURE_DIAG_REASON_MATCH_LIFT_IMPULSE_LOW,
                            gesture_lift_pos_impulse_ms,
                            GESTURE_MATCH_POS_IMPULSE_MIN_MS,
                            gesture_pronation_phi_deg);
                    }
                    if (!match_pronation_ok) {
                        send_gesture_diag(
                            GESTURE_DIAG_WAIT_REJECT,
                            GESTURE_DIAG_REASON_MATCH_PRONATION_LOW,
                            gesture_pronation_phi_deg,
                            GESTURE_MATCH_PRONATION_MIN_DEG,
                            gesture_lift_pos_impulse_ms);
                    }
                    send_gesture_diag(
                        GESTURE_DIAG_RESET,
                        GESTURE_DIAG_REASON_MATCH_GATE_FAILED,
                        gesture_lift_pos_impulse_ms,
                        gesture_pronation_phi_deg,
                        (float)failure_bits);
#if GESTURE_DEBUG_HISTORY
                    gesture_trajectory_finish(
                        2, GESTURE_DIAG_REASON_MATCH_GATE_FAILED);
#endif
                    gesture_rearm_required = true;
                    reset_gesture_sequence();
                    gesture_quiet_since_ms = 0;
                    return;
                }

                int64_t motion_ms = now - gesture_lift_event_start_ms;
                APP_TRACE(">>> Gesture MATCH: palm-up dwell + lift + palm-down hold\n");
                send_gesture_diag(GESTURE_DIAG_MOTION_COMPLETE,
                                  GESTURE_DIAG_REASON_NONE,
                                  (float)motion_ms,
                                  gesture_gyro_peak_abs_dps,
                                  fabsf(gesture_gyro_roll_deg));
                send_gesture_diag(GESTURE_DIAG_FINAL_READY,
                                  GESTURE_DIAG_REASON_NONE,
                                  gesture_lift_pos_impulse_ms,
                                  (float)hold_ms,
                                  gesture_lift_final_tilt_deg);
                {
                    float xy = gesture_gyro_hold_xy_peak_ratio();
                    bool xy_waived =
                        gesture_lift_before_flip &&
                        gesture_lift_pos_impulse_at_entry_ms >=
                            GESTURE_LIFT_XY_WAIVER_IMPULSE_MIN_MS;
                    uint8_t detail_reason =
                        (gesture_lift_before_flip ? 0x01 : 0x00) |
                        (xy_waived ? 0x02 : 0x00);

                    send_gesture_diag(GESTURE_DIAG_MATCH_DETAIL,
                                      detail_reason,
                                      xy,
                                      gesture_lift_pos_impulse_at_entry_ms,
                                      gesture_lift_roll_at_entry_deg);
                }
                send_gesture_diag(GESTURE_DIAG_MATCH,
                                  GESTURE_DIAG_REASON_NONE,
                                  gesture_pronation_phi_deg,
                                  gesture_lift_pos_impulse_ms,
                                  (float)hold_ms);
                gesture_rearm_required = true;
                /* Freeze lift axis before sequence reset clears lift state. */
                capture_recording_stop_axis_from_lift();
                {
                    float gx = gesture_gravity_lp_x;
                    float gy = gesture_gravity_lp_y;
                    float gz = gesture_gravity_lp_z;
                    /* Gyro optional during recording; linear stop is primary. */
                    recording_requested = true;
                    k_sem_give(&audio_control_wake);
                    gesture_block_until_ms = now + GESTURE_RETRIGGER_BLOCK_MS;
                    last_activity_ms = now;
                    reset_gesture_sequence();
                    gyro_set_enabled(true);
                    gesture_gravity_lp_x = gx;
                    gesture_gravity_lp_y = gy;
                    gesture_gravity_lp_z = gz;
                }
                gesture_quiet_since_ms = 0;
                return;
            }
        } else {
            bool hold_interrupted = gesture_final_since_ms > 0;
            if (hold_interrupted &&
                !final_pose_stable &&
                gesture_lift_final_tilt_deg >
                    GESTURE_LIFT_FINAL_TILT_MAX_DEG) {
                gesture_lift_pose_failed = true;
            }
            if (hold_interrupted &&
                (now - gesture_diag_last_report_ms) >= 250) {
                /* v1=rms, v2=tilt°, v3=|gy| dps — which gate dropped. */
                send_gesture_diag(GESTURE_DIAG_WAIT_REJECT,
                                  GESTURE_DIAG_REASON_FINAL_HOLD_INTERRUPTED,
                                  (final_rms < 1000.0f) ? final_rms
                                      : gesture_linear_accel_norm_ms2,
                                  gesture_lift_final_tilt_deg,
                                  gy_abs);
                gesture_diag_last_report_ms = now;
            }
            if (hold_interrupted) {
                gesture_final_since_ms = 0;
                gesture_hold_rms_exit_samples = 0;
                gesture_lift_hold_axis_valid = false;
                clear_accel_history();
            }
        }

        if (gesture_lift_stage == GESTURE_LIFT_WAIT_ACCEL &&
            (now - gesture_phase_start_ms) >=
                GESTURE_LIFT_START_TIMEOUT_MS) {
            float elapsed_ms =
                (float)(now - gesture_phase_start_ms);
            emit_lift_near_miss(now,
                                GESTURE_DIAG_REASON_LIFT_START_TIMEOUT,
                                elapsed_ms);
            send_gesture_diag(GESTURE_DIAG_RESET,
                              GESTURE_DIAG_REASON_FINAL_ACCEL_MISSING,
                              gesture_lift_peak_a_up_ms2,
                              gesture_lift_pos_impulse_ms,
                              elapsed_ms);
#if GESTURE_DEBUG_HISTORY
            gesture_trajectory_finish(
                2, GESTURE_DIAG_REASON_FINAL_ACCEL_MISSING);
#endif
            gesture_rearm_required = true;
            reset_gesture_sequence();
            gesture_quiet_since_ms = 0;
        }
    }
}

static void on_motion_started(float x, float y, float z)
{
    motion_active_start_ms = k_uptime_get();
    last_activity_ms = motion_active_start_ms;

    motion_vel_x = 0.0f; motion_vel_y = 0.0f; motion_vel_z = 0.0f;
    motion_distance   = 0.0f;
    motion_peak_speed = 0.0f;
    motion_speed_sum  = 0.0f;
    motion_speed_samples = 0;

    send_event_packet_xyz(0x10, x, y, z);
    APP_TRACE(">>> Motion active x=%.2f y=%.2f z=%.2f\n", (double)x, (double)y, (double)z);
}

static void on_motion_settled(float x, float y, float z)
{
    last_activity_ms = k_uptime_get();

    if (is_recording) {
        last_motion_time_ms = (uint32_t)k_uptime_get();
        int64_t elapsed = k_uptime_get() - motion_active_start_ms;
        uint32_t elapsed_ms = (uint32_t)(elapsed < 0 ? 0 : elapsed);
        float avg_speed = motion_speed_samples > 0
            ? motion_speed_sum / (float)motion_speed_samples : 0.0f;
        send_event_packet_settle(x, y, z, elapsed_ms, avg_speed, motion_peak_speed, motion_distance);
        APP_TRACE(">>> Motion settled x=%.2f y=%.2f z=%.2f elapsed=%u ms avg=%.3f peak=%.3f dist=%.3f\n",
               (double)x, (double)y, (double)z, elapsed_ms,
               (double)avg_speed, (double)motion_peak_speed, (double)motion_distance);
        return;
    }

    int64_t elapsed = k_uptime_get() - motion_active_start_ms;
    uint32_t elapsed_ms = (uint32_t)(elapsed < 0 ? 0 : elapsed);
    float avg_speed = motion_speed_samples > 0
        ? motion_speed_sum / (float)motion_speed_samples : 0.0f;
    if (inhibit_next_settle) {
        inhibit_next_settle = false;
        APP_TRACE(">>> Motion settled (inhibited after recording stop)\n");
        return;
    }

    last_motion_time_ms = (uint32_t)k_uptime_get();
    send_event_packet_settle(x, y, z, elapsed_ms, avg_speed, motion_peak_speed, motion_distance);
    APP_TRACE(">>> Motion settled x=%.2f y=%.2f z=%.2f elapsed=%u ms avg=%.3f peak=%.3f dist=%.3f\n",
           (double)x, (double)y, (double)z, elapsed_ms,
           (double)avg_speed, (double)motion_peak_speed, (double)motion_distance);

}


static void process_motion_sample(void)
{
    struct sensor_value accel[3];
    struct sensor_value gyro[3];
    float gx_raw_dps = 0.0f;
    float gy_raw_dps = 0.0f;
    float gz_raw_dps = 0.0f;
    float gy_dps = 0.0f;
    bool gyro_read_ok = false;
    bool gyro_ok = false;
    int ret = sensor_sample_fetch(imu);
    if (ret < 0) { LOG_ERR("fetch failed: %d", ret); return; }
    if (sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel) < 0) return;

    double x = sensor_value_to_double(&accel[0]);
    double y = sensor_value_to_double(&accel[1]);
    double z = sensor_value_to_double(&accel[2]);

    if (calibration_count < CALIBRATION_SAMPLES) {
        accumulate_calibration(x, y, z);
        return;
    }

    int64_t now = k_uptime_get();

    if (gyro_enabled) {
        if (sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro) == 0) {
            /* sensor_value gyro is rad/s; convert to dps. */
            gx_raw_dps = (float)(sensor_value_to_double(&gyro[0]) *
                                 GESTURE_RAD_TO_DEG);
            gy_raw_dps = (float)(sensor_value_to_double(&gyro[1]) *
                                 GESTURE_RAD_TO_DEG);
            gz_raw_dps = (float)(sensor_value_to_double(&gyro[2]) *
                                 GESTURE_RAD_TO_DEG);
            gyro_read_ok = true;
            gy_dps = gyro_y_corrected_dps(gy_raw_dps);
            gyro_ok = gyro_is_settled(now);
            gyro_sample_valid = gyro_ok;
            if (gyro_ok && !gyro_bias_valid &&
                fabsf(gy_raw_dps) < GESTURE_GYRO_BIAS_CAPTURE_MAX_DPS &&
                gesture_linear_accel_norm_ms2 <= GESTURE_QUIET_ACCEL_MS2) {
                gyro_bias_y_dps = gy_raw_dps;
                gyro_bias_valid = true;
                gy_dps = 0.0f;
            }
        }
    }

    process_gesture_sample((float)x, (float)y, (float)z,
                           gx_raw_dps, gy_raw_dps, gz_raw_dps,
                           gyro_read_ok, gy_dps, gyro_ok, now);

#if GESTURE_DEBUG_HISTORY
    if (gesture_host_collection_active) {
        uint8_t flags = (gyro_enabled ? BIT(0) : 0) |
                        (gyro_read_ok ? BIT(1) : 0) |
                        (gyro_ok ? BIT(2) : 0);
        gesture_host_collection_push(now, flags, (float)x, (float)y, (float)z,
                                     gx_raw_dps, gy_raw_dps, gz_raw_dps);
    }
    if ((gesture_history_flush_pending || gesture_trajectory_flush_pending ||
         gesture_host_collection_flush_pending) &&
        !is_recording && !recording_requested) {
        flush_host_collection();
        flush_gesture_trajectory();
        flush_gesture_history();
    }
#endif

    double delta    = motion_delta(x, y, z);
    double step     = sample_delta(x, y, z);
    update_step_window(step);
    double activity = step_window_sum;
    double peak     = step_window_peak();

    ARG_UNUSED(delta);

    if (!motion_active) {
        bool accel_active = activity >= MOTION_ENTRY_ACTIVITY_MS2 &&
                            peak >= MOTION_ENTRY_PEAK_MS2;
        if (accel_active) {
            active_high_count++;
        } else {
            active_high_count = 0;
            update_baseline(x, y, z);
        }

        if (active_high_count >= MOTION_START_WINDOWS &&
            (now - last_report_ms) >= REPORT_COOLDOWN_MS) {
            motion_active = true;
            settle_count = 0;
            active_high_count = 0;
            last_report_ms = now;
            z_excursion_peak = abs_double(z - baseline_z);
            atomic_inc(&detected_motion_count);

            APP_INFO("Motion! count=%d activity=%.3f peak=%.3f",
                    (int)atomic_get(&detected_motion_count), activity, peak);
            last_motion_time_ms = now;
            on_motion_started((float)x, (float)y, (float)z);
        }
        return;
    }

    /* Motion is active — update running z_excursion_peak */
    double z_dev = abs_double(z - baseline_z);
    if (z_dev > z_excursion_peak) {
        z_excursion_peak = z_dev;
    }

    /* Velocity & distance integration */
    static const float DT = MOTION_SAMPLE_INTERVAL_MS / 1000.0f;
    motion_vel_x += (float)(x - baseline_x) * DT;
    motion_vel_y += (float)(y - baseline_y) * DT;
    motion_vel_z += (float)(z - baseline_z) * DT;
    float spd = sqrtf(motion_vel_x * motion_vel_x +
                      motion_vel_y * motion_vel_y +
                      motion_vel_z * motion_vel_z);
    if (spd > motion_peak_speed) { motion_peak_speed = spd; }
    motion_distance += spd * DT;
    motion_speed_sum += spd;
    motion_speed_samples++;

    if ((activity >= MOTION_CONTINUE_ACTIVITY_MS2 || peak >= MOTION_CONTINUE_PEAK_MS2) &&
        (now - last_report_ms) >= REPORT_COOLDOWN_MS) {
        last_report_ms = now;
        last_motion_time_ms = now;
    }

    if (activity <= MOTION_SETTLE_ACTIVITY_MS2 &&
        peak <= MOTION_SETTLE_PEAK_MS2) {
        settle_count++;
    } else {
        settle_count = 0;
    }

    if (settle_count >= MOTION_SETTLE_WINDOWS) {
        motion_active = false;
        settle_count = 0;
        set_baseline(x, y, z);
        reset_step_window();
        APP_INFO("Motion settled: baseline=(%.3f, %.3f, %.3f)", x, y, z);
        on_motion_settled((float)x, (float)y, (float)z);
    }
}

/* ============================================================================
 * BLE Audio Service
 * ============================================================================ */

static void audio_tx_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    APP_INFO("Audio TX CCCD updated: %d", value);
    APP_TRACE(">>> Audio CCCD: %d (1=notify enabled)\n", value);
}

static ssize_t audio_rx_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len, uint16_t offset,
                              uint8_t flags)
{
    const uint8_t *data = buf;

    if (len >= 1) {
        if (data[0] == 0x01) {
            if (is_recording) {
                /* Host thinks we are idle (e.g. after a missed 0x01/0x02).
                 * Clear the stuck session then start again on the next loop. */
                stop_requested = true;
                recording_requested = true;
                k_sem_give(&audio_control_wake);
                APP_TRACE(">>> START while recording — restart\n");
            } else {
                APP_TRACE(">>> START command\n");
                stop_requested = false;
                recording_requested = true;
                k_sem_give(&audio_control_wake);
            }
        } else if (data[0] == 0x00) {
            APP_TRACE(">>> STOP command\n");
            stop_requested = true;
            k_sem_give(&audio_control_wake);
        } else if (data[0] == 0x02) {
            /* Claim primary role */
            int idx = conn_index(conn);
            if (idx >= 0) {
                primary_idx = idx;
                inhibit_next_settle = false;
                APP_TRACE(">>> conn[%d] claimed primary\n", idx);
                /* Update connection params: fast for primary, slow for secondary */
                k_work_schedule(&conn_param_work, K_MSEC(200));
            }
        } else if (data[0] == 0x03) {
            /* Yield primary role to the other connection */
            bool found = false;
            for (int i = 0; i < MAX_CONNS; i++) {
                if (connections[i] && connections[i] != conn) {
                    primary_idx = i;
                    APP_TRACE(">>> Primary yielded to conn[%d]\n", i);
                    found = true;
                    break;
                }
            }
            if (!found) {
                /* Keep the yielding connection as primary — clearing would
                 * drop all event/audio delivery until the next claim. */
                int self = conn_index(conn);
                if (self >= 0) {
                    primary_idx = self;
                    APP_TRACE(">>> Primary yield ignored (no peer); conn[%d] stays primary\n",
                           self);
                } else {
                    primary_idx = -1;
                    APP_TRACE(">>> Primary yielded but no peer present; primary cleared\n");
                }
            }
            /* Update connection params after yield */
            k_work_schedule(&conn_param_work, K_MSEC(200));
        } else if (data[0] == CMD_SET_OPERATION_MODE && len >= 2) {
            if (data[1] == OPERATION_MODE_NORMAL ||
                data[1] == OPERATION_MODE_DRIVING) {
                operation_mode_pending = data[1];
                APP_TRACE(">>> Operation mode request: %s%s\n",
                       data[1] == OPERATION_MODE_DRIVING ? "DRIVING" : "NORMAL",
                       (is_recording || recording_requested) ? " (pending)" : "");
                /* Ack on accept too. While recording, apply_pending_operation_mode()
                 * returns early and would otherwise notify nothing, so the host
                 * could never observe a non-none pending. */
                send_operation_mode_status();
            } else {
                APP_TRACE(">>> Invalid operation mode: %u\n", data[1]);
                send_operation_mode_status();
            }
        } else if (data[0] == CMD_SET_GESTURE_DETECT && len >= 2) {
            gesture_detect_enabled = (data[1] != 0);
            if (!gesture_detect_enabled &&
                !is_recording && !recording_requested) {
#if GESTURE_DEBUG_HISTORY
                if (gesture_trajectory_committed) {
                    gesture_trajectory_finish(2, GESTURE_DIAG_REASON_SEQUENCE_TIMEOUT);
                } else if (gesture_trajectory_active) {
                    gesture_trajectory_discard();
                }
#endif
                reset_gesture_sequence();
                gesture_rearm_required = true;
            }
            APP_TRACE(">>> Gesture detect: %s\n",
                   gesture_detect_effective() ? "ON" : "OFF");
            send_gesture_detect_status();
        } else if (data[0] == 0x55 || data[0] == 0x52 || data[0] == 0x53 ||
                   data[0] == 0x54 || data[0] == CMD_IMU_REG_WRITE ||
                   data[0] == CMD_IMU_REG_READ || data[0] == CMD_SET_GESTURE_CAPTURE ||
                   data[0] == 0x04) {
            struct tap_debug_command cmd = {.command = data[0], .op = len > 1 ? data[1] : 0};
            bool valid = len == 2;
            if (cmd.command == 0x04) valid = len == 1;
            else if (cmd.command == CMD_IMU_REG_WRITE) {
                valid = len == 3;
                if (valid) cmd.arg = data[2];
            } else if (cmd.command == 0x53 && cmd.op != 0) {
                valid = len == 4 && cmd.op <= 2;
                if (valid) cmd.arg = sys_get_le16(data + 2);
            } else if (cmd.command == 0x54 && cmd.op == 1) {
                valid = len == 42;
                if (valid) { uint8_t wire[40]; memcpy(wire, data + 2, 40);
                             tap_config_wire(&cmd.config, wire, true); }
            } else if (cmd.command == 0x55 || cmd.command == 0x54) valid &= cmd.op <= 2;
            else if (cmd.command == 0x52 || cmd.command == CMD_SET_GESTURE_CAPTURE) valid &= cmd.op <= 1;
            if (!valid) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
            cmd.conn = bt_conn_ref(conn);
            if (k_msgq_put(&tap_debug_queue, &cmd, K_NO_WAIT)) {
                bt_conn_unref(cmd.conn);
                return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
            }
        } else if (data[0] == CMD_REBOOT_BOOTLOADER && len >= 3 &&
                   data[1] == REBOOT_MAGIC_1 && data[2] == REBOOT_MAGIC_2) {
            bool accepted = !(is_recording || recording_requested);
            uint8_t ack[5] = { 0x00, 0x55, EVT_REBOOT_ACK, 1,
                               accepted ? 1 : 0 };

            APP_TRACE(">>> Reboot to bootloader requested: %s\n",
                   accepted ? "accepted" : "REFUSED (recording)");
            notify_one(conn, ack, sizeof(ack));

            if (accepted) {
                /* Long enough for the ack to actually reach the air: the
                 * connection is at the 30-50 ms idle interval and notify_one()
                 * only queues the packet. */
                k_work_schedule(&reboot_work, K_MSEC(500));
            }
        }
    }

    return len;
}

BT_GATT_SERVICE_DEFINE(audio_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(AUDIO_UUID_SERVICE)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(AUDIO_UUID_TX_CHAR),
        BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
    BT_GATT_CCC(audio_tx_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(AUDIO_UUID_RX_CHAR),
        BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, audio_rx_write, NULL),
);

/* Operation mode ack: [0x00][0x55][0x40][effective][pending].
 * pending == OPERATION_MODE_PENDING_NONE means nothing is deferred.  Sent to
 * every connection, not just the primary: Android yields primary to Mac Handy
 * under ConnectionPriority.MAC_HANDY and still needs to track the mode. */
static void send_operation_mode_status(void)
{
    uint8_t pkt[5] = {0x00, 0x55, EVT_OPERATION_MODE,
                      operation_mode, operation_mode_pending};

    notify_all_conns(pkt, sizeof(pkt));
}

/* True only when capture is both compiled in and switched on, so the ack never
 * claims a lean build is collecting. */
static bool gesture_capture_effective(void)
{
#if GESTURE_DEBUG_HISTORY
    return atomic_get(&debug_active) && gesture_capture_enabled;
#else
    return false;
#endif
}

/* Capture switch ack: [0x00][0x55][0x39][enabled].  All connections, for the
 * same reason as the operation mode ack. */
static void send_gesture_capture_status(void)
{
    uint8_t pkt[4] = {0x00, 0x55, EVT_GESTURE_CAPTURE,
                      gesture_capture_effective() ? 1 : 0};

    notify_all_conns(pkt, sizeof(pkt));
}

/* Gesture detect is independent of capture and of driving mode. */
static bool gesture_detect_effective(void)
{
    return gesture_detect_enabled;
}

/* Detect switch ack: [0x00][0x55][0x3A][enabled]. */
static void send_gesture_detect_status(void)
{
    uint8_t pkt[4] = {0x00, 0x55, EVT_GESTURE_DETECT,
                      gesture_detect_effective() ? 1 : 0};

    notify_all_conns(pkt, sizeof(pkt));
}

/* ---------------------------------------------------------------------------
 * Bounded control-plane notify
 *
 * bt_gatt_notify() allocates its ATT PDU with K_FOREVER: att.c gives a timeout
 * only to ATT_RESPONSE and ATT_CONFIRMATION, so a notification waits forever.
 * The buffers come from the att_pool shared by every notify in the system and
 * are released only once the packet reaches the controller, which needs an ACL
 * credit, which comes back only after the peer link-layer-ACKs.  A peer that
 * goes quiet therefore parks the caller indefinitely.
 *
 * The main loop is the only thing feeding the watchdog and it notifies from six
 * different places (motion, tap, greetings, tap diag, light-sleep events,
 * battery) plus the debug trajectory dumps, so it must never take that path.
 * Reserve a small slice of the pool for control traffic and drop instead of
 * blocking when the slice is exhausted.
 *
 * Budget: BT_ATT_TX_COUNT=12 = AUDIO_NOTIFY_IN_FLIGHT(6) + CTRL(4) + 2 spare
 * for the MCUmgr SMP transport during OTA.
 * --------------------------------------------------------------------------- */
#define CTRL_NOTIFY_IN_FLIGHT   4
#define CTRL_NOTIFY_WAIT_MS     20

K_SEM_DEFINE(ctrl_notify_slots, CTRL_NOTIFY_IN_FLIGHT, CTRL_NOTIFY_IN_FLIGHT);
static uint32_t ctrl_notify_dropped;

static void ctrl_notify_done(struct bt_conn *conn, void *user_data)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(user_data);
    k_sem_give(&ctrl_notify_slots);
}

/* Drop-in replacement for bt_gatt_notify() on the control characteristic.
 * Returns 0 on success, -EAGAIN when the slice was busy (packet dropped). */
static int notify_one(struct bt_conn *conn, const void *data, uint16_t len)
{
    struct bt_gatt_notify_params params;
    int err;

    if (!conn) {
        return -ENOTCONN;
    }
    if (k_sem_take(&ctrl_notify_slots, K_MSEC(CTRL_NOTIFY_WAIT_MS)) != 0) {
        ctrl_notify_dropped++;
        return -EAGAIN;
    }

    memset(&params, 0, sizeof(params));
    params.attr = &audio_svc.attrs[2];
    params.data = data;
    params.len  = len;
    params.func = ctrl_notify_done;

    err = bt_gatt_notify_cb(conn, &params);
    if (err) {
        /* func is not called when the request never got queued. */
        k_sem_give(&ctrl_notify_slots);
        if (err != -ENOTCONN && err != -EINVAL) {
            ctrl_notify_dropped++;
        }
    }
    return err;
}

/* Notify every active BLE client. Tap/diagnostic events must reach Android even
 * when Mac Handy holds the primary (audio) role. */
static void notify_all_conns(const void *data, uint16_t len)
{
    int n = 0;

    for (int i = 0; i < MAX_CONNS; i++) {
        if (!connections[i]) {
            continue;
        }
        if (notify_one(connections[i], data, len) == 0) {
            n++;
        }
    }
    if (n == 0) {
        APP_TRACE(">>> notify_all: no active conn\n");
    }
}

/* Send event packet via TX characteristic: [0x00][0x55][event_code] */
static void send_event_packet(uint8_t event_code)
{
    uint8_t pkt[3] = { 0x00, 0x55, event_code };

    /* Tap events are diagnostic / control UI signals for every host. */
    if (event_code == EVT_DOUBLE_TAP || event_code == EVT_SINGLE_TAP ||
        event_code == 0x20 || event_code == 0x21) {
        APP_TRACE(">>> send_event 0x%02x -> all conns (primary[%d])\n",
               event_code, primary_idx);
        notify_all_conns(pkt, sizeof(pkt));
        return;
    }

    if (!get_primary_conn()) {
        APP_TRACE(">>> send_event 0x%02x: no primary conn\n", event_code);
        return;
    }
    APP_TRACE(">>> send_event 0x%02x -> primary[%d]\n", event_code, primary_idx);
    notify_one(get_primary_conn(), pkt, sizeof(pkt));
}

/* Tap register snapshot: [0x00][0x55][0xD0][ret][CTRL1_XL][CTRL6_C][TAP_CFG]
 * [TAP_THS_6D][INT_DUR2][WAKE_UP_THS][MD1_CFG][INT1_CTRL][nonzero_lo][nonzero_hi]
 * [int1_level] */
static void send_tap_diag(void)
{
    static const uint8_t regs[] = {
        LSM6DS3TR_C_REG_CTRL1_XL, LSM6DS3TR_C_REG_CTRL6_C,
        LSM6DS3TR_C_REG_TAP_CFG,  LSM6DS3TR_C_REG_TAP_THS_6D,
        LSM6DS3TR_C_REG_INT_DUR2, LSM6DS3TR_C_REG_WAKE_UP_THS,
        LSM6DS3TR_C_REG_MD1_CFG,  LSM6DS3TR_C_REG_INT1_CTRL,
    };
    uint8_t pkt[15] = { 0x00, 0x55, EVT_TAP_DIAG,
                        (uint8_t)(int8_t)tap_last_read_ret };

    for (size_t i = 0; i < ARRAY_SIZE(regs); i++) {
        uint8_t val = 0xFF;

        if (i2c_reg_read_byte_dt(&imu_i2c, regs[i], &val) < 0) {
            val = 0xEE;
        }
        pkt[4 + i] = val;
    }
    pkt[12] = (uint8_t)(tap_nonzero_count & 0xFF);
    pkt[13] = (uint8_t)(tap_nonzero_count >> 8);
    pkt[14] = (uint8_t)(int8_t)gpio_pin_get_dt(&imu_int1);
    notify_all_conns(pkt, sizeof(pkt));
}

/* Register poke ack: [0x00][0x55][0xD2][reg][value_read_back][ret] */
static void send_tap_reg_ack(uint8_t reg, uint8_t value, int ret)
{
    uint8_t pkt[6] = { 0x00, 0x55, EVT_TAP_REG_ACK, reg, value,
                       (uint8_t)(int8_t)ret };

    APP_TRACE(">>> IMU reg 0x%02x = 0x%02x (ret=%d)\n", reg, value, ret);
    notify_all_conns(pkt, sizeof(pkt));
}

/* Deliver the per-connection greeting once that client has subscribed.  Polled
 * from the main loop; see conn_needs_greeting for why this cannot be event
 * driven. */
/* Boot report: [00][55][D3][cause u32 LE][crash_valid][stage]
 *              [prev_up_s u16 LE][hb u16 LE][ndrop u16 LE]
 * See EVT_BOOT_REPORT for how to read it. */
static void send_boot_report(struct bt_conn *conn)
{
    uint8_t pkt[15];

    pkt[0] = 0x00;
    pkt[1] = 0x55;
    pkt[2] = EVT_BOOT_REPORT;
    sys_put_le32(boot_reset_cause, &pkt[3]);
    pkt[7] = boot_crash_valid ? 1 : 0;
    pkt[8] = boot_crash_stage;
    sys_put_le16(boot_crash_up_s, &pkt[9]);
    sys_put_le16(boot_crash_hb, &pkt[11]);
    sys_put_le16(boot_crash_ndrop, &pkt[13]);

    notify_one(conn, pkt, sizeof(pkt));
    APP_TRACE(">>> boot_report cause=0x%08x crash=%u stage=%u(%s) prev_up=%us hb=%u ndrop=%u\n",
           boot_reset_cause, boot_crash_valid, boot_crash_stage,
           stage_str(boot_crash_stage), boot_crash_up_s, boot_crash_hb,
           boot_crash_ndrop);
}

static void send_pending_greetings(void)
{
    for (int i = 0; i < MAX_CONNS; i++) {
        if (!conn_needs_greeting[i] || !connections[i]) {
            continue;
        }
        if (!bt_gatt_is_subscribed(connections[i], &audio_svc.attrs[2],
                                   BT_GATT_CCC_NOTIFY)) {
            continue;
        }
        conn_needs_greeting[i] = false;

        /* First, so it is the anchor line in logcat for this connection:
         * why the firmware last restarted, and where it was when it died. */
        send_boot_report(connections[i]);

        uint8_t pkt[5] = {0x00, 0x55, EVT_OPERATION_MODE,
                          operation_mode, operation_mode_pending};
        notify_one(connections[i], pkt, sizeof(pkt));

        /* Capture and detect live in RAM and a reset clears them, so state the
         * truth here rather than leaving the host to assume last request. */
        uint8_t cap[4] = {0x00, 0x55, EVT_GESTURE_CAPTURE,
                          gesture_capture_effective() ? 1 : 0};
        notify_one(connections[i], cap, sizeof(cap));

        uint8_t det[4] = {0x00, 0x55, EVT_GESTURE_DETECT,
                          gesture_detect_effective() ? 1 : 0};
        notify_one(connections[i], det, sizeof(det));
        APP_TRACE(">>> Greeted conn[%d]: mode=%u capture=%u detect=%u\n", i,
               operation_mode, cap[3], det[3]);

        /* send_tap_diag() notifies every client; an extra copy for an already
         * connected peer is harmless and keeps the I2C reads in one place. */
        tap_diag_once_pending = atomic_get(&debug_active);
    }
}

/* motion_active/xyz packet: [0x00][0x55][code][f32_x][f32_y][f32_z] = 15 bytes */
static void send_event_packet_xyz(uint8_t event_code, float x, float y, float z)
{
    if (!get_primary_conn()) { return; }
    uint8_t pkt[15];
    pkt[0] = 0x00; pkt[1] = 0x55; pkt[2] = event_code;
    memcpy(&pkt[3],  &x, 4);
    memcpy(&pkt[7],  &y, 4);
    memcpy(&pkt[11], &z, 4);
    notify_one(get_primary_conn(), pkt, sizeof(pkt));
}

/* Gesture diagnostic packet:
 * [0x00][0x55][0x30][stage][reason][f32 value1][f32 value2][f32 value3]
 * = 17 bytes. Values depend on stage and are documented in
 * docs/flex_pronation_gesture.md. */
static void send_gesture_diag(uint8_t stage, uint8_t reason,
                              float value1, float value2, float value3)
{
    if (!atomic_get(&debug_active)) return;
#if GESTURE_DEBUG_HISTORY
    /* Always buffer milestones/rejects for end-of-session dump. */
    if (stage != GESTURE_DIAG_FINAL_SAMPLE) {
        gesture_history_push(stage, reason, value1, value2, value3);
    }
#endif
    if (!get_primary_conn()) { return; }
    uint8_t pkt[17];
    pkt[0] = 0x00;
    pkt[1] = 0x55;
    pkt[2] = 0x30;
    pkt[3] = stage;
    pkt[4] = reason;
    memcpy(&pkt[5], &value1, 4);
    memcpy(&pkt[9], &value2, 4);
    memcpy(&pkt[13], &value3, 4);
    notify_one(get_primary_conn(), pkt, sizeof(pkt));
}

#if GESTURE_DEBUG_HISTORY
static void flush_gesture_history(void)
{
    struct bt_conn *conn = get_primary_conn();
    uint8_t pkt[19];
    uint8_t i;
    uint8_t session;
    uint8_t count;

    if (!atomic_get(&debug_active) || !conn || gesture_history_count == 0) {
        gesture_history_flush_pending = false;
        return;
    }

    count = gesture_history_count;
    if (gesture_trajectory_session > gesture_history_session) {
        gesture_history_session = gesture_trajectory_session;
    } else {
        gesture_history_session++;
    }
    session = gesture_history_session;
    pkt[0] = 0x00;
    pkt[1] = 0x55;
    pkt[2] = EVT_GESTURE_HISTORY_BEGIN;
    pkt[3] = count;
    pkt[4] = session;
    notify_one(conn, pkt, 5);
    k_msleep(GESTURE_HISTORY_FLUSH_GAP_MS);

    for (i = 0; i < count && atomic_get(&debug_active); i++) {
        const gesture_history_entry_t *e = &gesture_history[i];
        /* [00 55 34][u16 t_ms LE][stage][reason][f32×3] = 19 bytes */
        pkt[0] = 0x00;
        pkt[1] = 0x55;
        pkt[2] = EVT_GESTURE_HISTORY_ENTRY;
        memcpy(&pkt[3], &e->t_ms, 2);
        pkt[5] = e->stage;
        pkt[6] = e->reason;
        memcpy(&pkt[7], &e->v1, 4);
        memcpy(&pkt[11], &e->v2, 4);
        memcpy(&pkt[15], &e->v3, 4);
        if (notify_one(conn, pkt, 19) == -EAGAIN) {
            /* Peer is backed up. Truncate rather than spend
             * CTRL_NOTIFY_WAIT_MS per remaining entry in the main loop. */
            APP_TRACE(">>> Gesture history truncated at %u/%u\n", i, count);
            break;
        }
        k_msleep(GESTURE_HISTORY_FLUSH_GAP_MS);
    }

    pkt[0] = 0x00;
    pkt[1] = 0x55;
    pkt[2] = EVT_GESTURE_HISTORY_END;
    pkt[3] = count;
    pkt[4] = session;
    if (atomic_get(&debug_active)) notify_one(conn, pkt, 5);

    gesture_history_count = 0;
    gesture_history_flush_pending = false;
    APP_TRACE(">>> Gesture history flushed session=%u count=%u\n",
           session, count);
}
#else
static void flush_gesture_history(void)
{
}
#endif

#if GESTURE_DEBUG_HISTORY
/* Held for the whole of flush_trajectory_batch(), which sleeps between chunks.
 *
 * Two threads reach a flush: audio_thread() on recording stop and the main
 * thread from process_motion_sample().  Both parked in the 5 ms gap, they sent
 * one 377-sample window as two interleaved sessions on 0.0.91.  Atomic because
 * the test and the set must not straddle a preemption. */
static atomic_t trajectory_flush_busy;

static bool flush_trajectory_batch(const gesture_trajectory_sample_t *samples,
                                   uint16_t sample_count, uint8_t result,
                                   uint8_t reason, bool overflow,
                                   const char *label)
{
    struct bt_conn *conn = get_primary_conn();
    uint8_t pkt[223];
    uint16_t start = 0;
    uint16_t sent = 0;
    uint8_t session;
    bool notify_error = false;

    if (!atomic_get(&debug_active) || sample_count == 0) {
        return true;
    }
    if (!atomic_cas(&trajectory_flush_busy, 0, 1)) {
        APP_TRACE(">>> %s skipped: a batch is already in flight\n", label);
        return false;
    }
    if (!conn) {
        APP_TRACE(">>> %s discarded: no primary connection\n", label);
        atomic_set(&trajectory_flush_busy, 0);
        return false;
    }

    session = ++gesture_trajectory_session;
    pkt[0] = 0x00;
    pkt[1] = 0x55;
    pkt[2] = EVT_GESTURE_TRAJECTORY_BEGIN;
    pkt[3] = GESTURE_TRAJECTORY_VERSION;
    pkt[4] = session;
    pkt[5] = result;
    pkt[6] = reason;
    memcpy(&pkt[7], &sample_count, 2);
    {
        /* Nominal only.  The IMU poll drops to SLEEP_POLL_INTERVAL_MS while
         * light sleep is active, so real spacing is 25 or 50 ms depending on
         * when each sample was taken.  Consumers must use each sample's t_ms
         * rather than assuming a uniform rate. */
        uint16_t period_ms = MOTION_SAMPLE_INTERVAL_MS;
        memcpy(&pkt[9], &period_ms, 2);
    }
    memcpy(&pkt[11], &gyro_bias_y_dps, 4);
    if (notify_one(conn, pkt, 15) < 0) {
        notify_error = true;
    }
    k_msleep(GESTURE_HISTORY_FLUSH_GAP_MS);

    while (start < sample_count && atomic_get(&debug_active)) {
        uint8_t count = (uint8_t)MIN(
            GESTURE_TRAJECTORY_CHUNK_SAMPLES,
            sample_count - start);
        size_t offset = 7;

        pkt[0] = 0x00;
        pkt[1] = 0x55;
        pkt[2] = EVT_GESTURE_TRAJECTORY_CHUNK;
        pkt[3] = session;
        memcpy(&pkt[4], &start, 2);
        pkt[6] = count;
        for (uint8_t i = 0; i < count; i++) {
            const gesture_trajectory_sample_t *sample = &samples[start + i];
            memcpy(&pkt[offset], &sample->t_ms, 2);
            pkt[offset + 2] = sample->flags;
            memcpy(&pkt[offset + 3], &sample->ax, 4);
            memcpy(&pkt[offset + 7], &sample->ay, 4);
            memcpy(&pkt[offset + 11], &sample->az, 4);
            memcpy(&pkt[offset + 15], &sample->gx, 4);
            memcpy(&pkt[offset + 19], &sample->gy, 4);
            memcpy(&pkt[offset + 23], &sample->gz, 4);
            offset += 27;
        }
        {
            int nerr = notify_one(conn, pkt, offset);

            if (nerr < 0) {
                notify_error = true;
                if (nerr == -EAGAIN) {
                    /* Peer backed up: truncate. END reports it via BIT(1). */
                    break;
                }
            } else {
                sent += count;
            }
        }
        start += count;
        k_msleep(GESTURE_HISTORY_FLUSH_GAP_MS);
    }

    pkt[0] = 0x00;
    pkt[1] = 0x55;
    pkt[2] = EVT_GESTURE_TRAJECTORY_END;
    pkt[3] = session;
    memcpy(&pkt[4], &sent, 2);
    pkt[6] = (overflow ? BIT(0) : 0) |
             (notify_error ? BIT(1) : 0);
    if (atomic_get(&debug_active)) (void)notify_one(conn, pkt, 7);
    atomic_set(&trajectory_flush_busy, 0);
    APP_TRACE(">>> %s flushed session=%u count=%u sent=%u flags=0x%02x\n",
           label, session, sample_count, sent, pkt[6]);
    return true;
}

static void flush_gesture_trajectory(void)
{
    if (!gesture_trajectory_flush_pending || gesture_trajectory_count == 0) {
        return;
    }
    /* Discard even when the batch was skipped as busy: being busy means this
     * same window is already on the air, so retrying would only duplicate it. */
    (void)flush_trajectory_batch(gesture_trajectory, gesture_trajectory_count,
                                 gesture_trajectory_result,
                                 gesture_trajectory_reason,
                                 gesture_trajectory_overflow,
                                 "Gesture trajectory");
    gesture_trajectory_discard();
}

static void flush_host_collection(void)
{
    if (!gesture_host_collection_flush_pending ||
        gesture_host_collection_count == 0) {
        return;
    }
    (void)flush_trajectory_batch(gesture_host_collection,
                                 gesture_host_collection_count,
                                 3, GESTURE_DIAG_REASON_NONE,
                                 gesture_host_collection_overflow,
                                 "Host IMU collection");
    gesture_host_collection_count = 0;
    gesture_host_collection_flush_pending = false;
    gesture_host_collection_overflow = false;
    if (gesture_phase == GESTURE_WAITING && !is_recording &&
        !recording_requested) {
        gyro_set_enabled(false);
    }
}
#else
static void flush_gesture_trajectory(void)
{
}
static void flush_host_collection(void)
{
}
#endif

/* motion_settled extended packet: [0x00][0x55][0x11][f32_x][f32_y][f32_z][u32_elapsed_ms]
 *   [f32_avg_speed][f32_peak_speed][f32_distance] = 31 bytes */
static void send_event_packet_settle(float x, float y, float z, uint32_t elapsed_ms,
                                     float avg_speed, float peak_speed, float distance)
{
    if (!get_primary_conn()) { return; }
    uint8_t pkt[31];
    pkt[0] = 0x00; pkt[1] = 0x55; pkt[2] = 0x11;
    memcpy(&pkt[3],  &x,          4);
    memcpy(&pkt[7],  &y,          4);
    memcpy(&pkt[11], &z,          4);
    memcpy(&pkt[15], &elapsed_ms, 4);
    memcpy(&pkt[19], &avg_speed,  4);
    memcpy(&pkt[23], &peak_speed, 4);
    memcpy(&pkt[27], &distance,   4);
    notify_one(get_primary_conn(), pkt, sizeof(pkt));
}

/* ============================================================================
 * Audio Streaming
 * ============================================================================ */

static void audio_stats_reset(void)
{
    memset(&audio_stats, 0, sizeof(audio_stats));
    audio_stats.session_start_ms = k_uptime_get();
}

static void audio_stats_print(const char *reason)
{
    int64_t now = k_uptime_get();
    int64_t session_ms = now - audio_stats.session_start_ms;
    int64_t dmic_ms = 0;

    if (audio_stats.dmic_stop_ms > 0) {
        dmic_ms = audio_stats.dmic_stop_ms - audio_stats.session_start_ms;
    } else if (session_ms > 0) {
        dmic_ms = session_ms;
    }

    if (!atomic_get(&debug_active)) return;
    APP_TRACE("Audio stats (%s): sess=%lldms dmic=%lldms cap=%u sent=%u bytes=%u "
           "ntfy=%u wait=%u wait_max=%ums retry=%u read_err=%u last_err=%d "
           "overrun=%u q_hi=%u\n",
           reason,
           (long long)session_ms,
           (long long)dmic_ms,
           audio_stats.frames_captured,
           audio_stats.frames_sent,
           audio_stats.bytes_sent,
           audio_stats.notifies_sent,
           audio_stats.notify_wait_count,
           audio_stats.notify_wait_max_ms,
           audio_stats.notify_retry_count,
           audio_stats.dmic_read_errors,
           (int)audio_stats.last_dmic_errno,
           audio_stats.dmic_overruns,
           audio_stats.queue_high_watermark);
}

static void audio_queue_purge(void)
{
    struct audio_pcm_frame drop;

    while (k_msgq_get(&audio_frame_q, &drop, K_NO_WAIT) == 0) {
    }
}

static void audio_queue_note_depth(void)
{
    uint32_t depth = (uint32_t)k_msgq_num_used_get(&audio_frame_q);

    if (depth > audio_stats.queue_high_watermark) {
        audio_stats.queue_high_watermark = depth;
    }
}

static void audio_notify_complete(struct bt_conn *conn, void *user_data)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(user_data);
    k_sem_give(&audio_notify_slots);
}

static int send_audio_notification_deadline(const uint8_t *data, uint16_t len,
                                            int64_t deadline_ms)
{
    while (k_uptime_get() < deadline_ms) {
        if (!get_primary_conn()) {
            return -ENOTCONN;
        }

        int64_t wait_start = k_uptime_get();
        int64_t remaining_ms = deadline_ms - wait_start;
        if (remaining_ms <= 0) {
            break;
        }
        if (remaining_ms > 100) {
            remaining_ms = 100;
        }

        if (k_sem_take(&audio_notify_slots, K_MSEC(remaining_ms)) != 0) {
            uint32_t waited = (uint32_t)(k_uptime_get() - wait_start);

            audio_stats.notify_wait_count++;
            if (waited > audio_stats.notify_wait_max_ms) {
                audio_stats.notify_wait_max_ms = waited;
            }
            continue;
        }

        uint32_t waited = (uint32_t)(k_uptime_get() - wait_start);
        if (waited > 0) {
            audio_stats.notify_wait_count++;
            if (waited > audio_stats.notify_wait_max_ms) {
                audio_stats.notify_wait_max_ms = waited;
            }
        }

        struct bt_conn *conn = get_primary_conn();
        if (!conn) {
            k_sem_give(&audio_notify_slots);
            return -ENOTCONN;
        }

        struct bt_gatt_notify_params params = {
            .attr = &audio_svc.attrs[2],
            .data = data,
            .len = len,
            .func = audio_notify_complete,
            .user_data = NULL,
        };
        int ret = bt_gatt_notify_cb(conn, &params);
        if (ret == 0) {
            audio_stats.notifies_sent++;
            return 0;
        }

        k_sem_give(&audio_notify_slots);
        if (ret != -ENOMEM && ret != -EAGAIN) {
            return ret;
        }
        audio_stats.notify_retry_count++;
        k_msleep(AUDIO_NOTIFY_RETRY_MS);
    }

    return -ECANCELED;
}

static int send_audio_notification(const uint8_t *data, uint16_t len)
{
    while (is_recording && !stop_requested) {
        int ret = send_audio_notification_deadline(
            data, len, k_uptime_get() + 100);
        if (ret != -ECANCELED) {
            return ret;
        }
        /* Still recording: keep trying through transient slot starvation. */
    }

    return -ECANCELED;
}

static bool drain_audio_notifications(void)
{
    int acquired = 0;
    int64_t deadline = k_uptime_get() + AUDIO_NOTIFY_DRAIN_TIMEOUT_MS;

    while (acquired < AUDIO_NOTIFY_IN_FLIGHT) {
        int64_t remaining_ms = deadline - k_uptime_get();
        if (remaining_ms <= 0 ||
            k_sem_take(&audio_notify_slots, K_MSEC(remaining_ms)) != 0) {
            break;
        }
        acquired++;
    }

    for (int i = 0; i < acquired; i++) {
        k_sem_give(&audio_notify_slots);
    }

    if (acquired != AUDIO_NOTIFY_IN_FLIGHT) {
        APP_TRACE("Audio notify drain timed out: %d/%d slots returned\n",
               acquired, AUDIO_NOTIFY_IN_FLIGHT);
        return false;
    }
    return true;
}

static int stream_audio_frame_deadline(const uint8_t *pcm, size_t audio_size,
                                       int64_t deadline_ms, bool use_deadline)
{
    size_t total_samples = audio_size / sizeof(int16_t);
    size_t offset = 0;

    while (offset < total_samples) {
        size_t samples_to_send = total_samples - offset;
        if (samples_to_send > PCM_PACKET_SIZE / sizeof(int16_t)) {
            samples_to_send = PCM_PACKET_SIZE / sizeof(int16_t);
        }

        tx_packet[0] = seq_num;
        tx_packet[1] = 0xAA;
        memcpy(&tx_packet[2], &pcm[offset * sizeof(int16_t)],
               samples_to_send * sizeof(int16_t));

        int ret;
        if (use_deadline) {
            ret = send_audio_notification_deadline(
                tx_packet, 2 + (samples_to_send * sizeof(int16_t)), deadline_ms);
        } else {
            ret = send_audio_notification(
                tx_packet, 2 + (samples_to_send * sizeof(int16_t)));
        }
        if (ret != 0) {
            return ret;
        }

        seq_num++;
        offset += samples_to_send;
        audio_stats.bytes_sent += samples_to_send * sizeof(int16_t);
    }

    audio_stats.frames_sent++;
    return 0;
}

static int stream_audio_frame(const uint8_t *pcm, size_t audio_size)
{
    return stream_audio_frame_deadline(pcm, audio_size, 0, false);
}

static int stream_queued_frames(bool blocking)
{
    struct audio_pcm_frame frame;
    k_timeout_t timeout = blocking ? K_MSEC(20) : K_NO_WAIT;

    while (is_recording && !stop_requested && get_primary_conn()) {
        int get_ret = k_msgq_get(&audio_frame_q, &frame, timeout);
        if (get_ret != 0) {
            return 0;
        }

        int ret = stream_audio_frame(frame.pcm, frame.nbytes);
        if (ret < 0 && ret != -ECANCELED && ret != -ENOTCONN) {
            APP_TRACE("Audio notify failed: %d\n", ret);
            k_msleep(AUDIO_NOTIFY_RETRY_MS);
        } else if (ret == -ENOTCONN || ret == -ECANCELED) {
            return ret;
        }

        /* After the first frame, prefer draining without sleeping. */
        timeout = K_NO_WAIT;
    }

    return 0;
}

static void drain_queued_frames_before_stop(void)
{
    struct audio_pcm_frame frame;
    int64_t deadline = k_uptime_get() + AUDIO_QUEUE_DRAIN_TIMEOUT_MS;

    while (k_uptime_get() < deadline) {
        if (k_msgq_get(&audio_frame_q, &frame, K_MSEC(10)) != 0) {
            if (k_msgq_num_used_get(&audio_frame_q) == 0) {
                break;
            }
            continue;
        }
        if (!get_primary_conn()) {
            break;
        }
        if (stream_audio_frame_deadline(frame.pcm, frame.nbytes,
                                        deadline, true) != 0) {
            break;
        }
    }

    audio_queue_purge();
}

/*
 * Dedicated capture path: always free DMIC blocks promptly by copying into the
 * software queue. BLE Notify backpressure must never stall dmic_read().
 */
static void audio_capture_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int consec_fail = 0;

    APP_TRACE("Audio capture thread started\n");

    while (1) {
        if (!capture_enabled) {
            consec_fail = 0;
            k_sem_take(&audio_capture_wake, K_FOREVER);
            continue;
        }

        int16_t *audio_buffer = NULL;
        size_t audio_size = 0;
        int ret = audio_capture_get_data(&audio_buffer, &audio_size);
        if (ret < 0) {
            audio_stats.dmic_read_errors++;
            audio_stats.last_dmic_errno = (uint32_t)(-ret);
            consec_fail++;

            /*
             * Transient stalls are expected under radio contention. Only mark
             * the capture path fatal after repeated failures; the sender keeps
             * draining whatever was already queued.
             */
            if (consec_fail >= AUDIO_CAPTURE_MAX_CONSEC_FAIL) {
                LOG_WRN("Audio read failed: %d (giving up after %d errors)\n",
                       ret, consec_fail);
                if (audio_stats.dmic_stop_ms == 0) {
                    audio_stats.dmic_stop_ms = k_uptime_get();
                }
                capture_enabled = false;
                (void)audio_capture_stop();
                capture_fatal = true;
                k_sem_give(&audio_control_wake);
                consec_fail = 0;
            }
            continue;
        }

        consec_fail = 0;
        if (audio_buffer == NULL || audio_size == 0) {
            continue;
        }

        struct audio_pcm_frame frame;
        size_t copy_len = audio_size;

        if (copy_len > AUDIO_FRAME_BYTES) {
            copy_len = AUDIO_FRAME_BYTES;
        }
        frame.nbytes = (uint16_t)copy_len;
        memcpy(frame.pcm, audio_buffer, copy_len);
        audio_stats.frames_captured++;

        if (k_msgq_put(&audio_frame_q, &frame, K_NO_WAIT) != 0) {
            audio_stats.dmic_overruns++;
        } else {
            audio_queue_note_depth();
        }
    }
}

static void audio_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int ret;
    bool dmic_available = false;
    bool dmic_session_active = false;

    APP_TRACE("Audio thread started\n");

    ret = audio_capture_init();
    if (ret < 0) {
        LOG_WRN("Audio init failed: %d — audio disabled\n", ret);
    } else {
        dmic_available = true;
        APP_TRACE("Audio capture initialized\n");
    }

    while (1) {
        if (recording_requested && !is_recording) {
            recording_requested = false;
            stop_requested = false;
            capture_fatal = false;
            seq_num = 0;
            audio_stats_reset();
            audio_queue_purge();

            if (dmic_available) {
                mic_power_on();
                ret = audio_capture_start();
                if (ret < 0) {
                    LOG_WRN("Audio start failed: %d\n", ret);
                    mic_power_off();
                    dmic_session_active = false;
                    capture_enabled = false;
                } else {
                    dmic_session_active = true;
                    capture_enabled = true;
                    k_sem_give(&audio_capture_wake);
                    APP_TRACE("Audio capture running\n");
                }
            }

            is_recording = true;
            gyro_set_enabled(true);
            sync_primary_conn_speed();
            APP_TRACE("Recording started\n");
            send_event_packet(0x01);
        }

        if ((stop_requested || capture_fatal) && is_recording) {
            bool was_fatal = capture_fatal;

            stop_requested = false;
            capture_fatal = false;
            capture_enabled = false;

            if (dmic_session_active) {
                if (audio_stats.dmic_stop_ms == 0) {
                    audio_stats.dmic_stop_ms = k_uptime_get();
                }
                ret = audio_capture_stop();
                if (ret < 0) {
                    LOG_WRN("Audio stop failed: %d\n", ret);
                }
                mic_power_off();
                dmic_session_active = false;
            }

            /* Flush any frames captured before stop under light contention. */
            drain_queued_frames_before_stop();
            is_recording = false;
            reset_recording_stop_state();
            gyro_set_enabled(false);
#if GESTURE_DEBUG_HISTORY
            if (gesture_trajectory_active && gesture_trajectory_committed) {
                gesture_trajectory_finish(1, GESTURE_DIAG_REASON_NONE);
            }
#endif

            /* Preserve ordering: all accepted PCM notifications must complete before stop. */
            (void)drain_audio_notifications();
            APP_TRACE("Recording stopped%s\n", was_fatal ? " (capture fault)" : "");
            audio_stats_print(was_fatal ? "fault" : "stop");
            send_event_packet(0x02);
            flush_host_collection();
            flush_gesture_trajectory();
            flush_gesture_history();
            battery_update();
            last_activity_ms = k_uptime_get();   /* reset idle timer after recording */
            last_recording_stop_ms = last_activity_ms;
            sync_primary_conn_speed();
            continue;
        }

        if (is_recording && get_primary_conn()) {
            (void)stream_queued_frames(true);
            continue;
        }

        if (!get_primary_conn() && dmic_session_active) {
            capture_enabled = false;
            if (audio_stats.dmic_stop_ms == 0) {
                audio_stats.dmic_stop_ms = k_uptime_get();
            }
            (void)audio_capture_stop();
            mic_power_off();
            dmic_session_active = false;
            audio_queue_purge();
            if (is_recording) {
                is_recording = false;
                reset_recording_stop_state();
                gyro_set_enabled(false);
                (void)drain_audio_notifications();
                APP_TRACE("Recording stopped (disconnected)\n");
                audio_stats_print("disconnect");
                send_event_packet(0x02);
                flush_host_collection();
                flush_gesture_trajectory();
                flush_gesture_history();
            }
        }

        k_sem_take(&audio_control_wake, K_FOREVER);
    }
}

/* ============================================================================
 * BLE Connection Callbacks
 * ============================================================================ */

static void exchange_func(struct bt_conn *conn, uint8_t att_err,
                          struct bt_gatt_exchange_params *params)
{
    if (att_err) {
        LOG_WRN(">>> MTU exchange failed: %d\n", att_err);
    } else {
        APP_TRACE(">>> MTU exchange done\n");
    }
}

static struct bt_gatt_exchange_params exchange_params[MAX_CONNS];

/* ============================================================================
 * Deferred advertising restart
 * ============================================================================ */

/* Fast tier == the plain BT_LE_ADV_CONN interval (100-150 ms): what an
 * undiscovered node should do.  Slow tier keeps the second slot reachable for
 * the Mac client without burning 100-150 ms of radio forever while one peer is
 * already connected. */
static const struct bt_le_adv_param adv_param_fast = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONNECTABLE,
        BT_GAP_ADV_FAST_INT_MIN_2,   /* 100 ms */
        BT_GAP_ADV_FAST_INT_MAX_2,   /* 150 ms */
        NULL);
static const struct bt_le_adv_param adv_param_slow = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONNECTABLE,
        BT_GAP_ADV_SLOW_INT_MIN,     /* 1 s   */
        BT_GAP_ADV_SLOW_INT_MAX,     /* 1.2 s */
        NULL);

typedef enum {
    ADV_TIER_UNKNOWN = 0,
    ADV_TIER_OFF,
    ADV_TIER_SLOW,
    ADV_TIER_FAST,
} adv_tier_t;

static adv_tier_t adv_tier;
static int64_t adv_conn_change_ms;   /* last time active_count() changed */

static void adv_note_conn_change(void)
{
    adv_conn_change_ms = k_uptime_get();
}

static void adv_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    int n = active_count();
    adv_tier_t want;
    const struct bt_le_adv_param *param;
    const char *label;
    int ret;

    if (n >= MAX_CONNS) {
        want = ADV_TIER_OFF;
    } else if (n > 0 &&
               (k_uptime_get() - adv_conn_change_ms) >= ADV_FAST_BURST_MS) {
        want = ADV_TIER_SLOW;
    } else {
        want = ADV_TIER_FAST;
    }

    if (want == adv_tier) {
        return;
    }

    /* BT_LE_ADV_CONN has no BT_LE_ADV_OPT_ONE_TIME, so BT_ADV_PERSIST is set and
     * the host already resumed advertising from inside the LE Connection
     * Complete handler.  bt_le_adv_start() would then return -EALREADY WITHOUT
     * reprogramming the interval, so the tier must be stopped first. */
    (void)bt_le_adv_stop();

    if (want == ADV_TIER_OFF) {
        adv_tier = want;
        APP_TRACE(">>> Adv off (conns=%d)\n", n);
        return;
    }

    if (want == ADV_TIER_SLOW) {
        param = &adv_param_slow;
        label = "slow(1000-1200ms)";
    } else {
        param = &adv_param_fast;
        label = "fast(100-150ms)";
    }

    ret = bt_le_adv_start(param, adv_data, ARRAY_SIZE(adv_data),
                          scan_rsp, ARRAY_SIZE(scan_rsp));
    if (ret) {
        LOG_WRN("Advertising start failed: %d", ret);
        LOG_WRN(">>> Adv start failed: %d\n", ret);
        adv_tier = ADV_TIER_UNKNOWN;
        k_work_schedule(&adv_work, K_MSEC(500));
        return;
    }
    adv_tier = want;
    APP_TRACE(">>> Adv tier=%s conns=%d\n", label, n);
}

/* ============================================================================
 * Serial command thread — reads from USB CDC ACM UART
 *   'r' → recording start
 *   's' → recording stop
 *   'b' → reboot into the UF2 bootloader (same path as RX 0x60)
 * ============================================================================ */

static void serial_rx_ready(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        uint8_t bytes[16];
        int count = uart_fifo_read(dev, bytes, sizeof(bytes));
        if (count <= 0) break;
        for (int i = 0; i < count; i++) {
            (void)k_msgq_put(&serial_rx_q, &bytes[i], K_NO_WAIT);
        }
    }
}

static void serial_cmd_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(uart_dev)) {
        APP_TRACE(">>> Serial: console UART not ready\n");
        return;
    }

    if (uart_irq_callback_user_data_set(uart_dev, serial_rx_ready, NULL)) {
        LOG_ERR("Serial RX interrupt setup failed");
        return;
    }
    uart_irq_rx_enable(uart_dev);
    APP_TRACE(">>> Serial command thread ready ('r'=start 's'=stop 'b'=bootloader)\n");

    while (true) {
        unsigned char ch;
        if (!k_msgq_get(&serial_rx_q, &ch, K_FOREVER)) {
            if (ch == 'r' || ch == 'R') {
                APP_TRACE(">>> SERIAL: recording start\n");
                stop_requested = false;
                recording_requested = true;
                k_sem_give(&audio_control_wake);
            } else if (ch == 's' || ch == 'S') {
                APP_TRACE(">>> SERIAL: recording stop\n");
                stop_requested = true;
                k_sem_give(&audio_control_wake);
            } else if (ch == 'b' || ch == 'B') {
                if (is_recording || recording_requested) {
                    APP_TRACE(">>> SERIAL: bootloader REFUSED (recording)\n");
                } else {
                    APP_TRACE(">>> SERIAL: reboot to bootloader\n");
                    /* Via the work item rather than inline so both entry points
                     * take the same path, and so this thread's printk can drain
                     * before the reset. */
                    k_work_schedule(&reboot_work, K_MSEC(500));
                }
            }
        }
    }
}

/* Local table rather than CONFIG_BT_HCI_ERR_TO_STR (EXPERIMENTAL, and it pulls
 * in the full error table).  These are the reasons that actually show up on a
 * peripheral link. */
static const char *hci_reason_str(uint8_t reason)
{
    switch (reason) {
    case 0x08: return "CONN_TIMEOUT(supervision)";
    case 0x13: return "REMOTE_USER_TERM";
    case 0x15: return "REMOTE_POWER_OFF";
    case 0x16: return "LOCAL_HOST_TERM";
    case 0x22: return "LL_RESPONSE_TIMEOUT";
    case 0x28: return "INSTANT_PASSED";
    case 0x2A: return "DIFF_TRANSACTION_COLLISION";
    case 0x3B: return "UNACCEPTABLE_CONN_PARAM";
    case 0x3E: return "CONN_FAIL_TO_ESTABLISH";
    default:   return "?";
    }
}

static void ble_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) { LOG_ERR("BLE connection failed: %d", err); return; }

    APP_INFO("BLE connected");
    APP_TRACE(">>> Connected!\n");

    int slot = free_slot();
    if (slot < 0) {
        LOG_WRN("No free connection slot, rejecting");
        bt_conn_disconnect(conn, BT_HCI_ERR_CONN_LIMIT_EXCEEDED);
        return;
    }

    connections[slot] = bt_conn_ref(conn);
    conn_started_ms[slot] = k_uptime_get();
    conn_param_seen[slot] = false;
    had_conn_this_cycle = true;
    adv_note_conn_change();

    if (primary_idx < 0) {
        primary_idx = slot;
        APP_TRACE(">>> conn[%d] is primary\n", slot);
    } else {
        APP_TRACE(">>> conn[%d] is secondary, notifying primary\n", slot);
        uint8_t pkt[3] = { 0x00, 0x55, 0x31 };
        notify_one(get_primary_conn(), pkt, sizeof(pkt));
    }

    /* Notifying here would reach nobody: the client subscribes about a second
     * from now. The main loop sends the greeting once it has -- see
     * send_pending_greetings(). */
    conn_needs_greeting[slot] = true;

    led_sync_runtime_state();

    exchange_params[slot].func = exchange_func;
    int ret = bt_gatt_exchange_mtu(conn, &exchange_params[slot]);
    if (ret) {
        LOG_WRN(">>> MTU exchange request failed: %d\n", ret);
    }

    /* Idle primary stays at 30–50 ms until recording needs the fast
     * audio interval.  Do not request params from this callback. */
    k_work_schedule(&conn_param_work, K_MSEC(200));

    if (active_count() < MAX_CONNS) {
        /* Schedule advertising restart via work queue — calling bt_le_adv_start()
         * directly from within a BT callback can fail on Nordic NCS. */
        k_work_schedule(&adv_work, K_MSEC(100));
        APP_TRACE(">>> Advertising restart scheduled\n");
    }
}

static void ble_disconnected(struct bt_conn *conn, uint8_t reason)
{
    int64_t now = k_uptime_get();
    int idx = conn_index(conn);

    APP_INFO("BLE disconnected: %d", reason);

    if (idx < 0) {
        APP_TRACE(">>> DISCONN (unknown slot) reason=0x%02x %s\n",
               reason, hci_reason_str(reason));
        return;
    }

    /* Everything needed to tell "the peer/link went away" from "we reset"
     * apart, without a sniffer:
     *   reason  - who dropped it and why
     *   age     - how long the link lived
     *   params  - what the link was actually running at, or never-updated,
     *             which would mean the peer ignored every parameter request
     *   ndrop   - control notifies dropped because the peer stopped ACKing */
    APP_TRACE(">>> DISCONN idx=%d reason=0x%02x %s up=%llds age=%llds "
           "params=%u/%u/%u(%s) since_rec=%llds ndrop=%u\n",
           idx, reason, hci_reason_str(reason),
           now / 1000, (now - conn_started_ms[idx]) / 1000,
           conn_param_interval[idx], conn_param_latency[idx],
           conn_param_timeout[idx],
           conn_param_seen[idx] ? "negotiated" : "never-updated",
           last_recording_stop_ms ? (now - last_recording_stop_ms) / 1000 : -1,
           ctrl_notify_dropped);

    bt_conn_unref(connections[idx]);
    connections[idx] = NULL;
    conn_needs_greeting[idx] = false;
    conn_param_seen[idx] = false;
    adv_note_conn_change();

    if (primary_idx == idx) {
        primary_idx = -1;
        for (int i = 0; i < MAX_CONNS; i++) {
            if (connections[i]) {
                primary_idx = i;
                APP_TRACE(">>> conn[%d] promoted to primary\n", i);
                break;
            }
        }
        if (primary_idx < 0) {
            is_recording = false;
            stop_requested = true;
            k_sem_give(&audio_control_wake);
            reset_recording_stop_state();
            gyro_set_enabled(false);
            /* Latch is global, not per-conn: clear it so the next connection
             * re-requests its parameters instead of assuming the old state. */
            ble_primary_want_fast = false;
        }
    } else if (primary_idx >= 0) {
        APP_TRACE(">>> Secondary disconnected, notifying primary\n");
        uint8_t pkt[3] = { 0x00, 0x55, 0x32 };
        notify_one(get_primary_conn(), pkt, sizeof(pkt));
    }

    led_sync_runtime_state();

    /* Schedule advertising restart via work queue */
    k_work_schedule(&adv_work, K_MSEC(100));
    APP_TRACE(">>> Advertising restart scheduled\n");
}

static void ble_le_param_updated(struct bt_conn *conn, uint16_t interval,
                                 uint16_t latency, uint16_t timeout)
{
    int idx = conn_index(conn);

    if (idx >= 0) {
        conn_param_interval[idx] = interval;
        conn_param_latency[idx]  = latency;
        conn_param_timeout[idx]  = timeout;
        conn_param_seen[idx]     = true;
    }

    APP_TRACE(">>> le_param_updated[%d]: interval=%u (%u.%02u ms) latency=%u timeout=%u (%u ms)\n",
           idx, interval, (interval * 125U) / 100U, (interval * 125U) % 100U,
           latency, timeout, timeout * 10U);
}

static struct bt_conn_cb conn_callbacks = {
    .connected        = ble_connected,
    .disconnected     = ble_disconnected,
    .le_param_updated = ble_le_param_updated,
};

/* How long to stay unconnected before SYSTEMOFF.
 * A peer that was connected and vanished is usually coming back (Doze, out of
 * range, app restart), so give it room; a node nobody ever connected to, or one
 * whose battery is nearly flat, sleeps on the short fuse. */
static uint32_t systemoff_fuse_ms(void)
{
    if (bat_last_pct < ADV_LOW_BATT_PCT) {
        return ADV_NO_CONN_SLEEP_MS;
    }
    return had_conn_this_cycle ? ADV_AFTER_DROP_SLEEP_MS : ADV_NO_CONN_SLEEP_MS;
}

static void enter_systemoff_for_tap_wake(void)
{
    uint8_t tap_src = 0;
    int ret;

    APP_TRACE(">>> SYSTEMOFF: %u min unconnected (had_conn=%d), tap to wake\n",
           (unsigned)(systemoff_fuse_ms() / 60000U), (int)had_conn_this_cycle);
    APP_INFO("Entering SYSTEMOFF; tap to resume advertising");

    rgb_set(false, false, false);
    adv_tier = ADV_TIER_UNKNOWN;

    if (wdt_channel_id >= 0) {
        wdt_feed(wdt, wdt_channel_id);
    }

    (void)bt_le_adv_stop();

    if (!device_is_ready(imu) || !i2c_is_ready_dt(&imu_i2c)) {
        LOG_WRN(">>> SYSTEMOFF aborted: IMU not ready\n");
        adv_tier = ADV_TIER_UNKNOWN;
        k_work_schedule(&adv_work, K_MSEC(100));
        unconnected_since_ms = k_uptime_get();
        return;
    }

    ret = configure_tap_wake();
    if (ret < 0) {
        LOG_WRN(">>> SYSTEMOFF aborted: MD1_CFG %d\n", ret);
        goto restore_adv;
    }

    ret = i2c_reg_read_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_TAP_SRC, &tap_src);
    if (ret < 0) {
        LOG_WRN(">>> SYSTEMOFF aborted: TAP_SRC %d\n", ret);
        goto restore_adv;
    }

    k_msleep(10);
    if (gpio_pin_get_dt(&imu_int1) > 0) {
        (void)i2c_reg_read_byte_dt(&imu_i2c, LSM6DS3TR_C_REG_TAP_SRC, &tap_src);
        k_msleep(10);
    }
    if (gpio_pin_get_dt(&imu_int1) > 0) {
        LOG_WRN(">>> SYSTEMOFF aborted: INT1 high (TAP_SRC=0x%02x)\n", tap_src);
        goto restore_adv;
    }

    ret = gpio_pin_interrupt_configure_dt(&imu_int1, GPIO_INT_LEVEL_ACTIVE);
    if (ret < 0) {
        LOG_WRN(">>> SYSTEMOFF aborted: INT1 sense %d\n", ret);
        goto restore_adv;
    }

    if (wdt_channel_id >= 0) {
        wdt_feed(wdt, wdt_channel_id);
    }

    sys_poweroff();

restore_adv:
    (void)configure_tap_detection();
    adv_tier = ADV_TIER_UNKNOWN;
    k_work_schedule(&adv_work, K_MSEC(100));
    unconnected_since_ms = k_uptime_get();
}

/* Hand the device over to the Adafruit UF2 bootloader, which reads GPREGRET at
 * boot and stays in mass-storage mode when it finds the magic.
 *
 * Requires USB to be connected: with no VBUS the bootloader does not stay in DFU
 * mode at all and falls straight through to the app.
 *
 * On the watchdog: the nRF52840 WDT has no STOP task, so it cannot be turned
 * off, and a soft reset does not clear it -- only a pin reset, POR, or SYSTEMOFF
 * wake does. The bootloader therefore inherits a running watchdog, which the
 * manual double-tap route (a pin reset) never had to deal with. Measured on
 * bootloader 0.6.2-12-g459adc9: it feeds the dog itself and the volume stays up
 * indefinitely, which matters because a 1.44 MB write can take 16 s against a
 * 12 s WDT_TIMEOUT_MS. Feed it here anyway so the handover starts from a full
 * window rather than a partly spent one. */
static void reboot_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    APP_TRACE(">>> Rebooting into UF2 bootloader (GPREGRET=0x%02x)\n",
           BOOTLOADER_MAGIC_UF2);

    if (wdt_channel_id >= 0) {
        wdt_feed(wdt, wdt_channel_id);
    }
    (void)bt_le_adv_stop();

    nrf_power_gpregret_set(NRF_POWER, 0, BOOTLOADER_MAGIC_UF2);
    sys_reboot(SYS_REBOOT_COLD);
}

/* ============================================================================
 * Main
 * ============================================================================ */

int main(void)
{
    int ret;

    usb_enable(NULL);
    k_msleep(500);

    APP_TRACE("\n");
    APP_TRACE("============================================\n");
    APP_TRACE("nordic-main: XIAO nRF52840 Sense\n");
    APP_TRACE("Build: %s %s\n", __DATE__, __TIME__);
    APP_TRACE("============================================\n\n");

    log_reset_cause();

    /* Arm the watchdog FIRST, before anything that can hang.
     * It used to be armed after boot_write_img_confirmed(), which meant a bad
     * OTA image that hung during init had no dog and sat there forever -- the
     * only way back was a physical reset. Armed here, such a hang resets within
     * WDT_TIMEOUT_MS and MCUboot reverts the unconfirmed image by itself.
     * Every wait in the boot path below must therefore feed it. */
    configure_watchdog();

    APP_INFO("Starting nordic-main");

    ret = configure_leds();
    if (ret < 0) LOG_WRN("LED setup failed: %d", ret);

    led_set_state(LED_BOOT);  /* White 1 second */

    ret = configure_mic_power();
    if (ret < 0) LOG_WRN("Mic power GPIO setup failed: %d", ret);

    ret = battery_init();
    if (ret < 0) LOG_WRN("Battery monitor init failed: %d", ret);

    if (!device_is_ready(imu)) {
        LOG_WRN("IMU not ready — gesture detection disabled");
    } else {
        ret = configure_motion_detection();
        if (ret < 0) LOG_WRN("Motion detection setup failed: %d", ret);
        else {
            ret = configure_tap_detection();
            if (ret < 0) LOG_WRN("Tap detection disabled: %d", ret);
        }
    }

    k_work_init_delayable(&adv_work, adv_work_handler);
    k_work_init_delayable(&conn_param_work, conn_param_work_handler);
    k_work_init_delayable(&reboot_work, reboot_work_handler);

    ret = bt_enable(NULL);
    if (ret) { LOG_ERR("BLE enable failed: %d", ret); return ret; }

    bt_conn_cb_register(&conn_callbacks);

    ret = bt_set_name(BLE_DEVICE_NAME);
    if (ret) { LOG_ERR("BLE name set failed: %d", ret); return ret; }

    ret = bt_le_adv_start(&adv_param_fast, adv_data, ARRAY_SIZE(adv_data),
                          scan_rsp, ARRAY_SIZE(scan_rsp));
    if (ret) { LOG_ERR("BLE adv failed: %d", ret); return ret; }
    adv_tier = ADV_TIER_FAST;
    adv_note_conn_change();

    APP_INFO("BLE advertising: %s", BLE_DEVICE_NAME);

    /* Capture runs above sender so DMIC blocks are freed under BLE backpressure. */
    k_thread_create(&audio_capture_thread_data, audio_capture_stack,
                    K_THREAD_STACK_SIZEOF(audio_capture_stack),
                    audio_capture_thread, NULL, NULL, NULL, 13, 0, K_NO_WAIT);
    /* Sender/control thread (priority 14, below BLE stack) */
    k_thread_create(&audio_thread_data, audio_stack, K_THREAD_STACK_SIZEOF(audio_stack),
                    audio_thread, NULL, NULL, NULL, 14, 0, K_NO_WAIT);

    /* Start serial command thread (priority 10) */
    k_thread_create(&serial_thread_data, serial_stack, K_THREAD_STACK_SIZEOF(serial_stack),
                    serial_cmd_thread, NULL, NULL, NULL, 10, 0, K_NO_WAIT);

    APP_INFO("nordic-main ready");

    /* Boot LED: white for ~1 second, then advertising blink or idle */
    {
        int64_t boot_led_until = k_uptime_get() + 1000;

        while (k_uptime_get() < boot_led_until) {
            if (wdt_channel_id >= 0) {
                wdt_feed(wdt, wdt_channel_id);
            }
            k_msleep(MAIN_LOOP_INTERVAL_MS);
        }
    }
    led_set_state(LED_IDLE);
    led_sync_runtime_state();

    /* Auto-confirm OTA image after 3s total — basic sanity that we didn't crash */
    {
        int64_t confirm_until = k_uptime_get() + 2000;

        while (k_uptime_get() < confirm_until) {
            if (wdt_channel_id >= 0) {
                wdt_feed(wdt, wdt_channel_id);
            }
            led_tick();
            k_msleep(MAIN_LOOP_INTERVAL_MS);
        }
    }
    if (!boot_is_img_confirmed()) {
        int img_ret = boot_write_img_confirmed();
        if (img_ret < 0) {
            LOG_WRN("Image confirm failed: %d", img_ret);
        } else {
            APP_INFO("Image confirmed");
            APP_TRACE("Image confirmed (permanent)\n");
        }
    }

    /* Initial battery reading after BLE is ready */
    battery_update();

    /* Main loop: watchdog feed + LED state management + blink tick + IMU motion */
    int64_t last_motion_sample_ms = k_uptime_get();
    int64_t last_battery_ms = k_uptime_get();
    int64_t last_heartbeat_ms = 0;
    int64_t last_pass_ms = 0;
    uint32_t hb_seq = 0;
    last_activity_ms = k_uptime_get();
    while (1) {
        k_msleep(MAIN_LOOP_INTERVAL_MS);

        if (wdt_channel_id >= 0) {
            wdt_feed(wdt, wdt_channel_id);
        }

        led_sync_runtime_state();

        int64_t now_ms = k_uptime_get();

        /* A pass should take ~MAIN_LOOP_INTERVAL_MS. Anything much longer means
         * something in the body blocked, which is the failure this loop must
         * never hit: it is the only thing feeding the watchdog. */
        if (last_pass_ms &&
            (now_ms - last_pass_ms) > MAIN_LOOP_STALL_WARN_MS) {
            LOG_WRN(">>> STALL %lld ms at stage=%u(%s) conns=%d ndrop=%u\n",
                   now_ms - last_pass_ms, cb.stage, stage_str(cb.stage),
                   active_count(), ctrl_notify_dropped);
        }
        last_pass_ms = now_ms;
        cb.up_s = (uint32_t)(now_ms / 1000);
        cb.conns = (uint8_t)active_count();
        cb.recording = is_recording ? 1 : 0;
        cb.ndrop = ctrl_notify_dropped;
        loop_stage(STAGE_IDLE);

        process_tap_debug_command();
        apply_pending_operation_mode();

        /* IMU motion detection — poll faster when active, slower when sleeping */
        int64_t imu_poll_ms = light_sleep_active
                              ? SLEEP_POLL_INTERVAL_MS
                              : MOTION_SAMPLE_INTERVAL_MS;
        if (device_is_ready(imu) &&
            (now_ms - last_motion_sample_ms) >= imu_poll_ms) {
            last_motion_sample_ms = now_ms;
            loop_stage(STAGE_MOTION);
            process_motion_sample();
        }
        loop_stage(STAGE_TAP);
        process_tap_event(now_ms);
        process_tap_stats_request();

        loop_stage(STAGE_GREET);
        send_pending_greetings();

        if (tap_diag_once_pending) {
            tap_diag_once_pending = false;
            loop_stage(STAGE_TAPDIAG);
            if (atomic_get(&debug_active)) send_tap_diag();
        }
        loop_stage(STAGE_IDLE);

        led_tick();

        /* Light sleep management */
        if (!motion_active && !is_recording &&
            gesture_phase == GESTURE_WAITING) {
            if (!light_sleep_active &&
                (now_ms - last_activity_ms) >= SLEEP_IDLE_TIMEOUT_MS) {
                light_sleep_active = true;
                loop_stage(STAGE_SLEEPNFY);
                send_event_packet(0x20);
                loop_stage(STAGE_IDLE);
                APP_TRACE(">>> Light sleep enter\n");
            }
        } else {
            if (light_sleep_active) {
                light_sleep_active = false;
                loop_stage(STAGE_SLEEPNFY);
                send_event_packet(0x21);
                loop_stage(STAGE_IDLE);
                APP_TRACE(">>> Light sleep wake\n");
            }
            last_activity_ms = now_ms;
        }

        if (active_count() > 0) {
            unconnected_since_ms = 0;
            sync_primary_conn_speed();
            /* Drop out of the post-connect fast advertising burst. */
            if (adv_tier == ADV_TIER_FAST &&
                (now_ms - adv_conn_change_ms) >= ADV_FAST_BURST_MS) {
                k_work_schedule(&adv_work, K_NO_WAIT);
            }
        } else {
            if (unconnected_since_ms == 0) {
                unconnected_since_ms = now_ms;
            }
            if (nrf_power_usbregstatus_vbusdet_get(NRF_POWER)) {
                unconnected_since_ms = now_ms;
            } else if ((now_ms - unconnected_since_ms) >= systemoff_fuse_ms() &&
                       !is_recording && !recording_requested) {
                enter_systemoff_for_tap_wake();
            }
        }

        /* Heartbeat: one line every 30 s while connected. hb_seq restarts at 1
         * after a reboot, so a '#1' following a '#N' is a reset, not a link
         * drop -- the two are indistinguishable from the peer's side. */
        if (active_count() > 0 &&
            (now_ms - last_heartbeat_ms) >= HEARTBEAT_INTERVAL_MS) {
            last_heartbeat_ms = now_ms;
            cb.hb_seq = ++hb_seq;
            APP_TRACE(">>> HB #%u up=%llds conns=%d rec=%d slp=%d bat=%u%% "
                   "ndrop=%u adv=%d",
                   hb_seq, now_ms / 1000, active_count(), (int)is_recording,
                   (int)light_sleep_active, bat_last_pct, ctrl_notify_dropped,
                   (int)adv_tier);
            for (int i = 0; atomic_get(&debug_active) && i < MAX_CONNS; i++) {
                struct bt_conn_info info;

                if (connections[i] &&
                    bt_conn_get_info(connections[i], &info) == 0) {
                    APP_TRACE(" c%d=%u/%u/%u", i, info.le.interval,
                           info.le.latency, info.le.timeout);
                }
            }
            APP_TRACE("\n");
        }

        /* Battery polling — force immediate re-read on USB disconnect */
        {
            static bool vbus_prev_main;
            bool vbus_now = nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
            if (!vbus_now && vbus_prev_main) {
                last_battery_ms = 0; /* trigger immediate ADC read */
            }
            vbus_prev_main = vbus_now;
        }
        if ((now_ms - last_battery_ms) >= BATTERY_POLL_INTERVAL_MS) {
            last_battery_ms = now_ms;
            loop_stage(STAGE_BATTERY);
            battery_update();
            loop_stage(STAGE_IDLE);
        }
    }

    return 0;
}
