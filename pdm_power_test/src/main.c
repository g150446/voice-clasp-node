/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * XIAO nRF54L15 Sense shared IMU/PDM rail power test.
 *
 * The microphone cannot have VDD removed independently of the IMU. This test
 * determines whether stopping its PDM clock makes its current contribution
 * disappear while the accelerometer remains active at 416 Hz.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/audio/dmic.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#define LED_NODE DT_ALIAS(led0)
#define IMU_NODE DT_ALIAS(imu0)
#define PDM_NODE DT_NODELABEL(pdm20)

#define TRANSITION_SETTLE_MS 1000
#define MEASURE_MS 20000

#define SAMPLE_RATE_HZ 16000
#define SAMPLE_WIDTH_BITS 16
#define CHANNEL_COUNT 1
#define AUDIO_FRAME_MS 20
#define AUDIO_FRAME_SAMPLES (SAMPLE_RATE_HZ * AUDIO_FRAME_MS / 1000)
#define AUDIO_BLOCK_SIZE (AUDIO_FRAME_SAMPLES * sizeof(int16_t))
#define AUDIO_BLOCK_COUNT 4

K_MEM_SLAB_DEFINE_STATIC(pdm_slab, AUDIO_BLOCK_SIZE, AUDIO_BLOCK_COUNT, 4);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);
static const struct device *const dmic = DEVICE_DT_GET(PDM_NODE);
static const struct device *const shared_regulator =
	DEVICE_DT_GET(DT_NODELABEL(pdm_imu_pwr));
static const struct device *const gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

/* The driver owns this definition; NON_STATIC exposes it for this test. */
PINCTRL_DT_DEV_CONFIG_DECLARE(PDM_NODE);
static const struct pinctrl_dev_config *const pdm_pinctrl =
	PINCTRL_DT_DEV_CONFIG_GET(PDM_NODE);

static bool dmic_configured;
static bool dmic_suspended;
static unsigned int cycle_number;

static void print_result(const char *operation, int ret)
{
	if (ret < 0) {
		printk("!!! %s failed: %d\n", operation, ret);
	}
}

static int keep_led_off(void)
{
	if (!gpio_is_ready_dt(&led)) {
		return -ENODEV;
	}

	return gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
}

static int configure_imu(void)
{
	struct sensor_value accel_odr = { .val1 = 416, .val2 = 0 };
	struct sensor_value gyro_off = { .val1 = 0, .val2 = 0 };
	struct sensor_value accel_range;
	int ret;

	sensor_g_to_ms2(2, &accel_range);

	ret = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &gyro_off);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_FULL_SCALE, &accel_range);
	if (ret < 0) {
		return ret;
	}

	return sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
			       SENSOR_ATTR_SAMPLING_FREQUENCY, &accel_odr);
}

static void print_acceleration(const char *state)
{
	struct sensor_value x;
	struct sensor_value y;
	struct sensor_value z;
	int ret;

	ret = sensor_sample_fetch_chan(imu, SENSOR_CHAN_ACCEL_XYZ);
	if (ret == 0) {
		ret = sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &x);
	}
	if (ret == 0) {
		ret = sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &y);
	}
	if (ret == 0) {
		ret = sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &z);
	}

	if (ret < 0) {
		printk("!!! %s IMU proof FAILED: %d\n", state, ret);
		return;
	}

	printk(">>> %s IMU proof: x=%.3f y=%.3f z=%.3f m/s^2\n",
	       state, sensor_value_to_double(&x), sensor_value_to_double(&y),
	       sensor_value_to_double(&z));
}

static void run_quiet_state(const char *state, const char *description,
			    bool prove_imu)
{
	printk(">>> STATE %s: %s\n", state, description);
	k_msleep(TRANSITION_SETTLE_MS);
	printk(">>> MEASURE %s: 20 seconds\n", state);
	if (prove_imu) {
		print_acceleration(state);
	}
	k_msleep(MEASURE_MS);
}

static int configure_dmic_once(void)
{
	static struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_WIDTH_BITS,
		.pcm_rate = SAMPLE_RATE_HZ,
		.block_size = AUDIO_BLOCK_SIZE,
		.mem_slab = &pdm_slab,
	};
	static struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 1000000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
			.req_num_chan = CHANNEL_COUNT,
		},
	};
	int ret;

	if (dmic_configured) {
		return 0;
	}

	cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	ret = dmic_configure(dmic, &cfg);
	if (ret == 0) {
		dmic_configured = true;
	}
	return ret;
}

static void free_queued_audio(void)
{
	while (true) {
		void *block = NULL;
		size_t size;
		int ret = dmic_read(dmic, 0, &block, &size, 0);

		if (ret < 0) {
			break;
		}
		if (block != NULL) {
			k_mem_slab_free(&pdm_slab, block);
		}
	}
}

static int prepare_dmic_default(void)
{
	int ret;

	if (dmic_suspended) {
		ret = pm_device_action_run(dmic, PM_DEVICE_ACTION_RESUME);
		if (ret < 0) {
			return ret;
		}
		dmic_suspended = false;
	}

	return pinctrl_apply_state(pdm_pinctrl, PINCTRL_STATE_DEFAULT);
}

static void run_s2_recording(void)
{
	int64_t started;
	bool measurement_started = false;
	int ret;

	ret = prepare_dmic_default();
	print_result("PDM default pinctrl", ret);
	if (ret < 0) {
		return;
	}

	ret = configure_dmic_once();
	print_result("DMIC configure", ret);
	if (ret < 0) {
		return;
	}

	ret = dmic_trigger(dmic, DMIC_TRIGGER_START);
	print_result("DMIC start", ret);
	if (ret < 0) {
		return;
	}

	printk(">>> STATE S2: IMU 416 Hz + PDM recording\n");
	started = k_uptime_get();

	while ((k_uptime_get() - started) <
	       (TRANSITION_SETTLE_MS + MEASURE_MS)) {
		void *block = NULL;
		size_t size;

		ret = dmic_read(dmic, 0, &block, &size, 100);
		if (ret == 0 && block != NULL) {
			k_mem_slab_free(&pdm_slab, block);
		}

		if (!measurement_started &&
		    (k_uptime_get() - started) >= TRANSITION_SETTLE_MS) {
			measurement_started = true;
			printk(">>> MEASURE S2: 20 seconds\n");
			print_acceleration("S2");
		}
	}
}

static int stop_dmic(void)
{
	int ret = dmic_trigger(dmic, DMIC_TRIGGER_STOP);

	if (ret < 0) {
		return ret;
	}

	/* STOP is asynchronous; let the final release event complete. */
	k_msleep(100);
	free_queued_audio();
	return 0;
}

static void enter_s0(void)
{
	int ret = regulator_disable(shared_regulator);

	if (ret < 0 && ret != -EALREADY) {
		print_result("shared rail off", ret);
	}
	run_quiet_state("S0", "shared rail OFF; IMU and microphone OFF", false);
}

static bool enter_s1(void)
{
	int ret = regulator_enable(shared_regulator);

	if (ret < 0 && ret != -EALREADY) {
		print_result("shared rail on", ret);
		return false;
	}
	k_msleep(10);

	ret = configure_imu();
	print_result("IMU configure (accel 416 Hz, gyro off)", ret);
	if (ret < 0) {
		return false;
	}

	if (cycle_number == 1U) {
		run_quiet_state("S1",
				"IMU 416 Hz; PDM never configured (target baseline)",
				true);
	} else {
		run_quiet_state("S1",
				"IMU 416 Hz; PDM previously configured but stopped",
				true);
	}
	return true;
}

int main(void)
{
	int ret;

	printk("\n============================================\n");
	printk("pdm_power_test: XIAO nRF54L15 Sense\n");
	printk("Mic VDD is shared with IMU; testing effective sleep\n");
	printk("Build: %s %s\n", __DATE__, __TIME__);
	printk("============================================\n");

	print_result("LED off", keep_led_off());

	if (!device_is_ready(shared_regulator) || !device_is_ready(imu) ||
	    !device_is_ready(dmic) || !device_is_ready(gpio1)) {
		printk("!!! Required device not ready: regulator=%d imu=%d dmic=%d gpio1=%d\n",
		       device_is_ready(shared_regulator), device_is_ready(imu),
		       device_is_ready(dmic), device_is_ready(gpio1));
		return 0;
	}

	while (true) {
		cycle_number++;
		printk("\n>>> CYCLE %u START\n", cycle_number);

		enter_s0();
		if (!enter_s1()) {
			printk("!!! Cycle aborted because IMU baseline is unavailable\n");
			k_msleep(5000);
			continue;
		}

		run_s2_recording();

		ret = stop_dmic();
		print_result("DMIC stop", ret);
		run_quiet_state("S3", "PDM stopped only", true);

		ret = pinctrl_apply_state(pdm_pinctrl, PINCTRL_STATE_SLEEP);
		print_result("PDM sleep pinctrl", ret);
		run_quiet_state("S3D", "PDM stopped; CLK and DIN disconnected", true);

		ret = gpio_pin_configure(gpio1, 12, GPIO_OUTPUT_LOW);
		print_result("P1.12 GPIO low", ret);
		run_quiet_state("S4", "PDM stopped; PDM_CLK forced GPIO low", true);

		ret = pm_device_action_run(dmic, PM_DEVICE_ACTION_SUSPEND);
		printk(">>> S5 suspend result: %d%s\n", ret,
		       ret == -ENOSYS ? " (DMIC driver has no device PM callback)" : "");
		if (ret == 0) {
			dmic_suspended = true;
		}
		run_quiet_state("S5", "S4 plus DMIC device suspend attempt", true);
	}

	return 0;
}
