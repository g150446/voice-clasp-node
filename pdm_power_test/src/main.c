/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * XIAO nRF54L15 Sense shared IMU/PDM rail power test.
 *
 * The IMU and microphone share IMU&MIC_3V3. Three 20-second states isolate
 * the shared-rail baseline and the current added by an active PDM microphone:
 *
 *   S0: shared rail off; IMU off;        microphone off
 *   S1: shared rail on;  IMU power-down; microphone asleep
 *   S2: shared rail on;  IMU power-down; microphone active
 *
 * The CPU is allowed to enter System ON idle in every state. In microphone
 * active states it wakes only to return completed PDM buffers to the slab.
 * State markers are printed once at each transition; the onboard LED remains
 * off because its load would contaminate the current measurement.
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

#define LED_NODE DT_ALIAS(led0)
#define IMU_NODE DT_ALIAS(imu0)
#define PDM_NODE DT_NODELABEL(pdm20)

/* PDM_CLK is P1.12. Driving it low guarantees the microphone sees no clock. */
#define PDM_CLK_PIN 12

#define STATE_DURATION_MS 20000
#define SHARED_RAIL_STARTUP_MS 10

#define SAMPLE_RATE_HZ 16000
#define SAMPLE_WIDTH_BITS 16
#define CHANNEL_COUNT 1
#define AUDIO_FRAME_MS 20
#define AUDIO_FRAME_SAMPLES (SAMPLE_RATE_HZ * AUDIO_FRAME_MS / 1000)
#define AUDIO_BLOCK_SIZE (AUDIO_FRAME_SAMPLES * sizeof(int16_t))
#define AUDIO_BLOCK_COUNT 4

K_MEM_SLAB_DEFINE_STATIC(pdm_slab, AUDIO_BLOCK_SIZE, AUDIO_BLOCK_COUNT, 4);

enum test_state {
	STATE_RAIL_OFF,
	STATE_IMU_OFF_MIC_SLEEP,
	STATE_IMU_OFF_MIC_ACTIVE,
};

struct state_desc {
	const char *name;
	const char *description;
	bool mic_active;
};

static const struct state_desc states[] = {
	[STATE_RAIL_OFF] = {
		"S0", "shared rail off; IMU off; microphone off", false
	},
	[STATE_IMU_OFF_MIC_SLEEP] = {
		"S1", "shared rail on; IMU power-down; microphone sleep", false
	},
	[STATE_IMU_OFF_MIC_ACTIVE] = {
		"S2", "shared rail on; IMU power-down; microphone active", true
	},
};

static const enum test_state cycle[] = {
	STATE_RAIL_OFF,
	STATE_IMU_OFF_MIC_SLEEP,
	STATE_IMU_OFF_MIC_ACTIVE,
};

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);
static const struct device *const dmic = DEVICE_DT_GET(PDM_NODE);
static const struct device *const shared_regulator =
	DEVICE_DT_GET(DT_NODELABEL(pdm_imu_pwr));
static const struct device *const gpio1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));

/* The driver owns this definition; CONFIG_PINCTRL_DYNAMIC exposes it here. */
PINCTRL_DT_DEV_CONFIG_DECLARE(PDM_NODE);
static const struct pinctrl_dev_config *const pdm_pinctrl =
	PINCTRL_DT_DEV_CONFIG_GET(PDM_NODE);

static bool dmic_configured;
static bool dmic_running;
static bool clk_forced_low;
static bool shared_rail_enabled = true;
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

static int power_down_imu(void)
{
	struct sensor_value odr_off = { .val1 = 0, .val2 = 0 };
	int ret;

	ret = sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_off);
	if (ret < 0) {
		return ret;
	}

	return sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
			       SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_off);
}

static int enable_shared_rail(void)
{
	int ret;

	if (shared_rail_enabled) {
		return 0;
	}

	ret = regulator_enable(shared_regulator);
	if (ret < 0 && ret != -EALREADY) {
		return ret;
	}
	shared_rail_enabled = true;
	k_msleep(SHARED_RAIL_STARTUP_MS);
	return 0;
}

static int disable_shared_rail(void)
{
	int ret;

	if (!shared_rail_enabled) {
		return 0;
	}

	ret = regulator_disable(shared_regulator);
	if (ret < 0 && ret != -EALREADY) {
		return ret;
	}
	shared_rail_enabled = false;
	return 0;
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
		uint32_t size;
		int ret = dmic_read(dmic, 0, &block, &size, 0);

		if (ret < 0) {
			break;
		}
		if (block != NULL) {
			k_mem_slab_free(&pdm_slab, block);
		}
	}
}

static int stop_dmic(void)
{
	int ret;

	if (!dmic_running) {
		return 0;
	}

	ret = dmic_trigger(dmic, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		return ret;
	}
	dmic_running = false;

	/* STOP is asynchronous; allow its final buffer release to complete. */
	k_msleep(100);
	free_queued_audio();
	return 0;
}

static int restore_pdm_pins(void)
{
	int ret;

	if (clk_forced_low) {
		ret = gpio_pin_configure(gpio1, PDM_CLK_PIN, GPIO_DISCONNECTED);
		if (ret < 0) {
			return ret;
		}
		clk_forced_low = false;
	}

	return pinctrl_apply_state(pdm_pinctrl, PINCTRL_STATE_DEFAULT);
}

static int start_dmic(void)
{
	int ret;

	if (dmic_running) {
		return 0;
	}

	ret = restore_pdm_pins();
	if (ret < 0) {
		return ret;
	}

	ret = configure_dmic_once();
	if (ret < 0) {
		return ret;
	}

	ret = dmic_trigger(dmic, DMIC_TRIGGER_START);
	if (ret == 0) {
		dmic_running = true;
	}
	return ret;
}

static void sleep_dmic(void)
{
	int ret;

	print_result("DMIC stop", stop_dmic());
	print_result("PDM sleep pinctrl",
		     pinctrl_apply_state(pdm_pinctrl, PINCTRL_STATE_SLEEP));

	ret = gpio_pin_configure(gpio1, PDM_CLK_PIN, GPIO_OUTPUT_LOW);
	print_result("PDM_CLK GPIO low", ret);
	if (ret == 0) {
		clk_forced_low = true;
	}
}

static void enter_state(enum test_state state)
{
	int ret;

	switch (state) {
	case STATE_RAIL_OFF:
		sleep_dmic();
		if (shared_rail_enabled) {
			print_result("IMU power-down", power_down_imu());
		}
		print_result("shared rail off", disable_shared_rail());
		break;

	case STATE_IMU_OFF_MIC_SLEEP:
		ret = enable_shared_rail();
		print_result("shared rail on", ret);
		if (ret == 0) {
			print_result("IMU power-down", power_down_imu());
		}
		sleep_dmic();
		break;

	case STATE_IMU_OFF_MIC_ACTIVE:
		ret = enable_shared_rail();
		print_result("shared rail on", ret);
		if (ret != 0) {
			break;
		}
		print_result("IMU power-down", power_down_imu());
		print_result("DMIC start", start_dmic());
		break;
	}
}

static void hold_state(enum test_state state, int64_t deadline_ms)
{
	if (!states[state].mic_active) {
		int64_t remaining_ms = deadline_ms - k_uptime_get();

		if (remaining_ms > 0) {
			k_msleep(remaining_ms);
		}
		return;
	}

	while (k_uptime_get() < deadline_ms) {
		void *block = NULL;
		uint32_t size;
		int64_t remaining_ms = deadline_ms - k_uptime_get();
		int32_t timeout_ms = (int32_t)MIN(remaining_ms, 100);

		if (timeout_ms <= 0) {
			break;
		}
		if (dmic_read(dmic, 0, &block, &size, timeout_ms) == 0 &&
		    block != NULL) {
			k_mem_slab_free(&pdm_slab, block);
		}
	}
}

static void run_state(enum test_state state, int64_t deadline_ms)
{
	const struct state_desc *desc = &states[state];

	printk(">>> STATE %s: %s; duration=%d ms\n", desc->name,
	       desc->description, STATE_DURATION_MS);
	enter_state(state);
	printk(">>> MEASURE %s: until uptime %lld ms\n", desc->name,
	       (long long)deadline_ms);

	hold_state(state, deadline_ms);
	if (k_uptime_get() > deadline_ms + 10) {
		printk("!!! %s transition deadline missed by %lld ms\n", desc->name,
		       (long long)(k_uptime_get() - deadline_ms));
	}
}

int main(void)
{
	int64_t next_transition_ms;

	printk("\n============================================\n");
	printk("pdm_power_test: XIAO nRF54L15 Sense\n");
	printk("Shared rail baseline test; LED stays off\n");
	printk("Build: %s %s\n", __DATE__, __TIME__);
	printk("Cycle: S0(rail off) S1(IMU off/mic sleep) "
	       "S2(IMU off/mic active)\n");
	printk("State duration: %d ms; cycle duration: %d ms\n",
	       STATE_DURATION_MS, STATE_DURATION_MS * ARRAY_SIZE(cycle));
	printk("============================================\n");

	print_result("LED off", keep_led_off());

	if (!device_is_ready(shared_regulator) || !device_is_ready(imu) ||
	    !device_is_ready(dmic) || !device_is_ready(gpio1)) {
		printk("!!! Required device not ready: regulator=%d imu=%d dmic=%d gpio1=%d\n",
		       device_is_ready(shared_regulator), device_is_ready(imu),
		       device_is_ready(dmic), device_is_ready(gpio1));
		return 0;
	}

	/* regulator-boot-on already owns the initial reference. Do not call
	 * regulator_enable() here: S0 must release that single reference so the
	 * fixed regulator driver actually drives P0.01 low.
	 */
	shared_rail_enabled = true;

	next_transition_ms = k_uptime_get();
	while (true) {
		cycle_number++;
		printk("\n>>> CYCLE %u START (60 seconds)\n", cycle_number);

		for (size_t i = 0; i < ARRAY_SIZE(cycle); i++) {
			next_transition_ms += STATE_DURATION_MS;
			run_state(cycle[i], next_transition_ms);
		}
	}

	return 0;
}
