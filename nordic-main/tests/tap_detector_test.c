/* Host test: cc -std=c11 -Wall -Wextra -Werror -Isrc
 * tests/tap_detector_test.c src/tap_detector.c -lm -o /tmp/tap_detector_test */
#include "tap_detector.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static struct tap_detector detector;
static unsigned singles, doubles;

static void feed(float x, float y, float z, unsigned samples)
{
    for (unsigned i = 0; i < samples; i++) {
        enum tap_result result = tap_detector_push(&detector, x, y, z);
        singles += result == TAP_SINGLE;
        doubles += result == TAP_DOUBLE;
    }
}
static void idle(unsigned n) { feed(0, 0, 1, n); }
static void start(void)
{
    tap_detector_reset(&detector);
    singles = doubles = 0;
    idle(250);
}
static void strike(void) { feed(0, 0, 2.6f, 2); }
static void gradual_strike(float sign)
{
    const float wave[] = {0.2f, 0.5f, 0.9f, 1.4f, 1.0f, 0.5f, 0.1f, 0};
    for (unsigned i = 0; i < sizeof(wave) / sizeof(wave[0]); i++) {
        feed(0, 0, 1 + sign * wave[i], 1);
    }
}
/* Gravity split 45 degrees across board X and Z, as when the wrist is rotated.
 * A tap on the raised surface then drives the board X axis, lateral to board Z. */
static void idle_tilt(unsigned n) { feed(0.7071f, 0, 0.7071f, n); }
static void strike_tilt(void) { feed(2.4f, 0, 0.7071f, 2); }
static void start_tilt(void)
{
    tap_detector_reset(&detector);
    singles = doubles = 0;
    idle_tilt(250);
}

int main(void)
{
    start(); idle(1000); assert(singles == 0 && doubles == 0);
    start(); strike(); idle(100);
    assert(singles == 0 && doubles == 0); /* single must wait for second */
    idle(200); assert(singles == 1 && doubles == 0);
    start(); strike(); idle(80); strike(); idle(300);
    assert(singles == 0 && doubles == 1);
    start(); strike(); idle(144); strike(); idle(300);
    assert(singles == 0 && doubles == 1); /* second at maximum gap */
    start(); strike(); idle(32); strike(); idle(300);
    assert(singles == 0 && doubles == 1); /* minimum gap */
    start(); strike(); idle(8); strike(); idle(300);
    assert(singles == 1 && doubles == 0); /* mechanical bounce */
    start(); strike(); idle(500); strike(); idle(300);
    assert(singles == 2 && doubles == 0);
    start(); feed(0, 0, 2.6f, 50); idle(300);
    assert(singles == 0 && doubles == 0); /* broad motion, any axis: width rejects */
    start(); feed(2, 0, 1, 2); idle(300);
    assert(singles == 1 && doubles == 0); /* lateral impulse is a valid tap */
    start(); feed(2, 0, 2.6f, 2); idle(300);
    assert(singles == 1 && doubles == 0); /* multi-axis impulse is a valid tap */
    start();
    for (unsigned i = 0; i < 2000; i++) {
        feed(0.2f * sinf(i * 0.12f), 0, 1 + 0.7f * sinf(i * 0.35f), 1);
    }
    idle(300); assert(singles == 0 && doubles == 0); /* vibration */
    start(); strike(); idle(80); tap_detector_reset(&detector);
    idle(250); strike(); idle(300);
    assert(singles == 1 && doubles == 0); /* FIFO gap breaks pairing */
    start(); strike(); idle(80); strike(); idle(50); strike(); idle(300);
    assert(singles == 0 && doubles == 1); /* residual after double */
    start(); feed(0, 0, -1, 250); feed(0, 0, -2.6f, 2);
    feed(0, 0, -1, 300); assert(singles == 1 && doubles == 0);
    start(); strike(); idle(80); feed(0, 0, 2.6f, 400); idle(300);
    assert(singles == 1 && doubles == 0); /* invalid second must not stall single */
    start(); strike(); idle(80); strike(); idle(330);
    strike(); idle(80); strike(); idle(300);
    assert(singles == 0 && doubles == 2); /* double remains usable after 700 ms */
    start(); strike(); idle(80); strike(); idle(330); strike(); idle(200);
    assert(singles == 0 && doubles == 1); /* single suppression lasts 1500 ms */
    start(); gradual_strike(1); idle(300);
    assert(singles == 1 && doubles == 0); /* short impulse with gradual rise */
    start(); gradual_strike(-1); idle(300);
    assert(singles == 1 && doubles == 0);
    start(); gradual_strike(1); idle(80); gradual_strike(1); idle(300);
    assert(singles == 0 && doubles == 1);
    start();
    for (unsigned i = 1; i <= 40; i++) feed(0, 0, 1 + 0.04f * i, 1);
    idle(300); assert(singles == 0 && doubles == 0); /* slow movement */
    /* Regression for the reported failure: at a 45-degree posture the tap lands
     * lateral to board Z. The old axis-dominance gate rejected the first strike
     * of a double, so a deliberate double tap was reported as a single. */
    start_tilt(); strike_tilt(); idle_tilt(200);
    assert(singles == 1 && doubles == 0); /* tilted single detected */
    start_tilt(); strike_tilt(); idle_tilt(60); strike_tilt(); idle_tilt(300);
    assert(singles == 0 && doubles == 1); /* tilted double keeps both strikes */
    start_tilt(); feed(2.2f, 0, 0.7071f, 50); idle_tilt(300);
    assert(singles == 0 && doubles == 0); /* sustained tilt motion still rejected */
    struct tap_config c = tap_default_config;
    assert(tap_config_valid(&c));
    c.peak_g = NAN; assert(!tap_config_valid(&c));
    c = tap_default_config; c.min_gap_ms = c.max_gap_ms;
    assert(!tap_config_valid(&c));
    c = tap_default_config; c.max_pulse_ms = 140; c.gravity_alpha = 0.001f;
    assert(tap_config_valid(&c));
    start();
    for (unsigned i = 0; i < 50; i++)
        assert(tap_detector_push_config(&detector, 0, 0, 2.6f, &c) == TAP_NONE);
    unsigned tuned_single = 0;
    for (unsigned i = 0; i < 300; i++)
        tuned_single += tap_detector_push_config(&detector, 0, 0, 1, &c) == TAP_SINGLE;
    assert(tuned_single == 1); /* same broad waveform can be replayed with a candidate */
    puts("tap_detector: 26 orientation-invariant scenarios + config validation/tuning passed");
}
