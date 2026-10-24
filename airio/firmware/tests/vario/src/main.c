/*
 * Host tests for the climb-rate filter and tone mapping.
 */

#include <math.h>
#include <stdlib.h>
#include <zephyr/ztest.h>

#include "tone.h"
#include "vario.h"

#define SAMPLE_HZ 50
#define DT        (1.0f / SAMPLE_HZ)

static float noise(float sigma)
{
	float sum = 0.0f;

	for (int i = 0; i < 12; i++) {
		sum += (float)rand() / (float)RAND_MAX;
	}
	return (sum - 6.0f) * sigma;
}

/* Run the filter on a constant climb and return the climb it reports. */
static float settle(float climb_ms, float noise_m, float seconds)
{
	struct vario_filter f;
	float alt = 1000.0f;

	vario_init(&f, 0.3f, 0.15f);
	for (int i = 0; i < (int)(seconds * SAMPLE_HZ); i++) {
		alt += climb_ms * DT;
		vario_update(&f, alt + noise(noise_m), DT);
	}
	return vario_climb(&f);
}

ZTEST(vario, test_pressure_altitude_round_trip)
{
	zassert_within(vario_pressure_to_altitude(101325.0f), 0.0f, 0.1f);
	/* ISA: about 898.7 hPa at 1000 m. */
	zassert_within(vario_pressure_to_altitude(89874.6f), 1000.0f, 1.0f);

	for (float h = 0.0f; h <= 5000.0f; h += 500.0f) {
		zassert_within(vario_pressure_to_altitude(vario_altitude_to_pressure(h)), h,
			       0.5f, "h=%f", (double)h);
	}
}

ZTEST(vario, test_still_air_reads_zero)
{
	srand(1);
	zassert_within(settle(0.0f, 0.15f, 20.0f), 0.0f, 0.15f);
}

ZTEST(vario, test_tracks_climb_and_sink)
{
	srand(2);
	zassert_within(settle(1.5f, 0.15f, 20.0f), 1.5f, 0.2f);
	zassert_within(settle(-3.0f, 0.15f, 20.0f), -3.0f, 0.2f);
}

ZTEST(vario, test_responds_within_two_seconds)
{
	struct vario_filter f;
	float alt = 1000.0f;

	vario_init(&f, 0.3f, 0.15f);
	for (int i = 0; i < 10 * SAMPLE_HZ; i++) {
		vario_update(&f, alt, DT);
	}

	/* Step into a 2 m/s thermal: reach 75% of it within 2 s. */
	for (int i = 0; i < 2 * SAMPLE_HZ; i++) {
		alt += 2.0f * DT;
		vario_update(&f, alt, DT);
	}
	zassert_true(vario_climb(&f) > 1.5f, "climb %f", (double)vario_climb(&f));
}

ZTEST(tone, test_silent_between_thresholds)
{
	const struct tone_config cfg = TONE_CONFIG_DEFAULT;
	struct tone t;

	tone_for_climb(&cfg, 0.0f, &t);
	zassert_equal(t.freq_hz, 0);
	tone_for_climb(&cfg, -1.9f, &t);
	zassert_equal(t.freq_hz, 0);
}

ZTEST(tone, test_lift_beeps_higher_and_faster)
{
	const struct tone_config cfg = TONE_CONFIG_DEFAULT;
	struct tone weak, strong;

	tone_for_climb(&cfg, 0.5f, &weak);
	tone_for_climb(&cfg, 3.0f, &strong);

	zassert_true(weak.freq_hz > 0 && weak.off_ms > 0);
	zassert_true(strong.freq_hz > weak.freq_hz);
	zassert_true(strong.on_ms + strong.off_ms < weak.on_ms + weak.off_ms);
}

ZTEST(tone, test_sink_is_continuous_and_low)
{
	const struct tone_config cfg = TONE_CONFIG_DEFAULT;
	struct tone sink, hard_sink;

	tone_for_climb(&cfg, -2.5f, &sink);
	tone_for_climb(&cfg, -6.0f, &hard_sink);

	zassert_true(sink.freq_hz > 0);
	zassert_equal(sink.off_ms, 0);
	zassert_true(hard_sink.freq_hz < sink.freq_hz);
	zassert_true(hard_sink.freq_hz >= 200);
}

ZTEST_SUITE(vario, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(tone, NULL, NULL, NULL, NULL, NULL);
