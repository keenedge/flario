#include "tone.h"

/* Lift: pitch rises and beeps get faster with climb. */
#define LIFT_BASE_HZ      700.0f
#define LIFT_HZ_PER_MS    100.0f
#define LIFT_MAX_HZ       1600.0f
#define LIFT_CYCLE_MAX_MS 600.0f
#define LIFT_CYCLE_MIN_MS 150.0f
#define LIFT_CYCLE_MS_PER_MS 75.0f

/* Sink: continuous low tone that drops with sink rate. */
#define SINK_BASE_HZ   400.0f
#define SINK_HZ_PER_MS 40.0f
#define SINK_MIN_HZ    200.0f

static float clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

void tone_for_climb(const struct tone_config *cfg, float climb_ms, struct tone *out)
{
	if (climb_ms >= cfg->lift_threshold) {
		float freq = clampf(LIFT_BASE_HZ + LIFT_HZ_PER_MS * climb_ms, LIFT_BASE_HZ,
				    LIFT_MAX_HZ);
		float cycle = clampf(LIFT_CYCLE_MAX_MS - LIFT_CYCLE_MS_PER_MS * climb_ms,
				     LIFT_CYCLE_MIN_MS, LIFT_CYCLE_MAX_MS);

		out->freq_hz = (uint16_t)freq;
		out->on_ms = (uint16_t)(cycle / 2.0f);
		out->off_ms = (uint16_t)(cycle - cycle / 2.0f);
	} else if (climb_ms <= cfg->sink_threshold) {
		float freq = clampf(SINK_BASE_HZ + SINK_HZ_PER_MS * (climb_ms - cfg->sink_threshold),
				    SINK_MIN_HZ, SINK_BASE_HZ);

		out->freq_hz = (uint16_t)freq;
		out->on_ms = 0;
		out->off_ms = 0;
	} else {
		out->freq_hz = 0;
		out->on_ms = 0;
		out->off_ms = 0;
	}
}
