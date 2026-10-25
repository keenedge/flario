/*
 * Maps climb rate to the beeper pattern. Pure C, no Zephyr dependencies.
 */

#ifndef AIRIO_TONE_H_
#define AIRIO_TONE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct tone {
	uint16_t freq_hz; /* 0 = silent */
	uint16_t on_ms;
	uint16_t off_ms;  /* 0 = continuous tone */
};

struct tone_config {
	float lift_threshold; /* m/s, beeping starts at or above this */
	float sink_threshold; /* m/s, sink tone starts at or below this (negative) */
};

#define TONE_CONFIG_DEFAULT {.lift_threshold = 0.1f, .sink_threshold = -2.0f}

void tone_for_climb(const struct tone_config *cfg, float climb_ms, struct tone *out);

#ifdef __cplusplus
}
#endif

#endif /* AIRIO_TONE_H_ */
