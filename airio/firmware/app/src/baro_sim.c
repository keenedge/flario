/*
 * Synthetic baro for native_sim: a repeating flight of a thermal climb, a
 * glide and a sink, with baro-like noise added, so the filter and beeper
 * logic can be exercised without hardware.
 */

#include "baro.h"
#include "vario.h"

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(baro_sim, LOG_LEVEL_INF);

#define START_ALT_M  1000.0f
#define NOISE_SIGMA_M 0.15f

struct segment {
	uint32_t duration_ms;
	float climb_ms;
};

static const struct segment flight[] = {
	{10000, 0.0f},  /* on launch */
	{20000, 1.5f},  /* weak thermal */
	{15000, 3.0f},  /* core */
	{20000, -1.0f}, /* glide */
	{10000, -3.0f}, /* sink */
};

static float alt_m = START_ALT_M;
static int64_t last_ms;

static uint32_t flight_duration_ms(void)
{
	uint32_t total = 0;

	for (size_t i = 0; i < ARRAY_SIZE(flight); i++) {
		total += flight[i].duration_ms;
	}
	return total;
}

static float climb_at(uint32_t t_ms)
{
	t_ms %= flight_duration_ms();

	for (size_t i = 0; i < ARRAY_SIZE(flight); i++) {
		if (t_ms < flight[i].duration_ms) {
			return flight[i].climb_ms;
		}
		t_ms -= flight[i].duration_ms;
	}
	return 0.0f;
}

/* Roughly Gaussian noise from the sum of uniform samples. */
static float noise(float sigma)
{
	float sum = 0.0f;

	for (int i = 0; i < 12; i++) {
		sum += (float)rand() / (float)RAND_MAX;
	}
	return (sum - 6.0f) * sigma;
}

int baro_init(void)
{
	srand(1);
	last_ms = k_uptime_get();
	LOG_INF("baro: synthetic flight, %u s loop", flight_duration_ms() / 1000);
	return 0;
}

int baro_read(float *pressure_pa)
{
	int64_t now = k_uptime_get();
	float dt = (float)(now - last_ms) / 1000.0f;

	alt_m += climb_at((uint32_t)now) * dt;
	last_ms = now;

	*pressure_pa = vario_altitude_to_pressure(alt_m + noise(NOISE_SIGMA_M));
	return 0;
}
