/*
 * airio: small BLE vario on a Seeed XIAO nRF52840.
 *
 * v0 loop: read the baro at 50 Hz, run the climb-rate filter, and map the
 * climb rate to the beeper. The IMU is brought up but not fused yet.
 */

#include "baro.h"
#include "beeper.h"
#include "tone.h"
#include "vario.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#if DT_NODE_EXISTS(DT_ALIAS(imu))
#include "imu.h"
#endif

LOG_MODULE_REGISTER(airio, LOG_LEVEL_INF);

#define SAMPLE_HZ      50
#define SAMPLE_PERIOD  K_MSEC(1000 / SAMPLE_HZ)
#define REPORT_EVERY   SAMPLE_HZ /* once a second */
#define WARMUP_SAMPLES (2 * SAMPLE_HZ) /* stay silent while the filter settles */

/* Filter tuning: about 0.3 m/s^2 of pilot vertical acceleration, 0.15 m baro noise. */
#define ACCEL_NOISE 0.3f
#define BARO_NOISE  0.15f

static K_TIMER_DEFINE(sample_timer, NULL, NULL);

int main(void)
{
	struct vario_filter filter;
	const struct tone_config tone_cfg = TONE_CONFIG_DEFAULT;
	struct tone t;
	float pressure_pa;
	uint32_t n = 0;

	LOG_INF("airio starting");

	if (baro_init() < 0 || beeper_init() < 0) {
		return 0;
	}

#if DT_NODE_EXISTS(DT_ALIAS(imu))
	(void)imu_init();
#endif

	vario_init(&filter, ACCEL_NOISE, BARO_NOISE);
	k_timer_start(&sample_timer, SAMPLE_PERIOD, SAMPLE_PERIOD);

	while (true) {
		k_timer_status_sync(&sample_timer);

		if (baro_read(&pressure_pa) < 0) {
			continue;
		}

		vario_update(&filter, vario_pressure_to_altitude(pressure_pa),
			     1.0f / SAMPLE_HZ);

		if (++n <= WARMUP_SAMPLES) {
			continue;
		}

		tone_for_climb(&tone_cfg, vario_climb(&filter), &t);
		beeper_set(&t);

		if (n % REPORT_EVERY == 0) {
			LOG_INF("alt %.1f m, climb %+.2f m/s", (double)vario_altitude(&filter),
				(double)vario_climb(&filter));
#if DT_NODE_EXISTS(DT_ALIAS(imu))
			imu_log();
#endif
		}
	}

	return 0;
}
