/*
 * Climb-rate filter: a two-state (altitude, vertical speed) Kalman filter
 * fed with barometric altitude. Pure C with no Zephyr dependencies so it can
 * be unit tested and replayed against recorded flights on a host.
 */

#ifndef AIRIO_VARIO_H_
#define AIRIO_VARIO_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct vario_filter {
	float alt;      /* m */
	float vel;      /* m/s, positive up */
	float p[2][2];  /* state covariance */
	float q;        /* process noise: vertical acceleration variance, (m/s^2)^2 */
	float r;        /* measurement noise: baro altitude variance, m^2 */
	bool initialised;
};

/*
 * accel_noise: expected vertical acceleration of the pilot, 1 sigma (m/s^2).
 *              Larger values make the vario react faster but noisier.
 * baro_noise:  baro altitude noise, 1 sigma (m).
 */
void vario_init(struct vario_filter *f, float accel_noise, float baro_noise);

/* Feed one baro altitude sample taken dt seconds after the previous one. */
void vario_update(struct vario_filter *f, float alt_m, float dt_s);

static inline float vario_climb(const struct vario_filter *f)
{
	return f->vel;
}

static inline float vario_altitude(const struct vario_filter *f)
{
	return f->alt;
}

/* ISA pressure altitude (m) for a pressure in Pa, against a 1013.25 hPa datum. */
float vario_pressure_to_altitude(float pressure_pa);

/* Inverse of vario_pressure_to_altitude(). */
float vario_altitude_to_pressure(float alt_m);

#ifdef __cplusplus
}
#endif

#endif /* AIRIO_VARIO_H_ */
