#include "vario.h"

#include <math.h>

#define ISA_P0_PA    101325.0f
#define ISA_EXPONENT 0.190263f /* 1 / 5.25588 */
#define ISA_SCALE_M  44330.8f

void vario_init(struct vario_filter *f, float accel_noise, float baro_noise)
{
	f->alt = 0.0f;
	f->vel = 0.0f;
	f->p[0][0] = 1.0f;
	f->p[0][1] = 0.0f;
	f->p[1][0] = 0.0f;
	f->p[1][1] = 1.0f;
	f->q = accel_noise * accel_noise;
	f->r = baro_noise * baro_noise;
	f->initialised = false;
}

void vario_update(struct vario_filter *f, float alt_m, float dt_s)
{
	if (!f->initialised) {
		f->alt = alt_m;
		f->vel = 0.0f;
		f->initialised = true;
		return;
	}

	/* Predict with a constant-velocity model driven by white acceleration. */
	float dt2 = dt_s * dt_s;
	float dt3 = dt2 * dt_s;
	float dt4 = dt3 * dt_s;

	f->alt += f->vel * dt_s;

	float p00 = f->p[0][0] + dt_s * (f->p[1][0] + f->p[0][1]) + dt2 * f->p[1][1] +
		    f->q * dt4 / 4.0f;
	float p01 = f->p[0][1] + dt_s * f->p[1][1] + f->q * dt3 / 2.0f;
	float p10 = f->p[1][0] + dt_s * f->p[1][1] + f->q * dt3 / 2.0f;
	float p11 = f->p[1][1] + f->q * dt2;

	/* Correct with the baro altitude. */
	float y = alt_m - f->alt;
	float s = p00 + f->r;
	float k0 = p00 / s;
	float k1 = p10 / s;

	f->alt += k0 * y;
	f->vel += k1 * y;

	f->p[0][0] = (1.0f - k0) * p00;
	f->p[0][1] = (1.0f - k0) * p01;
	f->p[1][0] = p10 - k1 * p00;
	f->p[1][1] = p11 - k1 * p01;
}

float vario_pressure_to_altitude(float pressure_pa)
{
	return ISA_SCALE_M * (1.0f - powf(pressure_pa / ISA_P0_PA, ISA_EXPONENT));
}

float vario_altitude_to_pressure(float alt_m)
{
	return ISA_P0_PA * powf(1.0f - alt_m / ISA_SCALE_M, 1.0f / ISA_EXPONENT);
}
