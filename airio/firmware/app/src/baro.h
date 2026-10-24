/*
 * Barometer source. On hardware this is the BMP581 (or BMP580) on i2c1; on
 * native_sim it is a synthetic flight so the whole app runs without a board.
 */

#ifndef AIRIO_BARO_H_
#define AIRIO_BARO_H_

int baro_init(void);

/* Read the latest pressure in Pa. Call at the vario sample rate. */
int baro_read(float *pressure_pa);

#endif /* AIRIO_BARO_H_ */
