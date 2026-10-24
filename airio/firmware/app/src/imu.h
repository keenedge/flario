/*
 * LSM6DSV16X bring-up. Fusing vertical acceleration into the climb-rate
 * filter is the next step; for now the IMU is checked and logged.
 */

#ifndef AIRIO_IMU_H_
#define AIRIO_IMU_H_

int imu_init(void);

/* Log the current accelerometer reading. */
void imu_log(void);

#endif /* AIRIO_IMU_H_ */
