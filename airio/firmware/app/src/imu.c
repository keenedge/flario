#include "imu.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

static const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu));

int imu_init(void)
{
	if (!device_is_ready(imu)) {
		LOG_ERR("%s not ready", imu->name);
		return -ENODEV;
	}

	LOG_INF("imu: %s", imu->name);
	return 0;
}

void imu_log(void)
{
	struct sensor_value accel[3];

	if (!device_is_ready(imu) || sensor_sample_fetch_chan(imu, SENSOR_CHAN_ACCEL_XYZ) < 0 ||
	    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel) < 0) {
		return;
	}

	LOG_INF("accel %.2f %.2f %.2f m/s^2", sensor_value_to_double(&accel[0]),
		sensor_value_to_double(&accel[1]), sensor_value_to_double(&accel[2]));
}
