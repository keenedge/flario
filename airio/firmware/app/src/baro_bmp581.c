#include "baro.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(baro, LOG_LEVEL_INF);

static const struct device *const baro = DEVICE_DT_GET(DT_ALIAS(baro));

int baro_init(void)
{
	if (!device_is_ready(baro)) {
		LOG_ERR("%s not ready", baro->name);
		return -ENODEV;
	}

	LOG_INF("baro: %s", baro->name);
	return 0;
}

int baro_read(float *pressure_pa)
{
	struct sensor_value press;
	int ret;

	ret = sensor_sample_fetch(baro);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_channel_get(baro, SENSOR_CHAN_PRESS, &press);
	if (ret < 0) {
		return ret;
	}

	/* SENSOR_CHAN_PRESS is in kPa. */
	*pressure_pa = (float)(sensor_value_to_double(&press) * 1000.0);
	return 0;
}
