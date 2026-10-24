#include "beeper.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(beeper, LOG_LEVEL_INF);

#define BEEPER_NODE DT_PATH(zephyr_user)
#define HAS_PWM     DT_NODE_HAS_PROP(BEEPER_NODE, pwms)

#if HAS_PWM
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec buzz_a = PWM_DT_SPEC_GET_BY_NAME(BEEPER_NODE, buzz_a);
static const struct pwm_dt_spec buzz_b = PWM_DT_SPEC_GET_BY_NAME(BEEPER_NODE, buzz_b);
#endif

/* Idle poll interval while silent, so a new tone starts promptly. */
#define IDLE_MS 20

static K_MUTEX_DEFINE(tone_lock);
static struct tone current;

static void get_tone(struct tone *t)
{
	k_mutex_lock(&tone_lock, K_FOREVER);
	*t = current;
	k_mutex_unlock(&tone_lock);
}

static void output(uint16_t freq_hz)
{
#if HAS_PWM
	if (freq_hz == 0) {
		pwm_set_pulse_dt(&buzz_a, 0);
		pwm_set_pulse_dt(&buzz_b, 0);
		return;
	}

	uint32_t period_ns = NSEC_PER_SEC / freq_hz;

	/* B is inverted in devicetree, so the piezo sees twice the swing. */
	pwm_set_dt(&buzz_a, period_ns, period_ns / 2);
	pwm_set_dt(&buzz_b, period_ns, period_ns / 2);
#else
	ARG_UNUSED(freq_hz);
#endif
}

static const char *mode_name(const struct tone *t)
{
	if (t->freq_hz == 0) {
		return "silent";
	}
	return t->off_ms == 0 ? "sink" : "lift";
}

void beeper_set(const struct tone *t)
{
	k_mutex_lock(&tone_lock, K_FOREVER);
	if (mode_name(t) != mode_name(&current)) {
		LOG_INF("%s, %u Hz", mode_name(t), t->freq_hz);
	}
	current = *t;
	k_mutex_unlock(&tone_lock);
}

static void beeper_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct tone t;

	while (true) {
		get_tone(&t);

		if (t.freq_hz == 0) {
			output(0);
			k_msleep(IDLE_MS);
		} else if (t.off_ms == 0) {
			output(t.freq_hz);
			k_msleep(IDLE_MS);
		} else {
			output(t.freq_hz);
			k_msleep(t.on_ms);
			output(0);
			k_msleep(t.off_ms);
		}
	}
}

K_THREAD_DEFINE(beeper_tid, 1024, beeper_thread, NULL, NULL, NULL, 5, 0, 0);

int beeper_init(void)
{
#if HAS_PWM
	if (!pwm_is_ready_dt(&buzz_a) || !pwm_is_ready_dt(&buzz_b)) {
		LOG_ERR("beeper PWM not ready");
		return -ENODEV;
	}
#endif
	return 0;
}
