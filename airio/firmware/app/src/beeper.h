/*
 * Piezo beeper. On hardware it drives BZ1 differentially from two PWM
 * channels (one inverted) on D2/D3; on native_sim it logs tone changes.
 */

#ifndef AIRIO_BEEPER_H_
#define AIRIO_BEEPER_H_

#include "tone.h"

int beeper_init(void);

/* Change the pattern; the beeper thread picks it up on its next cycle. */
void beeper_set(const struct tone *t);

#endif /* AIRIO_BEEPER_H_ */
