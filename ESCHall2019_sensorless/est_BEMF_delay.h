#ifndef EST_BEMF_H
#define EST_BEMF_H

#include "ADC_2019sensorless.h"
#include "config.h"
volatile uint32_t prevTickTime;
volatile uint32_t period_us_per_tick;

#include "IntervalTimer.h"
IntervalTimer delayCommutateTimer;

extern void commutate_isr(uint8_t phase, commutateMode_t caller);
void delayCommutate_isr();

extern volatile bool dir;
extern uint8_t curPhase_ADC;
// extern commutateMode_t commutateMode;

void BEMFcrossing_isr() {
	uint32_t curTickTime = micros();
	period_us_per_tick = curTickTime - prevTickTime;
	prevTickTime = curTickTime;
	delayCommutateTimer.begin(delayCommutate_isr, period_us_per_tick/2);
}
void delayCommutate_isr() {
	delayCommutateTimer.end();

	uint8_t pos_BEMF = dir ? (curPhase_ADC+1)%6 : (curPhase_ADC+5)%6;

	commutate_isr(pos_BEMF, MODE_SENSORLESS_DELAY);
}

float getSpeed_erps() {
	return 6000000.0/period_us_per_tick;
}

#endif