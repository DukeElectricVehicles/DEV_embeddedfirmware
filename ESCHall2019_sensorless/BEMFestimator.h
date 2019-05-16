#ifndef BEMF_EST_H
#define BEMF_EST_H

#include "ADC_2019sensorless.h"
volatile uint32_t prevTickTime;
volatile uint32_t period_us_per_tick;

#include "IntervalTimer.h"
IntervalTimer delayCommutateTimer;
IntervalTimer postSwitchDelayTimer;

extern void commutate_isr();
void delayCommutate();
void postSwitchDelay();

void BEMFcrossing_isr() {
	uint32_t curTickTime = micros();
	period_us_per_tick = curTickTime - prevTickTime;
	prevTickTime = curTickTime;
	delayCommutateTimer.begin(delayCommutate, period_us_per_tick/2);
}
void delayCommutate() {
	delayCommutateTimer.end();
	commutate_isr();
	postSwitchDelayTimer.begin(postSwitchDelay, min(period_us_per_tick/10, 1000));
}
void postSwitchDelay() {
	updateCmp();
	postSwitchDelayTimer.end();
}

float getSpeed_erps() {
	return 6000000.0/period_us_per_tick;
}

#endif