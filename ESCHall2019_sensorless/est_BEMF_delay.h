#ifndef EST_BEMF_H
#define EST_BEMF_H

#include "ADC_2019sensorless.h"
#include "config.h"
volatile uint32_t prevTickTime;
volatile uint32_t period_bemfdelay_usPerTick = 0;
extern volatile uint32_t period_commutation_usPerTick;

// #include "IntervalTimer.h"
// IntervalTimer delayCommutateTimer;
volatile uint32_t delayCommutateTimer = 0;
volatile bool delayCommutateFinished = true;

volatile void BEMFcrossing_isr();
extern void commutate_isr(uint8_t phase, commutateMode_t caller);
void delayCommutate_isr();

extern volatile bool dir;
extern volatile uint8_t curPhase_ADC;
volatile uint8_t triggerPhase_delay;
// extern commutateMode_t commutateMode;

volatile void BEMFcrossing_isr() {
	if (!delayCommutateFinished || (triggerPhase_delay == curPhase_ADC)) {
		return;
	}
	delayCommutateFinished = false;
	triggerPhase_delay = curPhase_ADC;
	uint32_t curTickTime = micros();
	uint32_t elapsedTime = curTickTime - prevTickTime;
	period_bemfdelay_usPerTick = min(constrain(
			elapsedTime,
			period_bemfdelay_usPerTick - (PERIODSLEWLIM_US_PER_S*elapsedTime/1e6),
			period_bemfdelay_usPerTick + (PERIODSLEWLIM_US_PER_S*elapsedTime/1e6)),
		10000);
	prevTickTime = curTickTime;

	// if (delayCommutateFinished){
	// 	delayCommutateTimer = 0;
	// }
	delayCommutateTimer = curTickTime + period_bemfdelay_usPerTick/2;
	// delayCommutateTimer.begin(delayCommutate_isr, period_bemfdelay_usPerTick/2);
}
void delayCommutate_isr() {
	// delayCommutateTimer.end();
	delayCommutateFinished = true;

	uint8_t pos_BEMF = dir ? (triggerPhase_delay+1)%6 : (triggerPhase_delay+5)%6;

	commutate_isr(pos_BEMF, MODE_SENSORLESS_DELAY);

	volatile static bool GPIO0on;
	digitalWrite(0, triggerPhase_delay==0);
	digitalWrite(1, pos_BEMF==0);
	GPIO0on = !GPIO0on;
}

float getSpeed_erps() {
	return 6000000.0/period_bemfdelay_usPerTick;
}

#endif