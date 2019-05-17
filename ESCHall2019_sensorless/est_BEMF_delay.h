#ifndef EST_BEMF_H
#define EST_BEMF_H

#include "ADC_2019sensorless.h"
#include "config.h"
volatile uint32_t prevTickTime;
volatile uint32_t period_us_per_tick = 0;

// #include "IntervalTimer.h"
// IntervalTimer delayCommutateTimer;
volatile uint32_t delayCommutateTimer = 0;
volatile bool delayCommutateFinished = true;


volatile void BEMFcrossing_isr();
extern void commutate_isr(uint8_t phase, commutateMode_t caller);
void delayCommutate_isr();

extern volatile bool dir;
extern volatile uint8_t curPhase_ADC;
// extern commutateMode_t commutateMode;

volatile void BEMFcrossing_isr() {
	if (!delayCommutateFinished) {
		return;
	}
	delayCommutateFinished = false;
	uint32_t curTickTime = micros();
	period_us_per_tick = min(curTickTime - prevTickTime, 10000);
	prevTickTime = curTickTime;

	// if (delayCommutateFinished){
	// 	delayCommutateTimer = 0;
	// }
	delayCommutateTimer = curTickTime + period_us_per_tick/2;
	// delayCommutateTimer.begin(delayCommutate_isr, period_us_per_tick/2);
}
void delayCommutate_isr() {
	// delayCommutateTimer.end();
	delayCommutateFinished = true;

	uint8_t pos_BEMF = dir ? (curPhase_ADC+1)%6 : (curPhase_ADC+5)%6;

	commutate_isr(pos_BEMF, MODE_SENSORLESS_DELAY);
	
	volatile static bool GPIO0on;
	digitalWrite(0, GPIO0on);
	GPIO0on = !GPIO0on;
}

float getSpeed_erps() {
	return 6000000.0/period_us_per_tick;
}

#endif