#ifndef EST_BEMF_H
#define EST_BEMF_H

#include "ADC_2019sensorless.h"
#include "config.h"
volatile uint32_t prevTickTime_BEMFdelay;
volatile uint32_t period_bemfdelay_usPerTick = 0;
extern volatile uint32_t period_commutation_usPerTick;

// #include "IntervalTimer.h"
// IntervalTimer delayCommutateTimer;
volatile uint32_t delayCommutateTimer = 0;
volatile bool delayCommutateFinished = true;

volatile void BEMFcrossing_isr();
extern void commutate_isr(uint8_t phase, commutateMode_t caller);
void delayCommutate_isr();

void updateCmp_BEMFdelay();
void updatePhase_BEMFdelay(uint8_t drivePhase);
static const uint8_t floatPhases[6] = {2, 0, 1, 2, 0, 1};
static const uint8_t highPhases[6]  = {0, 2, 2, 1, 1, 0};
// static const uint8_t floatPhases[6] = {VS_C, VS_A, VS_B, VS_C, VS_A, VS_B};
// static const uint8_t highPhases[6] = {VS_A, VS_C, VS_C, VS_B, VS_B, VS_A};
static const bool isRisingEdges[6] = {true, false, true, false, true, false};

volatile uint8_t floatPhase, highPhase;
volatile bool isRisingEdge;

volatile uint8_t curPhase_BEMFdelay;
float duty = 0;
uint16_t cmpVal;
volatile bool cmpOn = false;

extern volatile bool dir;
extern volatile uint8_t curPhase_BEMFdelay;
volatile uint8_t triggerPhase_delay;
// extern commutateMode_t commutateMode;

void updateBEMFdelay(uint32_t curTimeMicros) {
}

volatile void BEMFcrossing_isr() {
	if (!delayCommutateFinished || (triggerPhase_delay == curPhase_BEMFdelay)) {
		return;
	}
	delayCommutateFinished = false;
	triggerPhase_delay = curPhase_BEMFdelay;
	uint32_t curTickTime = micros();
	uint32_t elapsedTime = curTickTime - prevTickTime_BEMFdelay;
	period_bemfdelay_usPerTick = min(constrain(
			elapsedTime,
			period_bemfdelay_usPerTick - (PERIODSLEWLIM_US_PER_S*elapsedTime/1e6),
			period_bemfdelay_usPerTick + (PERIODSLEWLIM_US_PER_S*elapsedTime/1e6)),
		100000);
	prevTickTime_BEMFdelay = curTickTime;

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
	// digitalWrite(0, triggerPhase_delay==0);
	digitalWrite(1, pos_BEMF==0);
	GPIO0on = !GPIO0on;
}

float getSpeed_erps() {
	return 6000000.0/period_bemfdelay_usPerTick;
}


// IntervalTimer postSwitchDelayTimer;
extern volatile uint32_t timeToUpdateCmp;
extern volatile uint32_t period_commutation_usPerTick;
void updatePhase_BEMFdelay(uint8_t drivePhase) {
	if (curPhase_BEMFdelay != drivePhase){
		curPhase_BEMFdelay = drivePhase;
		if (curPhase_BEMFdelay >= 6){
			return;
		}
		timeToUpdateCmp = micros() + min(period_commutation_usPerTick/5, 200);
		cmpOn = false;

		floatPhase = floatPhases[curPhase_BEMFdelay];
		highPhase = highPhases[curPhase_BEMFdelay];
		isRisingEdge = isRisingEdges[curPhase_BEMFdelay] ^ (!dir);
		// adc->disableCompare(ADC_0);
		// adc->startContinuous(floatPhases[curPhase_BEMFdelay], ADC_0);
		// adc->startContinuous(highPhases[curPhase_BEMFdelay], ADC_1);
		// bool suc = postSwitchDelayTimer.begin(updateCmp_ADC, min(period_commutation_usPerTick/10, 1000));
	}
}
void updateCmp_BEMFdelay() {
	cmpOn = true;
}
void BEMFdelay_update(volatile uint16_t vsx_cnts[3]) {
  if (!cmpOn || (vsx_cnts[floatPhase] < 500)){
  	return;
  }

  volatile bool floatGt0 = (vsx_cnts[floatPhase] > (vsx_cnts[highPhase]/2)); // don't think i need to multiply by duty since it's properly phase aligned
  if (floatGt0 ^ !isRisingEdge){
  	BEMFcrossing_isr();
  	
	static volatile int16_t LEDon;
	#define LED_DIV 1
	digitalWrite(0, HIGH);
	// LEDon = !LEDon;
	LEDon ++;
	LEDon %= LED_DIV*2;
  }
  else {
  	digitalWrite(0, LOW);
  }
}

#endif