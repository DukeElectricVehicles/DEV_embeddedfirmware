#ifndef EST_BEMF_H
#define EST_BEMF_H

#include "ADC_2019sensorless.h"
#include "config.h"
volatile uint32_t prevTickTime_BEMFdelay;
volatile uint32_t period_bemfdelay_usPerTick = 0;
extern volatile uint32_t period_commutation_usPerTick;

#include "IntervalTimer.h"
IntervalTimer delayCommutateTimer;
IntervalTimer delayMissedTickTimer;
// IntervalTimer delayCmpTimer;
// volatile uint32_t delayCommutateTimer = 0;
volatile bool delayCommutateFinished = true;

void BEMFcrossing_service();
extern void allOff_isr();
extern void commutate_isr(uint8_t phase, commutateMode_t caller);
void delayCommutate_isr();
void safetyOff_delay();
void missedTick_delay();

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
uint16_t cmpVal;
volatile bool cmpOn = false;

extern volatile bool dir;
extern volatile uint8_t curPhase_BEMFdelay;
volatile uint8_t triggerPhase_delay;

extern volatile int16_t phaseAdvance_Q10;

bool leftSensorless = false;
extern void exitSensorless();

volatile bool BEMFcrossingFlag = false;
extern volatile uint32_t ADCreadTime;
extern volatile uint16_t vsx_cnts[3];

void updateBEMFdelay(uint32_t curTimeMicros) {
}

#define ZCMAfilterBase2 5
void BEMFcrossing_service() {
	test = 15;
	static uint32_t prevZCtimes_us[1<<ZCMAfilterBase2];
	static uint16_t prevZCtimesInd = 0;

	BEMFcrossingFlag = false;

	if (!delayCommutateFinished || (triggerPhase_delay == curPhase_BEMFdelay)) {
		return;
	}

	// uint32_t eventTime_us = micros();
	delayMissedTickTimer.end();
	delayCommutateTimer.end();

	cli();
	uint32_t eventTime_us = ADCreadTime;
	// correction for discrete ADC sampling
	#ifdef useTRIGDELAYCOMPENSATION
		uint32_t triggerDelay_us = period_bemfdelay_usPerTick * abs((int16_t)(vsx_cnts[floatPhase] - (vsx_cnts[highPhase]>>1))) / vsx_cnts[highPhase];
		triggerDelay_us = constrain(triggerDelay_us, 0, (uint32_t)1000000/PWM_FREQ);
		eventTime_us -= triggerDelay_us;
	#endif
	sei();

	uint32_t elapsedTime_us = eventTime_us - prevTickTime_BEMFdelay;
	if (elapsedTime_us < (0.9*period_bemfdelay_usPerTick)) {
		if (elapsedTime_us > 1000)
			period_bemfdelay_usPerTick *= .9;
		return;
	}
	triggerPhase_delay = curPhase_BEMFdelay;

	period_bemfdelay_usPerTick = (eventTime_us - prevZCtimes_us[prevZCtimesInd]) >> ZCMAfilterBase2;
	prevZCtimes_us[prevZCtimesInd++] = eventTime_us;
	prevZCtimesInd %= 1<<ZCMAfilterBase2;
	// period_bemfdelay_usPerTick = min(constrain(
	// 		elapsedTime_us,
	// 		period_bemfdelay_usPerTick - (PERIODSLEWLIM_US_PER_S*elapsedTime_us >> 20), ///1e6),
	// 		period_bemfdelay_usPerTick + (PERIODSLEWLIM_US_PER_S*elapsedTime_us >> 20)), ///1e6)),
	// 	100000);
	prevTickTime_BEMFdelay = eventTime_us;

	// if (delayCommutateFinished){
	// 	delayCommutateTimer = 0;
	// }
	// delayCommutateTimer = eventTime_us + (period_bemfdelay_usPerTick>>1) - (((int32_t)(phaseAdvance_Q10 * (period_bemfdelay_usPerTick>>1))) >> 10);
	delayCommutateFinished = false;
	delayCommutateTimer.priority(10);
	delayMissedTickTimer.priority(10);
	if (!delayCommutateTimer.begin(delayCommutate_isr,  // returns false if interval is negative (or if no more hardware resources)
			(period_bemfdelay_usPerTick>>1)
			- (((int32_t)(phaseAdvance_Q10 * (period_bemfdelay_usPerTick>>1))) >> 10)
			// - triggerDelay_us // included in eventTime_us
			- (micros()-eventTime_us)))
		delayCommutate_isr();
	if (!delayMissedTickTimer.begin(safetyOff_delay, 
			(period_bemfdelay_usPerTick)
			// - (triggerDelay_us)
			- (micros()-eventTime_us)))
		safetyOff_delay();
}
void delayCommutate_isr() {
	test = 16;
	delayCommutateTimer.end();

	uint8_t pos_BEMF = dir ? (triggerPhase_delay+1)%6 : (triggerPhase_delay+5)%6;

	commutate_isr(pos_BEMF, MODE_SENSORLESS_DELAY);

	// volatile static bool GPIO0on;
	// // digitalWrite(0, triggerPhase_delay==0);
	// digitalWrite(1, pos_BEMF==0);
	// GPIO0on = !GPIO0on;

	delayCommutateFinished = true;
}

float getSpeed_erps() {
	test = 17;
	return 6000000.0/period_bemfdelay_usPerTick;
}

void safetyOff_delay() {
	test = 18;
	delayMissedTickTimer.begin(missedTick_delay, period_bemfdelay_usPerTick>>2);
	allOff_isr();
}
void missedTick_delay() {
	test = 19;
	delayCommutateFinished = true;
	delayMissedTickTimer.end();
	exitSensorless();
}

// IntervalTimer postSwitchDelayTimer;
extern volatile uint32_t timeToUpdateCmp;
extern volatile uint32_t period_commutation_usPerTick;
void updatePhase_BEMFdelay(uint8_t drivePhase) {
	test = 20;
	if (curPhase_BEMFdelay != drivePhase){
		curPhase_BEMFdelay = drivePhase;
		if (curPhase_BEMFdelay >= 6){
			return;
		}
		timeToUpdateCmp = micros() + min(period_commutation_usPerTick/5, 200);
		cmpOn = false;
		delayCommutateFinished = true;
		// delayCmpTimer.begin(updateCmp_BEMFdelay, min(period_commutation_usPerTick/5,200));

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
	test = 29;
	// delayCmpTimer.end();
	cmpOn = true;
}
void inline BEMFdelay_update() {
	test = 22;
	// if ((!delayCommutateFinished) && (micros() >= delayCommutateTimer)){
	// delayCommutate_isr();
	// }
	if (micros() > timeToUpdateCmp) {
		updateCmp_BEMFdelay();
	}

	if (!cmpOn || (vsx_cnts[floatPhase] < 100)){
		return;
	}

	// volatile bool floatGt0 = (vsx_cnts[floatPhase] > (vsx_cnts[highPhase]>>1)); // don't think i need to multiply by duty since it's properly phase aligned
	if (!isRisingEdge ^ (vsx_cnts[floatPhase] > (vsx_cnts[highPhase]>>1))){
		BEMFcrossingFlag = true;
		// BEMFcrossing_service(vsx_cnts);
		
		// static volatile int16_t LEDon;
		// #define LED_DIV 1
		// digitalWriteFast(0, HIGH);
		// // LEDon = !LEDon;
		// LEDon ++;
		// LEDon %= LED_DIV*2;
	}
	else {
		// digitalWriteFast(0, LOW);
	}
}

#endif