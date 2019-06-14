#ifndef EST_BEMF_H
#define EST_BEMF_H

#include "ADC_2019sensorless.h"
#include "config.h"

extern void allOff_isr();
extern void commutate_isr(uint8_t phase, commutateMode_t caller);

void BEMFcrossing_isr();
void commutate_BEMFdelay_isr();
extern void exitSensorless(); bool leftSensorless = false;

void updatePhase_BEMFdelay(uint8_t drivePhase);
static const uint8_t floatPhases[6] = {2, 0, 1, 2, 0, 1}; // {VS_C, VS_A, VS_B, VS_C, VS_A, VS_B};
static const uint8_t highPhases[6]  = {0, 2, 2, 1, 1, 0}; // {VS_A, VS_C, VS_C, VS_B, VS_B, VS_A};
static const bool isRisingEdges[6] = {true, false, true, false, true, false};
volatile uint8_t floatPhase;
volatile uint8_t highPhase;
volatile bool isRisingEdge;

// master parameters
extern volatile bool dir;
extern volatile int16_t phaseAdvance_Q10;
extern volatile commutateMode_t commutateMode;

// comutation parameters
volatile uint8_t mCurPhase_BEMFdelay;
volatile uint8_t mTriggerPhase_BEMFdelay;
volatile bool delayCommutateFinished = true;

volatile uint32_t mPrevZCTime_BEMFdelay_us;
volatile uint32_t period_bemfdelay_usPerTick = 0;
extern volatile uint32_t period_commutation_usPerTick;

// ADC stuff
extern volatile uint32_t ADCreadTime;
extern volatile uint16_t vsx_cnts[3];

// timers
volatile uint32_t delayCommutateTimer = 0;
volatile uint32_t delayCommutateDur = 0;
volatile uint32_t updateCmpTimer = 0;
// volatile uint32_t updateCmpDur = MINTICKDUR_US / 2 * (0.7);
#define updateCmpDur (MINTICKDUR_US / 2 / 2)
#if updateCmpDur != 390 // sanity check
	#error
#endif

void commutate_BEMFdelay_isr() {
	test = 16;
	uint8_t pos_BEMF = dir ? (mTriggerPhase_BEMFdelay+1)%6 : (mTriggerPhase_BEMFdelay+5)%6;
	commutate_isr(pos_BEMF, MODE_SENSORLESS_DELAY);
}

float getSpeed_erps() {
	test = 17;
	return 6000000.0/period_bemfdelay_usPerTick;
}

void updatePhase_BEMFdelay(uint8_t drivePhase) {
	test = 20;
	if (mCurPhase_BEMFdelay != drivePhase){
		mCurPhase_BEMFdelay = drivePhase;
		if (mCurPhase_BEMFdelay >= 6){
			return;
		}
		updateCmpTimer = micros();
		// updateCmpDur = MINTICKDUR_US / 2 * (0.7); // COMPILER PLZ OPTIMIZE THIS

		delayCommutateFinished = true;

		floatPhase = floatPhases[mCurPhase_BEMFdelay];
		highPhase = highPhases[mCurPhase_BEMFdelay];
		isRisingEdge = isRisingEdges[mCurPhase_BEMFdelay] ^ (!dir);
	}
}

void inline BEMFdelay_update() {
	test = 22;
	uint32_t curTime_us = micros();

	if (commutateMode == MODE_SENSORLESS_DELAY) {
		if ((int32_t)(curTime_us - mPrevZCTime_BEMFdelay_us) > period_bemfdelay_usPerTick){
			Serial.print((int32_t)(curTime_us - mPrevZCTime_BEMFdelay_us));
			Serial.print('\t');
			Serial.println(period_bemfdelay_usPerTick);
			allOff_isr();
			if ((int32_t)(curTime_us - mPrevZCTime_BEMFdelay_us) > (period_bemfdelay_usPerTick << 1)) {
				exitSensorless();
			}
		}
	}

	// awaiting commutation
	if (!delayCommutateFinished) {
		if ((int32_t)(curTime_us - delayCommutateTimer) >= delayCommutateDur) {
			commutate_BEMFdelay_isr();
			delayCommutateFinished = true;
			updateCmpTimer = curTime_us;
		}
		return;
	}

	if ((int32_t)(curTime_us - updateCmpTimer) < updateCmpDur) {
		return;
	}

	if (vsx_cnts[floatPhase] < 100){
		return;
	}

	if (!isRisingEdge ^ (vsx_cnts[floatPhase] > (vsx_cnts[highPhase]>>1))){
		BEMFcrossing_isr();
	}

}

#define ZCMAfilterBase2 5
static uint32_t prevZCtimes_us[1<<ZCMAfilterBase2];
static uint16_t prevZCtimesInd = 0;
void BEMFcrossing_isr() {
	test = 15;

	// cli();
	if (mTriggerPhase_BEMFdelay == mCurPhase_BEMFdelay) {
		return;
	}
	mTriggerPhase_BEMFdelay = mCurPhase_BEMFdelay;

	uint32_t eventTime_us = ADCreadTime;
	// correction for discrete ADC sampling
	#ifdef useTRIGDELAYCOMPENSATION
		uint32_t triggerDelay_us = period_bemfdelay_usPerTick * abs((int16_t)(vsx_cnts[floatPhase] - (vsx_cnts[highPhase]>>1))) / vsx_cnts[highPhase];
		triggerDelay_us = constrain(triggerDelay_us, 0, (uint32_t)1000000/PWM_FREQ);
		eventTime_us -= triggerDelay_us;
	#endif

	uint32_t elapsedTime_us = eventTime_us - mPrevZCTime_BEMFdelay_us;
	mPrevZCTime_BEMFdelay_us = eventTime_us;
	// period_bemfdelay_usPerTick = (eventTime_us - prevZCtimes_us[prevZCtimesInd]) >> ZCMAfilterBase2;
	period_bemfdelay_usPerTick = elapsedTime_us;
	prevZCtimes_us[prevZCtimesInd++] = eventTime_us;
	prevZCtimesInd %= 1<<ZCMAfilterBase2;

	delayCommutateTimer = eventTime_us;
	delayCommutateDur = (period_bemfdelay_usPerTick>>1) - (((int32_t)(phaseAdvance_Q10 * (period_bemfdelay_usPerTick>>1))) >> 10);
	// sei();
}

#endif