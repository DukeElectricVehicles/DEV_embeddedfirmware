#ifndef ADCSENSORLESS_H
#define ADCSENSORLESS_H

#include "ADC.h" // download into Libraries folder: https://github.com/pedvide/ADC/tree/776320e7d4913c820bfa0e9407b38d5b6d4f60df
#include "est_BEMF_delay.h"

ADC *adc = new ADC(); // adc object

#define ADC_RES_BITS 8
uint8_t floatPhase, highPhase;
uint8_t curPhase_ADC;
float duty = 0;
uint16_t cmpVal;

extern void BEMFcrossing_isr();

static const uint8_t floatPhases[6] = {VS_C, VS_A, VS_B, VS_C, VS_A, VS_B};
static const uint8_t highPhases[6] = {VS_A, VS_C, VS_C, VS_B, VS_B, VS_A};
static const bool isRisingEdges[6] = {true, false, true, false, true, false};
/*
  switch(pos){
    case 0://HIGH A, LOW B
      writeHigh(0b001);
      writeLow( 0b010);
      break;
    case 1://HIGH C, LOW B
      writeHigh(0b100);
      writeLow( 0b010);
      break;
    case 2://HIGH C, LOW A
      writeHigh(0b100);
      writeLow( 0b001);
      break;
    case 3://HIGH B, LOW A
      writeHigh(0b010);
      writeLow( 0b001);
      break;
    case 4://HIGH B, LOW C
      writeHigh(0b010);
      writeLow( 0b100);
      break;
    case 5://HIGH A, LOW C
      writeHigh(0b001);
      writeLow( 0b100);
      break;
    default:
      writeHigh(0b000);
      writeLow (0b000);
      break;
  }
*/

void updateZeroCross(uint8_t V_bus, bool isRisingEdge);
void updateCmp_ADC();

void setupADC(){
    pinMode(VS_A, INPUT);
    pinMode(VS_B, INPUT);
    pinMode(VS_C, INPUT);
    #ifdef VS_AOLD
    pinMode(VS_AOLD, INPUT);
    #endif
    #ifdef VS_COLD
    pinMode(VS_COLD, INPUT);
    #endif
    pinMode(13, OUTPUT);

    ///// ADC0 ////
    adc->setAveraging(32); // set number of averages
    adc->setResolution(ADC_RES_BITS); // set bits of resolution
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

	// adc->enableCompare(cmpVal/3.3*adc->getMaxValue(ADC_0), 1, ADC_0); // measurement will be ready if value > cmpVal
    // adc->enableCompareRange(1.0*adc->getMaxValue(ADC_0)/3.3, 2.0*adc->getMaxValue(ADC_0)/3.3, 0, 1, ADC_0); // ready if value lies out of [1.0,2.0] V
    // adc->enableInterrupts(ADC_0);

    adc->startContinuous(VS_A, ADC_0);

    ////// ADC1 /////
    #if ADC_NUM_ADCS>1
    adc->setAveraging(1, ADC_1); // set number of averages
    adc->setResolution(ADC_RES_BITS, ADC_1); // set bits of resolution
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED, ADC_1); // change the conversion speed
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED, ADC_1); // change the sampling speed
    
    adc->startContinuous(VS_C, ADC_1);
    #endif

    Serial.println("End setup");
}

uint16_t getThrottle_ADC() {
	static uint16_t prevThrot = 0;
	uint16_t toRet = (adc->analogRead(THROTTLE, ADC_0)) << (10 - ADC_RES_BITS);
	if (toRet > (1<<10)){
		return prevThrot;
	}
	prevThrot = toRet;
	return toRet;
}
// IntervalTimer postSwitchDelayTimer;
extern volatile uint32_t timeToUpdateCmp;
void updatePhase_ADC(uint8_t drivePhase) {
	if (curPhase_ADC != drivePhase){
		curPhase_ADC = drivePhase;
		if (curPhase_ADC >= 6){
			return;
		}
		// adc->disableCompare(ADC_0);
		// adc->startContinuous(floatPhases[curPhase_ADC], ADC_0);
		adc->startContinuous(highPhases[curPhase_ADC], ADC_1);
		timeToUpdateCmp = micros() + min(period_us_per_tick/10, 1000);
		// bool suc = postSwitchDelayTimer.begin(updateCmp_ADC, min(period_us_per_tick/10, 1000));
	}
}
void updateDuty_ADC(float duty_) {
	duty = duty_;
}
void updateCmp_ADC() {
	// postSwitchDelayTimer.end();
	updateZeroCross(adc->analogReadContinuous(ADC_1),isRisingEdges[curPhase_ADC]);
	// Serial.println("updated compare");
}

void updateZeroCross(uint8_t V_bus, bool isRisingEdge){
	cmpVal = V_bus * duty / 2;
	adc->enableCompare(cmpVal, isRisingEdge, ADC_0);
	adc->enableInterrupts(ADC_0);
	adc->startContinuous(floatPhases[curPhase_ADC], ADC_0);
}
// Make sure to call readSingle() to clear the interrupt.
void adc0_isr() {
	static volatile bool LEDon;
    adc->adc0->analogReadContinuous();
    // BEMFcrossing_isr();
    adc->disableInterrupts(ADC_0); // only catch zero crossing once
    digitalWriteFast(13, LEDon);
    LEDon = !LEDon;
}

#if ADC_NUM_ADCS>1
// void adc1_isr() {
//     adc->adc1->analogReadContinuous();
//     //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );
// }
#endif

#endif