#ifndef ADCSENSORLESS_H
#define ADCSENSORLESS_H

// #include "ADC.h" // download into Libraries folder: https://github.com/pedvide/ADC/tree/776320e7d4913c820bfa0e9407b38d5b6d4f60df
#include "est_BEMF_delay.h"

// ADC *adc = new ADC(); // adc object

#define ADC_RES_BITS 8
volatile uint8_t floatPhase, highPhase;
volatile bool isRisingEdge;
volatile uint8_t curPhase_ADC;
float duty = 0;
uint16_t cmpVal;
volatile bool cmpOn = false;

extern volatile void BEMFcrossing_isr();

volatile uint16_t vsx_cnts[3]; // vsA, vsB, vsC
static const uint8_t floatPhases[6] = {2, 0, 1, 2, 0, 1};
static const uint8_t highPhases[6]  = {0, 2, 2, 1, 1, 0};
// static const uint8_t floatPhases[6] = {VS_C, VS_A, VS_B, VS_C, VS_A, VS_B};
// static const uint8_t highPhases[6] = {VS_A, VS_C, VS_C, VS_B, VS_B, VS_A};
static const bool isRisingEdges[6] = {true, false, true, false, true, false};

volatile uint16_t thr_cnts;
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

void adc_isr();
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

	// ADC settings
	// Let:
	// 	ADC0_A be phase A (A7 = ADC0_SE7b = channel 7)
	// 	ADC0_B be phase C (A11 = ADC0_DM0 = channel 19)
	// 	ADC1_A be phase B (A10 = ADC0_DP0 / ADC1_DP3 = channel 3)
	// 	ADC1_B be throttle (A18 = ADC1_SE6b = channel 6)
	analogReadRes(12); // Set 12-bit ADC resolution (default is 10)
	ADC0_SC2 = ADC_SC2_ADTRG; // Hardware triggered (comes from PDB)
	ADC0_SC3 = 0; // Not continuous, no averaging, page 664
	ADC1_SC2 = ADC_SC2_ADTRG;
	ADC1_SC3 = 0;

	// page 98
	ADC0_SC1A = 7;// | ADC_SC1_AIEN;  // ADCx_CFG2[MUXSEL] bit selects between ADCx_SEn channels a and b. Refer to MUXSEL description in ADC chapter for details
	ADC0_SC1B = 19;// | ADC_SC1_AIEN;
	ADC1_SC1A = 3; // interrupt enabled - throttle is not time sensitive so just IRQ here instead
	ADC1_SC1B = 6 | ADC_SC1_AIEN;

	ADC0_CFG2 |= ADC_CFG2_MUXSEL; // for SE7b
	ADC1_CFG2 |= ADC_CFG2_MUXSEL; // for SE6b

	// Programmable Delay Block settings
	// Clock it first! If the PDB clock isn't turned on, the processor crashes when
	// accessing PDB registers.
	SIM_SCGC6 |= SIM_SCGC6_PDB; // Enable PDB in System Integration Module

	PDB0_SC = PDB_SC_TRGSEL(8); // FTM0 trigger input selected (which was set to FTM0_CH2)
	PDB0_CH0DLY0 = 596; // Almost immediately trigger the first ADC conversion
	PDB0_CH0C1 = (0b10 << 16) | (0b01 << 8) | (0b11); // Back-to-back turned on for channel 2,
	  // channel 1 set by its counter, and both channel 1 and 2 outputs turned on
	  // Back-to-back mode means that channel 2 (ADC0 'B' conversion) will start
	  // as soon as the channel 1 ('A' conversion) is completed.
	PDB0_CH1C1 = (0b11 << 16) | (0b00 << 8) | (0b11);
	PDB0_MOD = FTM0_MOD; // Same maximum count as FTM
	PDB0_SC |= PDB_SC_LDOK | PDB_SC_PDBEN; // Turn on the PDB

	attachInterruptVector(IRQ_ADC1, adc_isr); // When IRQ_ADC0 fires, code execution will
											 // jump to "adc0_irq()" function.
	NVIC_ENABLE_IRQ(IRQ_ADC1); // ADC complete interrupt
	NVIC_SET_PRIORITY(IRQ_ADC1, 10); // Zero = highest priority

 //    ///// ADC0 ////
 //    adc->setAveraging(32); // set number of averages
 //    adc->setResolution(ADC_RES_BITS); // set bits of resolution
 //    adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
 //    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

	// // adc->enableCompare(cmpVal/3.3*adc->getMaxValue(ADC_0), 1, ADC_0); // measurement will be ready if value > cmpVal
 //    // adc->enableCompareRange(1.0*adc->getMaxValue(ADC_0)/3.3, 2.0*adc->getMaxValue(ADC_0)/3.3, 0, 1, ADC_0); // ready if value lies out of [1.0,2.0] V
 //    // adc->enableInterrupts(ADC_0);

 //    adc->startContinuous(VS_A, ADC_0);

 //    ////// ADC1 /////
 //    #if ADC_NUM_ADCS>1
 //    adc->setAveraging(1, ADC_1); // set number of averages
 //    adc->setResolution(ADC_RES_BITS, ADC_1); // set bits of resolution
 //    adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED, ADC_1); // change the conversion speed
 //    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED, ADC_1); // change the sampling speed
	
 //    adc->startContinuous(VS_C, ADC_1);
 //    #endif

	Serial.println("End setup");
}

uint16_t getThrottle_ADC() {
	static uint16_t prevThrot = 0;
	// uint16_t toRet = (adc->analogRead(THROTTLE, ADC_0)) << (10 - ADC_RES_BITS);
	uint16_t toRet = thr_cnts >> 2; // analog read res is 12 bit
	if (toRet > (1<<10)){
		return prevThrot;
	}
	prevThrot = toRet;
	return toRet;
}
// IntervalTimer postSwitchDelayTimer;
extern volatile uint32_t timeToUpdateCmp;
extern volatile uint32_t period_commutation_usPerTick;
void updatePhase_ADC(uint8_t drivePhase) {
	if (curPhase_ADC != drivePhase){
		curPhase_ADC = drivePhase;
		if (curPhase_ADC >= 6){
			return;
		}
		timeToUpdateCmp = micros() + min(period_commutation_usPerTick/10, 10);
		cmpOn = false;

		floatPhase = floatPhases[curPhase_ADC];
		highPhase = highPhases[curPhase_ADC];
		isRisingEdge = isRisingEdges[curPhase_ADC] ^ (!dir);
		// adc->disableCompare(ADC_0);
		// adc->startContinuous(floatPhases[curPhase_ADC], ADC_0);
		// adc->startContinuous(highPhases[curPhase_ADC], ADC_1);
		// bool suc = postSwitchDelayTimer.begin(updateCmp_ADC, min(period_commutation_usPerTick/10, 1000));
	}
}
void updateCmp_ADC() {
	cmpOn = true;
}

// Make sure to call readSingle() to clear the interrupt.
void adc_isr() {
	 //    volatile int16_t phA_current = ADC0_RA - 2054;//DO NOT COMMENT THESE OUT, reading value changes state
  // volatile int16_t phB_current = ADC0_RB - 2050;

  // Serial.println("hello");

	// adc->adc0->analogReadContinuous();
	// // BEMFcrossing_isr();
	// adc->disableInterrupts(ADC_0); // only catch zero crossing once

  // vsx_cnts[0] = phA_current+2054;	//DO NOT COMMENT THESE OUT, reading value changes state
  // vsx_cnts[2] = phB_current+2050;	//DO NOT COMMENT THESE OUT, reading value changes state
  vsx_cnts[0] = ADC0_RA;	//DO NOT COMMENT THESE OUT, reading value changes state
  vsx_cnts[2] = ADC0_RB;	//DO NOT COMMENT THESE OUT, reading value changes state
  vsx_cnts[1] = ADC1_RA;	//DO NOT COMMENT THESE OUT, reading value changes state
  thr_cnts = ADC1_RB;			//DO NOT COMMENT THESE OUT, reading value changes state
  // while (true){
  	// Serial.println("YAY");
  // 	delay(10);
  // }

  if (!cmpOn || (vsx_cnts[floatPhase] < 500)){
  	return;
  }

  volatile bool floatGt0 = (vsx_cnts[floatPhase] > (vsx_cnts[highPhase]/2)); // don't think i need to multiply by duty since it's properly phase aligned
  if (floatGt0 ^ !isRisingEdge){
  	BEMFcrossing_isr();



		static volatile int16_t LEDon;
		#define LED_DIV 1
		digitalWriteFast(13, HIGH);
		// LEDon = !LEDon;
		LEDon ++;
		LEDon %= LED_DIV*2;
  }
  else {
  	digitalWriteFast(13, LOW);
  }
}

#if ADC_NUM_ADCS>1
// void adc1_isr() {
//     adc->adc1->analogReadContinuous();
//     //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );
// }
#endif

#endif