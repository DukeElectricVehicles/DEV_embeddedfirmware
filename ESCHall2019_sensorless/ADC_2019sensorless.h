#ifndef ADCSENSORLESS_H
#define ADCSENSORLESS_H

#include "est_BEMF_delay.h"

#define ADC_RES_BITS 12
#define ADCSAMPLEBUFFERSIZE 1000

extern volatile void BEMFdelay_update();

volatile uint16_t vsx_cnts[3]; // vsA, vsB, vsC
volatile uint16_t thr_cnts;
volatile uint16_t vsxSamples_cnts[ADCSAMPLEBUFFERSIZE][6];
volatile uint16_t vsxSample_ind = 0;
volatile bool ADCsampleCollecting = false, ADCsampleDone = false;
extern volatile uint32_t period_bemfdelay_usPerTick;
extern volatile uint32_t period_hallsimple_usPerTick;
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
	analogReadRes(ADC_RES_BITS); // Set 12-bit ADC resolution (default is 10)
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

	Serial.println("End ADC setup");
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

// Make sure to read registers to clear the interrupt.
void adc_isr() {
  vsx_cnts[0] = ADC0_RA;	//DO NOT COMMENT THESE OUT, reading value changes state
  vsx_cnts[2] = ADC0_RB;	//DO NOT COMMENT THESE OUT, reading value changes state
  vsx_cnts[1] = ADC1_RA;	//DO NOT COMMENT THESE OUT, reading value changes state
  thr_cnts = ADC1_RB;			//DO NOT COMMENT THESE OUT, reading value changes state
  
  BEMFdelay_update(vsx_cnts);

  if (ADCsampleCollecting) {
  	vsxSamples_cnts[vsxSample_ind][0] = vsx_cnts[highPhase];
  	vsxSamples_cnts[vsxSample_ind][1] = vsx_cnts[floatPhase];
  	vsxSamples_cnts[vsxSample_ind][2] = vsx_cnts[3-(highPhase+floatPhase)];
  	// memcpy((void*)vsxSamples_cnts[vsxSample_ind], (void*)vsx_cnts, sizeof(vsx_cnts));
  	vsxSamples_cnts[vsxSample_ind][3] = isRisingEdge;
  	vsxSamples_cnts[vsxSample_ind][4] = period_bemfdelay_usPerTick;
  	vsxSamples_cnts[vsxSample_ind++][5] = micros();
  	if (vsxSample_ind >= ADCSAMPLEBUFFERSIZE) {
  		ADCsampleCollecting = false;
  		ADCsampleDone = true;
  		vsxSample_ind = 0;
  	}
  }
}

#endif