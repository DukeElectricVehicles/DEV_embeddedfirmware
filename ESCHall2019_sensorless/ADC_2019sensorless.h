#ifndef ADCSENSORLESS_H
#define ADCSENSORLESS_H

#include "est_BEMF_delay.h"
#include "config.h"
#include "est_hall_simple.h"

#define ADC_RES_BITS 12
#define ADC_SAMPLETIME_NS 9000 // manually entered - please adjust from observation
#define ADC_DELAYTIME_NS 15000 //20000
#define ADCSAMPLEBUFFERSIZE 1000

#define MIN_ADCVALID_DUTY ((ADC_SAMPLETIME_NS + ADC_DELAYTIME_NS) * PWM_FREQ / 1e9 * MODULO * 1.2) // 1.2 is safety factor

extern void BEMFdelay_update();

volatile uint32_t ADCreadTime;
volatile uint16_t vsx_cnts[3]; // vsA, vsB, vsC
volatile uint16_t thr_cnts;
volatile uint16_t vsxSamples_cnts[ADCSAMPLEBUFFERSIZE][7];
volatile uint16_t vsxSample_ind = 0;
volatile bool ADCsampleCollecting = false, ADCsampleDone = false;
extern volatile uint32_t period_bemfdelay_usPerTick;
extern volatile uint32_t period_hallsimple_usPerTick;
extern volatile uint16_t duty;
extern volatile commutateMode_t commutateMode;

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
	#if (ADC_DELAYTIME_NS * F_CPU) >= (65535 * 1000000000)
		#error "PDB delay overflow"
	#endif
	PDB0_CH0DLY0 = (uint64_t) ADC_DELAYTIME_NS * F_CPU / 1e9; // 	DELAY FROM PWM TRIGGER
	PDB0_CH0C1 = (0b10 << 16) | (0b01 << 8) | (0b11); // Back-to-back turned on for channel 2,
	  // channel 1 set by its counter, and both channel 1 and 2 outputs turned on
	  // Back-to-back mode means that channel 2 (ADC0 'B' conversion) will start
	  // as soon as the channel 1 ('A' conversion) is completed.
	PDB0_CH1C1 = (0b11 << 16) | (0b00 << 8) | (0b11);
	PDB0_MOD = FTM0_MOD; // Same maximum count as FTM
	PDB0_SC |= PDB_SC_LDOK | PDB_SC_PDBEN; // Turn on the PDB

	attachInterruptVector(IRQ_ADC1, adc_isr); // When IRQ_ADC1 fires, code execution will
											 // jump to "adc_irq()" function.
	NVIC_ENABLE_IRQ(IRQ_ADC1); // ADC complete interrupt
	NVIC_SET_PRIORITY(IRQ_ADC1, 0); // Zero = highest priority

	Serial.println("End ADC setup");
}

uint16_t getThrottle_ADC() {
	test = 24;
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
	// 4 ADC reads take about 9  us @ 12 bit	
	// 						  6.7us @ 10 bit
	//						  2.5us @ 8 bit
	vsx_cnts[0] = ADC0_RA;	//DO NOT COMMENT THESE OUT, reading value changes state
	vsx_cnts[2] = ADC0_RB;	//DO NOT COMMENT THESE OUT, reading value changes state
	vsx_cnts[1] = ADC1_RA;	//DO NOT COMMENT THESE OUT, reading value changes state
	thr_cnts = ADC1_RB;			//DO NOT COMMENT THESE OUT, reading value changes state
	ADCreadTime = micros();
	test = 23;

	if ((duty > MIN_ADCVALID_DUTY) || (duty < (0.001*MODULO))) {
		BEMFdelay_update();
	} else if (commutateMode == MODE_SENSORLESS_DELAY){
        commutateMode = MODE_HALL;
        hallnotISR();
	}

	// if (!ADCsampleDone) { //(ADCsampleCollecting) {
	// 	vsxSamples_cnts[vsxSample_ind][0] = vsx_cnts[highPhase];
	// 	vsxSamples_cnts[vsxSample_ind][1] = vsx_cnts[floatPhase];
	// 	vsxSamples_cnts[vsxSample_ind][2] = vsx_cnts[3-(highPhase+floatPhase)];
	// 	// memcpy((void*)vsxSamples_cnts[vsxSample_ind], (void*)vsx_cnts, sizeof(vsx_cnts));
	// 	vsxSamples_cnts[vsxSample_ind][3] = isRisingEdge;
	// 	vsxSamples_cnts[vsxSample_ind][4] = period_bemfdelay_usPerTick;
	// 	vsxSamples_cnts[vsxSample_ind][5] = delayCommutateFinished;
	// 	vsxSamples_cnts[vsxSample_ind++][6] = micros();
	// 	if (vsxSample_ind >= ADCSAMPLEBUFFERSIZE) {
	// 		if (ADCsampleCollecting) {
	// 			ADCsampleCollecting = false;
	// 			ADCsampleDone = true;
	// 		}
	// 		vsxSample_ind = 0;
	// 	}
	// }
}

#endif