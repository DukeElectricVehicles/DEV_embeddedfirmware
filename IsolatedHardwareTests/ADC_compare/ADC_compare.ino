#include "ADC.h" // download into Libraries folder: https://github.com/pedvide/ADC/tree/776320e7d4913c820bfa0e9407b38d5b6d4f60df
#include "Metro.h"

#define VS_A A2
#define VS_B A10
#define VS_C A3	

// I'm sorry I left this code a mess - Gerry

uint8_t A_val0 = 0, A_val1 = 0;
float cmpVal = 1.0;
bool zeroCrossMode = false, risingEdge;

Metro printTimer(100);

ADC *adc = new ADC(); // adc object
uint8_t value, value2;

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(VS_A, INPUT);
    pinMode(VS_B, INPUT);
    pinMode(VS_C, INPUT);

    Serial.begin(115200);

    Serial.println("Begin setup");

    ///// ADC0 ////
    adc->setAveraging(1); // set number of averages
    adc->setResolution(8); // set bits of resolution
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

	// adc->enableCompare(cmpVal/3.3*adc->getMaxValue(ADC_0), 1, ADC_0); // measurement will be ready if value > cmpVal
    // adc->enableCompareRange(1.0*adc->getMaxValue(ADC_0)/3.3, 2.0*adc->getMaxValue(ADC_0)/3.3, 0, 1, ADC_0); // ready if value lies out of [1.0,2.0] V
    // adc->enableInterrupts(ADC_0);

    adc->startContinuous(VS_A, ADC_0);

    ////// ADC1 /////
    #if ADC_NUM_ADCS>1
    adc->setAveraging(1, ADC_1); // set number of averages
    adc->setResolution(8, ADC_1); // set bits of resolution
    adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED, ADC_1); // change the conversion speed
    adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED, ADC_1); // change the sampling speed
    
    adc->startContinuous(VS_C, ADC_1);
    #endif

    Serial.println("End setup");

}

void loop() {

	if (printTimer.check()){
		Serial.print(A_val0*3.3/adc->getMaxValue(ADC_0));
		Serial.print('\t');
		Serial.println(A_val1*3.3/adc->getMaxValue(ADC_1));
	}

	if (Serial.available()){
		char c = Serial.read();
		switch (c) {
			case 'c': // conversion active?
	            Serial.print("Converting? ADC0: ");
	            Serial.println(adc->isConverting(ADC_0));
	            #if ADC_NUM_ADCS>1
	            Serial.print("Converting? ADC1: ");
	            Serial.println(adc->isConverting(ADC_1));
	            #endif
	            break;
	        case 's': // stop conversion
	            adc->stopContinuous(ADC_0);
	            Serial.println("Stopped");
	            break;
	        case 't': // conversion successful?
	            Serial.print("Conversion successful? ADC0: ");
	            Serial.println(adc->isComplete(ADC_0));
	            #if ADC_NUM_ADCS>1
	            Serial.print("Conversion successful? ADC1: ");
	            Serial.println(adc->isComplete(ADC_1));
	            #endif
	            break;
	        case 'r': // restart conversion
	            Serial.println("Restarting conversions ");
	            adc->disableCompare(ADC_0);
	            adc->startContinuous(VS_A, ADC_0);
	            #if ADC_NUM_ADCS>1
	            adc->startContinuous(VS_C, ADC_1);
	            #endif
	            //adc->startContinuousDifferential(A10, A11, ADC_0);
	            break;
	        case 'v': // value
	            Serial.print("Value ADC0: ");
	            value = (uint16_t)adc->analogReadContinuous(ADC_0); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
	            Serial.println(value*3.3/adc->getMaxValue(ADC_0), DEC);
	            #if ADC_NUM_ADCS>1
	            Serial.print("Value ADC1: ");
	            value2 = (uint16_t)adc->analogReadContinuous(ADC_1); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
	            Serial.println(value2*3.3/adc->getMaxValue(ADC_1), DEC);
	            #endif
	            break;
	        case 'n': // new single conversion on readPin3
	            // this shows how even when both ADCs are busy with continuous measurements
	            // you can still call analogRead, it will pause the conversion, get the value and resume the continuous conversion automatically.
	            Serial.print("Single read on VS_B: ");
	            Serial.println(adc->analogRead(VS_B, ADC_1)*3.3/adc->getMaxValue(ADC_0), DEC);
	            break;
	        case '>': // only update on values > the number entered right after the symbol i.e. >1.2[enter]
	        	cmpVal = Serial.parseFloat();
				adc->enableCompare(cmpVal/3.3*adc->getMaxValue(ADC_0), 1, ADC_0); // measurement will be ready if value > cmpVal
				adc->enableInterrupts(ADC_0);
				Serial.print("Set compare value to ");
				Serial.print(cmpVal * 3.3 / adc->getMaxValue(ADC_0));
				Serial.println("V");
				break;
	        case '<': // only update on values < the number entered right after the symbol i.e. <1.2[enter]
	        	cmpVal = Serial.parseFloat();
				adc->enableCompare(cmpVal/3.3*adc->getMaxValue(ADC_0), 0, ADC_0); // measurement will be ready if value < cmpVal
				adc->enableInterrupts(ADC_0);
				Serial.print("Set compare value to ");
				Serial.print(cmpVal * 3.3 / adc->getMaxValue(ADC_0));
				Serial.println("V");
				break;
			case 'Z': // detect zero crossing for sensorless, assume VS_C is at 100% duty cycle
				zeroCrossMode = !zeroCrossMode;
				if (zeroCrossMode){
					updateZeroCross(adc->analogReadContinuous(ADC_1), adc->analogReadContinuous(ADC_0));
				} else {
					adc->disableCompare(ADC_0);
				}
				break;
		}
	}

}

void updateZeroCross(uint8_t V_bus, uint8_t curVal){
	uint8_t cmpVal = adc->analogRead(VS_C) / 2;
	risingEdge = cmpVal > (curVal);
	adc->enableCompare(cmpVal, risingEdge, ADC_0);
	adc->enableInterrupts(ADC_0);
	Serial.print("Set compare value to ");
	Serial.print(risingEdge ? '>' : '<');
	Serial.print(cmpVal * 3.3 / adc->getMaxValue(ADC_0));
	Serial.println("V");
}
// Make sure to call readSingle() to clear the interrupt.
void adc0_isr() {
    A_val0 = adc->adc0->analogReadContinuous();
    if (zeroCrossMode) {
		Serial.println("zero crossing!");
		updateZeroCross(adc->analogReadContinuous(ADC_1), A_val0);
	}
    //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );
}

#if ADC_NUM_ADCS>1
void adc1_isr() {
    A_val1 = adc->adc1->analogReadContinuous();
    //digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN) );
}
#endif