
#define INLA 22
#define INHA 23
#define INLB 9
#define INHB 10
#define INLC 6
#define INHC 20

#define pin_pwm_pha INHA
#define pin_pwm_phb INHB
#define pin_pwm_phc INHC


#define TPM_C 48000000            // core clock, for calculation only
#define PWM_FREQ 12000            //  PWM frequency [Hz]
#define MODULO (TPM_C / PWM_FREQ) // calculation the modulo for FTM0

#define PRESCALE 0b10
#define DEADTIME 0b100010


void setup() {


    FTM0_OUTINIT = 0;              // initialize to low
    FTM0_MODE = 0x04;              // Disable write protection
    FTM0_OUTMASK = 0xFF;           // Use mask to disable outputs while configuring
    FTM0_SC = 0x08 | 0x00;         // set system clock as source for FTM0
    FTM0_MOD = MODULO;             // Period register (max counter value)

    // FTM0_CNTIN = 0;                // Counter initial value (for alignment of ADC trigger)
    FTM0_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN; // for ADC (trigger on center pulse)
    // FTM0_EXTTRIG |= FTM_EXTTRIG_CH2TRIG;

    // add channels as needed here
    FTM0_COMBINE  = 0x00000033;    // COMBINE=1, COMP=1, DTEN=1, SYNCEN=1 for channels 0/1   // page 796  (COMP1 sets complement)
    FTM0_COMBINE |= 0x00003300;    // CH 2/3
    FTM0_COMBINE |= 0x00330000;    // CH 4/5
    FTM0_POL      = 0b00000000;    // Polarity - use this to invert signals (can take the functionality of COMP signal in COMBINE)
                                   // but preferably use the CxSC values instead since POL defines the "safe value"

    FTM0_SYNC = 0x02;              // PWM sync @ max loading point enable (set trigger to end once it hits the value)
    FTM0_SYNC |= 0x08;             // PWM sync outmask as well
    FTM0_DEADTIME = PRESCALE<<6;   // DeadTimer prescale systemClk/1                 // page 801
    FTM0_DEADTIME |= DEADTIME;     // 1uS DeadTime, max of 63 counts of 48Mhz clock  // page 801
    
    FTM0_C0V = 0;                  // This specifies where the trigger starts 
    FTM0_C1V = 0;                  // This specifies where the trigger ends (init to 0 for safety)
    FTM0_C2V = 0;
    FTM0_C3V = 0;
    FTM0_C4V = 0;
    FTM0_C5V = 0;
    FTM0_SYNC |= 0x80;             // set PWM value update
    FTM0_C0SC = 0x24;              // PWM output, edge aligned (ignored by combine), positive signal
    // FTM0_C1SC = 0x28;              // PWM output, edge aligned (ignored by combine), negative signal
    FTM0_C2SC = 0x24;              // PWM output, edge aligned (ignored by combine), positive signal
    // FTM0_C3SC = 0x28;              // PWM output, edge aligned (ignored by combine), negative signal
    FTM0_C4SC = 0x24;              // PWM output, edge aligned (ignored by combine), positive signal
    // FTM0_C5SC = 0x28;              // PWM output, edge aligned (ignored by combine), negative signal

    /*  For the next 2 lines, we need to figure out which "alternate function" we should mux the pin output to.
        From page 207 of the manual, find the column that maps the pin to the FTM peripheral.
          i.e. pin 22: FTM0_CH0 for PTC1 is in column ALT4, that means use PORT_PCR_MUX(4)
        Always OR with | PORT_PCR_DSE | PORT_PCR_SRE for outputting - not sure why or what these are
          pin 22: PTC1 - row 44, FTM0_CH0 is ALT4
          pin 23: PTC2 - row 45, FTM0_CH1 is ALT4
          pin 9:  PTC3 - row 46, FTM0_CH2 is ALT4
          pin 10: PTC4 - row 49, FTM0_CH3 is ALT4
          pin 6:  PTD4 - row 61, FTM0_CH4 is ALT4
          pin 20: PTD5 - row 62, FTM0_CH5 is ALT4
    */
    CORE_PIN22_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;    //config teensy output port pins
    CORE_PIN23_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;   //config teensy output port pins
    CORE_PIN9_CONFIG  = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
    CORE_PIN10_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
    CORE_PIN6_CONFIG  = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;
    CORE_PIN20_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;

    FTM0_OUTMASK = 0x0;            // Re-enables PWM output by "opening the mask"
    FTM0_MODE = 0x01;              // Enable FTM0 and reenable write protection
    FTM0_SYNC |= 0x80;

  // // FTM0 settings
  // // f_pwm = 20kHz
  // // up-down counting mode
  // // trigger out = FTM0_CH2 (channel unused for PWM generation)
  // analogWriteRes(16);
  // analogWriteFrequency(pin_pwm_pha, 20000*2); // Up-down mode screws with us, double the freq works
  // // FTM_Mod_Val = FTM0_MOD; // Make it easier on ourselves for later, just save this value                       
  // FTM0_SC |= FTM_SC_CPWMS; // Turn on center aligned mode
  // FTM0_EXTTRIG |= FTM_EXTTRIG_CH2TRIG; // Output trigger = Channel 2 compare flag (CH2 not used for PWM)
  // FTM0_C2V = FTM0_MOD; // Channel 2 will fire at the top of the PWM up-down count,
                          // when PWM waveform is at its low level

  // ADC settings
  analogReadRes(12); // Set 12-bit ADC resolution (default is 10)
  ADC0_SC2 = ADC_SC2_ADTRG; // Hardware triggered (comes from PDB)
  ADC0_SC3 = 0; // Not continuous, no averaging
  ADC0_SC1A = 8; // Channel A set to measure A-phase current, channel 8, ARM pin 35, teensy pin 16
  ADC0_SC1B = 9 | ADC_SC1_AIEN; // Interrupt enabled for channel B, 
                                       // and it measures B-phase current, ARM pin 36, teensy pin 17
 
  // Programmable Delay Block settings
  // Clock it first! If the PDB clock isn't turned on, the processor crashes when
  // accessing PDB registers.
  SIM_SCGC6 |= SIM_SCGC6_PDB; // Enable PDB in System Integration Module
  
  PDB0_SC = PDB_SC_TRGSEL(8); // FTM0 trigger input selected (which was set to FTM0_CH2)
  PDB0_CH0DLY0 = 1; // Almost immediately trigger the first ADC conversion
  PDB0_CH0C1 = (2 << 16) | (1 << 8) | (3); // Back-to-back turned on for channel 2,
      // channel 1 set by its counter, and both channel 1 and 2 outputs turned on
      // Back-to-back mode means that channel 2 (ADC0 'B' conversion) will start
      // as soon as the channel 1 ('A' conversion) is completed.
  PDB0_MOD = FTM0_MOD; // Same maximum count as FTM
  PDB0_SC |= PDB_SC_LDOK | PDB_SC_PDBEN; // Turn on the PDB
  
  attachInterruptVector(IRQ_ADC0, adc0_irq); // When IRQ_ADC0 fires, code execution will
                                             // jump to "adc0_irq()" function.
  NVIC_ENABLE_IRQ(IRQ_ADC0); // ADC complete interrupt
  NVIC_SET_PRIORITY(IRQ_ADC0, 10); // Zero = highest priority

  
  analogWrite(pin_pwm_pha, 1);
  analogWrite(pin_pwm_phb, 1);
  analogWrite(pin_pwm_phc, 1);
  // Immediately change to 0% duty cycle
  FTM0_C1V = 0;
  FTM0_C0V = 0;
  FTM0_C6V = 0;
}

void loop() {

}

void adc0_irq() {
    volatile int16_t phA_current = ADC0_RA - 2054;//DO NOT COMMENT THESE OUT, reading value changes state
  volatile int16_t phB_current = ADC0_RB - 2050;

  Serial.println("hello");
}