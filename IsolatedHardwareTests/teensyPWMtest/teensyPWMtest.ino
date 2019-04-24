#define TPM_C 48000000            // core clock, for calculation only
#define PWM_FREQ 1000            //  PWM frequency [Hz]
#define MODULO (TPM_C / PWM_FREQ) // calculation the modulo for FTM0

int PWMcounter = MODULO/3;
unsigned long prevTime;

void setup() {
  Serial.println("Beginning");
  prevTime = millis();
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(7, OUTPUT); // DRV enable
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(10,OUTPUT);

  // PWM setup
  /*  To use this, first figure out what pins you want to use.  They have to be on the same timer.  Refer to the table here:
        https://pjrc.com/teensy/td_pulse.html
      Now, check the Teensy schematic to find what pin names they are:
        https://www.pjrc.com/teensy/schematic.html
      For example, if you want to have complementary outputs for pins 10 and 20, we find that they are:
        pin 10: PTC4
        pin 9: PTC3
      Now, go to the manual:
        https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf
      Page 207:
        PTC4 is in row 49 and is FTM0 CH 3
        PTC3 is in row 46 and is FTM0 CH 2

      Another example:
        pin 6: PTD4 - row 61 FTM0_CH4
        pin 20: PTD5 - row 62 FTM0_CH5

  */
  FTM0_MODE = 0x04;              // Disable write protection
    //  FTM0_POL = 0;                  // Positive Polarity 
  FTM0_OUTMASK = 0xFF;           // Use mask to disable outputs while configuring
  FTM0_SC = 0x08 | 0x00;         // set system clock as source for FTM0
  FTM0_MOD = MODULO;             // Period register (max counter value)
  FTM0_CNTIN = 0;                // Counter initial value (optional, for alignment)
  
  FTM0_COMBINE = 0x00000033;     // COMBINE=1, COMP=1, DTEN=1, SYNCEN=1 for channels 0/1   // page 796  (COMP1 sets complement)
  FTM0_SYNC = 0x02;              // PWM sync @ max loading point enable (set trigger to end once it hits the value)
  FTM0_DEADTIME = 0b00<<6;       // DeadTimer prescale systemClk/1                 // page 801
  FTM0_DEADTIME |= 0b000000;     // 1uS DeadTime, max of 63 counts of 48Mhz clock  // page 801
  FTM0_C0V = 0;                  // This specifies where the trigger starts 
  FTM0_C1V = 0;                  // This specifies where the trigger ends (init to 0 for safety)
  FTM0_SYNC |= 0x80;             // set PWM value update
  FTM0_C0SC = 0x28;              // PWM output, edge aligned, positive signal
  // FTM0_C1SC = 0x28;              // PWM output, edge aligned, positive signal
    

  /*  For the next 2 lines, we need to figure out which "alternate function" we should mux the pin output to.
      First, refer to the schematic here:
        https://www.pjrc.com/teensy/schematic.html
      or the teensy header file here:
        https://github.com/PaulStoffregen/cores/blob/master/teensy3/core_pins.h
      to find out which port and channel the pin is on
        i.e. PIN22 is PTC1 - i.e. port C channel 1
      Then go to page 207 of the manual (https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf) and find the "pin name"
        i.e. PTC1 is in row 44
      Finally, find the column that maps the pin to the FTM peripheral.
        i.e. FTM0_CH0 for PTC1 is in column ALT4, that means use PORT_PCR_MUX(4)
      Always or with | PORT_PCR_DSE | PORT_PCR_SRE for outputting - not sure why or what these are
  */
  CORE_PIN22_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;    //config teensy output port pins
  CORE_PIN23_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE;   //config teensy output port pins

   FTM0_OUTMASK = 0x0;            // Re-enables PWM output
   
//   FTM0_C1V = MODULO*(1); // sets both low and high side
//   FTM0_SYNC |= 0x80;

    FTM0_C1V = PWMcounter;        // these two lines are how you set the duty cycle
    FTM0_SYNC |= 0x80;

    FTM0_MODE = 0x01;              // Enable FTM0
    FTM0_SYNC |= 0x80;

    digitalWrite(7, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
//  if ((millis() % 1000) == 0){
//    Serial.println("HI");
  if((millis()-prevTime) > 50){
    prevTime = millis();
    FTM0_C1V = PWMcounter;
    FTM0_SYNC |= 0x80;             // set PWM value update
    PWMcounter += MODULO/100;
    if (PWMcounter > MODULO){
      PWMcounter = 0;
    }
    Serial.print("updated PWM counter to "); Serial.println(PWMcounter);
  }
//  }

//  if ((millis()-prevTime) > 1000){
//    prevTime = millis();
//    FTM0_OUTMASK = 0x03 & (~(FTM0_OUTMASK & 3));
//  }
  
  //digitalWrite(10, !digitalRead(10));
//  delay(10);
}
