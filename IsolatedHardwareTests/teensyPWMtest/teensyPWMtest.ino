#define TPM_C 48000000            // core clock, for calculation only
static uint16_t PWM_FREQ = 1000;            //  PWM frequency [Hz]
static uint16_t MODULO = (TPM_C / PWM_FREQ); // calculation the modulo for FTM0

int PWMcounter = MODULO/3;
unsigned long prevTime;

char serialBuffer[100];
uint8_t serialBufferInd = 0;
uint8_t prescale = 0b10;
uint8_t deadtime = 0b100010;

typedef enum {
  STATE_DUMMYTEST,
  STATE_MOTORTEST
} state_t;
state_t state;

void setup() {
  Serial.println("Beginning");
  state = STATE_MOTORTEST;
  prevTime = millis();
  pinMode(7, OUTPUT); // DRV enable

  setupPWM();

  digitalWrite(7, HIGH);
}

void loop() {

  if (state == STATE_DUMMYTEST){
    if((millis()-prevTime) > 50){
      prevTime = millis();
      writePWM(PWMcounter, PWMcounter/2, MODULO-PWMcounter);
      PWMcounter += MODULO/100;
      if (PWMcounter > (MODULO*.9)){
        PWMcounter = MODULO*.1;
      }
      Serial.print("updated PWM counter to "); Serial.println(PWMcounter);
    }
  }

  if (Serial.available()){
    char input = Serial.read();
    switch(input){
      case '0': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
        serialBuffer[serialBufferInd] = input;
        serialBufferInd++;
        break;
      case 'P':
        serialBuffer[serialBufferInd] = '\0';
        serialBufferInd = 0;
        prescale = atoi(serialBuffer);
        switch (prescale){
          case 0b00: case 0b10: case 0b11:
            break;
          default:
            prescale = 0b00;
        }
        FTM0_DEADTIME = (prescale << 6) | deadtime;
        break;
      case 'T':
        serialBuffer[serialBufferInd] = '\0';
        serialBufferInd = 0;
        deadtime = atoi(serialBuffer);
        if (deadtime > 0b111111){
          deadtime = 0b111111;
        }
        FTM0_DEADTIME = (prescale << 6) | deadtime;
        break;
      case 'F':
        serialBuffer[serialBufferInd] = '\0';
        serialBufferInd = 0;
        PWM_FREQ = constrain(atoi(serialBuffer),733,1000000);
        MODULO = (TPM_C / PWM_FREQ);
        FTM0_MOD = MODULO;
        break;
      case 'm':
        state = STATE_MOTORTEST;
        serialBuffer[serialBufferInd] = '\0';
        serialBufferInd = 0;
        writeTrap(.05*MODULO, atoi(serialBuffer));
        break;
      case 'd':
        state = STATE_DUMMYTEST;
        break;
    }
  }
}

void setupPWM(){
  // PWM setup
  /*  To use this, first figure out what pins you want to use.  They have to be on the same timer.  Refer to the table here:
        https://pjrc.com/teensy/td_pulse.html
      Now, check the Teensy schematic to find what pin names they are:
        https://www.pjrc.com/teensy/schematic.html
        or the teensy header file here:
          https://github.com/PaulStoffregen/cores/blob/master/teensy3/core_pins.h
      For example, if you want to have complementary outputs for pins 9 and 10, we find that they are:
        pin 10: PTC4
        pin 9: PTC3
      Now, go to page 207 of the manual:
        https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf
      Page 207:
        PTC4 is in row 49 and is FTM0 CH 3
        PTC3 is in row 46 and is FTM0 CH 2
      Note that complementary/combined inputs have to have adjacent channels of the form
        2n and 2n+1
      Another example:
        pin 6: PTD4 - row 61 FTM0_CH4
        pin 20: PTD5 - row 62 FTM0_CH5
      Another example:
        pin 22: PTC1 - row 44 FTM0_CH0
        pin 23: PTC2 - row 45 FTM0_CH1
  */
  FTM0_OUTINIT = 0;              // initialize to low
  FTM0_MODE = 0x04;              // Disable write protection
  FTM0_OUTMASK = 0xFF;           // Use mask to disable outputs while configuring
  FTM0_SYNC |= 0x80;             // syncronize outmask update with sync signal
  FTM0_SC = 0x08 | 0x00;         // set system clock as source for FTM0
  FTM0_MOD = MODULO;             // Period register (max counter value)
  FTM0_CNTIN = 0;                // Counter initial value (optional, for alignment)

  // add channels as needed here
  FTM0_COMBINE  = 0x00000033;    // COMBINE=1, COMP=1, DTEN=1, SYNCEN=1 for channels 0/1   // page 796  (COMP1 sets complement)
  FTM0_COMBINE |= 0x00003300;    // CH 2/3
  FTM0_COMBINE |= 0x00330000;    // CH 4/5
  FTM0_POL      = 0b00000000;    // Polarity - use this to invert signals (can take the functionality of COMP signal in COMBINE)
                                 // but preferably use the CxSC values instead since POL defines the "safe value"

  FTM0_SYNC = 0x02;              // PWM sync @ max loading point enable (set trigger to end once it hits the value)
  FTM0_DEADTIME = prescale<<6;       // DeadTimer prescale systemClk/1                 // page 801
  FTM0_DEADTIME |= deadtime;     // 1uS DeadTime, max of 63 counts of 48Mhz clock  // page 801
  
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

}

void writePWM(uint16_t A, uint16_t B, uint16_t C){
  A = constrain(A, 0, MODULO);
  B = constrain(B, 0, MODULO);
  C = constrain(C, 0, MODULO);
  FTM0_C0V = MODULO/2 - A/2; // be careful trying to combine these,
  FTM0_C1V = MODULO/2 + A/2; // uint16 overflow may occur
  FTM0_C2V = MODULO/2 - B/2;
  FTM0_C3V = MODULO/2 + B/2;
  FTM0_C4V = MODULO/2 - C/2;
  FTM0_C5V = MODULO/2 + C/2;
  FTM0_SYNC |= 0x80;             // update
}

void writeTrap(uint16_t throttle, uint8_t phase){
  throttle = constrain(throttle, 0, MODULO);
  switch (phase){
    case 0:   //HIGH A, LOW C
      FTM0_C0V = MODULO/2 - throttle/2;
      FTM0_C1V = MODULO/2 + throttle/2;
      FTM0_OUTMASK = 0b001100;
      FTM0_C4V = 0;
      FTM0_C5V = 0;
      FTM0_SYNC |= 0x80;
      break;
    case 1:   //HIGH B, LOW C
      FTM0_OUTMASK = 0b000011;
      FTM0_C2V = MODULO/2 - throttle/2;
      FTM0_C3V = MODULO/2 + throttle/2;
      FTM0_C4V = 0;
      FTM0_C5V = 0;
      FTM0_SYNC |= 0x80;
      break;
    case 2:   //HIGH B, LOW A
      FTM0_C0V = 0;
      FTM0_C1V = 0;
      FTM0_C2V = MODULO/2 - throttle/2;
      FTM0_C3V = MODULO/2 + throttle/2;
      FTM0_OUTMASK = 0b110000;
      FTM0_SYNC |= 0x80;
      break;
    case 3:   //HIGH C, LOW A
      FTM0_C0V = 0;
      FTM0_C1V = 0;
      FTM0_OUTMASK = 0b001100;
      FTM0_C4V = MODULO/2 - throttle/2;
      FTM0_C5V = MODULO/2 + throttle/2;
      FTM0_SYNC |= 0x80;
      break;
    case 4:   //HIGH C, LOW B
      FTM0_OUTMASK = 0b000011;
      FTM0_C2V = 0;
      FTM0_C3V = 0;
      FTM0_C4V = MODULO/2 - throttle/2;
      FTM0_C5V = MODULO/2 + throttle/2;
      FTM0_SYNC |= 0x80;
      break;
    case 5:   //HIGH A, LOW B
      FTM0_C0V = MODULO/2 - throttle/2;
      FTM0_C1V = MODULO/2 + throttle/2;
      FTM0_C2V = 0;
      FTM0_C3V = 0;
      FTM0_OUTMASK = 0b110000;
      FTM0_SYNC |= 0x80;
      break;
    default:
      FTM0_OUTMASK = 0xFF;
      FTM0_SYNC |= 0x80;
      break;
  }
  Serial.print("wrote ");
  Serial.print(throttle);
  Serial.print('/');
  Serial.print(MODULO);
  Serial.print(" at phase ");
  Serial.print(phase);
  Serial.print('\n');
}

void writeFloat(){
  FTM0_OUTMASK = 0xFF;
  FTM0_SYNC |= 0x80;
}