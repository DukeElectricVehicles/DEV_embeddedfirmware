#ifndef MCPWM_H
#define MCPWM_H

#define BIT12TOMOD MODULO/4096   // approx
#if MODULO<4096
  #error "conversion from 12 bit to peripheral mod is bad"
#elif MODULO>=65536
  #error "MODULO register too large, either increase pwm frequency or set a clock divider"
#endif

#define PWM_TRIGSTART 0 //MODULO/2 - throttle/2
#define PWM_TRIGEND throttle //MODULO/2 + throttle/2

#define PRESCALE 0b10
#define DEADTIME 0b100010

void setupPWM();
void writePWM(uint16_t A, uint16_t B, uint16_t C);
void writeTrap(uint16_t throttle, uint8_t phase);
void writeLow(uint8_t phase, uint16_t throttle);
void writeHigh(uint8_t phase, uint16_t throttle);

typedef enum {
  PWM_COMPLEMENTARY,
  PWM_NONSYNCHRONOUS,
  PWM_BIPOLAR // not yet implemented
} pwmMode_t;
pwmMode_t pwmMode = PWM_NONSYNCHRONOUS;

extern bool FAULT;

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
  FTM0_SC = 0x08 | 0x00;         // set system clock as source for FTM0
  FTM0_MOD = MODULO;             // Period register (max counter value)

  // FTM0_CNTIN = 0;                // Counter initial value (must be 0 actually)
  FTM0_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN; // for ADC (trigger on center pulse)

  // add channels as needed here
  FTM0_COMBINE  = 0x00000033;    // COMBINE=1, COMP=1, DTEN=1, SYNCEN=1 for channels 0/1   // page 796  (COMP1 sets complement)
  FTM0_COMBINE |= 0x00003300;    // CH 2/3
  FTM0_COMBINE |= 0x00330000;    // CH 4/5
  FTM0_POL      = 0b00000000;    // Polarity - use this to invert signals (can take the functionality of COMP signal in COMBINE)
                                 // but preferably use the CxSC values instead since POL defines the "safe value"

  // FTM0_SYNCONF |= FTM_SYNCONF_SYNCMODE;
  // FTM0_CONF |= 0xF000; // counter run in BDM mode
  // FTM0_PWMLOAD |= FTM_PWMLOAD_LDOK;
  FTM0_SYNC |= FTM_SYNC_CNTMAX;              // PWM sync @ max loading point enable (set trigger to end once it hits the value)

  FTM0_SYNC |= 0x08;             // PWM sync outmask as well
  FTM0_DEADTIME = PRESCALE<<6;   // DeadTimer prescale systemClk/1                 // page 801
  FTM0_DEADTIME |= DEADTIME;     // 1uS DeadTime, max of 63 counts of 48Mhz clock  // page 801
  
  FTM0_C0V = 0;                  // This specifies where the trigger starts 
  FTM0_C1V = 1000;                  // This specifies where the trigger ends (init to 0 for safety)
  FTM0_C2V = 0;
  FTM0_C3V = 1000;
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
  
  FTM0_MODE &= ~(FTM_MODE_WPDIS);   // re-enable write protection
  FTM0_MODE |= 0x01;                // enable FTM0

  FTM0_SYNC |= 0x80;

  writeTrap(0, -1);
}

void setPWMMode(pwmMode_t newMode){
  pwmMode = newMode;
  setupPWM();
}

void writePWM(uint16_t A, uint16_t B, uint16_t C){
  if (FAULT){
    FTM0_OUTMASK = 0xFFFF;
    FTM0_SYNC |= 0x80;
    return;
  }

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

  // throttle = 500;
  // FTM0_C0V = MODULO/2 - MODULO/4;
  // FTM0_C1V = MODULO/2 + MODULO/4;
  // FTM0_C2V = MODULO/2 - MODULO/4;
  // FTM0_C3V = MODULO/2 + MODULO/4;
  // FTM0_C4V = MODULO/2 - MODULO/4;
  // FTM0_C5V = MODULO/2 + MODULO/4;
  // FTM0_OUTMASK = 0x0;
  // FTM0_SYNC |= 0x80;
  // // Serial.println("wrote trap");
  // return;

  if (FAULT){
    FTM0_OUTMASK = 0xFFFF;
    FTM0_SYNC |= 0x80;
    return;
  }

  throttle = constrain(throttle * BIT12TOMOD, 0, MODULO);
  switch (phase){
    case 0:   //HIGH A, LOW B
      FTM0_C0V = PWM_TRIGSTART;
      FTM0_C1V = PWM_TRIGEND + 1; // +1 because... idk why actually but it's needed for full duty cycle
      FTM0_C2V = 0;
      FTM0_C3V = 0;
      FTM0_C4V = dir ? PWM_TRIGSTART : 0; // preload register
      FTM0_C5V = dir ? PWM_TRIGEND : 0;
      FTM0_OUTMASK = (pwmMode!=PWM_NONSYNCHRONOUS) ? 0b110000 : 0b110001;
      break;
    case 1:   //HIGH C, LOW B
      FTM0_OUTMASK = (pwmMode!=PWM_NONSYNCHRONOUS) ? 0b000011 : 0b010011;
      FTM0_C0V = !dir ? PWM_TRIGSTART : 0; // preload register
      FTM0_C1V = !dir ? PWM_TRIGEND : 0;
      FTM0_C2V = 0;
      FTM0_C3V = 0;
      FTM0_C4V = PWM_TRIGSTART;
      FTM0_C5V = PWM_TRIGEND + 1;
      break;
    case 2:   //HIGH C, LOW A
      FTM0_C0V = 0;
      FTM0_C1V = 0;
      FTM0_OUTMASK = (pwmMode!=PWM_NONSYNCHRONOUS) ? 0b001100 : 0b011100;
      FTM0_C2V = dir ? PWM_TRIGSTART : 0; // preload register
      FTM0_C3V = dir ? PWM_TRIGEND : 0;
      FTM0_C4V = PWM_TRIGSTART;
      FTM0_C5V = PWM_TRIGEND + 1;
      break;
    case 3:   //HIGH B, LOW A
      FTM0_C0V = 0;
      FTM0_C1V = 0;
      FTM0_C2V = PWM_TRIGSTART;
      FTM0_C3V = PWM_TRIGEND + 1;
      FTM0_C4V = !dir ? PWM_TRIGSTART : 0; // preload register
      FTM0_C5V = !dir ? PWM_TRIGEND : 0;
      FTM0_OUTMASK = (pwmMode!=PWM_NONSYNCHRONOUS) ? 0b110000 : 0b110100;
      break;
    case 4:   //HIGH B, LOW C
      FTM0_OUTMASK = (pwmMode!=PWM_NONSYNCHRONOUS) ? 0b000011 : 0b000111;
      FTM0_C0V = dir ? PWM_TRIGSTART : 0; // preload register
      FTM0_C1V = dir ? PWM_TRIGEND : 0;
      FTM0_C2V = PWM_TRIGSTART;
      FTM0_C3V = PWM_TRIGEND + 1;
      FTM0_C4V = 0;
      FTM0_C5V = 0;
      break;
    case 5:   //HIGH A, LOW C
      FTM0_C0V = PWM_TRIGSTART;
      FTM0_C1V = PWM_TRIGEND + 1;
      FTM0_C2V = !dir ? PWM_TRIGSTART : 0; // preload register
      FTM0_C3V = !dir ? PWM_TRIGEND : 0;
      FTM0_OUTMASK = (pwmMode!=PWM_NONSYNCHRONOUS) ? 0b001100 : 0b001101;
      FTM0_C4V = 0;
      FTM0_C5V = 0;
      break;
    default:
      FTM0_OUTMASK = 0xFF;
      break;
  }
  FTM0_OUTINIT = 0;              // initialize to low
  FTM0_SYNC |= 0x80;
}
// write the phase to the low side gates
// 1-hot encoding for the phase
// 001 = A, 010 = B, 100 = C
void writeLow(uint8_t phase, uint16_t throttle){
  if (throttle == 0){
    phase = 0;
  }
  digitalWriteFast(INLA, (phase&(1<<0)));
  digitalWriteFast(INLB, (phase&(1<<1)));
  digitalWriteFast(INLC, (phase&(1<<2)));
}
// write the phase to the high side gates
// 1-hot encoding for the phase
// 001 = A, 010 = B, 100 = C
void writeHigh(uint8_t phase, uint16_t throttle){
  switch(phase){
  case 0b001: // Phase A
    analogWrite(INHB, 0);
    analogWrite(INHC, 0);
    analogWrite(INHA, throttle);
    break;
  case 0b010: // Phase B
    analogWrite(INHA, 0);
    analogWrite(INHC, 0);
    analogWrite(INHB, throttle);
    break;
  case 0b100:// Phase C
    analogWrite(INHA, 0);
    analogWrite(INHB, 0);
    analogWrite(INHC, throttle);
    break;
  default://ALL OFF
    analogWrite(INHA, 0);
    analogWrite(INHB, 0);
    analogWrite(INHC, 0);
  }
}
#endif