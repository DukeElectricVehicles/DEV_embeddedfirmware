#define BMS_3 1
#define ESC_v2_1 2

#define INA_ID 0
#define BOARD BMS_3

#if BOARD==BMS_3
  #define AL -1
  #define AH -1
  #define BL -1
  #define BH -1
#elif BOARD==ESC_v2_1
  #define AL 22
  #define AH 23
  #define BL 9
  #define BH 10
#endif

#define OC_LIMIT 2 // undefine to disable current limit
#define ALERT_PIN 12

#include "INA.h"

uint32_t lastLoopTime = 0;
bool debugPrint = false;

char buffer[100];
float trueCurrentVals_A[256];
float measCurrentVals_A[256];
uint8_t currentValsInd = 0;
float trueVoltageVals_V[256];
float measVoltageVals_V[256];
uint8_t voltageValsInd = 0;

void INAOC_isr();

void setup() {

  Serial.begin(115200);
  Serial.println("Beginning");
  INAinit();

  memset(buffer, 0, sizeof(buffer));
  memset(trueCurrentVals_A, 0, sizeof(trueCurrentVals_A));
  memset(measCurrentVals_A, 0, sizeof(measCurrentVals_A));
  memset(trueVoltageVals_V, 0, sizeof(trueVoltageVals_V));
  memset(measVoltageVals_V, 0, sizeof(measVoltageVals_V));
  pinMode(AL, OUTPUT);
  pinMode(AH, OUTPUT);
  pinMode(BL, OUTPUT);
  pinMode(BH, OUTPUT);
  analogWriteResolution(12);
  analogWriteFrequency(AL, 8000);
  digitalWrite(AL, HIGH);
  digitalWrite(AH, LOW);
  digitalWrite(BL, LOW);

  Serial.println("INA226 test and calibration");
  printInstructions();

  lastLoopTime = millis();
}

void loop() {
  static uint32_t thisLoopTime = 0;

  thisLoopTime = millis();

  updateINA();

  if (debugPrint && ((thisLoopTime - lastLoopTime) > 100)){
    Serial.print(InaVoltage_V,4);
    Serial.print(" ");
    Serial.print(InaCurrent_A,4);
    Serial.print(" ");
    Serial.print(InaPower_W,4);
    Serial.print(" ");
    Serial.print(digitalRead(ALERT_PIN));
    Serial.print("\n");

    #if BOARD == BMS_3
      pinMode(2, OUTPUT);
      digitalWrite(2, HIGH);
    #endif
  }

  if (Serial.available()){
    char input = Serial.read();
    float tmp;
    switch(input) {
      case 'h':
        printInstructions();
        break;
      case 'd':
        debugPrint = !debugPrint;
        break;
      case 'I':
      case 'C':
        tmp = Serial.parseFloat();
        trueCurrentVals_A[currentValsInd] = tmp;
        measCurrentVals_A[currentValsInd] = InaCurrent_A;
        currentValsInd++;
        break;
      case 'V':
        tmp = Serial.parseFloat();
        trueVoltageVals_V[voltageValsInd] = tmp;
        measVoltageVals_V[voltageValsInd] = InaVoltage_V;
        voltageValsInd++;
        break;
      case 'p':
        printStoredVals();
        break;
      case '=':
        calcParams();
        break;
      case 'D':
        tmp = Serial.parseFloat();
        analogWrite(BH, tmp*4096);
        break;
      case 'i':
        currentValsInd--;
        break;
      case 'v':
        voltageValsInd--;
        break;
    }
  }
}

void INAOC_isr() {
  analogWrite(BH, 0);
  Serial.println("\nOVER CURRENT EVENT");
}

void printInstructions(){
  Serial.println("Commands:");
  Serial.println("h - print these instructions");
  Serial.println("d - start printing live values");
  Serial.println("I# - log a measured current value (in A) for calibration, i.e. I0.3 means you are measuring 300mA on a multimeter right now");
  Serial.println("C# - same as I#");
  Serial.println("V# - log a measured voltage value (in V) for calibration");
  Serial.println("i - delete the last current measurement");
  Serial.println("v - delete the last voltage measurement");
  Serial.println("p - print the stores current and voltage values from reference");
  Serial.println("= - calculate the calibrations and offsets");
  Serial.println("D - start writing a duty cycle; this is for calibration purposes so that you can generate a reference current, only applies to ESC2019_sensorless");
  Serial.println();
}
void printStoredVals(){
  Serial.println("Current measurements:");
  for (uint8_t i = 0; i<currentValsInd; i++){
    Serial.print(trueCurrentVals_A[i], 4);
    Serial.print("A\t");
    Serial.print(measCurrentVals_A[i], 4);
    Serial.print("A\n");
  }
  Serial.println("Voltage measurements:");
  for (uint8_t i = 0; i<voltageValsInd; i++){
    Serial.print(trueVoltageVals_V[i], 3);
    Serial.print("V\t");
    Serial.print(measVoltageVals_V[i], 3);
    Serial.print("V\n");
  }
}

float vals[10] = {
0.3060, 0.4845,
0.1024, 0.1545,
0.2050, 0.3095,
0.2980, 0.4720,
0.5028, 0.7645};
void calcParams(){
  // float dummyCoeffs[2];
  // float dummyR2;
  // float dummyX[5];
  // float dummyY[5];
  // for (int i=0; i<5; i++){
  //   dummyX[i] = vals[2*i+1];
  //   dummyY[i] = vals[2*i];
  // }
  // dummyR2 = simpLinReg(&dummyX[0], &dummyY[0], &dummyCoeffs[0], 5);
  // Serial.print("Y = ");
  // Serial.print(dummyCoeffs[0],2);
  // Serial.print(" * X + ");
  // Serial.print(dummyCoeffs[1],2);
  // Serial.print("\nR2 = ");
  // Serial.println(dummyR2, 6);
  // Serial.println(dummyCoeffs[0],6);
  // Serial.println(dummyCoeffs[1],6);

  float currentCoeffs[2], voltageCoeffs[2];
  float currentR2, voltageR2;
  currentR2 = simpLinReg(&measCurrentVals_A[0], &trueCurrentVals_A[0], &currentCoeffs[0], currentValsInd);
  voltageR2 = simpLinReg(&measVoltageVals_V[0], &trueVoltageVals_V[0], &voltageCoeffs[0], voltageValsInd);
  Serial.print("Ireal = ");
  Serial.print(currentCoeffs[0],2);
  Serial.print(" * Imeas + ");
  Serial.print(currentCoeffs[1],2);
  Serial.print("\n\tR2 = ");
  Serial.println(currentR2, 6);
  Serial.print("Vreal = ");
  Serial.print(voltageCoeffs[0],2);
  Serial.print(" * Vmeas + ");
  Serial.print(voltageCoeffs[1],2);
  Serial.print("\n\tR2 = ");
  Serial.println(voltageR2, 6);

  Serial.println(currentCoeffs[0],6);
  Serial.println(currentCoeffs[1],6);
  Serial.println(voltageCoeffs[0],6);
  Serial.println(voltageCoeffs[1],6);
}

// from http://jwbrooks.blogspot.com/2014/02/arduino-linear-regression-function.html
float simpLinReg(float* x, float* y, float* lrCoef, int n){
  // pass x and y arrays (pointers), lrCoef pointer, and n.  The lrCoef array is comprised of the slope=lrCoef[0] and intercept=lrCoef[1].  n is length of the x and y arrays.
  // http://en.wikipedia.org/wiki/Simple_linear_regression

  // initialize variables
  float xbar=0;
  float ybar=0;
  float xybar=0;
  float xsqbar=0;
  float x2bar=0;
  float y2bar=0;
  
  // calculations required for linear regression
  for (int i=0; i<n; i++){
    xbar=xbar+x[i];
    ybar=ybar+y[i];
    x2bar += x[i]*x[i];
    y2bar += y[i]*y[i];
    xybar=xybar+x[i]*y[i];
    xsqbar=xsqbar+x[i]*x[i];
  }
  xbar=xbar/n;
  ybar=ybar/n;
  x2bar /= n;
  y2bar /= n;
  xybar=xybar/n;
  xsqbar=xsqbar/n;
  
  // simple linear regression algorithm
  lrCoef[0]=(xybar-xbar*ybar)/(xsqbar-xbar*xbar);
  lrCoef[1]=ybar-lrCoef[0]*xbar;

  // Gerry's addition
  // float rxy = (xybar - xbar*ybar) / sqrt((x2bar - xbar*xbar)*(y2bar-ybar*ybar));
  float R2 = (xybar - xbar*ybar)*(xybar - xbar*ybar) / ((x2bar - xbar*xbar)*(y2bar - ybar*ybar)); //rxy*rxy;
  return R2;
}