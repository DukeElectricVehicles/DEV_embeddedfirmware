#ifndef AUTODETECTHALLS_H
#define AUTODETECTHALLS_H

#define SIN30 0.5
#define COS30 0.8660254038

bool detectingHalls = false;

typedef struct {
	double transitionToTime[8];
	double transitionToTheta[8];
	uint8_t nextPosition[8];
} hallTransitions_t;

hallTransitions_t mostRecentTransitions;

hallTransitions_t runOpenLoop(double speed_rad_per_s, uint16_t maxPWM){
	detectingHalls = true;
	hallISRflag = false;
	hallTransitions_t toRet;
	memset(toRet.transitionToTime, 0, sizeof(toRet.transitionToTime));
	memset(toRet.transitionToTheta, 0, sizeof(toRet.transitionToTheta));
	memset(toRet.nextPosition, 0, sizeof(toRet.nextPosition));

	digitalWrite(INLA, LOW);
	digitalWrite(INLB, LOW);
	digitalWrite(INLC, LOW);

	maxPWM = maxPWM / 2;
	double theta = 0;
	double c, s;
	uint32_t startTime = micros();
	uint32_t lastLoopTime = micros();
	uint32_t tmpTime;
	uint8_t lastHallPos = hallPos;
	while (theta < (2*PI)){
		if (hallISRflag){
			cli();
			toRet.nextPosition[lastHallPos] = hallPos;
			lastHallPos = hallPos;
			toRet.transitionToTime[hallPos] = micros();
			toRet.transitionToTheta[hallPos] = theta;
			hallISRflag = false;
			sei();
		}

		// write next phase
		c = maxPWM * cos(theta);
		s = maxPWM * sin(theta);
		/* 	cos(theta) = cos(theta)
			cos(theta+120deg) = -1/2 * cos(theta) - sqrt(3)/2 * sin(theta)
			cos(theta+240deg) = -1/2 * cos(theta) + sqrt(3)/2 * sin(theta)
		*/
		analogWrite(INHA, maxPWM + c);
		analogWrite(INHB, maxPWM - SIN30 * c - COS30 * s);
		analogWrite(INHC, maxPWM - SIN30 * c + COS30 * s);
		// Serial.print(maxPWM + c); Serial.print('\t');
		// Serial.print(maxPWM - SIN30 * c - COS30 * s); Serial.print('\t');
		// Serial.print(maxPWM - SIN30 * c + COS30 * s); Serial.print('\n');

		tmpTime = micros();
		theta += speed_rad_per_s * (tmpTime - lastLoopTime) / 1000000.0;
		lastLoopTime = tmpTime;
	}

	analogWrite(INHA, 0);
	analogWrite(INHB, 0);
	analogWrite(INHC, 0);

	for (uint8_t i = 0; i<8; i++){
		if (toRet.transitionToTime[i]!=0){
			toRet.transitionToTime[i] -= startTime;
		}
	}

	detectingHalls = false;
	mostRecentTransitions = toRet;
	return toRet;
}

void printHallTransitions(hallTransitions_t transitions){
	for (int i = 0; i<8; i++){
		Serial.print(i, BIN);
		Serial.print('\t');
		Serial.print(transitions.nextPosition[i], BIN);
		Serial.print('\t');
		Serial.print(transitions.transitionToTheta[i]);
		Serial.print('\t');
		Serial.print(transitions.transitionToTime[i]/1000000.0,3);
		Serial.print('\n');
	}
}

#endif