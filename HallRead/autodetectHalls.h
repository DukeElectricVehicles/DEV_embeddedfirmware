#ifndef AUTODETECTHALLS_H
#define AUTODETECTHALLS_H

#ifdef ESC2019_SENSORLESS
	#include "MCpwm_2019sensorless.h"
#else
	#include "MCpwm.h"
#endif

#define SIN30 0.5
#define COS30 0.8660254038

bool detectingHalls = false;

typedef struct {
	double transitionToTime[8];
	double transitionToTheta[8];
	double positionTheta[8];
	uint8_t nextPosition[8];
	bool badCycle;
} hallTransitions_t;

hallTransitions_t mostRecentTransitions;
void printHallTransitions(hallTransitions_t transitions);

hallTransitions_t runOpenLoop(double speed_rad_per_s, uint16_t maxPWM){
	detectingHalls = true;
	hallISRflag = false;
	hallTransitions_t toRet;
	memset(toRet.transitionToTime, 0, sizeof(toRet.transitionToTime));
	memset(toRet.transitionToTheta, 0, sizeof(toRet.transitionToTheta));
	memset(toRet.positionTheta, 0, sizeof(toRet.transitionToTheta));
	memset(toRet.nextPosition, 0, sizeof(toRet.nextPosition));
	toRet.badCycle = false;

	setupPWM();
	writePWM(0,0,0);
	pinMode(DRV_EN_GATE, OUTPUT);
	digitalWrite(DRV_EN_GATE, HIGH);
	delay(10);

	maxPWM = maxPWM / 2;
	double theta = 0;
	double c, s;
	uint32_t startTime = micros();
	uint32_t lastLoopTime = micros();
	uint32_t tmpTime;
	uint8_t lastHallPos = hallPos;
	while ((theta < (2*PI)) && (theta > (-2*PI))){
		if (hallISRflag){
			cli();
			toRet.nextPosition[lastHallPos] = hallPos;
			toRet.transitionToTime[lastHallPos] = micros();
			toRet.transitionToTheta[lastHallPos] = theta;
			lastHallPos = hallPos;
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

		writePWM(	maxPWM + c,
					maxPWM - SIN30 * c - COS30 * s,
					maxPWM - SIN30 * c + COS30 * s);
		// Serial.print(maxPWM + c); Serial.print('\t');
		// Serial.print(maxPWM - SIN30 * c - COS30 * s); Serial.print('\t');
		// Serial.print(maxPWM - SIN30 * c + COS30 * s); Serial.print('\n');

		tmpTime = micros();
		theta += speed_rad_per_s * (tmpTime - lastLoopTime) / 1000000.0;
		lastLoopTime = tmpTime;
	}

	writePWM(0,0,0);
	digitalWrite(DRV_EN_GATE, LOW);
	delay(10);

	for (uint8_t i = 0; i<8; i++){
		if (toRet.transitionToTime[i]!=0){
			toRet.transitionToTime[i] -= startTime;
		}
	}

	uint8_t cycleInd = 1;
	for (uint8_t i=0; i<6; i++){
		uint8_t next = toRet.nextPosition[cycleInd];

		float diffHall = toRet.transitionToTheta[next] - toRet.transitionToTheta[cycleInd];
		float hallAngle;
		if (fabsf(diffHall) < M_PI){
			hallAngle = (toRet.transitionToTheta[next] + toRet.transitionToTheta[cycleInd]) / 2;
		} else {
			hallAngle = (toRet.transitionToTheta[next] + toRet.transitionToTheta[cycleInd]) / 2 + M_PI;
		}
		hallAngle = fmodf(hallAngle + 2*M_PI, 2*M_PI);

		toRet.positionTheta[next] = hallAngle;

		cycleInd = next;
	}
	if (cycleInd!=1){
		toRet.badCycle = true;
		Serial.println("cycle issue:");
		printHallTransitions(toRet);
	}

	detectingHalls = false;
	mostRecentTransitions = toRet;
	return toRet;
}
void runOpenLoops(double speed_rad_per_s, uint16_t maxPWM, double thetas[6], uint8_t numRevs=1){
	detectingHalls = true;
	hallISRflag = false;
	hallTransitions_t toRet[numRevs];
	for (uint8_t i = 0; i<numRevs; i++) {
		memset(toRet[i].transitionToTime, 0, sizeof(toRet[i].transitionToTime));
		memset(toRet[i].transitionToTheta, 0, sizeof(toRet[i].transitionToTheta));
		memset(toRet[i].positionTheta, 0, sizeof(toRet[i].transitionToTheta));
		memset(toRet[i].nextPosition, 0, sizeof(toRet[i].nextPosition));
		toRet[i].badCycle = false;
	}

	setupPWM();
	writePWM(0,0,0);
	pinMode(DRV_EN_GATE, OUTPUT);
	digitalWrite(DRV_EN_GATE, HIGH);
	delay(10);

	maxPWM = maxPWM / 2;
	double theta = 0;
	double c, s;
	uint32_t startTime = micros();
	uint32_t lastLoopTime = micros();
	uint32_t tmpTime;
	uint8_t lastHallPos = hallPos;
	uint8_t toRetInd_ticks = 0;
	// while ((theta < (2*PI*numRevs)) && (theta > (-2*PI*numRevs))){
	while (toRetInd_ticks < 6*numRevs) {
		if (fabsf(theta) > (2*PI*(numRevs+1))){
			break; // something went wrong and we've gone too far
		}
		if (hallISRflag){
			cli();
			toRet[toRetInd_ticks/6].nextPosition[lastHallPos] = hallPos;
			toRet[toRetInd_ticks/6].transitionToTime[lastHallPos] = micros();
			toRet[toRetInd_ticks/6].transitionToTheta[lastHallPos] = theta;
			toRetInd_ticks++;
			lastHallPos = hallPos;
			hallISRflag = false;
			sei();

			// if (toRetInd_ticks%6==0) {
		 //    	for (uint8_t i=0; i<6; i++){
			// 		Serial.print(toRet[toRetInd_ticks/6-1].positionTheta[i+1] / M_PI * 100); Serial.print('\t');
		 //    	}
		 //    	Serial.println();
		 //    }
		}

		// write next phase
		c = maxPWM * cos(theta);
		s = maxPWM * sin(theta);
		/* 	cos(theta) = cos(theta)
			cos(theta+120deg) = -1/2 * cos(theta) - sqrt(3)/2 * sin(theta)
			cos(theta+240deg) = -1/2 * cos(theta) + sqrt(3)/2 * sin(theta)
		*/

		writePWM(	maxPWM + c,
					maxPWM - SIN30 * c - COS30 * s,
					maxPWM - SIN30 * c + COS30 * s);
		// Serial.print(maxPWM + c); Serial.print('\t');
		// Serial.print(maxPWM - SIN30 * c - COS30 * s); Serial.print('\t');
		// Serial.print(maxPWM - SIN30 * c + COS30 * s); Serial.print('\n');

		tmpTime = micros();
		theta += speed_rad_per_s * (tmpTime - lastLoopTime) / 1000000.0;
		lastLoopTime = tmpTime;
	}

	writePWM(0,0,0);
	digitalWrite(DRV_EN_GATE, LOW);
	delay(10);

	for (uint8_t j = 0; j<numRevs; j++){
		for (uint8_t i = 0; i<8; i++){
			if (toRet[j].transitionToTime[i]!=0){
				toRet[j].transitionToTime[i] -= startTime;
			}
		}
	}

	// average transitions to get center points
	uint8_t cycleInd = 1;
	for (uint8_t toRetInd=0; toRetInd<numRevs; toRetInd++) {
		for (uint8_t i=0; i<6; i++){
			uint8_t next = toRet[toRetInd].nextPosition[cycleInd];

			float diffHall = toRet[toRetInd].transitionToTheta[next] - toRet[toRetInd].transitionToTheta[cycleInd];
			float hallAngle;
			if (fabsf(diffHall) < M_PI){
				hallAngle = (toRet[toRetInd].transitionToTheta[next] + toRet[toRetInd].transitionToTheta[cycleInd]) / 2;
			} else {
				hallAngle = (toRet[toRetInd].transitionToTheta[next] + toRet[toRetInd].transitionToTheta[cycleInd]) / 2 + M_PI;
			}
			hallAngle = fmodf(hallAngle + 2*M_PI, 2*M_PI);

			toRet[toRetInd].positionTheta[next] = hallAngle;

			cycleInd = next;
		}
		if (cycleInd!=1){
			Serial.println("cycle issue:");
			toRet[toRetInd].badCycle = true;
			printHallTransitions(toRet[toRetInd]);
		}
	}

	detectingHalls = false;
	mostRecentTransitions = toRet[numRevs-1];

    double sumC[6];
    double sumS[6];
    memset(sumC, 0, sizeof(sumC));
    memset(sumS, 0, sizeof(sumS));
    for (uint8_t toRetInd=0; toRetInd<numRevs; toRetInd++){
    	for (uint8_t i=0; i<6; i++){
    		sumC[i] += cos(toRet[toRetInd].positionTheta[i+1]);
    		sumS[i] += sin(toRet[toRetInd].positionTheta[i+1]);
			Serial.print(toRet[toRetInd].positionTheta[i+1] / M_PI * 100); Serial.print('\t');
    	}
    	Serial.println();
    }

    // double thetas[6];
	for (uint8_t i=0; i<6; i++){
		double c = sumC[i] / numRevs;
		double s = sumS[i] / numRevs;
		thetas[i] = fmodf(atan2(s, c) + 2*PI, 2*PI) / M_PI * 100;
		double mag = sqrt(c*c + s*s);
		Serial.print(i); Serial.print('\t');
		Serial.print(thetas[i]); Serial.print('\t');
		Serial.print(c); Serial.print('\t');
		Serial.print(s); Serial.print('\t');
		Serial.print(mag); Serial.print('\n');
	}

	// return thetas;
}

void printHallTransitions(hallTransitions_t transitions){
	for (int i = 0; i<8; i++){
		Serial.print('\t');
		Serial.print(i, DEC);
		Serial.print('\t');
		Serial.print(transitions.nextPosition[i], DEC);
		Serial.print('\t');
		Serial.print(transitions.transitionToTheta[i],4);
		Serial.print('\t');
		Serial.print(transitions.transitionToTime[i]/1000000.0,3);
		Serial.print('\t');
		Serial.print('\t');
		Serial.print(transitions.positionTheta[i],3);
		Serial.print('\n');
	}
	Serial.print('\n');
}

#endif