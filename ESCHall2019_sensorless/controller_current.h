#ifndef CONTROLLER_CURRENT_H
#define CONTROLLER_CURRENT_H

extern pwmMode_t pwmMode;

extern float Kv;
extern float Rs;
extern float maxCurrent, minCurrent;
extern float rpm, Vbus;

float IsetpointSlewLim_A_per_s = 75;
float Isetpoint = 0, Isetpointsetpoint;
float uP, uI, u;
float kP = 0.3, kI = 2;
float limP = 8, limI = 20, lim = 20;

uint32_t lastUpdateTime_CC_us = 0;

void inline moveTowards(float *origV, float newV, float lim);

float getOpenLoopCurrent(float duty) {
	return (duty*Vbus - rpm/Kv) / Rs;
}
float getOpenLoopVoltage(float I) {
	// return rpm/Kv + I*Rs;
	return 0;
}

void bumplessPIDupdate_I(float duty, float current) {
	lastUpdateTime_CC_us = micros();
	float Vhat = getOpenLoopVoltage(current);
	uI = (duty*Vbus) - Vhat;
	uI = constrain(uI, -limI, limI);
	Isetpoint = current;
}
float getPIDvoltage_I(float Imeas) {
	if (Vbus == 0) {
		return 0;
	}
	float lowerLimit = 0;
	uint32_t newT_us = micros();
	moveTowards(&Isetpoint, Isetpointsetpoint, IsetpointSlewLim_A_per_s * (newT_us-lastUpdateTime_CC_us) / 1e6);
	float error = (Isetpoint - Imeas);
	uP = error*kP;
	if (	(duty >= (1.00*MODULO)) && (error > 0) 	){
		// uI *= .99; // don't do this so that we can maintain 100% duty cycle
	} else if (	(duty < (0.002*MODULO)) && (error < 0)	){	// can't go any lower
		// moveTowards(&Isetpoint, 0, 2 * IsetpointSlewLim_A_per_s * (newT_us-lastUpdateTime_CC_us) / 1e6);
		if ((rpm > 100) && (pwmMode == PWM_COMPLEMENTARY)) {
			lowerLimit = 0.0015*Vbus;
		}
		// uI += .1;
	} else {
		uI += error*kI*(newT_us - lastUpdateTime_CC_us) / 1e6;
	}
	uP = constrain(uP, -limP, limP);
	uI = constrain(uI, -limI, limI);
	u = constrain(uP + uI, -lim, lim);
	lastUpdateTime_CC_us = newT_us;

	return constrain(getOpenLoopVoltage(Isetpoint) + u, lowerLimit, Vbus);
}
void setSetpoint_I(float Isetpoint_new) {
	Isetpointsetpoint = Isetpoint_new;
}

void inline moveTowards(float *origV, float newV, float slewLim) {
	float diff = newV - *origV;
	if (diff > slewLim) {
		*origV = *origV + slewLim;
	} else if (diff < -slewLim) {
		*origV = *origV - slewLim;
	} else {
		*origV = newV;
	}
}

#endif