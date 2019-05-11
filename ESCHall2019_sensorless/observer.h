#ifndef OBSERVER_H
#define OSBERVER_H

// #include "utils.h"

#define PLL_KP 2000
#define PLL_KI 40000

static volatile float m_phase;
static volatile float m_phase_now_observer;
static volatile float m_pll_phase=0;
static volatile float m_pll_speed=0;
static float speed = 0;

static void pll_run(float phase, float dt, volatile float *phase_var,
		volatile float *speed_var);
float utils_angle_difference(float angle1, float angle2);
void utils_norm_angle(float *angle);
#define UTILS_IS_NAN(x)		((x) != (x))
#define UTILS_NAN_ZERO(x)	(x = UTILS_IS_NAN(x) ? 0.0 : x)

float updateHall(uint8_t hallPos){

	return hallPos * 1.8; // 360/200
	
	static uint8_t prevHallPos = -1;
	static uint32_t prevTime = 0;
	static uint32_t hallAngle = 0;
	static uint32_t prevTickTime = 0;

	uint32_t curTime = micros();
	float dt = min(curTime - prevTime, 1000) / 1000000.0;
	prevTime = curTime;

	if (hallPos < 201){
		if (prevHallPos != hallPos){ // real pos is average of transitions
			uint32_t tickTime = micros();
			uint32_t dttick = (tickTime-prevTickTime);
			int8_t delHall = (((int16_t)hallPos-prevHallPos + 300) % 200) - 100;
			speed += constrain(.01 * (delHall/200.0*1000000/dttick - speed),-50,50);
			prevTickTime = tickTime;
		}
	}
	if ((micros() - prevTickTime) > 100000){
		speed /= 1.1;
		// speed = constrain(((speed>0) ? 1 : -1) / 6.0 * 1000000/(micros()-prevTickTime),-100,100);
	}

	if (fabsf(speed) < 500){
		m_phase_now_observer = hallPos / 200.0 * 360;
	} else {
		if (hallPos < 201){
			if (prevHallPos != hallPos){ // real pos is average of transitions
				if (abs(hallPos - prevHallPos) < 100){
					hallAngle = (hallPos + prevHallPos) / 2;
				} else {
					hallAngle = (hallPos + prevHallPos + 200) / 2;
				}
				hallAngle %= 200;
			}
		}

		float diff = utils_angle_difference(m_phase_now_observer, hallAngle / 200 * 360);
		if (fabsf(diff) < 16){
			m_phase_now_observer += m_pll_speed * dt;
		} else {
			m_phase_now_observer -= diff / 100;
		}
	}

	utils_norm_angle((float*) &m_phase_now_observer);

	pll_run(m_phase_now_observer, dt, &m_pll_phase, &m_pll_speed);

	prevHallPos = hallPos;

	return m_phase_now_observer;
}

static void pll_run(float phase, float dt, volatile float *phase_var,
		volatile float *speed_var) {
	UTILS_NAN_ZERO(*phase_var);
	float delta_theta = phase - *phase_var + 180;
	utils_norm_angle(&delta_theta);
	delta_theta -= 180;
	UTILS_NAN_ZERO(*speed_var);
	if (*speed_var > 30000){
		*speed_var = 30000;
	}
	*phase_var += (*speed_var + PLL_KP * delta_theta) * dt;
	utils_norm_angle((float*)phase_var);
	*speed_var += PLL_KI * delta_theta * dt;
}

/**
 * Make sure that 0 <= angle < 360
 *
 * @param angle
 * The angle to normalize.
 */
void utils_norm_angle(float *angle) {
	*angle = fmodf(*angle, 360.0);

	if (*angle < 0.0) {
		*angle += 360.0;
	}
}

/**
 * Get the difference between two angles. Will always be between -180 and +180 degrees.
 * @param angle1
 * The first angle
 * @param angle2
 * The second angle
 * @return
 * The difference between the angles
 */
float utils_angle_difference(float angle1, float angle2) {
//	utils_norm_angle(&angle1);
//	utils_norm_angle(&angle2);
//
//	if (fabsf(angle1 - angle2) > 180.0) {
//		if (angle1 < angle2) {
//			angle1 += 360.0;
//		} else {
//			angle2 += 360.0;
//		}
//	}
//
//	return angle1 - angle2;

	// Faster in most cases
	float difference = angle1 - angle2;
	while (difference < -180.0) difference += 2.0 * 180.0;
	while (difference > 180.0) difference -= 2.0 * 180.0;
	return difference;
}

#endif