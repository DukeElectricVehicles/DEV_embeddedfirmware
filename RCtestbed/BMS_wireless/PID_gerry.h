#ifndef PID_GERRY_H
#define PID_GERRY_H


class PID {

	public:

		float* m_error;
		float* m_control;
		float m_kP, m_kI, m_kD;
		float m_limLow = -1e9, m_PLimLow = -1e9, m_ILimLow = -1e9, m_DLimLow = -1e9;
		float m_limHigh = 1e9, m_PLimHigh = 1e9, m_ILimHigh = 1e9, m_DLimHigh = 1e9;
		float m_slewLimNeg = -1e9, m_slewLimPos = 1e9;

		float m_errorI, m_errorD;
		float m_errorPrev;

		uint32_t m_lastUpdateTime_us;
		float m_dt_s;


		PID(float kP, float kI, float kD, float* error, float* control): 
			m_kP(kP), m_kI(kI), m_kD(kD), m_error(error), m_control(control)
		{
			m_errorI = 0;
			m_errorPrev = 0;
			*control = m_kP * (*error);
			m_lastUpdateTime_us = micros();
			m_dt_s = 0;
		};

		void update()
		{
			uint32_t newT = micros();
			m_dt_s = (newT - m_lastUpdateTime_us) / 1000000.0;
			m_lastUpdateTime_us = newT;

			m_errorI += (*m_error) * m_dt_s;
			m_errorI = constrain(m_errorI, m_ILimLow/m_kI, m_ILimHigh/m_kI);
			m_errorD = (m_dt_s==0) ? 0 : (*m_error - m_errorPrev) / m_dt_s;
			*m_control = constrain(
							constrain(
								constrain(m_kP * (*m_error), m_PLimLow, m_PLimHigh) + 
										  m_kI * m_errorI + // constrained earlier
								constrain(m_kD * m_errorD, m_DLimLow, m_DLimHigh), 
								m_limLow, m_limHigh),
							(*m_control) + m_slewLimNeg*m_dt_s, (*m_control) + m_slewLimPos*m_dt_s);

			m_errorPrev = *m_error;
		};
		void update(float kP, float kI, float kD)
		{
			m_dt_s = (micros() - m_lastUpdateTime_us) / 1000000.0;
			m_errorI += (*m_error) * m_dt_s;
			m_errorD = (*m_error - m_errorPrev) / m_dt_s;
			*m_control = (kP * (*m_error)) + (kI * m_errorI) + (kD * m_errorD);

			m_lastUpdateTime_us += m_dt_s * 1000000L;
			m_errorPrev = *m_error;
		};
		void reset()
		{
			m_errorI = 0;
			m_errorD = 0;
			m_lastUpdateTime_us = micros();
		};

		void setLim(float limLow, float limHigh) {m_limLow = limLow; m_limHigh = limHigh;};
		void setPLim(float PLimLow, float PLimHigh) {m_PLimLow = PLimLow; m_PLimHigh = PLimHigh;};
		void setILim(float ILimLow, float ILimHigh) {m_ILimLow = ILimLow; m_ILimHigh = ILimHigh;};
		void setDLim(float DLimLow, float DLimHigh) {m_DLimLow = DLimLow; m_DLimHigh = DLimHigh;};
		void setSlewLim(float slewLimNeg, float slewLimPos) {m_slewLimNeg = slewLimNeg; m_slewLimPos = slewLimPos;};
};


#endif