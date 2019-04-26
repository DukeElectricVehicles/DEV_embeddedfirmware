/*

	Path following using the RTK GPS
	Gerry Chen - Apr 22, 2019

*/

#ifndef PATH_FOLLOW_RTK_H
#define PATH_FOLLOW_RTK_H

#ifndef useRTK
	#error "this version of path following is not supported without RTK"
#endif
#ifdef useAngleControl
	#error "haven't yet coded closed loop angle control with GPS based path following"
#endif

extern msg_pos_llh_t curPosLLH_RTK;
extern msg_vel_ned_t curVelNED_RTK;
extern bool isPathComplete;

static uint8_t curWaypointInd = 0;

double waypoints_RTK[][2] = {
{36.0021857, -78.9455007},
{36.0022074, -78.945545},
{36.0022313, -78.9456147},
{36.0022725, -78.9457488},
{36.0023246, -78.945891},
{36.0023799, -78.9460129},
{36.0023972, -78.9460598},
{36.0024027, -78.9461042},
{36.0023962, -78.9461418},
{36.0023712, -78.9461657},
{36.0023148, -78.9462035},
{36.0022628, -78.9462491},
{36.0022120, -78.946284},
{36.002177, -78.9463054},
{36.0021543, -78.9463161},
{36.002129, -78.9463161},
{36.002109, -78.9463054},
{36.0020946, -78.9462839},
{36.0020848, -78.9462611},
{36.0020740, -78.9462303},
{36.0020545, -78.9461713},
{36.0020187, -78.9460734},
{36.001980, -78.9459674},
{36.0019438, -78.9458735},
{36.0019156, -78.9458146},
{36.0018876, -78.9457520},
{36.001870, -78.9457032},
{36.0018592, -78.9456603},
{36.0018602, -78.9456254},
{36.0018776, -78.9455986},
{36.001908, -78.945574},
{36.0019536, -78.9455463},
{36.0020208, -78.9455128},
{36.002046, -78.9454967},
{36.0020707, -78.9454819},
{36.0021065, -78.9454592},
{36.0021391, -78.945452},
{36.0021651, -78.9454672},
{36.0021857, -78.9455007}
};

float nLPF,eLPF;
double delLat, delLon;
double delWayLat, delWayLon;
double proj;
double curHeading, desHeading;

void initPathFollow(){ // initialize the waypoint stuff
	curWaypointInd = 0;
	delWayLat = waypoints_RTK[curWaypointInd+1][0] - waypoints_RTK[curWaypointInd][0];
	delWayLon = waypoints_RTK[curWaypointInd+1][1] - waypoints_RTK[curWaypointInd][1];
	double delWayMag = sqrt(delWayLat*delWayLat + delWayLon*delWayLon);
	delWayLat = delWayLat / delWayMag;
	delWayLon = delWayLon / delWayMag;
}
float getWaypointDir(){
	// how much do we care about position vs heading accuracy?

	if (isPathComplete){
		return 0;
	}

	nLPF += .1*(curVelNED_RTK.n - nLPF);
	eLPF += .1*(curVelNED_RTK.e - eLPF);

	delLat = waypoints_RTK[curWaypointInd][0] - curPosLLH_RTK.lat;
	delLon = waypoints_RTK[curWaypointInd][1] - curPosLLH_RTK.lon;

	// first check if we have gotten past the waypoint
	// 	direction to waypoint should be within +/- 90 degrees of direction from this waypoint to the next waypoint
	//	=> use dot product
	proj = (delLat*delWayLat) + (delLon*delWayLon);
	if (proj < 0){ // met waypoint
		curWaypointInd++;
		if (curWaypointInd >= (sizeof(waypoints_RTK)/sizeof(waypoints_RTK[0]))){ // finished
			isPathComplete = true;
			curWaypointInd = 0;
			return 0;
		} else if (curWaypointInd == ((sizeof(waypoints_RTK)/sizeof(waypoints_RTK[0])) - 1)){ // last waypoint
			delWayLat = waypoints_RTK[curWaypointInd][0] - waypoints_RTK[curWaypointInd-1][0];
			delWayLon = waypoints_RTK[curWaypointInd][1] - waypoints_RTK[curWaypointInd-1][1];
		} else {
			delWayLat = waypoints_RTK[curWaypointInd+1][0] - waypoints_RTK[curWaypointInd][0];
			delWayLon = waypoints_RTK[curWaypointInd+1][1] - waypoints_RTK[curWaypointInd][1];
		}
		double delWayMag = sqrt(delWayLat*delWayLat + delWayLon*delWayLon);
		delWayLat = delWayLat / delWayMag;
		delWayLon = delWayLon / delWayMag;
	}

	// now calculate what angle we should steer at
	curHeading = ((abs(nLPF) > 500) || (abs(eLPF) > 500)) ? atan2(nLPF, eLPF) : curHeading;
	desHeading = atan2(delLat, delLon);
	return fmod(desHeading-curHeading + 5*PI, 2*PI) - PI;
}

float getProgress(){
	return ((float) curWaypointInd) / (sizeof(waypoints_RTK)/sizeof(waypoints_RTK[0]));
}

#endif