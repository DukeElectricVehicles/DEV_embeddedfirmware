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

#define COS36 0.8090169944
#define R_EARTH 6371000
#define RADPERDEG 0.01745329252
#define WAYPOINTDISTTHRESH_M 10

extern msg_pos_llh_t curPosLLH_RTK;
extern msg_vel_ned_t curVelNED_RTK;
extern bool isPathComplete;
extern bool manualPrint;

static uint16_t curWaypointInd = 0;

#include "grossHallLoop4.h"

float nLPF,eLPF;
double delLat_m, delLon_m, del2_m2;
double setLat_m, setLon_m;
double delWayLat_m, delWayLon_m;
double proj;
double curHeading, desHeading;

void initPathFollow(){ // initialize the waypoint stuff
	curWaypointInd = 0;
	delWayLat_m = (waypoints_RTK[curWaypointInd+1][0] - waypoints_RTK[curWaypointInd][0]) * RADPERDEG * R_EARTH;
	delWayLon_m = (waypoints_RTK[curWaypointInd+1][1] - waypoints_RTK[curWaypointInd][1]) * RADPERDEG * R_EARTH * COS36;
	double delWayMag = sqrt(delWayLat_m*delWayLat_m + delWayLon_m*delWayLon_m);
	delWayLat_m = delWayLat_m / delWayMag;
	delWayLon_m = delWayLon_m / delWayMag;
}
float getWaypointDir(){
	// how much do we care about position vs heading accuracy?

	if (isPathComplete){
		return 0;
	}

	nLPF += .1*(curVelNED_RTK.n - nLPF);
	eLPF += .1*(curVelNED_RTK.e - eLPF);

	delLat_m = (waypoints_RTK[curWaypointInd][0] - curPosLLH_RTK.lat) * RADPERDEG * R_EARTH;
	delLon_m = (waypoints_RTK[curWaypointInd][1] - curPosLLH_RTK.lon) * RADPERDEG * R_EARTH * COS36;
	setLat_m = delLat_m;// + delWayLat_m;
	setLon_m = delLon_m;// + delWayLon_m;
	del2_m2 = delLat_m*delLat_m + delLon_m*delLon_m;

	// first check if we have gotten past the waypoint
	// 	direction to waypoint should be within +/- 90 degrees of direction from this waypoint to the next waypoint
	//	=> use dot product
	proj = (delLat_m*delWayLat_m) + (delLon_m*delWayLon_m);
	if ((proj < 0) && (del2_m2 < (WAYPOINTDISTTHRESH_M*WAYPOINTDISTTHRESH_M))){ // met waypoint
		manualPrint = true;
		curWaypointInd++;
		if (curWaypointInd >= (sizeof(waypoints_RTK)/sizeof(waypoints_RTK[0]))){ // finished
			isPathComplete = true;
			curWaypointInd = 0;
			return 0;
		} else if (curWaypointInd == ((sizeof(waypoints_RTK)/sizeof(waypoints_RTK[0])) - 1)){ // last waypoint
			delWayLat_m = (waypoints_RTK[curWaypointInd][0] - waypoints_RTK[curWaypointInd-1][0]) * RADPERDEG * R_EARTH;
			delWayLon_m = (waypoints_RTK[curWaypointInd][1] - waypoints_RTK[curWaypointInd-1][1]) * RADPERDEG * R_EARTH * COS36;
		} else {
			delWayLat_m = (waypoints_RTK[curWaypointInd+1][0] - waypoints_RTK[curWaypointInd][0]) * RADPERDEG * R_EARTH;
			delWayLon_m = (waypoints_RTK[curWaypointInd+1][1] - waypoints_RTK[curWaypointInd][1]) * RADPERDEG * R_EARTH * COS36;
		}
		double delWayMag = sqrt(delWayLat_m*delWayLat_m + delWayLon_m*delWayLon_m);
		delWayLat_m = delWayLat_m / delWayMag;
		delWayLon_m = delWayLon_m / delWayMag;
	}

	// now calculate what angle we should steer at
	curHeading = ((abs(nLPF) > 500) || (abs(eLPF) > 500)) ? atan2(nLPF, eLPF) : curHeading;
	desHeading = atan2(setLat_m, setLon_m);
	return fmod(desHeading-curHeading + 5*PI, 2*PI) - PI;
}

float getProgress(){
	return ((float) curWaypointInd) / (sizeof(waypoints_RTK)/sizeof(waypoints_RTK[0]));
}

#endif