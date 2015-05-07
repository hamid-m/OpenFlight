
#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "mission_interface.h"

extern void run_mission(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct mission *missionData_ptr) {
	double xt,yt,zt;
	double x,y,z;
	double R = 6378137;
	double d;
	xt = R*cos(44.725699*D2R)*cos(-93.075225*D2R);
	yt = R*cos(44.725699*D2R)*sin(-93.075225*D2R);
	zt = R*sin(-93.075225*D2R);
	
	x = R*cos(navData_ptr->lat)*cos(navData_ptr->lon);
	y = R*cos(navData_ptr->lat)*sin(navData_ptr->lon);
	z = R*sin(navData_ptr->lon);
	
	d = pow(pow(x-xt,2)+pow(y-yt,2)+pow(z-zt,2),0.5);
	if(missionData_ptr->mode == 2){
		if(d<20){
			missionData_ptr->haveGPS = 0;
			missionData_ptr->researchNav = 1;
			missionData_ptr->researchGuidance = 1;
		}
	}
	if(missionData_ptr->mode == 1){
		missionData_ptr->haveGPS = 1;
		missionData_ptr->researchNav = 0;
		missionData_ptr->researchGuidance = 0;
	}
}
