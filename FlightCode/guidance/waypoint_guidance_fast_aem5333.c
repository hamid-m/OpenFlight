
/*
 *	\brief Waypoint tracker get_guidance law
 *
 *	\details Designed by Peter Bauer. Waypoints specified as relative distances start (0,0) point is the point where autopilot is switched
 *	\ingroup guidance_fcns
 *
 * \author SZTAKI
 * \author Systems and Control Laboratory
 * \copyright Copyright 2012 
 *
 * $Id$
 */

#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "../system_id/systemid_interface.h"
#include "../utils/matrix.h"
#include "guidance_interface.h"



//////////////////////////////////////////////////////////////
//#include "../utils/matrix.c" //Required for SIL sim only. Also must comment out #include <unistd.h> and #include <termios.h> inside matrix.c

//#include "../aircraft/thor_config.h"  // for SIL sim only, use "thor" or "faser"
#include AIRCRAFT_UP1DIR
//////////////////////////////////////////////////////////////



//local function definition
void LatLonAltToEcef2(MATRIX vector, MATRIX position);
double distance2waypoint(MATRIX wp_curr, MATRIX pos_ned);
double wraparound(double dta);
double mysign(double v);
static double heading_control (double head_ref, double head_angle);
static double altitude_control(double alt_ref, double altitude);

#ifdef AIRCRAFT_THOR
	static double alt_gain[2] 	= {0.023,0.0010}; 		// PI gains for altitude tracker
	static double head_gain 	= 1.5;					// P gain for heading tracker
#endif
#ifdef AIRCRAFT_TYR
	static double alt_gain[2] 	= {0.023,0.0010}; 		// PI gains for altitude tracker
	static double head_gain 	= 1.5;					// P gain for heading tracker
#endif
#ifdef AIRCRAFT_FASER
	static double alt_gain[2] 	= {0.021,0.0017};
	static double head_gain 	= 1.2;
#endif
#ifdef AIRCRAFT_IBIS
	static double alt_gain[2] 	= {0.021,0.0017};
	static double head_gain 	= 1.2;
#endif
#ifdef AIRCRAFT_BALDR
	static double alt_gain[2] 	= {0.021,0.0017};
	static double head_gain 	= 1.2;
#endif


static double e[2] = {0,0};
static double integrator[1] = {0};


//////////////////////////////////////////////////////////////
// Waypoint definition
#define numofwaypoints 3
static short nextwaypoint=0;

// order: NO, EA, IAS, alt
// FISRT ROW MUST BE {0, 0, IAS, alt} !!!!


static double waypoints[numofwaypoints][4] =    {{0.0, 0.0, 20.0, 60.0},
{ 0.0, -50, 20.0, 60.0 },
{ 0.0, -101, 20.0, 60.0 },
};



// Control parameters
#define FTOL    20.0        // tolerance of feasability checking circle [m]
#define WPTOL   20.0        // tolerance for reaching waypoints [m]
#define PHI0    40*D2R      // considered maximum bank angle [deg]

//local variables
static MATRIX pos_lla, pos_ecef, pos_ecef0, T_ecef2ned, v_ned, tmp31, pos_ned, WP_goal;

static short guide_init=0, guide_start=0;   // init for initialization of matrices, start for start of waypoint guidance                                                      
static short pinit=0, lc=10;    // pinit for initialization of tracking, lc is the mode flag 
static double d2WP, xA, yA, xT, yT, xC, yC, psiT, cpsi, dpsi, psi, Rt, R2;


extern void get_guidance(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr, struct mission *missionData_ptr){       
	
	// Snapshots
	if(time <= 50*TIMESTEP){
		controlData_ptr->signal_1 = psi;  // store initial HEADING on dummy variable signal_1
		controlData_ptr->signal_0 = sensorData_ptr->adData_ptr->h_filt;   	// store initial ALTITUDE on dummy variable signal_0
	}
	
	if (time>0.05){
    
	#ifdef AIRCRAFT_THOR
		controlData_ptr->ias_cmd = 17;				//Trim airspeed (m/s)
	#elif defined AIRCRAFT_TYR
		controlData_ptr->ias_cmd = 17;				//Trim airspeed (m/s)
	#else
		controlData_ptr->ias_cmd = 23;
	#endif


	// Initialization of algorithm and variables	
    if (guide_init==0)  // init variables
    {
        pos_lla     = mat_creat(3,1,ZERO_MATRIX);   // LAT, LON, alt vector
        pos_ecef    = mat_creat(3,1,ZERO_MATRIX);   // ECEF position vector
        pos_ecef0   = mat_creat(3,1,ZERO_MATRIX);   // ECEF position vector of initial point
        v_ned       = mat_creat(3,1,ZERO_MATRIX);   // vector of NED velocity
        pos_ned     = mat_creat(3,1,ZERO_MATRIX);   // vector of NED position
        tmp31       = mat_creat(3,1,ZERO_MATRIX);   // temporary 3x1 vector
        T_ecef2ned  = mat_creat(3,3,ZERO_MATRIX);   // transformation matrix from ECEF to NED
        WP_goal     = mat_creat(2,1,ZERO_MATRIX);   // Vector of North / East goal coordinates (actaul waypoint)       
        guide_init=1;                               // guidance initialization finished
    }	    
    
    
    if (guide_start==0)
    {
		pos_lla[0][0] =  44.725699*D2R;
		pos_lla[1][0] = -93.073960*D2R;
        pos_lla[2][0]=navData_ptr->alt;
        
        // transform starting point from LLA to ECEF
        LatLonAltToEcef2(pos_ecef0, pos_lla);
        
        // transformation matrix for LaunchPoint ECEF |--> NED for initial point
        T_ecef2ned[0][0] = -sin(pos_lla[0][0])*cos(pos_lla[1][0]); T_ecef2ned[0][1] = -sin(pos_lla[0][0])*sin(pos_lla[1][0]); T_ecef2ned[0][2] =  cos(pos_lla[0][0]);
        T_ecef2ned[1][0] = -sin(pos_lla[1][0]);                    T_ecef2ned[1][1] =  cos(pos_lla[1][0]);                    T_ecef2ned[1][2] =  0.0;
        T_ecef2ned[2][0] = -cos(pos_lla[0][0])*cos(pos_lla[1][0]); T_ecef2ned[2][1] = -cos(pos_lla[0][0])*sin(pos_lla[1][0]); T_ecef2ned[2][2] = -sin(pos_lla[0][0]);
        
        //save next waypoint coordinates
        WP_goal[0][0]=waypoints[nextwaypoint][0];
        WP_goal[1][0]=waypoints[nextwaypoint][1];       
        
        guide_start=1;
    }
    else
        cpsi=0; //to handle SIL initial zero data
    
    // current vehicle position/vel (computed at every iteration)
    pos_lla[0][0]=navData_ptr->lat;
    pos_lla[1][0]=navData_ptr->lon;
    pos_lla[2][0]=navData_ptr->alt;
    v_ned[0][0] = navData_ptr->vn;
    v_ned[1][0] = navData_ptr->ve;
    v_ned[2][0] = navData_ptr->vd;
    // azimuth angle from ground speed
    psi=atan2(v_ned[1][0], v_ned[0][0]);
    
    // Transfrom the difference of actual coordinates from the starting point into NED frame
    LatLonAltToEcef2(pos_ecef, pos_lla);
    mat_sub(pos_ecef, pos_ecef0, tmp31);
    mat_mul(T_ecef2ned, tmp31, pos_ned);
    
    // Calculate waypoint distance from actual point
    d2WP=distance2waypoint(WP_goal, pos_ned);
    
    // change of target waypoint: check if vehicle is within x meters of the target waypoint
    if ((d2WP < WPTOL) && (guide_start==1)) {
        if(nextwaypoint == numofwaypoints-1) nextwaypoint = 0; // change back to first waypoint
        else  nextwaypoint++;
        
        // select new waypoint parameters        
        WP_goal[0][0] = waypoints[nextwaypoint][0];
        WP_goal[1][0] = waypoints[nextwaypoint][1];
        
        pinit=0;
    }
    
    // actual aircraft position
    xA=pos_ned[0][0];   // North
    yA=pos_ned[1][0];   // East
    
    // Target coordinates
    xT=WP_goal[0][0];
    yT=WP_goal[1][0];
    
    if (pinit==0)   // decide about tracking method
    {
        if (sensorData_ptr->adData_ptr->ias_filt>10)   // only for SIL because nonzero initial conditions
        {
            psiT=atan2(yT-yA, xT-xA);    // Aircraft target azimuth angle
            // azimuth angle correction if required:
            if (fabs(psiT-wraparound(psi))>PI)
                psiT=psiT+mysign(wraparound(psi))*PI2;
            
            Rt=pow(controlData_ptr->ias_cmd, 2)/g/tan(PHI0);  // possible turn radius
            // center of circle:
            dpsi=mysign(psiT-wraparound(psi))*90*D2R;            
            xC=xA+Rt*cos(psi+dpsi);
            yC=yA+Rt*sin(psi+dpsi);
            // check feasibility
            R2=sqrt(pow(WP_goal[0][0]-xC, 2)+pow(WP_goal[1][0]-yC, 2)); // distance of waypoint from circle center
            if (R2<Rt+FTOL)   // infeasible problem turn into opposite direction
            {
                cpsi=-dpsi*4;
                lc=8;
                pinit=1;
            }
            else    // feasible problem, go to waypoint
            {
                cpsi=psiT-wraparound(psi);
                lc=0;
                pinit=1;
            }
        } // end if nonzero velocity
    }   // end decide about tracking method
   
    if (lc==8)  // check if the algorithm can change to circular segment tracking
    {
        // calculate actual circle parameters
        Rt=pow(controlData_ptr->ias_cmd, 2)/g/tan(PHI0);  // turn radius
        // center of circle:
        psiT=atan2(yT-yA, xT-xA);
        // azimuth angle correction if required:
        if (fabs(psiT-wraparound(psi))>PI)
            psiT=psiT+mysign(wraparound(psi))*PI2;
        // center of circle:
        dpsi=mysign(psiT-wraparound(psi))*90*D2R;       
        xC=xA+Rt*cos(psi+dpsi);
        yC=yA+Rt*sin(psi+dpsi);
        // check feasibility
        R2=sqrt(pow(WP_goal[0][0]-xC, 2)+pow(WP_goal[1][0]-yC, 2));
        if (R2>=Rt+FTOL)   // fasible problem, track circle
        {
            cpsi=psiT-wraparound(psi);
            lc=0;
        }
        else
            cpsi=-dpsi*4;
    }
    else if (lc==0) {
        psiT=atan2(yT-yA, xT-xA);
        // azimuth angle correction if required:
        if (fabs(psiT-wraparound(psi))>PI)
            psiT=psiT+mysign(wraparound(psi))*PI2;
        cpsi=psiT-wraparound(psi);
    }
    
    // send out control command
    controlData_ptr->psi_cmd=cpsi;
    
    // send out diagnostic variable
    controlData_ptr->r_cmd=((nextwaypoint)*10+lc);
    }
	
	controlData_ptr->phi_cmd = heading_control(controlData_ptr->psi_cmd, 0);
    controlData_ptr->theta_cmd = altitude_control(controlData_ptr->h_cmd, sensorData_ptr->adData_ptr->h_filt - controlData_ptr->signal_0);
}
    
/////////////////////////////////////////////////////////////////////////////////////////
///// LOCAL FUNCTIONS ////////////////////////////

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// signum function
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static double heading_control (double head_ref, double head_angle)
{
	// Heading tracking controller implemented here
	double roll_limit = 45*D2R; // Roll angle saturation limit (45 degrees)
	double head_out;

	e[1] = head_ref - head_angle;
	head_out = head_gain*e[1];

	if	(fabs(head_out) >= roll_limit){
		head_out = sign(head_out)*roll_limit;
	}

    return head_out;
}

static double altitude_control(double alt_ref, double altitude)
{
	double pitch_limit = 0.349066; // Pitch angle saturation limit (20 degrees)
	double h_out;

	// Altitude tracker
	e[0] = alt_ref - altitude;
	integrator[0] += e[0]*TIMESTEP; // altitude error integral

		//	proportional term + integral term   (output of altitude PI block)
	h_out = alt_gain[0]*e[0] + alt_gain[1]*integrator[0];

	if	(abs(h_out) >= pitch_limit){
		h_out = sign(h_out)*pitch_limit;
	}

	return h_out;  //rad
}

double mysign(double v) {
    double temp=0;
    
    if (v<0)
        temp = -1;
    else if (v>0)
        temp = 1;
    else
        temp = 0;
    
    return temp;
}

void LatLonAltToEcef2(MATRIX vector, MATRIX position) {
    
    #define EARTH_RADIUS 6378137         /* earth semi-major axis radius (m) */
            #define ECC2		 0.0066943799901 /* major eccentricity squared */
            double Rn, alt, denom;
    double sinlat, coslat, coslon, sinlon;
    
    
    sinlat = sin(position[0][0]);
    coslat = cos(position[0][0]);
    coslon = cos(position[1][0]);
    sinlon = sin(position[1][0]);
    alt = position[2][0];
    
    denom = (1.0 - (ECC2 * pow(sinlat, 2)));
    denom = fabs(denom);
    
    Rn = EARTH_RADIUS / sqrt(denom);
    
    vector[0][0] = (Rn + alt) * coslat * coslon;
    vector[1][0] = (Rn + alt) * coslat * sinlon;
    vector[2][0] = (Rn * (1.0 - ECC2) + alt) * sinlat;
}

double distance2waypoint(MATRIX wp_curr, MATRIX pos_ned) {
    return sqrt( pow((wp_curr[0][0]-pos_ned[0][0]), 2) + pow((wp_curr[1][0]-pos_ned[1][0]), 2) );
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// wrap around for -180 and + 180
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double wraparound(double dta) {
    
    //bound heading angle between -180 and 180
    if(dta >  PI) dta -= PI2;
    if(dta < -PI) dta += PI2;
    
    return dta;
}

void close_guidance(void){
	//free memory space
	mat_free(pos_lla);
	mat_free(pos_ecef);
	mat_free(pos_ecef0);
	mat_free(T_ecef2ned);
	mat_free(v_ned);
	mat_free(tmp31);
	mat_free(pos_ned);
	mat_free(WP_goal);    
	e[0] = 0;
	e[1] = 0;
	integrator[0] = 0;
}
