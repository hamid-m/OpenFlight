/*! \file daq.c
 *	\brief Data acquisition source code
 *
 *	\details This file implements the init_daq() and get_daq() functions for the UAV.
 *	\ingroup daq_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: daq.c 1014 2014-01-15 18:54:42Z brtaylor $
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <cyg/posix/pthread.h>
#include <cyg/kernel/kapi.h>
#include <cyg/cpuload/cpuload.h>
#include <cyg/gpt/mpc5xxx_gpt.h>
#include <cyg/io/mpc5xxx_gpio.h>
#include <cyg/io/i2c_mpc5xxx.h>
#include <cyg/io/io.h>
#include <cyg/io/serialio.h>

#include "../globaldefs.h"
#include "../utils/serial_mpc5200.h"
#include "../utils/misc.h"
#include "../utils/scheduling.h"
#include "../extern_vars.h"
#include "AirData/airdata_interface.h"
#include "AirData/airdata_constants.h"
#include "GPS/gps_interface.h"
#include "IMU/imu_interface.h"
#include "ADC/adc_interface.h"
#include "PWM/pwm_interface.h"
#include "GPIO/gpio_interface.h"
#include "../navigation/nav_interface.h"
#include "daq_interface.h"
#include AIRCRAFT_UP1DIR


#if(defined(AIRCRAFT_THOR) || defined(AIRCRAFT_TYR) || defined(AIRCRAFT_FASER) || defined(AIRCRAFT_IBIS) || defined(AIRCRAFT_BALDR))
	/// Arrays of calibration coefficients using macros defined in aircraft/XXX_config.h.
	static double pwm_dthr_cal[] = PWMIN_DTHR_CAL;
	static double pwm_de_cal[] = PWMIN_DE_CAL;
	static double pwm_dr_cal[] = PWMIN_DR_CAL;
	static double pwm_da_l_cal[] = PWMIN_DA_L_CAL;
	static double pwm_da_r_cal[] = PWMIN_DA_R_CAL;

	/// Compute order of polynomial calibration (length of array - 1)
	static int pwm_dthr_ord = sizeof(pwm_dthr_cal)/sizeof(*pwm_dthr_cal) - 1;
	static int pwm_de_ord   = sizeof(pwm_de_cal)/sizeof(*pwm_de_cal) - 1;
	static int pwm_dr_ord   = sizeof(pwm_dr_cal)/sizeof(*pwm_dr_cal) - 1;
	static int pwm_da_l_ord = sizeof(pwm_da_l_cal)/sizeof(*pwm_da_l_cal) - 1;
	static int pwm_da_r_ord = sizeof(pwm_da_r_cal)/sizeof(*pwm_da_r_cal) - 1;
#endif



/// Low Pass Filter for speed and altitude signals initialization
static double u_alt[2] = {0,0}; //input of altitude low pass filter { u(k), u(k-1) }
static double y_alt[2] = {0,0}; //output of altitude low pass filter { y(k), y(k-1) }

static double u_speed[2] = {0,0}; //input of altitude low pass filter { u(k), u(k-1) }
static double y_speed[2] = {0,0}; //output of altitude low pass filter { y(k), y(k-1) }


double lp_filter(double signal, double *u, double *y)
{
	const int m=1;  //m = order of denominator of low pass filter

	u[m] = signal;

	y[m] = 0.9608*y[m-1] + 0.0392*u[m-1];	// these coefficients come from a discretized low pass filter with a pole at 2 rad/sec

	u[m-1] = u[m];		// initialize past values for next frame
	y[m-1] = y[m];

	return y[m];
}


void init_daq(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr)
{
	// Assign GPS serial port info.
	sensorData_ptr->gpsData_ptr->baudRate = GPS_BAUDRATE;
	sensorData_ptr->gpsData_ptr->portName = GPS_PORT;
	
	#ifdef AIRCRAFT_IBIS
		sensorData_ptr->gpsData_l_ptr->baudRate = GPS_BAUDRATE_L;
		sensorData_ptr->gpsData_l_ptr->portName = GPS_PORT_L;		
		init_gps(sensorData_ptr->gpsData_l_ptr);		

		sensorData_ptr->gpsData_r_ptr->baudRate = GPS_BAUDRATE_R;
		sensorData_ptr->gpsData_r_ptr->portName = GPS_PORT_R;		
		init_gps(sensorData_ptr->gpsData_r_ptr);				
	#endif
	
	/* Initialize sensors */
	init_gps(sensorData_ptr->gpsData_ptr);		/* GPS */
	init_imu();		/* IMU */
	init_airdata();		/* Ps,Pd */			
	init_gpio();	

	#if(defined(AIRCRAFT_FASER) || defined(AIRCRAFT_IBIS) || defined(AIRCRAFT_BALDR))
		init_adc();
	#endif
		
	// initialize as no data, no lock
	sensorData_ptr->gpsData_ptr->err_type = got_invalid;
	sensorData_ptr->gpsData_l_ptr->err_type = got_invalid;
	sensorData_ptr->gpsData_r_ptr->err_type = got_invalid;
	sensorData_ptr->gpsData_ptr->navValid = 1;
	sensorData_ptr->gpsData_l_ptr->navValid = 1;
	sensorData_ptr->gpsData_r_ptr->navValid = 1;
	navData_ptr->err_type = got_invalid;
    // initialize GPS_TOW to 0, to ensure old_GPS_TOW is set to 0 initially when checking if GPS data is new
	sensorData_ptr->gpsData_ptr->GPS_TOW = 0;
	sensorData_ptr->gpsData_l_ptr->GPS_TOW = 0;
	sensorData_ptr->gpsData_r_ptr->GPS_TOW = 0;

}

void get_daq(struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr){		

	// local pointers to keep things tidy
	struct imu *imuData_ptr = sensorData_ptr->imuData_ptr;
	struct gps *gpsData_ptr = sensorData_ptr->gpsData_ptr;
	struct airdata *adData_ptr = sensorData_ptr->adData_ptr;	
	
	#ifdef AIRCRAFT_IBIS
		struct gps *gpsData_l_ptr = sensorData_ptr->gpsData_l_ptr;
		struct gps *gpsData_r_ptr = sensorData_ptr->gpsData_r_ptr;
		double alpha_cl, beta_cl;
		uint16_t signal_counts[24];
		uint16_t pwm_signals[8];
		struct surface *surfData_ptr = sensorData_ptr->surfData_ptr;
	#endif	
	
	#ifdef AIRCRAFT_FASER
		double alpha_cl, beta_cl;
		uint16_t signal_counts[24];
		uint16_t pwm_signals[8];
		struct surface *surfData_ptr = sensorData_ptr->surfData_ptr;
	#endif

	#ifdef AIRCRAFT_BALDR
		double alpha_cl, beta_cl;
		uint16_t signal_counts[24];
		uint16_t pwm_signals[8];
		struct surface *surfData_ptr = sensorData_ptr->surfData_ptr;
	#endif

	#ifdef AIRCRAFT_THOR
		double alpha_cl, beta_cl;
		uint16_t pwm_signals[4];
	#endif

	#ifdef AIRCRAFT_TYR
		uint16_t pwm_signals[4];
	#endif

	int adstatus;

	// IMU Sensor
	read_imu(imuData_ptr);

	// **** IMU Equations ****
	// TODO: correct accels for CG offset
	// **** End IMU ****

	//**** Air Data Sensors ****			
	adstatus = read_airdata(adData_ptr);

	// Set the status bits
	adData_ptr->status &=  0xFF0; // reset the lowest 4 bits.
	adData_ptr->status |=  (uint16_t)adstatus; // set new status.
	
	// Compute pressure altitude; bias removal results in AGL altitude
	adData_ptr->h = AIR_DATA_K1*(1-pow((adData_ptr->Ps)/AIR_DATA_P0,AIR_DATA_K2)) - adData_ptr->bias[0]; //[ m ]
	
	// Compute Indicated Airspeed (IAS). This equation accounts for compressibility effects. Sensor bias is removed prior to calculation
	adData_ptr->ias = copysign(AIR_DATA_K3*sqrt(fabs(pow(fabs((adData_ptr->Pd-adData_ptr->bias[1])/AIR_DATA_P0 +1),AIR_DATA_K4)-1)),adData_ptr->Pd); 


    // Filter altitude and airspeed signals
	adData_ptr->h_filt = lp_filter(adData_ptr->h, u_alt, y_alt);  	    // filtered ALTITUDE
	adData_ptr->ias_filt   = lp_filter(adData_ptr->ias, u_speed, y_speed);	// filtered AIRSPEED



	#if(defined(AIRCRAFT_THOR) || defined(AIRCRAFT_FASER) || defined(AIRCRAFT_IBIS) || defined(AIRCRAFT_BALDR))
		// Only THOR, FASER and IBIS have 5-hole pitot probes.
		// Compute angle of attack
		alpha_cl = (adData_ptr->Pd_aoa-adData_ptr->bias[2])/(mymax(adData_ptr->Pd-adData_ptr->bias[1],0.02)); // Minimum dynamic pressure at +0.02KPa (10 m/s) to prevent div/zero
		adData_ptr->aoa = (0.9898*pow(alpha_cl,3) - 0.0666*pow(alpha_cl,2) + 13.566*alpha_cl + 0.5297)*D2R;

		// Compute angle of sideslip
		beta_cl = (adData_ptr->Pd_aos-adData_ptr->bias[3])/(mymax(adData_ptr->Pd-adData_ptr->bias[1],0.02));  // Minimum dynamic pressure at +0.02KPa (10 m/s) to prevent div/zero
		adData_ptr->aos = beta_cl/0.068*D2R;

		// Correct pitot aoa/aos for angular rates and installation biases
		adData_ptr->aoa += PITOT_ALPHA_BIAS + PITOT2CG_X*imuData_ptr->q/mymax(adData_ptr->ias,10) - PITOT2CG_Y*imuData_ptr->p/mymax(adData_ptr->ias,5);
		adData_ptr->aos += PITOT_BETA_BIAS - PITOT2CG_X*imuData_ptr->r/mymax(adData_ptr->ias,10) + PITOT2CG_Z*imuData_ptr->p/mymax(adData_ptr->ias,5);
	#endif
	//**** End Air Data ****


	
	#if(defined(AIRCRAFT_IBIS) || defined(AIRCRAFT_FASER) || defined(AIRCRAFT_BALDR))		// Read ADC chip (alpha/beta vanes, control surface positions)
		adstatus = read_adc(&signal_counts[0]);
/*		
		// Reset upper 6 status bits
		adData_ptr->status &= 0x3FF;	
		adData_ptr->status |=  (uint16_t)adstatus; // set new status.

		// Convert signals in counts to engineering units. Calibration parameters set in /aircraft/faser_config.h
		adData_ptr->l_alpha = L_ALPHA1_SLOPE*((double)signal_counts[6]) + L_ALPHA1_BIAS;
		adData_ptr->l_beta = L_BETA_SLOPE*((double)signal_counts[4]) + L_BETA_BIAS;
		
		adData_ptr->r_alpha = R_ALPHA1_SLOPE*((double)signal_counts[7]) + R_ALPHA1_BIAS;
		adData_ptr->r_beta = R_BETA_SLOPE*((double)signal_counts[5]) + R_BETA_BIAS;

		// Correct vane aoa/aos for angular rates and installation biases
		adData_ptr->l_alpha += L_VANE_ALPHA_BIAS + L_VANE2CG_X*imuData_ptr->q/mymax(adData_ptr->ias,10) - L_VANE2CG_Y*imuData_ptr->p/mymax(adData_ptr->ias,5);
		adData_ptr->r_alpha += R_VANE_ALPHA_BIAS + R_VANE2CG_X*imuData_ptr->q/mymax(adData_ptr->ias,10) - R_VANE2CG_Y*imuData_ptr->p/mymax(adData_ptr->ias,5);

		adData_ptr->l_beta += L_VANE_BETA_BIAS - L_VANE2CG_X*imuData_ptr->r/mymax(adData_ptr->ias,10) + L_VANE2CG_Z*imuData_ptr->p/mymax(adData_ptr->ias,5);
		adData_ptr->r_beta += R_VANE_BETA_BIAS - R_VANE2CG_X*imuData_ptr->r/mymax(adData_ptr->ias,10) + R_VANE2CG_Z*imuData_ptr->p/mymax(adData_ptr->ias,5);

		// TODO: set channels for new ADC wiring.
		surfData_ptr->dr_pos = DR_SLOPE*(double)signal_counts[3] + DR_BIAS;
		surfData_ptr->de_pos = DE_SLOPE*(double)signal_counts[11] + DE_BIAS;
		surfData_ptr->da_l_pos = DA_L_SLOPE*(double)signal_counts[14] + DA_L_BIAS;	//10,11,12,13
		surfData_ptr->da_r_pos = DA_R_SLOPE*(double)signal_counts[13] + DA_R_BIAS;
		surfData_ptr->df_l_pos = DF_L_SLOPE*(double)signal_counts[15] + DF_L_BIAS;
		surfData_ptr->df_r_pos = DF_R_SLOPE*(double)signal_counts[12] + DF_R_BIAS;
	*/	
		adData_ptr->r_beta = (double)signal_counts[3];
		adData_ptr->r_alpha = (double)signal_counts[5];
		adData_ptr->l_beta = (double)signal_counts[7];
		adData_ptr->l_alpha = (double)signal_counts[11];
		surfData_ptr->da_l_pos = (double)signal_counts[13];
		surfData_ptr->da_r_pos = (double)signal_counts[15];
		surfData_ptr->dr_pos = (double)signal_counts[19];
		surfData_ptr->de_pos = (double)signal_counts[21];
		
	#endif


	#if(defined(AIRCRAFT_THOR) || defined(AIRCRAFT_TYR) ||defined(AIRCRAFT_FASER)  || defined(AIRCRAFT_IBIS) || defined(AIRCRAFT_BALDR))
		// Read 4 PWM signals from ATMega328
		read_pwm(&pwm_signals[0]);

		// Apply calibration equations
		controlData_ptr->dthr_in = 	polyval(pwm_dthr_cal, (double)pwm_signals[PWMIN_DTHR_CH],pwm_dthr_ord);
		controlData_ptr->de_in = 	polyval(pwm_de_cal, (double)pwm_signals[PWMIN_DE_CH]/PWMIN_SCALING,pwm_de_ord);
		controlData_ptr->dr_in = 	polyval(pwm_dr_cal, (double)pwm_signals[PWMIN_DR_CH]/PWMIN_SCALING,pwm_dr_ord);
		controlData_ptr->da_l_in = 	polyval(pwm_da_l_cal, (double)pwm_signals[PWMIN_DA_L_CH]/PWMIN_SCALING,pwm_da_l_ord);
		controlData_ptr->da_r_in = 	polyval(pwm_da_r_cal, (double)pwm_signals[PWMIN_DA_R_CH]/PWMIN_SCALING,pwm_da_r_ord);

		#endif



	//**** GPS Sensor(s) ****
	if(-1 == read_gps(gpsData_ptr)){
		if(++gpsData_ptr->read_calls > BASE_HZ)
			gpsData_ptr->err_type = got_invalid;
	}else {gpsData_ptr->read_calls=0;	}

	#ifdef AIRCRAFT_IBIS
		if(-1 == read_gps(gpsData_l_ptr)){
			if(++gpsData_l_ptr->read_calls > BASE_HZ)
				gpsData_l_ptr->err_type = got_invalid;
		}else {gpsData_l_ptr->read_calls=0;}
		
		if(-1 == read_gps(gpsData_r_ptr)){
			if(++gpsData_r_ptr->read_calls > BASE_HZ)
				gpsData_r_ptr->err_type = got_invalid;
		}else {gpsData_r_ptr->read_calls=0;}
	#endif
	//**** End GPS ****

	// Read GPIOs; data dump, control mode
	read_gpio(controlData_ptr);
	
}

