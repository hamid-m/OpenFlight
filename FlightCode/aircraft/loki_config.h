/*! \file loki_config.h
 *	\brief Configuration defines for the Loki aircraft.
 *
 *	\details This file contains the geometry, surface calibrations and limits, and serial port configuration for the Loki aircraft.
 *	\ingroup aircraft_cfg
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: loki_config.h 756 2012-01-04 18:51:43Z murch $
 */
#ifndef LOKI_CONFIG_H_
#define LOKI_CONFIG_H_

#define AIRCRAFT_LOKI	///< Aircraft name. Used in daq() for sensor processing.

// GPS Sensor Configuration
#define GPS_PORT 		SERIAL_PORT2	///< Serial port for GPS receiver, used in init_daq()
#define GPS_BAUDRATE	B57600			///< Baud rate of serial port for GPS receiver, used in init_daq()

// Downlink telemetry configuration
#define TELEMETRY_PORT SERIAL_PORT3		///< Serial port for telemetry()
#define TELEMETRY_BAUDRATE B115200		///< Baud rate of serial port for telemetry()

// Control Surface Trims
#define ELEVATOR_TRIM  -0.0314 ///< [rad], approximate elevator trim value
#define AILERON_TRIM    0.0175 ///< [rad], approximate aileron trim value
#define RUDDER_TRIM     0.0 ///< [rad], approximate rudder trim value
#define THROTTLE_TRIM   0.7 ///< [ND], approximate throttle trim value


// *****  Need to verify aileron channel assignments! *****

// Control Surface Channel Assignments
#define PWM_THROTTLE_CH  4 ///<  PWM channel for throttle
#define PWM_ELEVATOR_CH  3 ///<  PWM channel for elevator
#define PWM_RUDDER_CH 	 5 ///<  PWM channel for rudder
#define PWM_L_AILERON_CH 7 ///<  PWM channel for left aileron
#define PWM_R_AILERON_CH 2 ///<  PWM channel for right aileron
#define PWM_L_FLAP_CH 	 0 ///<  PWM channel for left flap
#define PWM_R_FLAP_CH	 1 ///<  PWM channel for right flap

// Control surface PWM calibration parameters
#define PWM_THROTTLE_CAL  { 2180,    2997} ///< linear calibration for throttle
#define PWM_ELEVATOR_CAL  { 2498.28, 4087} ///< linear calibration for elevator
#define PWM_RUDDER_CAL    {-2498.28, 4087} ///< linear calibration for rudder
#define PWM_L_AILERON_CAL { 2498.28, 4087} ///< linear calibration for left aileron
#define PWM_R_AILERON_CAL {-2498.28, 4087} ///< linear calibration for right aileron
#define PWM_L_FLAP_CAL    { 2498.28, 4087} ///< linear calibration for left flap
#define PWM_R_FLAP_CAL    { 2498.28, 4087} ///< linear calibration for right flap

// Control Surface Limits, max and min
#define 	THROTTLE_MAX 	 1.0
#define		THROTTLE_MIN	 0.0
#define		RUDDER_MAX		 0.4363	///< [rad], 25deg
#define		RUDDER_MIN		-0.4363 ///< [rad],-25deg
#define		ELEVATOR_MAX	 0.4363	///< [rad], 25deg
#define		ELEVATOR_MIN	-0.4363 ///< [rad],-25deg
#define 	L_AILERON_MAX	 0.4363	///< [rad], 25deg
#define 	L_AILERON_MIN	-0.4363 ///< [rad],-25deg
#define 	R_AILERON_MAX	 0.4363	///< [rad], 25deg
#define 	R_AILERON_MIN	-0.4363 ///< [rad],-25deg
#define 	L_FLAP_MAX	 	 0.4363	///< [rad], 25deg
#define 	L_FLAP_MIN		-0.4363 ///< [rad],-25deg
#define 	R_FLAP_MAX	 	 0.4363	///< [rad], 25deg
#define 	R_FLAP_MIN		-0.4363	///< [rad],-25deg
	
#endif	
