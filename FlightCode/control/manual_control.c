/*! \file manual_control.c
 *	\brief Manual control source code
 *
 *	\details Manual control commands are passed directly through.
 *
 *	\ingroup control_fcns
 *
 *	\author University of Minnesota
 *	\author Aerospace Engineering and Mechanics
 *	\copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: manual_control.c 797 2012-04-16 15:41:52Z murch $
 */

#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "control_interface.h" 

/// Return control outputs based on references and feedback signals.
extern void get_control(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct control *controlData_ptr) {

	controlData_ptr->dthr = controlData_ptr->dthr_in; // throttle
    controlData_ptr->de = controlData_ptr->de_in; // Elevator deflection [rad]
    controlData_ptr->dr = controlData_ptr->dr_in; // Rudder deflection [rad]
	controlData_ptr->da_l = controlData_ptr->da_l_in; // left aileron
    controlData_ptr->da_r = controlData_ptr->da_r_in; // Aileron deflection [rad], right

	subtract_trim_bias(controlData_ptr);
}

// Reset of controller
extern void reset_control(struct control *controlData_ptr){
	// Here: code to reset the controller
	controlData_ptr->dthr = 0; // throttle
	controlData_ptr->de   = 0; // elevator
	controlData_ptr->dr   = 0; // rudder
	controlData_ptr->da_l = 0; // left aileron
	controlData_ptr->da_r = 0; // right aileron
	controlData_ptr->df_l = 0; // left flap
	controlData_ptr->df_r = 0; // right flap	

}
