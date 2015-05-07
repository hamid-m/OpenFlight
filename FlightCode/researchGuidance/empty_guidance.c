/*! \file straight_level.c
 *	\brief Straight and level pitch/roll commands
 *
 *	\details
 *	\ingroup guidance_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: straight_level.c 860 2012-07-18 18:27:25Z joh07594 $
 */

#include <stdlib.h>
#include <math.h>

#include "../globaldefs.h"
#include "researchguidance_interface.h"

extern void get_researchGuidance(double time, struct sensordata *sensorData_ptr, struct nav *navData_ptr, struct researchControl *researchControlData_ptr, struct mission *missionData_ptr){

		researchControlData_ptr->phi_cmd = 0;
		researchControlData_ptr->theta_cmd = 0;
	}

void close_researchGuidance(void){
  //free memory space.  For example, use `mat_free`, 
  // reset any initialization flags or integrators.
}