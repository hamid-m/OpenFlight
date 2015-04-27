
#ifndef MISSION_INTERFACE_H_
#define MISSION_INTERFACE_H_

/// Standard function to call the mission manager
extern void run_mission(
		struct sensordata *sensorData_ptr,	///< pointer to sensorData structure
		struct nav *navData_ptr,			///< pointer to navData structure
		struct mission *missionData_ptr		///< pointer to missionData structure
);

#endif /* MISSION_INTERFACE_H_ */
