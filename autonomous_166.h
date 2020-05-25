#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "camera.h"
#include "encoder.h"

/***********************************************************************
Header:			Autonomous_166.h 
Date modified:  03-03-07
Header name:  	autonomous_166.h
Used in:		autonomous_166.c, camera_166.c
**********************************************************************/
/* def of gripper_open was changed in manip.h so stable def is here */
#define AUTO_GRIPPER_OPEN				0
#define AUTO_GRIPPER_CLOSED				1

/* MISSION constants */
#define MISSION_NOT_INITIALIZED 0
#define DEAD_RECKONING          7
#define MISSION_CLOSING_ON_GRIPPER 8
#define MISSION_INITIALIZING	1
#define MISSION_ACTIVATED 		2
#define MISSION_STAGE1 			3
#define MISSION_ABORTED 		4
#define MISSION_ACCOMPLISHED 	5
#define MISSION_COMPLETED	 	6


/* which light are we going after? */
#define GOAL_LEFT 		0
#define GOAL_CENTER 	1
#define GOAL_RIGHT 		2

/* CASEs */
/* position on the field */
#define FAR_STATE_1 0
#define FAR_STATE_2 1
#define FAR_STATE_3 2
#define MIDRANGE_STATE_1 3
#define MIDRANGE_STATE_2 4
#define MIDRANGE_STATE_3 5
#define CLOSE_STATE_1 6
#define CLOSE_STATE_2 7
#define CLOSE_STATE_3 8
#define THERE 9
#define CLOSE 10

//Dead Reckoning Counters
#define DEAD_COUNT_BURST 32		// burst power
#define DEAD_COUNT_MIN 150		// burst power
#define DEAD_COUNT_MAX 210		// drive slower til in camera view

//Switch limits
//Switch Position 0 is not used to value flakiness

#define SWITCH_STATE_1_MIN 0
#define SWITCH_STATE_1_MAX 10

#define SWITCH_STATE_2_MIN 85
#define SWITCH_STATE_2_MAX 95

#define SWITCH_STATE_3_MIN 175
#define SWITCH_STATE_3_MAX 195

#define SWITCH_STATE_4_MIN 380
#define SWITCH_STATE_4_MAX 405

#define SWITCH_STATE_5_MIN 590
#define SWITCH_STATE_5_MAX 610

#define SWITCH_STATE_6_MIN 790
#define SWITCH_STATE_6_MAX 820

//definitions for autonomous switch
// use unsigned char but treat these as constants
#define AUTO_DO_NOTHING			1
#define AUTO_ARM_LOW	        2 
#define AUTO_FWD_FLOOR          3       
#define AUTO_SCORE_MID          4
#define AUTO_REV_MOVE_ARM		5
#define AUTO_ARM_FLOOR          6 

//#define AUTO_MID_CENTER 	    2
//#define AUTO_LOW_CENTER 	    3
//#define AUTO_MID_LEFT	 	3
//#define AUTO_MID_RIGHT 		4
//#define AUTO_LOW_LEFT	 	6
//#define AUTO_LOW_RIGHT 		7
//#define AUTO_HIGH_CENTER 	8
//#define AUTO_HIGH_LEFT 		9
//#define AUTO_HIGH_RIGHT 	10


// function prototypes
void do_auto(void);
void auto_state_machine(void);
void auto_drive(void);
unsigned char determine_goal(void);
void do_camera(void);
unsigned int auto_switch_find(void);
void go_and_move_arm(unsigned char);
void do_auto_arm(unsigned char);
