#include <stdio.h>

#include "manipulator.h"
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "user_routines.h"
#include "camera.h"
#include "autonomous_166.h"
#include "camera_166.h"
#include "crab_drive.h"  
#include "manipulator.h"
/*******************************************************************************
*
*	TITLE:		autonomous_166.c
*
*	DATE:		03-Feb-2007
*
*	COMMENTS:
*	
* camera ranges
*     Min = (1,1)      
*     Center = (80,120)
*     Max = (159,239)
*******************************************************************************/
// look at these vars from user_routine_fast.c
extern unsigned char autonomous_goal;  				// default
extern unsigned char auto_manip_position; 			// middle position

// current state of autonomous 
static unsigned char auto_state = MISSION_NOT_INITIALIZED;

// goal of autonomous (left, center, right)
unsigned char goal=GOAL_CENTER;

// robot position on field
unsigned char position;			

// direction of light: t=center(target), l=left, r=right
unsigned char light_direction = 't';

// is there only one light in view? TRUE/FALSE
unsigned char one_light;
	
unsigned char counter=0;
unsigned char keeper_placed = FALSE;
unsigned char target_not_seen = FALSE;
unsigned char error_reported = FALSE;

// counters
static unsigned char print_count = 100;	//so will print first time
static unsigned char camera_count = 0;
static unsigned char there_count = 0;
static unsigned char abort_count = 0;
static unsigned int dead_count = 0;
static unsigned char dead_print_count = 30;	//so will print first time

#define ENCODER_DEAD_VALUE 150

// TIMING PARAMETERS - treat as constants
#define RETREAT_COUNTS				500
#define PRINT_RESET					400
#define POSITION_DEBUG_RESET		50
#define ABORT_NUM					500

// number of loops until camera is read
#define CAM_START					50

// DRIVE PARAMETERS 
//no power
#define no_pwr	127	
// set up drive parameters
unsigned char drive_x = 127;
unsigned char drive_y = 127;
unsigned char drive_z = 127;
//fwd drive and reverse drive parameters
#define REVERSE_AUTO_SPEED		64
#define FORWARD_AUTO_SPEED		192
#define DRIVE_LOOP_COUNT	38
#define MANIP_DESIRED_POS	3

// fwd/back drive instructions for different distance from light
// to be adjusted depending on desired speed
// if no x,y or z use no_pwr
unsigned char x_INIT = 127;		//forward while camera is initializing
unsigned char y_INIT = 170;		

// fwd drive while in dead reckoning mode A
unsigned char x_DED_REK_1 = 127;		//forward while camera is initializing
unsigned char y_DED_REK_1 = 162;		

// fwd drive while in dead reckoning mode A
unsigned char x_DED_REK_2 = 127;		//forward while camera is initializing
unsigned char y_DED_REK_2 = 158;		

// FAR1 distance; approach directly (at center)
#define center_x_FAR1	127	
#define center_y_FAR1	160
// FAR distance; approach from left
#define left_x_FAR1		87
#define left_y_FAR1		170
// FAR distance; approach from right
#define right_x_FAR1	167
#define right_y_FAR1	170

// FAR2 distance	
#define center_x_FAR2	127
#define center_y_FAR2	158
#define left_x_FAR2		127
#define left_y_FAR2		170
#define right_x_FAR2	127
#define right_y_FAR2	170

// FAR3 distance
#define center_x_FAR3	127
#define center_y_FAR3	156
#define left_x_FAR3		127
#define left_y_FAR3		170
#define right_x_FAR3	127
#define right_y_FAR3	170

// MIDRANGE1 distance
#define center_x_MIDRANGE1	127
#define center_y_MIDRANGE1	154
#define left_x_MIDRANGE1	97
#define left_y_MIDRANGE1	170
#define right_x_MIDRANGE1	157
#define right_y_MIDRANGE1	170

// MIDRANGE2 distance
#define center_x_MIDRANGE2	127
#define center_y_MIDRANGE2	152
#define left_x_MIDRANGE2	110
#define left_y_MIDRANGE2	160
#define right_x_MIDRANGE2	144
#define right_y_MIDRANGE2	160

// MIDRANGE3 distance
#define center_x_MIDRANGE3	127
#define center_y_MIDRANGE3	150
#define left_x_MIDRANGE3	110
#define left_y_MIDRANGE3	160
#define right_x_MIDRANGE3	144
#define right_y_MIDRANGE3	160

// CLOSE1 distance
#define center_x_CLOSE1	127
#define center_y_CLOSE1	150
#define left_x_CLOSE1	114
#define left_y_CLOSE1	160
#define right_x_CLOSE1	140
#define right_y_CLOSE1	160

// CLOSE2 distance
#define center_x_CLOSE2	127
#define center_y_CLOSE2	140
#define left_x_CLOSE2	117
#define left_y_CLOSE2	160
#define right_x_CLOSE2	134
#define right_y_CLOSE2	160

// CLOSE3 distance
#define center_x_CLOSE3	127
#define center_y_CLOSE3	140
#define left_x_CLOSE3	127
#define left_y_CLOSE3	160
#define right_x_CLOSE3	127
#define right_y_CLOSE3	160

// BACKUP when done
#define center_y_BACK	60

// rotation adjustment for turning toward light
//unsigned char z_adjust = 5;
//until is debugged
#define z_adjust	0
// seeing one light =1, none=0, more=2
unsigned char one_light;

//Encoder Value
long encoder;
#define USE_ENCODER 0
#define END_DEAD_RECKONING 38
#define MAX_GRIP_COUNT 40
#define AUTO_DEBUG 1

/***********************************************************************
Function name:  do_auto
Date modified:  2-03-07
Called From:	should be called from user_routines_fast
Parameters:     unsigned int auto_goal, unsigned int manip_position
Returns: 		void
Purpose: 		Autonomous mode loop processing
Notes: 
**********************************************************************/
void do_auto() { 
	static unsigned char display=TRUE;
	if(display==TRUE)
	{
		printf("do_auto has been called.\r");
		display=FALSE;
	}
	
	Camera_Handler();
	auto_state_machine();
    
    if (auto_state == MISSION_ACTIVATED)
	{
        auto_drive();
    }
}

/***********************************************************************
Function name:  auto_state_machine
Date modified:  2-03-07
Called From:	do_auto
Parameters:     none
Returns: 		void
Purpose: 		Autonomous mode loop processing
Notes: 
**********************************************************************/
void auto_state_machine() {  
 	
	/**************************************************
	* this is c4lled every loop 
	* 
	* Go left or right or center?
	*
	**************************************************/

	static unsigned char loopcount = 0;
	static unsigned char grip_count = 0;
	loopcount++;

	// make sure drive is initialized	
	drive_x = no_pwr;
	drive_y = no_pwr;
	drive_z = no_pwr;

	print_count++;

	// only show data every few loops
	if(print_count >= PRINT_RESET)
	{
		// reset the counter
		print_count = 0;
#if AUTO_DEBUG
		printf("auto_state_machine: %d\r", auto_state);
#endif
	}

	switch (auto_state) 
	{
       	case MISSION_NOT_INITIALIZED:
			/*	Initialize camera */
			Initialize_Camera();

	    	/*    Looks for primarily green light */
//	  		Track_Color(0,100,200,255,0,100);
	  		Track_Color(0,245,200,255,0,245);
	    	    
			// set the local variable from user_routines_fast.c	
	    	goal = autonomous_goal;
			Reset_Encoder_2_Count();

#if AUTO_DEBUG
			printf("auto_drive: MISSION_NOT_INITIALIZED \r"); 
#endif
			// close gripper around keeper
			autonomous_manipulator_master_control(MIDDLE,AUTO_GRIPPER_CLOSED);

	    	auto_state = MISSION_CLOSING_ON_GRIPPER; /* grip the keeper */
			break;

       	case MISSION_CLOSING_ON_GRIPPER:
			grip_count++;
			// close gripper around keeper

#if AUTO_DEBUG
			printf("auto_drive: MISSION_CLOSING_ON_GRIPPER \r"); 
#endif
			autonomous_manipulator_master_control(MIDDLE,AUTO_GRIPPER_CLOSED);

	    	if (grip_count >= MAX_GRIP_COUNT)
			{
				auto_state = DEAD_RECKONING; /* start drive */
			}
			break;

        case DEAD_RECKONING:
//USe either encoder values or loop counter to manage the autonomous period
#if AUTO_DEBUG
			printf("auto_state_machine: DEAD_RECKONING\r");
#endif
#if USE_ENCODER
			encoder=Get_Encoder_2_Count();
			printf("Encoder Value: %ld\r",encoder);

			if((encoder < ENCODER_DEAD_VALUE) && (encoder > -ENCODER_DEAD_VALUE))
			{
				if(encoder<=DEAD_COUNT_BURST)
				{
					crab_drive(x_DED_REK_1,(y_DED_REK_1),no_pwr); 
				}
				else if(encoder <= DEAD_COUNT_MIN)
				{					
					crab_drive(x_DED_REK_1, y_DED_REK_1, no_pwr);
				}
				else if(encoder<=DEAD_COUNT_MAX)
				{
					if(T_Packet_Data.pixels<2)
					{
						auto_state=MISSION_INITIALIZING;
					}										
					crab_drive(x_DED_REK_2, y_DED_REK_2, no_pwr);
				}
				else
				{
					if(T_Packet_Data.pixels<2)
					{
						auto_state=MISSION_ABORTED;
					}
					else
					{
						if(T_Packet_Data.pixels > 2)
						{
							auto_state=MISSION_INITIALIZING;
						}
						else
						{
							auto_state=MISSION_ABORTED;
						}
					}
				}
			}
			else
			{
				printf("Goes here directly\r");
				auto_state=MISSION_INITIALIZING;
#if AUTO_DEBUG
			printf("going to MISSION INITIALIZING...\r");
#endif
				dead_count=0;
			}
#else
			dead_count++;
			if(dead_count < END_DEAD_RECKONING)
			{
				crab_drive(no_pwr, FORWARD_AUTO_SPEED, no_pwr);									
			}
			else
			{	
					auto_state=MISSION_ABORTED;			
			}	
			if(T_Packet_Data.pixels>2)
			{
					auto_state=MISSION_INITIALIZING;
#if AUTO_DEBUG
			printf("going to MISSION INITIALIZING...\r");
#endif	
			}	
			break;
				
		case MISSION_INITIALIZING:
			//move fwd a bit
		
			// start reading camera and change states if ready
			if (camera_count <= CAM_START)
			{
				camera_count++;
			}
			else
			{
	    	   	do_camera();
	
			    auto_state = MISSION_ACTIVATED; /* init complete */
#if AUTO_DEBUG
				printf("auto_state_machine: MISSION_ACTIVATED");
#endif
			}
			break;
    
		case MISSION_ACTIVATED: 

			if (keeper_placed == TRUE) 
			{
					auto_state = MISSION_ACCOMPLISHED;
#if AUTO_DEBUG
				printf("auto_state_machine: MISSION_ACCOMPLISHED\r");
#endif
			}
			else if (abort_count == ABORT_NUM)
			{
				auto_state = MISSION_ABORTED;
				printf("auto_state_machine: MISSION_ABORT!!! NO LIGHT\r");
			}
			else
			{
				camera_count++;
				do_camera();
        		auto_drive();
			}
			break;

		case MISSION_ACCOMPLISHED:
			// move backwards for x counts
			if (there_count == 0)
			{
				// first time around
#if AUTO_DEBUG
				printf("auto_state_machine: RETREATING");
#endif
				crab_drive(no_pwr, center_y_BACK, no_pwr);
			}
			if (there_count > RETREAT_COUNTS)
			{
				auto_state = MISSION_COMPLETED;
#if AUTO_DEBUG
				printf("auto_state_machine: MISSION_COMPLETED\r");
#endif
			}
			there_count++;
			break;

		case MISSION_ABORTED:
			// no operations here
			crab_drive(no_pwr, no_pwr, no_pwr);
			dead_print_count ++;
			if (dead_print_count>= 50)
			{
				printf("mission aborted state/r");
				dead_print_count = 0;
			}	
			break;

		case MISSION_COMPLETED:
			// no operations here
			crab_drive(no_pwr, no_pwr, no_pwr);
			dead_print_count ++;
			if (dead_print_count>= 50)
			{
				printf("mission complete state/r");
				dead_print_count = 0;
			}	
			break;
			break;

		default:
			printf("error in auto_state_machine, auto_state= %d\r", auto_state);
			break;
	}
}


/***********************************************************************
Function name:  auto_drive
Date modified:  2-03-07
Called From:	do_auto
Parameters:     none
Returns: 		void
Purpose: 		Autonomous mode loop processing
Notes: 
**********************************************************************/
void auto_drive() {

	static unsigned char drive_count = 0;
	// Initialize drive
	drive_x = no_pwr;
	drive_y = no_pwr;
	drive_z = no_pwr;
	
	// this is called every loop while active

  	position = determine_closeness(T_Packet_Data.my);
  	light_direction = target_at_center(T_Packet_Data.mx);

	drive_count++;
	// only show data every 10 loops
	if(drive_count >= POSITION_DEBUG_RESET)
	{
		// reset the counter
		drive_count = 0;
#if AUTO_DEBUG
//		printf("auto_drive: position state= %d\r", position);
#endif
	}	

	/**************************************************
	* this is c4lled every loop 
	* Set up drive parameters
	* Go left or right or center?	
	**************************************************/
	switch (position) { 
	    case FAR_STATE_1:	// near base
	        //move toward target
	        //call joystick x,y,0 if center of mass is reasonably close to 
			//center
#if AUTO_DEBUG
			if (print_count == 0) {
				printf("auto_drive: FAR_1\r");
			}
#endif

			// z value will be adjusted to focus on camera
		    if(goal==GOAL_CENTER) { 
				drive_x = center_x_FAR1;
				drive_y = center_y_FAR1;
		    }
		    if(goal==GOAL_LEFT) {
				drive_x = left_x_FAR1;
				drive_y = left_y_FAR1;
		    }
		    if(goal==GOAL_RIGHT) {
				drive_x = right_x_FAR1;
				drive_y = right_y_FAR1;
		    }
			break;

		case FAR_STATE_2:
#if AUTO_DEBUG
			if (print_count == 0) {
				printf("auto_drive: FAR_2\r");
			}
#endif
		    if(goal==GOAL_CENTER) { 
				drive_x = center_x_FAR2;
				drive_y = center_y_FAR2;
		    }
		    if(goal==GOAL_LEFT) {
				drive_x = left_x_FAR2;
				drive_y = left_y_FAR2;
		    }
		    if(goal==GOAL_RIGHT) {
				drive_x = right_x_FAR2;
				drive_y = right_y_FAR2;
		    }
			break;

		case FAR_STATE_3:
#if AUTO_DEBUG
			if (print_count == 0) {
				printf("auto_drive: FAR_3\r");
			}
#endif
		    if(goal==GOAL_CENTER) { 
				drive_x = center_x_FAR3;
				drive_y = center_y_FAR3;
		    }
		    if(goal==GOAL_LEFT) {
				drive_x = left_x_FAR3;
				drive_y = left_y_FAR3;
		    }
		    if(goal==GOAL_RIGHT) {
				drive_x = right_x_FAR3;
				drive_y = right_y_FAR3;
		    }
			break;
	
	    case MIDRANGE_STATE_1:	//middle of field
#if AUTO_DEBUG
			if (print_count == 0) {
				printf("auto_drive: MIDRANGE_1\r"); 
			}	
#endif	       
			// strafe toward desired goal, turn toward camera
		    if(goal==GOAL_CENTER) { 
				drive_x = center_x_MIDRANGE1;
				drive_y = center_y_MIDRANGE1;
		    }
		    if(goal==GOAL_LEFT) {
				drive_x = left_x_MIDRANGE1;
				drive_y = left_y_MIDRANGE1;
		    }
		    if(goal==GOAL_RIGHT) {
				drive_x = right_x_MIDRANGE1;
				drive_y = right_y_MIDRANGE1;
		    }

			//adjust the manipulator

#if AUTO_DEBUG
			if (print_count == 1) {
				printf("auto_drive: ADJUSTING MANIPULATOR \r"); 
			}	
#endif	       
			autonomous_manipulator_master_control(MIDDLE, AUTO_GRIPPER_CLOSED);
			break;

		case MIDRANGE_STATE_2:
#if AUTO_DEBUG
			if (print_count == 0) {
				printf("auto_drive: MIDRANGE_2\r");
			}
#endif
		    if(goal==GOAL_CENTER) { 
				drive_x = center_x_MIDRANGE2;
				drive_y = center_y_MIDRANGE2;
		    }
		    if(goal==GOAL_LEFT) {
				drive_x = left_x_MIDRANGE2;
				drive_y = left_y_MIDRANGE2;
		    }
		    if(goal==GOAL_RIGHT) {
				drive_x = right_x_MIDRANGE2;
				drive_y = right_y_MIDRANGE2;
		    }
			break;

		case MIDRANGE_STATE_3:
#if AUTO_DEBUG
			if (print_count == 0) {
				printf("auto_drive: MIDRANGE_3\r");
			}
#endif
		    if(goal==GOAL_CENTER) { 
				drive_x = center_x_MIDRANGE3;
				drive_y = center_y_MIDRANGE3;
		    }
		    if(goal==GOAL_LEFT) {
				drive_x = left_x_MIDRANGE3;
				drive_y = left_y_MIDRANGE3;
		    }
		    if(goal==GOAL_RIGHT) {
				drive_x = right_x_MIDRANGE3;
				drive_y = right_y_MIDRANGE3;
		    }
			break;

		case CLOSE_STATE_1:
#if AUTO_DEBUG
			if (print_count == 0) {
				printf("auto_drive: CLOSE_1\r"); 
			}
#endif
			if(light_direction=='t') 
			{
				drive_x = center_x_CLOSE1;
				drive_y = center_y_CLOSE1;
			}
			else if(light_direction=='l')
			{
				drive_x = left_x_CLOSE1;
				drive_y = left_y_CLOSE1;
			}
			else if (light_direction=='r')
			{
				drive_x = right_x_CLOSE1;
				drive_y = right_y_CLOSE1;
			}
			break;

		case CLOSE_STATE_2:
#if AUTO_DEBUG
			if (print_count == 0) {
				printf("auto_drive: CLOSE_2\r"); 
			}
#endif
			if(light_direction=='t') 
			{
				drive_x = center_x_CLOSE2;
				drive_y = center_y_CLOSE2;
			}
			else if(light_direction=='l')
			{
				drive_x = left_x_CLOSE2;
				drive_y = left_y_CLOSE2;
			}
			else if (light_direction=='r')
			{
				drive_x = right_x_CLOSE2;
				drive_y = right_y_CLOSE2;
			}
			break;
			
		
		case CLOSE_STATE_3:
#if AUTO_DEBUG
			if (print_count == 0) {
				printf("auto_drive: CLOSE_3\r"); 
			}
#endif
			if(light_direction=='t') 
			{
				drive_x = center_x_CLOSE3;
				drive_y = center_y_CLOSE3;
			}
			else if(light_direction=='l')
			{
				drive_x = left_x_CLOSE3;
				drive_y = left_y_CLOSE3;
			}
			else if (light_direction=='r')
			{
				drive_x = right_x_CLOSE3;
				drive_y = right_y_CLOSE3;
			}
			break;
	
	    case THERE: // on target
			// don't drive
			//ASSUME that this = true: (if manipulator adjust complete)

			there_count++;
			
#if AUTO_DEBUG
			if (print_count == 0) {
				printf("auto_drive: THERE %d\r", there_count); 
			}
#endif

			// 0p3n 73h m4n1pu|470r 4nd p|4(3 73h k33p3r 0|\| 5p1d3r |_36
			// wait a bit 
			if (there_count > 20)
			{
				// open up the gripper, place keeper on target 
#if AUTO_DEBUG
				printf("auto_drive: PLACING KEEPER \r"); 
#endif
				autonomous_manipulator_master_control(MIDDLE,AUTO_GRIPPER_OPEN);
				keeper_placed = TRUE;
				there_count = 0;
			}
			break;
	}
	
	// adjust z parameter based on target position
	if (light_direction == 'l')
	{
		drive_z = no_pwr - z_adjust;
	}
	else if (light_direction == 'r')	
	{
		drive_z = no_pwr + z_adjust;
	}

#if AUTO_DEBUG
	if (print_count==1)
	{
		printf("auto_drive: calling crab_drive %d, %d, %d, light direction = %c\r", 
				drive_x, drive_y, drive_z, light_direction);
	}
#endif
	crab_drive(drive_x, drive_y, drive_z);
}


/***********************************************************************
Function name:  do_camera
Date modified:  2-04-07
Called From:	do_auto
Parameters:     none
Returns: 		
Purpose: 		Alternatively checks for one light & adjust camera if
				see more than one. Periodically sets back to normal
				to retest. 
Notes: 			We should see only one light when close to target
**********************************************************************/
void do_camera() 
{  
	static unsigned char do_cam_count = 0;
	static unsigned char debug_cam_count = 0;
	
#if AUTO_DEBUG
//	printf("do_camera: do_cam_count= %d \r",
//			do_cam_count);
#endif
	debug_cam_count++;
	if (debug_cam_count == 5)
	{
		do_cam_count++;
		camera_debug();
	}

	do_cam_count++;
	if (do_cam_count == 20)
	{
		// revert back to normal window
   		if ( one_light == 2) 
		{	
			// 0 = revert to normal; 1 = adjust to one target
			adjust_window(0);
		}	    
        else 
		{ 
			// find out if camera sees one light or more than one
			one_light = single_light(determine_closeness(T_Packet_Data.my), T_Packet_Data.confidence);
				
			// tell the camera autonomous code to focus on only one light
			if ( one_light == 0) 	// no light found!
			{
				abort_count++;	// abort mission if no lights seen
			}
			else 
			{   // one or more lights seen
				abort_count = 0;
				if ( one_light == 2) // more than one light
				{	
					// 0 = revert to normal; 1 = adjust to one target
					adjust_window(1);
					
				}
			}	   
		}
		do_cam_count = 0; // reset count	              	    
	}
}



/***********************************************************************
Function name:  auto_switch_find()
Date modified:  4-06-07
Made by:        Sid Soundararajan
Called From:	
Parameters:     void
Returns: 		unsigned int switch number
Purpose: 		Find which autonomous should be done
Notes: 
**********************************************************************/
unsigned int auto_switch_find()
{
	int switch_read;
	int switch_value = AUTO_DO_NOTHING;

	switch_read=Get_Analog_Value(rc_ana_in01);

	if((switch_read>=SWITCH_STATE_1_MIN) && (switch_read<=SWITCH_STATE_1_MAX))
		switch_value=AUTO_DO_NOTHING;
	else if((switch_read>=SWITCH_STATE_2_MIN) && (switch_read<=SWITCH_STATE_2_MAX))
		switch_value=AUTO_ARM_FLOOR;
	else if((switch_read>=SWITCH_STATE_3_MIN) && (switch_read<=SWITCH_STATE_3_MAX))
		switch_value=AUTO_FWD_FLOOR;
	else if((switch_read>=SWITCH_STATE_4_MIN) && (switch_read<=SWITCH_STATE_4_MAX))
		switch_value=AUTO_SCORE_MID;
	else if((switch_read>=SWITCH_STATE_5_MIN) && (switch_read<=SWITCH_STATE_5_MAX))
		switch_value=AUTO_REV_MOVE_ARM;
	else if((switch_read>=SWITCH_STATE_6_MIN) && (switch_read<=SWITCH_STATE_6_MAX))
		switch_value=AUTO_DO_NOTHING;
	else
		printf("ERROR: Switch is not in range\r");
	return switch_value;
}

// back = 1, go backwards, back = 0 go forwards
void go_and_move_arm(unsigned char back)
{
	static unsigned char loop_count = 0;
	
	//do we drive?
	static unsigned char drive = 1;
	unsigned char y_val = REVERSE_AUTO_SPEED; 
	loop_count++;
	
	if (back == 0)
	{
		y_val = FORWARD_AUTO_SPEED;
	}

	//first, go back a bit (~ 1 sec)
	if(drive)
	{
		//drive 
		crab_drive(127,y_val,127);

		//if time is up, we don't drive any more
		if(loop_count > DRIVE_LOOP_COUNT)
		{
			drive = 0;
		}
	}
	//next, move arm to low
	else
	{
		crab_drive(127,127,127);
		autonomous_manipulator_master_control(MANIP_DESIRED_POS,AUTO_GRIPPER_OPEN);
	}
	return;
}

void do_auto_arm(unsigned char position)
{
	autonomous_manipulator_master_control(position,AUTO_GRIPPER_OPEN);
}

