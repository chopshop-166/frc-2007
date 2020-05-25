/*******************************************************************************
* FILE NAME: manipulator.c
*
* WRITTEN BY:	Robert Harris	--	General Coding and Debugging
*			 	Eric Finn		--	Debugging
*				Beth Finn		--	Mentor
*				Per Hamnqvist	--	Mentor
*
* DESCRIPTION:
*  This file contains the functions created for the manipulator
*
* USAGE:
*  Functions created for the manipulator can be used inside of user_routines.c 
*  and autonomous modes.
*
*******************************************************************************/

#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "camera.h"
#include "encoder.h"
#include "eeprom.h"
#include "manipulator.h"

#define MANIP_DEBUG 0
#define LED_DEBUG 0
#define MANIP_GENERAL_INFO 0

char direction_base = 0; // direction of base encoder count
char direction_elbow = 0; // direction of elbow encoder count
char direction_wrist = 0; // direction of wrist encoder count
char direction_gripper = 0; // direction of gripper encoder count


extern unsigned char rotary_dial; // value from rotary dial on copilot box
extern char rocker_3_position_switch; // value from rocker switch on copilot box
extern unsigned char mini_joystick_1; // value from joystick 1 on copilot box
extern unsigned char mini_joystick_2; // value from joystick 2 on copilot box
extern unsigned char mini_joystick_3; // value from joystick 3 on copilot box

unsigned char manipulator_counter = 0; // counter used in manipulator code
unsigned char auto_counter = 0; // print throttle counter
int encoder_base = 0; // variable used to hold base encoder count 
int encoder_elbow = 0; // variable used to hold elbow encoder count
int encoder_wrist = 0; // variable used to hold wrist encoder count
int encoder_gripper = 0; // variable used to hold gripper encoder count


extern int Base_Position_Box; // value used to hold preset position from EEPROM
extern int Base_Position_Floor;	// value used to hold preset position from EEPROM	
extern int Base_Position_Low; // value used to hold preset position from EEPROM		
extern int Base_Position_Middle; // value used to hold preset position from EEPROM	
extern int Base_Position_High; // value used to hold preset position from EEPROM	
extern int Elbow_Position_Box; // value used to hold preset position from EEPROM	
extern int Elbow_Position_Floor; // value used to hold preset position from EEPROM
extern int Elbow_Position_Low; // value used to hold preset position from EEPROM	
extern int Elbow_Position_Middle; // value used to hold preset position from EEPROM	
extern int Elbow_Position_High;	// value used to hold preset position from EEPROM	
extern int Wrist_Position_Box; // value used to hold preset position from EEPROM	
extern int Wrist_Position_Floor; // value used to hold preset position from EEPROM	
extern int Wrist_Position_Low; // value used to hold preset position from EEPROM		
extern int Wrist_Position_Middle; // value used to hold preset position from EEPROM	
extern int Wrist_Position_High;	// value used to hold preset position from EEPROM	
extern int Gripper_Open; // value used to hold preset position from EEPROM			
extern int Gripper_Closed; // value used to hold preset position from EEPROM		

/***************** Manipulator Automatic Adjustment Code **************************

Inputs: 
	long goal_joint_position: the desired value for the joint in question
	unsigned char joint_to_be_moved: the joint in question, where 5 = base joint, 6 = elbow joint, 7 = wrist joint, 8 = gripper joint	
	unsigned char desired_speed of movement: the desired speed of movement, where 0 is no movement and 127 is maximum speed

Output:
	A value from 0 to 254 for the motor on the joint in question

**********************************************************************************/


unsigned char manipulator_automatic_adjustment(long goal_joint_position, unsigned char joint_to_be_moved, unsigned char desired_speed_of_movement, long upper_soft_stop, long lower_soft_stop)
{

unsigned char hold_value; // hold value for different steps of the function


#if 0
auto_counter++;
//if (auto_counter >= 6)
{
//printf("manipulator_automatic_adjustment: joint to be moved= %d\r", joint_to_be_moved);
printf("encoder_gripper =  %d\r", encoder_gripper);
auto_counter=0;
}
#endif


	switch(joint_to_be_moved) // determines which joint's encoder needs to be used and compares it to the goal position
	{

	case BASE:		//base (shoulder) joint

		
#if MANIP_DEBUG
if (manipulator_counter == 1) 
printf("base encoder value %d\r", encoder_base);
#endif

if((encoder_base >= (goal_joint_position - ENCODER_BASE_DEAD_ZONE)) && (encoder_base <= (goal_joint_position + ENCODER_BASE_DEAD_ZONE))) // if they are within the dead zone, the correct position
		{
			return 127; // do not move the motor
		}
		else	// if they aren't equal
		{
			if(encoder_base < goal_joint_position)	// if the goal joint position is higher than the current value
			{

				hold_value = 0;	// set hold value to correspond to scenario 
				direction_base = 1; // set the encoder direction to positive
			}
			else
			{

				if(encoder_base < lower_soft_stop)	// if the goal joint position is lower than the current value
					return 127; // return neutral

				hold_value = 1;  // set hold value to correspond to scenario 
				direction_base = -1; // set the encoder direction to negative
			}
		}

	break;

	case ELBOW:		//elbow (middle) joint


#if MANIP_DEBUG
	if (manipulator_counter == 2) {
		printf("elbow encoder value %d\r", encoder_elbow);
	}
#endif


		if((encoder_elbow >= (goal_joint_position - ENCODER_ELBOW_DEAD_ZONE)) && (encoder_elbow <= (goal_joint_position + ENCODER_ELBOW_DEAD_ZONE))) // if they are within the dead zone, the correct position
		{
			return 127; // do not move the motor
		}
		else	// if they aren't equal
		{
			if(encoder_elbow < goal_joint_position)	// if the goal joint position is higher than the current value
			{
			

				hold_value = 0;	// set hold value to correspond to scenario 
				direction_elbow = 1; // set the encoder direction to positive
			}
			else
			{

				if(encoder_elbow < lower_soft_stop)	// if the goal joint position is lower than the current value
					return 127; // return neutral



				hold_value = 1;  // set hold value to correspond to scenario 
				direction_elbow = -1; // set the encoder direction to negative
			}
		}

	break;

	case WRIST:		//wrist joint

#if MANIP_DEBUG
if (manipulator_counter == 3)
printf("wrist encoder value %d\r", encoder_wrist);
#endif


if((encoder_wrist >= (goal_joint_position - ENCODER_WRIST_DEAD_ZONE)) && (encoder_wrist <= (goal_joint_position + ENCODER_WRIST_DEAD_ZONE))) // if they are within the dead zone, the correct position
		{
			return 127; // do not move the motor
		}
		else	// if they aren't equal
		{
			if(encoder_wrist < goal_joint_position)	// if the goal joint position is higher than the current value
			{

#if MANIP_DEBUG
				if (manipulator_counter == 2)	{
					printf("DID NOT STOP");
				}
#endif

				hold_value = 0;	// set hold value to correspond to scenario 
				direction_wrist = 1; // set the encoder direction to positive
			}
			else
			{

				if(encoder_wrist < lower_soft_stop)	// if the goal joint position is lower than the current value
					return 127; // return neutral

				#if MANIP_DEBUG
				if (manipulator_counter == 2)
				printf("did not stop");
				#endif

				hold_value = 1;  // set hold value to correspond to scenario 
				direction_wrist = -1; // set the encoder direction to negative
			}
		}

	break;

	case GRIPPER:		//gripper joint

	
#if 1
	if (manipulator_counter == 4) {
		printf("\gripper encoder value %d\r", encoder_gripper);
	}
#endif

		if((encoder_gripper >= (goal_joint_position - ENCODER_GRIPPER_DEAD_ZONE)) && (encoder_gripper <= (goal_joint_position + ENCODER_GRIPPER_DEAD_ZONE))) // if they are within the dead zone, the correct position
		{
			return 127; // do not move the motor
		}
		else
		{
			if(encoder_gripper < goal_joint_position)	// if the goal joint position is higher than the current value
			{
//				if(encoder_gripper > upper_soft_stop) // if the current value exceeds the upper soft stop
//					return 127;	// return neutral

//				if(encoder_gripper < lower_soft_stop)	// if the goal joint position is lower than the current value
//					return 127; // return neutral

				hold_value = 0;	// set hold value to correspond to scenario 
				direction_gripper = -1; // set the encoder direction to negative
			}
			else
			{
//				if(encoder_gripper > upper_soft_stop) // if the current value exceeds the upper soft stop
//					return 127;	// return neutral

				if(encoder_gripper < lower_soft_stop)	// if the goal joint position is lower than the current value
					return 127; // return neutral

				hold_value = 1;  // set hold value to correspond to scenario 
				direction_gripper = 1; // set the encoder direction to positive
			}
		}

	break;
	}

	if(hold_value == 1)	// if scenario where the goal joint position is higher than the current value	
	{

		hold_value = 127 + desired_speed_of_movement; // set output to hold_value

		if((joint_to_be_moved == GRIPPER) && (LIMIT_SWITCH_GRIPPER == 1)) // if the gripper is the joint in question and the gripper override is on
			return 127; // return neutral

	}
	else	// if scenario where the goal joint position is higher than the current value	
	{

		if((joint_to_be_moved == BASE) && (LIMIT_SWITCH_BASE == 1)) // if the base is the joint in question and the base override is on
			return 127; // return neutral

		if((joint_to_be_moved == ELBOW) && (LIMIT_SWITCH_ELBOW == 1)) // if the elbow is the joint in question and the elbow override is on
			return 127; // return neutral

		if((joint_to_be_moved == WRIST) && (LIMIT_SWITCH_WRIST == 1)) // if the wrist is the joint in question and the wrist override is on
			return 127; // return neutral



		hold_value = 127 - desired_speed_of_movement; // set output to hold value
	}



	return hold_value; // return the value determined by the mathematical logic
}


/********************* Manipulator Manual Adjustment Code *************************

Inputs: 
	unsigned char input_value: 0-254 value from analog input device
	unsigned char reduction_factor: a percentage of 0-100 
Output:
	A value from 0 to 254 for a joint motor

**********************************************************************************/

unsigned char manipulator_manual_adjustment(int input_value, unsigned char reduction_factor, unsigned char joint_to_be_moved)
{

	int hold_value;	// hold value for different steps of the function

	switch(joint_to_be_moved) //  set encoder direction values and check limit switches based on which joint is inputed
	{

	case BASE:

		if(input_value < (127 - 7)) // if base value is backwards
		{
			direction_base = 1; // set encoder direction as positive
		}
		else if(input_value > (127 + 7)) // if base direction is forwards
		{
			direction_base = -1; // set encoder direction as negative
		}

		if((input_value < (127 - 7)) && (LIMIT_SWITCH_BASE == 1)) // if base value is backwards and base is at home
		{
			return 127; // return neutral
		}

	break;

	case ELBOW:

		if(input_value < (127 - 7)) // if elbow value is backwards
		{
			direction_elbow = 1; // set encoder direction as positive
		}
		else if(input_value > (127 + 7)) // if elbow direction is forwards
		{
			direction_elbow = -1; // set encoder  direction as negative
		}

		if((input_value < (127 - 7)) && (LIMIT_SWITCH_ELBOW == 1)) // if elbow value is backwards and elbow is at home
		{
			return 127; // return neutral
		}
		
	break;

	case WRIST:

		if(input_value < (127 - 7)) // if wrist value is backwards
		{
			direction_wrist = 1; // set encoder direction as positive
		}
		else if(input_value > (127 + 7)) // if wrist value is forwards
		{
			direction_wrist = -1; // set encoder direction as negative
		}

		if((input_value < (127 - 7)) && (LIMIT_SWITCH_WRIST == 1) && (rotary_dial)) // if wrist direction is backwards and wrist is at home, and we're not in full override
		{
			return 127; // return neutral
		}
		break;
	}


	if ((input_value >= (127 - 7)) && (input_value <= (127 + 7))) //if in the dead zone
		return 127;	//return neutral

	if (input_value < 127)						// if backwards
	{
		hold_value = -1 * (input_value - 127);	// convert to 0 - 127 number
	}
	else										// if forwards
	{
		hold_value = input_value - 127;		// convert to 0 - 127 number	
	}

	hold_value = hold_value * reduction_factor ; //multiply by percentage input_value * reduction_factor 
	hold_value = hold_value / 100;	//divide by 100 to regain 0-127 value

	if (input_value < 127)						// if backwards
	{
		hold_value = (-1 * hold_value) + 127;	// convert to 0 - 254 number
	}
	else										// if forwards
	{
		hold_value = hold_value + 127;			// convert to 0 - 254 number	
	}



return hold_value; //return the determined value
} 


/**************** Teleoperated Manipulator Master Control Code ********************

Inputs: 
	Uses global variables from copilot box and home base limit switches as well as encoders functions (but no inputs are passed into function)
Output:
	Sets PWM05, PWM06, PWM07 and PWM08 (but no direct output)

**********************************************************************************/



void teleoperated_manipulator_master_control(void)
{

static unsigned char 	base_override = 0; // used to hold base override on or off
static unsigned char	elbow_override = 0; // used to hold elbow override on or off
static unsigned char	wrist_override = 0; // used to hold wrist override on or off
static unsigned char	last_rotary_dial = 0; // used to reset overrides when rotary dial changes
unsigned char			hold_value; // used as a general value holder in this function




#if LED_DEBUG

printf("in teleoperated_master control \r");

#endif


	manipulator_counter ++; // add to the counter	
	if (manipulator_counter == 16) // if counter equals 16
		manipulator_counter = 0; // reset counter

#if MANIP_DEBUG
	if (manipulator_counter == 0)
		printf("MANIPULATOR DEBUG LOOP RESET\r\r");
	if (manipulator_counter == 5)
		printf("teleoperated_manipulator_mc: Rotary Dial: %d\r",rotary_dial);
#endif


	if(manipulator_counter == 15) // if counter equals 15
	{
		encoder_base = (int)ENCODER_BASE;  //  set base encoder global variable to current value from base encoder
		encoder_elbow = (int)ENCODER_ELBOW; //  set elbow encoder global variable to current value from elbow encoder
		encoder_wrist = (int)ENCODER_WRIST; //  set wrist encoder global variable to current value from wrist encoder
		encoder_gripper = (int)ENCODER_GRIPPER; //  set gripper encoder global variable to current value from gripper encoder
#if MANIP_DEBUG
		printf("checking encoder counts");
#endif
	}

#if MANIP_GENERAL_INFO
printf("encoders: base, elbow, wrist, gripper, \r%d,%d,%d,%d\r", encoder_base, encoder_elbow, encoder_wrist, encoder_gripper);
#endif

	if(LIMIT_SWITCH_BASE == 1)	//home for encoder 3
		Reset_Encoder_3_Count();		//resets encoder 3
	
	if(LIMIT_SWITCH_ELBOW == 1)	//home for encoder 4
		Reset_Encoder_4_Count();		//resets encoder 4
	
	if(LIMIT_SWITCH_WRIST == 1)	//home for encoder 5
		Reset_Encoder_5_Count();		//resets encoder 5

	if(LIMIT_SWITCH_GRIPPER == 1)	//home for encoder 6
		Reset_Encoder_6_Count();		//resets encoder 6
		
#if MANIP_GENERAL_INFO		
printf("limit switches: base, elbow, wrist, gripper, \r%d,%d,%d,%d\r\r", rc_dig_in07, rc_dig_in08, rc_dig_in09, rc_dig_in10);
#endif

	if((mini_joystick_1 < (127 - BASE_DEAD_ZONE)) || (mini_joystick_1 > (127 + BASE_DEAD_ZONE))) //if base analog stick leaves dead zone
		base_override = 1;	// turn on base override

	if((mini_joystick_2 < (127 - ELBOW_DEAD_ZONE)) || (mini_joystick_2 > (127 + ELBOW_DEAD_ZONE))) //if elbow analog stick leaves dead zone
		elbow_override = 1;	// turn on elbow override

	if((mini_joystick_3 < (127 - WRIST_DEAD_ZONE)) || (mini_joystick_3 > (127 + WRIST_DEAD_ZONE))) //if wrist analog stick leaves dead zone
		wrist_override = 1;	// turn on wrist override




	if(last_rotary_dial != rotary_dial)	// if the rotary dial has changed
	{
		last_rotary_dial = rotary_dial;	// adjust the last known information for the rotary dial
		base_override = 0; // remove the base override	
		elbow_override = 0; // remove the elbow override
		wrist_override = 0; // remove the wrist override
	}


	//RED LED code


	if((base_override == 1) || (elbow_override == 1) || (wrist_override == 1))  // If any of the overrides are on
		{
		#if LED_DEBUG
		printf("Red LED on \r");
		#endif
		Relay2_red = 1;  // turn on the red LED
		}
	else // if an override is on
		{
		#if LED_DEBUG
		printf("Red LED off \r");
		#endif
		Relay2_red = 0; // turn off the red LED  
		}

	//GREEN LED code
	
	switch(rotary_dial)
	{

	case 0: // override


		Relay2_green = 0;
	break;


	case 1: // BOX

		if((encoder_base >= (BASE_POSITION_BOX - ENCODER_BASE_DEAD_ZONE)) && (encoder_base <= (BASE_POSITION_BOX + ENCODER_BASE_DEAD_ZONE))) // if base is within the dead zone
		{
			if((encoder_elbow >= (ELBOW_POSITION_BOX - ENCODER_ELBOW_DEAD_ZONE)) && (encoder_base <= (ELBOW_POSITION_BOX + ENCODER_ELBOW_DEAD_ZONE))) // if base is within the dead zone
			{
				if((encoder_base >= (WRIST_POSITION_BOX - ENCODER_WRIST_DEAD_ZONE)) && (encoder_base <= (WRIST_POSITION_BOX + ENCODER_WRIST_DEAD_ZONE))) // if wrist is within the dead zone
				{		
					Relay2_green = 1; // turn on the green LED because we are in alignment
					#if LED_DEBUG
					printf("Green LED on \r");
					#endif
				}
				else // if wrist is not in alignment
				{	
					#if LED_DEBUG
					printf("Green LED off \r");
					#endif
					Relay2_green = 0; // turn off the green LED because we are not in alignment
				}
			}
			else  // if elbow is not in alignment
			{
				#if LED_DEBUG
				printf("Green LED off \r");
				#endif
				Relay2_green = 0; // turn off the green LED because we are not in alignment
			}
		}
		else // if base is not in alignment
		{
			#if LED_DEBUG
			printf("Green LED off \r");
			#endif
			Relay2_green = 0; // turn off the green LED because we are not in alignment
		}

	break;

	case 2: // FLOOR

		if((encoder_base >= (BASE_POSITION_FLOOR - ENCODER_BASE_DEAD_ZONE)) && (encoder_base <= (BASE_POSITION_FLOOR + ENCODER_BASE_DEAD_ZONE))) // if base is within the dead zone
		{
			if((encoder_elbow >= (ELBOW_POSITION_FLOOR - ENCODER_ELBOW_DEAD_ZONE)) && (encoder_base <= (ELBOW_POSITION_FLOOR + ENCODER_ELBOW_DEAD_ZONE))) // if base is within the dead zone
			{
				if((encoder_base >= (WRIST_POSITION_FLOOR - ENCODER_WRIST_DEAD_ZONE)) && (encoder_base <= (WRIST_POSITION_FLOOR + ENCODER_WRIST_DEAD_ZONE))) // if wrist is within the dead zone
				{		
					Relay2_green = 1; // turn on the green LED because we are in alignment
					#if LED_DEBUG
					printf("Green LED on \r");
					#endif
					Relay2_green = 1; // turn on the green LED because we are in alignment
				}
				else // if wrist is not in alignment
				{	
					#if LED_DEBUG
					printf("Green LED off \r");
					#endif
					Relay2_green = 0; // turn off the green LED because we are not in alignment
				}
			}
			else  // if elbow is not in alignment
			{
				#if LED_DEBUG
				printf("Green LED off \r");
				#endif
				Relay2_green = 0; // turn off the green LED because we are not in alignment
			}
		}
		else // if base is not in alignment
		{
			#if LED_DEBUG
			printf("Green LED off \r");
			#endif
			Relay2_green = 0; // turn off the green LED because we are not in alignment
		}

	break;

	case 3: // LOW

		if((encoder_base >= (BASE_POSITION_LOW - ENCODER_BASE_DEAD_ZONE)) && (encoder_base <= (BASE_POSITION_LOW + ENCODER_BASE_DEAD_ZONE))) // if base is within the dead zone
		{
			if((encoder_elbow >= (ELBOW_POSITION_LOW - ENCODER_ELBOW_DEAD_ZONE)) && (encoder_base <= (ELBOW_POSITION_LOW + ENCODER_ELBOW_DEAD_ZONE))) // if base is within the dead zone
			{
				if((encoder_base >= (WRIST_POSITION_LOW - ENCODER_WRIST_DEAD_ZONE)) && (encoder_base <= (WRIST_POSITION_LOW + ENCODER_WRIST_DEAD_ZONE))) // if wrist is within the dead zone
				{		
					Relay2_green = 1; // turn on the green LED because we are in alignment
					#if LED_DEBUG
					printf("Green LED on \r");
					#endif
				}
				else // if wrist is not in alignment
				{	
					#if LED_DEBUG
					printf("Green LED off \r");
					#endif
					Relay2_green = 0; // turn off the green LED because we are not in alignment
				}
			}
			else  // if elbow is not in alignment
			{
				#if LED_DEBUG
				printf("Green LED off \r");
				#endif
				Relay2_green = 0; // turn off the green LED because we are not in alignment
			}
		}
		else // if base is not in alignment
		{
			#if LED_DEBUG
			printf("Green LED off \r");
			#endif
			Relay2_green = 0; // turn off the green LED because we are not in alignment
		}

	break;

	case 4: // MIDDLE

		if((encoder_base >= (BASE_POSITION_MIDDLE - ENCODER_BASE_DEAD_ZONE)) && (encoder_base <= (BASE_POSITION_MIDDLE + ENCODER_BASE_DEAD_ZONE))) // if base is within the dead zone
		{
			if((encoder_elbow >= (ELBOW_POSITION_MIDDLE - ENCODER_ELBOW_DEAD_ZONE)) && (encoder_base <= (ELBOW_POSITION_MIDDLE + ENCODER_ELBOW_DEAD_ZONE))) // if base is within the dead zone
			{
				if((encoder_base >= (WRIST_POSITION_MIDDLE - ENCODER_WRIST_DEAD_ZONE)) && (encoder_base <= (WRIST_POSITION_MIDDLE + ENCODER_WRIST_DEAD_ZONE))) // if wrist is within the dead zone
				{			
					Relay2_green = 1; // turn on the green LED because we are in alignment
					#if LED_DEBUG
					printf("Green LED on \r");
					#endif
				}
				else // if wrist is not in alignment
				{	
					#if LED_DEBUG
					printf("Green LED off \r");
					#endif
					Relay2_green = 0; // turn off the green LED because we are not in alignment
				}
			}
			else  // if elbow is not in alignment
			{
				#if LED_DEBUG
				printf("Green LED off \r");
				#endif
				Relay2_green = 0; // turn off the green LED because we are not in alignment
			}
		}
		else // if base is not in alignment
		{
			#if LED_DEBUG
			printf("Green LED off \r");
			#endif
			Relay2_green = 0; // turn off the green LED because we are not in alignment
		}

	break;

	case 5: // HIGH

		if((encoder_base >= (BASE_POSITION_HIGH - ENCODER_BASE_DEAD_ZONE)) && (encoder_base <= (BASE_POSITION_HIGH + ENCODER_BASE_DEAD_ZONE))) // if base is within the dead zone
		{
			if((encoder_elbow >= (ELBOW_POSITION_HIGH - ENCODER_ELBOW_DEAD_ZONE)) && (encoder_base <= (ELBOW_POSITION_HIGH + ENCODER_ELBOW_DEAD_ZONE))) // if base is within the dead zone
			{
				if((encoder_base >= (WRIST_POSITION_HIGH - ENCODER_WRIST_DEAD_ZONE)) && (encoder_base <= (WRIST_POSITION_HIGH + ENCODER_WRIST_DEAD_ZONE))) // if wrist is within the dead zone
				{		
					Relay2_green = 1; // turn on the green LED because we are in alignment
					#if LED_DEBUG
					printf("Green LED on \r");
					#endif
				}
				else // if wrist is not in alignment
				{	
					#if LED_DEBUG
					printf("Green LED off \r");
					#endif
					Relay2_green = 0; // turn off the green LED because we are not in alignment
				}
			}
			else  // if elbow is not in alignment
			{
				#if LED_DEBUG
				printf("Green LED off \r");
				#endif
				Relay2_green = 0; // turn off the green LED because we are not in alignment
			}
		}
		else // if base is not in alignment
		{
			#if LED_DEBUG
			printf("Green LED off \r");
			#endif
			Relay2_green = 0; // turn off the green LED because we are not in alignment
		}

	break;

	}

	// since the manipulator is entirely manual during teleoperated mode, 
	// this switch statement handles the rocker switch for gripper code
	switch(rocker_3_position_switch) 
	{
  	
	case 0:	// switch is in center position

		MOTOR_GRIPPER = 127; // no change

	break;
	
	case 1: // switch is in forward position

		if(LIMIT_SWITCH_GRIPPER == 1) // if the gripper is at home
		{
			MOTOR_GRIPPER = 127; // return neutral
			break; 
		}

	//	if(encoder_gripper < BASE_UPPER_LIMIT) // if not beyond limit
	//	{
		MOTOR_GRIPPER = 127 + GRIPPER_SPEED_FORWARD; // forward change
		direction_gripper = 1; // set encoder direction to positive

	//	}
	//	else	// if beyond limit
	//	{
	//	MOTOR_GRIPPER = 127; // return neutral
	//	}

	break;

	case -1: // switch is in backward position

	//	if(encoder_gripper > BASE_LOWER_LIMIT) // if not beyond limit
	//	{
		MOTOR_GRIPPER = 127 - GRIPPER_SPEED_BACKWARD; // backward change
		direction_gripper = -1; // set encoder direction to negative
	//	}
	//	else	// if beyond limit
	//	{
	//	MOTOR_GRIPPER = 127; // return neutral
	//	}

	break;
	}

//OI input not going to be 0, 1, etc

	switch(rotary_dial) //various positions on hieght selector dial
	{

	case 0: // manual override

		MOTOR_BASE = manipulator_manual_adjustment(mini_joystick_1, BASE_MANUAL_REDUCTION,BASE); // use manual adjustment for the base
		MOTOR_ELBOW = manipulator_manual_adjustment(mini_joystick_2, ELBOW_MANUAL_REDUCTION, ELBOW); // use manual adjustment for the elbow
		MOTOR_WRIST = manipulator_manual_adjustment(mini_joystick_3, WRIST_MANUAL_REDUCTION,WRIST); // use manual adjustment for the wrist		
		base_override = 1; // set base override variable to 1 to trigger red LED
		elbow_override = 1; // set elbow override variable to 1 to trigger red LED
		wrist_override = 1; // set wrist override variable to 1 to trigger red LED


	break;

	case 1: 	// position BOX	

#if MANIP_DEBUG
if (manipulator_counter == 6)
{	
printf("base_override= %d   elbow_override= %d  wrist_override= %d  \r", base_override, elbow_override, wrist_override);
printf("base goto value, base upper stop, base lower stop %d,%d,%d\r", Base_Position_Box, BASE_UPPER_LIMIT, BASE_LOWER_LIMIT);
printf("elbow goto value, elbow upper stop, elbow lower stop %d,%d,%d\r", Elbow_Position_Box, ELBOW_UPPER_LIMIT, ELBOW_LOWER_LIMIT);
printf("wrist goto value, wrist upper stop, wrist lower stop %d,%d,%d\r", Wrist_Position_Box, WRIST_UPPER_LIMIT, WRIST_LOWER_LIMIT);
}
#endif

		if(base_override == 0)  // if the base override is off
		{
			MOTOR_BASE = goto_box(BASE); // use special go home function to move base
		}
		else // if the base override is on
		{
			MOTOR_BASE = manipulator_manual_adjustment(mini_joystick_1, BASE_MANUAL_REDUCTION,BASE); // use manual adjustment for the base
		}
		if(elbow_override == 0)  // if the elbow override is off
		{
			MOTOR_ELBOW = goto_box(ELBOW); // use special go home function to move elbow
		}
		else // if the elbow override is on
		{
			MOTOR_ELBOW = manipulator_manual_adjustment(mini_joystick_2, ELBOW_MANUAL_REDUCTION, ELBOW); // use manual adjustment for the elbow
		}
		if(wrist_override == 0)  // if the wrist override is off
		{
			MOTOR_WRIST = goto_box(WRIST); // use special go home function to more wrist
		}
		else // if the wrist override is on
		{
			MOTOR_WRIST = manipulator_manual_adjustment(mini_joystick_3, WRIST_MANUAL_REDUCTION,WRIST); // use manual adjustment for the wrist
		}
	break;

	case 2: 	// position FLOOR
		
		if(base_override == 0)  // if the base override is off
`		{
			MOTOR_BASE = manipulator_automatic_adjustment(BASE_POSITION_FLOOR,BASE,BASE_SPEED,BASE_UPPER_LIMIT,BASE_LOWER_LIMIT); // use automatic adjustment for the base
		}
		else // if the base override is on
		{
			MOTOR_BASE = manipulator_manual_adjustment(mini_joystick_1, BASE_MANUAL_REDUCTION,BASE); // use manual adjustment for the base
		}
		if(elbow_override == 0)  // if the elbow override is off
		{
			MOTOR_ELBOW = manipulator_automatic_adjustment(ELBOW_POSITION_FLOOR,ELBOW,ELBOW_SPEED,ELBOW_UPPER_LIMIT,ELBOW_LOWER_LIMIT); // use automatic adjustment for the elbow
		}
		else // if the elbow override is on
		{
			MOTOR_ELBOW = manipulator_manual_adjustment(mini_joystick_2, ELBOW_MANUAL_REDUCTION,ELBOW); // use manual adjustment for the elbow
		}
		if(wrist_override == 0)  // if the wrist override is off
		{
			MOTOR_WRIST = manipulator_automatic_adjustment(WRIST_POSITION_FLOOR,WRIST,WRIST_SPEED,WRIST_UPPER_LIMIT,WRIST_LOWER_LIMIT); // use automatic adjustment for the wrist
		}
		else // if the wrist override is on
		{
			MOTOR_WRIST = manipulator_manual_adjustment(mini_joystick_3, WRIST_MANUAL_REDUCTION,WRIST); // use manual adjustment for the wrist
		}

	break;

	case 3: 	// position LOW
		
		if(base_override == 0)  // if the base override is off
		{
			MOTOR_BASE = manipulator_automatic_adjustment(BASE_POSITION_LOW,BASE,BASE_SPEED,BASE_UPPER_LIMIT,BASE_LOWER_LIMIT); // use automatic adjustment for the base
		}
		else // if the base override is on
		{
			MOTOR_BASE = manipulator_manual_adjustment(mini_joystick_1, BASE_MANUAL_REDUCTION,BASE); // use manual adjustment for the base
		}

		if(elbow_override == 0)  // if the elbow override is off
		{
			MOTOR_ELBOW = manipulator_automatic_adjustment(ELBOW_POSITION_LOW,ELBOW,ELBOW_SPEED,ELBOW_UPPER_LIMIT,ELBOW_LOWER_LIMIT); // use automatic adjustment for the elbow
		}
		else // if the elbow override is on
		{
			MOTOR_ELBOW = manipulator_manual_adjustment(mini_joystick_2, ELBOW_MANUAL_REDUCTION,ELBOW); // use manual adjustment for the elbow
		}

		if(wrist_override == 0)  // if the wrist override is off
		{
			MOTOR_WRIST = manipulator_automatic_adjustment(WRIST_POSITION_LOW,WRIST,WRIST_SPEED,WRIST_UPPER_LIMIT,WRIST_LOWER_LIMIT); // use automatic adjustment for the wrist
		}
		else // if the wrist override is on
		{
			MOTOR_WRIST = manipulator_manual_adjustment(mini_joystick_3, WRIST_MANUAL_REDUCTION, WRIST); // use manual adjustment for the wrist
		}

	break;

	case 4: 	// position MIDDLE
		
		if(base_override == 0)  // if the base override is off
		{
			MOTOR_BASE = manipulator_automatic_adjustment(BASE_POSITION_MIDDLE,BASE,BASE_SPEED,BASE_UPPER_LIMIT,BASE_LOWER_LIMIT); // use automatic adjustment for the base
		}
		else // if the base override is on
		{
			MOTOR_BASE = manipulator_manual_adjustment(mini_joystick_1, BASE_MANUAL_REDUCTION,BASE); // use manual adjustment for the base
		}

		if(elbow_override == 0)  // if the elbow override is off
		{
			MOTOR_ELBOW = manipulator_automatic_adjustment(ELBOW_POSITION_MIDDLE,ELBOW,ELBOW_SPEED,ELBOW_UPPER_LIMIT,ELBOW_LOWER_LIMIT); // use automatic adjustment for the elbow
		}
		else // if the elbow override is on
		{
			MOTOR_ELBOW = manipulator_manual_adjustment(mini_joystick_2, ELBOW_MANUAL_REDUCTION, ELBOW); // use manual adjustment for the elbow
		}

		if(wrist_override == 0)  // if the wrist override is off
		{
			MOTOR_WRIST = manipulator_automatic_adjustment(WRIST_POSITION_MIDDLE,WRIST,WRIST_SPEED,WRIST_UPPER_LIMIT,WRIST_LOWER_LIMIT); // use automatic adjustment for the wrist
		}
		else // if the wrist override is on
		{
			MOTOR_WRIST = manipulator_manual_adjustment(mini_joystick_3, WRIST_MANUAL_REDUCTION,WRIST); // use manual adjustment for the wrist
		}

	break;

	case 5: 	// position HIGH	
		
		if(base_override == 0)  // if the base override is off
		{
			MOTOR_BASE = manipulator_automatic_adjustment(BASE_POSITION_HIGH,BASE,BASE_SPEED,BASE_UPPER_LIMIT,BASE_LOWER_LIMIT); // use automatic adjustment for the base
		}
		else // if the base override is on
		{
			MOTOR_BASE = manipulator_manual_adjustment(mini_joystick_1, BASE_MANUAL_REDUCTION,BASE); // use manual adjustment for the base
		}

		if(elbow_override == 0)  // if the elbow override is off
		{
			MOTOR_ELBOW = manipulator_automatic_adjustment(ELBOW_POSITION_HIGH,ELBOW,ELBOW_SPEED,ELBOW_UPPER_LIMIT,ELBOW_LOWER_LIMIT); // use automatic adjustment for the elbow
		}
		else // if the elbow override is on
		{
			MOTOR_ELBOW = manipulator_manual_adjustment(mini_joystick_2, ELBOW_MANUAL_REDUCTION,ELBOW); // use manual adjustment for the elbow
		}

		if(wrist_override == 0)  // if the wrist override is off
		{
			MOTOR_WRIST = manipulator_automatic_adjustment(WRIST_POSITION_HIGH,WRIST,WRIST_SPEED,WRIST_UPPER_LIMIT,WRIST_LOWER_LIMIT); // use automatic adjustment for the wrist
		}
		else // if the wrist override is on
		{
			MOTOR_WRIST = manipulator_manual_adjustment(mini_joystick_3, WRIST_MANUAL_REDUCTION,WRIST); // use manual adjustment for the wrist
		}

	break;

	default:  // should not get here

printf("code error in teleoperated_manipulator_master_control: rotary_dial= %d  \r", rotary_dial);

	}
}




/****************** Autonmous Manipulator Master Control Code *********************

Inputs: 
	unsigned char desired_position: desired position of arm, where 1 = BOX, 2 = FLOOR, 3 = LOW, 4 = MIDDLE, and 5 = HIGH 
	unsigned char gripper_open_or_close: desired position of gripper, where 0 = OPEN and 1 = CLOSED
	Uses global variables from copilot box
Output:
	Sets PWM05, PWM06, PWM07 and PWM08 (but no direct output)

**********************************************************************************/


void autonomous_manipulator_master_control(unsigned char desired_position, unsigned char gripper_open_or_close)
{		

	switch(desired_position) // choose position that has been called
	{
	
	case 1: 	// position BOX	
		
			MOTOR_BASE = goto_box(BASE); // use special go home function to move base
			MOTOR_ELBOW = goto_box(ELBOW); // use special go home function to move elbow
			MOTOR_WRIST = goto_box(WRIST); // use special go home function to move wrist
		
	break;

	case 2: 	// position FLOOR
		
			MOTOR_BASE = manipulator_automatic_adjustment(BASE_POSITION_FLOOR,BASE,BASE_SPEED,BASE_UPPER_LIMIT,BASE_LOWER_LIMIT); // use automatic adjustment for the base
			MOTOR_ELBOW = manipulator_automatic_adjustment(ELBOW_POSITION_FLOOR,ELBOW,ELBOW_SPEED,ELBOW_UPPER_LIMIT,ELBOW_LOWER_LIMIT); // use automatic adjustment for the elbow
			MOTOR_WRIST = manipulator_automatic_adjustment(WRIST_POSITION_FLOOR,WRIST,WRIST_SPEED,WRIST_UPPER_LIMIT,WRIST_LOWER_LIMIT); // use automatic adjustment for the wrist

	break;

	case 3: 	// position LOW
		
			MOTOR_BASE = manipulator_automatic_adjustment(BASE_POSITION_LOW,BASE,BASE_SPEED,BASE_UPPER_LIMIT,BASE_LOWER_LIMIT); // use automatic adjustment for the base
			MOTOR_ELBOW = manipulator_automatic_adjustment(ELBOW_POSITION_LOW,ELBOW,ELBOW_SPEED,ELBOW_UPPER_LIMIT,ELBOW_LOWER_LIMIT); // use automatic adjustment for the elbow
			MOTOR_WRIST = manipulator_automatic_adjustment(WRIST_POSITION_LOW,WRIST,WRIST_SPEED,WRIST_UPPER_LIMIT,WRIST_LOWER_LIMIT); // use automatic adjustment for the wrist

	break;

	case 4: 	// position MIDDLE

			MOTOR_BASE = manipulator_automatic_adjustment(BASE_POSITION_MIDDLE,BASE,BASE_SPEED,BASE_UPPER_LIMIT,BASE_LOWER_LIMIT); // use automatic adjustment for the base
			MOTOR_ELBOW = manipulator_automatic_adjustment(ELBOW_POSITION_MIDDLE,ELBOW,ELBOW_SPEED,ELBOW_UPPER_LIMIT,ELBOW_LOWER_LIMIT); // use automatic adjustment for the elbow
			MOTOR_WRIST = manipulator_automatic_adjustment(WRIST_POSITION_MIDDLE,WRIST,WRIST_SPEED,WRIST_UPPER_LIMIT,WRIST_LOWER_LIMIT); // use automatic adjustment for the wrist

	break;

	case 5: 	// position HIGH	
		
			MOTOR_BASE = manipulator_automatic_adjustment(BASE_POSITION_HIGH,BASE,BASE_SPEED,BASE_UPPER_LIMIT,BASE_LOWER_LIMIT); // use automatic adjustment for the base
			MOTOR_ELBOW = manipulator_automatic_adjustment(ELBOW_POSITION_HIGH,ELBOW,ELBOW_SPEED,ELBOW_UPPER_LIMIT,ELBOW_LOWER_LIMIT); // use automatic adjustment for the elbow
			MOTOR_WRIST = manipulator_automatic_adjustment(WRIST_POSITION_HIGH,WRIST,WRIST_SPEED,WRIST_UPPER_LIMIT,WRIST_LOWER_LIMIT); // use automatic adjustment for the wrist

	break;

	default:  // should not get here

printf("code error in autonomous_manipulator_master_control: desired_position= %d  \r", desired_position);
		
	}	
#if 1
auto_counter++;
if (auto_counter >= 6)
{
printf("in autonomous_manipulator_master_control: gripper_open_or_close %d\r", gripper_open_or_close);
auto_counter=0;
}
#endif
	if(gripper_open_or_close)
	{
		MOTOR_GRIPPER = manipulator_automatic_adjustment(GRIPPER_OPEN,GRIPPER,GRIPPER_SPEED_FORWARD,GRIPPER_UPPER_LIMIT,GRIPPER_LOWER_LIMIT);
	}
	else
	{
		MOTOR_GRIPPER = manipulator_automatic_adjustment(GRIPPER_CLOSED,GRIPPER,GRIPPER_SPEED_FORWARD,GRIPPER_UPPER_LIMIT,GRIPPER_LOWER_LIMIT);
	}
		
}


/**************************** Encoder Reset Battery *******************************

Inputs: 
	Uses global variables from home base limit switches as well as encoders functions
Output:
	Resets Encoders 3-6 when their respective home positions are reached (but no direct output)

**********************************************************************************/

void encoder_reset_battery(void)
{


	manipulator_counter ++; // add to counter for manipulator code
	if (manipulator_counter == 16) // if the manipulator counter value is 16
		manipulator_counter = 0; // reset manipulator counter

#if MANIP_DEBUG
	if (manipulator_counter == 0)
		printf("MANIPULATOR DEBUG LOOP RESET\r\r");
#endif
	

	if(manipulator_counter == 15) // if the manipulator counter is at 15
	{
		encoder_base = (int)ENCODER_BASE; // set base encoder value equal to the current base encoder value 
		encoder_elbow = (int)ENCODER_ELBOW; // set elbow encoder value equal to the current elbow encoder value
		encoder_wrist = (int)ENCODER_WRIST; // set wrist encoder value equal to the current wrist encoder value
		encoder_gripper = (int)ENCODER_GRIPPER; // set gripper encoder value equal to the current gripper encoder value
#if MANIP_DEBUG
		printf("checking encoder counts");
#endif
	}


	if(LIMIT_SWITCH_BASE == 1)	//home for encoder 3
		Reset_Encoder_3_Count();		//resets encoder 3
	
	if(LIMIT_SWITCH_ELBOW == 1)	//home for encoder 4
		Reset_Encoder_4_Count();		//resets encoder 4
	
	if(LIMIT_SWITCH_WRIST == 1)	//home for encoder 5
		Reset_Encoder_5_Count();		//resets encoder 5

	if(LIMIT_SWITCH_GRIPPER == 1)	//home for encoder 6
		Reset_Encoder_6_Count();		//resets encoder 6 

}


/************************* Manipulator Calibration Function ***********************

Inputs: 
	Uses global variables found in user_routines.c and the encoder values (but no direct input
Output:
	Calls EEPROM_prep function to set arm values to their calibrated values  (but no direct output)

**********************************************************************************/

void manipulator_calibration(void)
{

	static unsigned char click_handler_1 = 0; // variable to check if the trigger switch has been pressed
	static unsigned char click_handler_2 = 0; // variable to check if the top switch has been pressed
	
	unsigned char hold_value_1; // hold value used in this function to set values to EEPROM
	unsigned char hold_value_2; // hold value used in this function to set values to EEPROM
	
	
	// Manipulator Calibration

	if (p4_sw_trig == 0) // if trigger is not pressed
	click_handler_1 = 0;// it is no longer being held down


	if((p4_sw_trig) && (p4_sw_trig != click_handler_1)) // if trigger has just been pressed
	{
		
		switch(rotary_dial)	
		{
		
		case 1:	// BOX

			
			Base_Position_Box = (int)ENCODER_BASE;	// set global int to encoder value
			hold_value_1 = Base_Position_Box & 0xFF;	// set lower byte
			hold_value_2 = (Base_Position_Box >> 8) & 0xff;	// set upper byte
			printf("initial value, write value lower, write value higher  \r%d\r%d,%d\r", Base_Position_Box,hold_value_1,hold_value_2);
			EEPROM_prep(BASE_BOX_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address	
			EEPROM_prep(BASE_BOX_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

			Elbow_Position_Box = (int)ENCODER_ELBOW;	// set global int to encoder value
			hold_value_1 = Elbow_Position_Box & 0xFF;	// set lower byte
			hold_value_2 = (Elbow_Position_Box >> 8) & 0xff;	// set upper byte
			EEPROM_prep(ELBOW_BOX_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(ELBOW_BOX_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

			Wrist_Position_Box = (int)ENCODER_WRIST;	// set global int to encoder value
			hold_value_1 = Wrist_Position_Box & 0xFF;	// set lower byte
			hold_value_2 = (Wrist_Position_Box >> 8) & 0xff;	// set upper byte
			EEPROM_prep(WRIST_BOX_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(WRIST_BOX_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

		break;	

		case 2:	// FLOOR

			Base_Position_Floor = (int)ENCODER_BASE;	// set global int to encoder value
			hold_value_1 = Base_Position_Floor & 0xFF;	// set lower byte
			hold_value_2 = (Base_Position_Floor >> 8) & 0xff;	// set upper byte
			EEPROM_prep(BASE_FLOOR_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(BASE_FLOOR_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

			Elbow_Position_Floor = (int)ENCODER_ELBOW;	// set global int to encoder value
			hold_value_1 = Elbow_Position_Floor & 0xFF;	// set lower byte
			hold_value_2 = (Elbow_Position_Floor >> 8) & 0xff;	// set upper byte
			EEPROM_prep(ELBOW_FLOOR_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(ELBOW_FLOOR_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address
	
			Wrist_Position_Floor = (int)ENCODER_WRIST;	// set global int to encoder value
			hold_value_1 = Wrist_Position_Floor & 0xFF;	// set lower byte
			hold_value_2 = (Wrist_Position_Floor >> 8) & 0xff;	// set upper byte
			EEPROM_prep(WRIST_FLOOR_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(WRIST_FLOOR_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

		break;	

		case 3:	// LOW

			Base_Position_Low = (int)ENCODER_BASE;	// set global int to encoder value
			hold_value_1 = Base_Position_Low & 0xFF;	// set lower byte
			hold_value_2 = (Base_Position_Low >> 8) & 0xff;	// set upper byte
			EEPROM_prep(BASE_LOW_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(BASE_LOW_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

			Elbow_Position_Low = (int)ENCODER_ELBOW;	// set global int to encoder value
			hold_value_1 = Elbow_Position_Low & 0xFF;	// set lower byte
			hold_value_2 = (Elbow_Position_Low >> 8) & 0xff;	// set upper byte
			EEPROM_prep(ELBOW_LOW_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(ELBOW_LOW_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

			Wrist_Position_Low = (int)ENCODER_WRIST;	// set global int to encoder value
			hold_value_1 = Wrist_Position_Low & 0xFF;	// set lower byte
			hold_value_2 = (Wrist_Position_Low >> 8) & 0xff;	// set upper byte
			EEPROM_prep(WRIST_LOW_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(WRIST_LOW_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

		break;	

		case 4:	// MIDDLE

			Base_Position_Middle = (int)ENCODER_BASE;	// set global int to encoder value
			hold_value_1 = Base_Position_Middle & 0xFF;	// set lower byte
			hold_value_2 = (Base_Position_Middle >> 8) & 0xff;	// set upper byte
			EEPROM_prep(BASE_MIDDLE_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(BASE_MIDDLE_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

			Elbow_Position_Middle = (int)ENCODER_ELBOW;	// set global int to encoder value
			hold_value_1 = Elbow_Position_Middle & 0xFF;	// set lower byte
			hold_value_2 = (Elbow_Position_Middle >> 8) & 0xff;	// set upper byte
			EEPROM_prep(ELBOW_MIDDLE_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(ELBOW_MIDDLE_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

			Wrist_Position_Middle = (int)ENCODER_WRIST;	// set global int to encoder value
			hold_value_1 = Wrist_Position_Middle & 0xFF;	// set lower byte
			hold_value_2 = (Wrist_Position_Middle >> 8) & 0xff;	// set upper byte
			EEPROM_prep(WRIST_MIDDLE_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(WRIST_MIDDLE_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

		break;	

		case 5:	// HIGH

			Base_Position_High = (int)ENCODER_BASE;	// set global int to encoder value
			hold_value_1 = Base_Position_High & 0xFF;	// set lower byte
			hold_value_2 = (Base_Position_High >> 8) & 0xff;	// set upper byte
			EEPROM_prep(BASE_HIGH_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(BASE_HIGH_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

			Elbow_Position_High = (int)ENCODER_ELBOW;	// set global int to encoder value
			hold_value_1 = Elbow_Position_High & 0xFF;	// set lower byte
			hold_value_2 = (Elbow_Position_High >> 8) & 0xff;	// set upper byte
			EEPROM_prep(ELBOW_HIGH_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(ELBOW_HIGH_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address
	
			Wrist_Position_High = (int)ENCODER_WRIST;	// set global int to encoder value
			hold_value_1 = Wrist_Position_High & 0xFF;	// set lower byte
			hold_value_2 = (Wrist_Position_High >> 8) & 0xff;	// set upper byte
			EEPROM_prep(WRIST_HIGH_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(WRIST_HIGH_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

		break;	

		}	
	}
	
	if(p4_sw_trig == 1)	// if trigger is being pressed
	click_handler_1 = 1; // it is being held down	
	

	// Gripper Calibration			
	
	if (p4_sw_top == 0) // if trigger is not pressed
	{
		click_handler_2 = 0;	// it is no longer being held down
	}

	if((p4_sw_top) && (p4_sw_top != click_handler_2)) // if trigger has just been pressed
	{
	
		switch(rotary_dial)
		{

		case 1: // OPEN
		case 2:
		case 3:

			Gripper_Open = (int)ENCODER_GRIPPER;	// set global int to encoder value
			hold_value_1 = Gripper_Open  & 0xFF;	// set lower byte
			hold_value_2 = (Gripper_Open >> 8) & 0xff;	// set upper byte
			EEPROM_prep(GRIPPER_OPEN_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(GRIPPER_OPEN_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

		break;

		case 4: // CLOSED
	    case 5:

			Gripper_Closed = (int)ENCODER_GRIPPER;	// set global int to encoder value
			hold_value_1 = Gripper_Closed  & 0xFF;	// set lower byte
			hold_value_2 = (Gripper_Closed >> 8) & 0xff;	// set upper byte
			EEPROM_prep(GRIPPER_CLOSED_EEPROM_LOW_BYTE,hold_value_1);	// set lower byte to an EEPROM address
			EEPROM_prep(GRIPPER_CLOSED_EEPROM_HIGH_BYTE,hold_value_2);	// set upper byte to an EEPROM address

		break;


		}	
	}

	if(p4_sw_top == 1)	// if trigger is being pressed
	click_handler_2 = 1; // it is being held down


}


/********************** Manipulator Initialization Function ***********************

Inputs: 
	Reads EEPROM values (but no direct input)
Output:
	Sets manipulator global variables in user_routines.c  (but no direct output)

**********************************************************************************/

void manipulator_initialization(void)	
{

	static unsigned char EEPROM_counter = 0;	// used to avoid running through too many EEPROM reads per code loop 
	unsigned char hold_data_1;	// used to temporarily hold lower bytes for EEPROM_writing
	unsigned char hold_data_2;	// used to temporarily hold lower bytes for EEPROM_writing

	if (EEPROM_counter < 5) // if counter has not reached the 5th code loop
	{
		EEPROM_counter ++;  // add to the counter
	}
	else // if counter has reached the 5th code loop
	{
		return; // get out of the function
	}


/////////////////////////////// Arm EEPROM Readings /////////////////////////////

	if (EEPROM_counter == 1) // Read the BASE presets on the 1st code loop
	{

		hold_data_1 = EEPROM_read(BASE_BOX_EEPROM_LOW_BYTE); // read lower byte
		hold_data_2 = EEPROM_read(BASE_BOX_EEPROM_HIGH_BYTE); // read upper byte
		Base_Position_Box = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(BASE_FLOOR_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(BASE_FLOOR_EEPROM_HIGH_BYTE); // read upper byte
		Base_Position_Floor = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(BASE_LOW_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(BASE_LOW_EEPROM_HIGH_BYTE); // read upper byte
		Base_Position_Low = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(BASE_MIDDLE_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(BASE_MIDDLE_EEPROM_HIGH_BYTE); // read upper byte
		Base_Position_Middle = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(BASE_HIGH_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(BASE_HIGH_EEPROM_HIGH_BYTE); // read upper byte
		Base_Position_High = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

//		printf("1");
	}


	if (EEPROM_counter == 2) // Read the ELBOW presets on the 1st code loop
	{

		hold_data_1 = EEPROM_read(ELBOW_BOX_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(ELBOW_BOX_EEPROM_HIGH_BYTE); // read upper byte
		Elbow_Position_Box = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(ELBOW_FLOOR_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(ELBOW_FLOOR_EEPROM_HIGH_BYTE); // read upper byte
		Elbow_Position_Floor = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(ELBOW_LOW_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(ELBOW_LOW_EEPROM_HIGH_BYTE); // read upper byte
		Elbow_Position_Low = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(ELBOW_MIDDLE_EEPROM_LOW_BYTE);	// read lower byte	
		hold_data_2 = EEPROM_read(ELBOW_MIDDLE_EEPROM_HIGH_BYTE); // read upper byte
		Elbow_Position_Middle = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(ELBOW_HIGH_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(ELBOW_HIGH_EEPROM_HIGH_BYTE); // read upper byte
		Elbow_Position_High = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

//		printf("2");
}

	if (EEPROM_counter == 3) // Read the WRIST presets on the 1st code loop
	{

		hold_data_1 = EEPROM_read(WRIST_BOX_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(WRIST_BOX_EEPROM_HIGH_BYTE); // read upper byte
		Wrist_Position_Box = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(WRIST_FLOOR_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(WRIST_FLOOR_EEPROM_HIGH_BYTE); // read upper byte
		Wrist_Position_Floor = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(WRIST_LOW_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(WRIST_LOW_EEPROM_HIGH_BYTE); // read upper byte
		Wrist_Position_Low = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(WRIST_MIDDLE_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(WRIST_MIDDLE_EEPROM_HIGH_BYTE); // read upper byte
		Wrist_Position_Middle = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

		hold_data_1 = EEPROM_read(WRIST_HIGH_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(WRIST_HIGH_EEPROM_HIGH_BYTE); // read upper byte
		Wrist_Position_High = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

//		printf("3");
	}


	if (EEPROM_counter == 4) // Read the GRIPPER presets on the 1st code loop
	{

		hold_data_1 = EEPROM_read(GRIPPER_OPEN_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(GRIPPER_OPEN_EEPROM_HIGH_BYTE); // read upper byte
		Gripper_Open = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value
	
		hold_data_1 = EEPROM_read(GRIPPER_CLOSED_EEPROM_LOW_BYTE);	// read lower byte
		hold_data_2 = EEPROM_read(GRIPPER_CLOSED_EEPROM_HIGH_BYTE); // read upper byte
		Gripper_Closed = (((unsigned int)hold_data_2 & 0xff) << 8) | ((unsigned int)hold_data_1 & 0xff); // mesh lower and upper bytes for actual value

//		printf("4");
	}


//	printf("Base Box, Elbow High, Gripper Open, Rotary Dial Position %d,%d,%d,%d\r", Base_Position_Box, Elbow_Position_High, Gripper_Open, rotary_dial);


}


/****************************** Go Home Function ***********************************

Inputs: 
	the joint in question and the limit switch values for that joint
Output:
	a 0-254 value for the motor on the joint in question

**********************************************************************************/

unsigned char goto_box(unsigned char joint_to_be_moved)
{

unsigned char hold_value; // hold value for various use in function 

	switch(joint_to_be_moved) // use joint in question variable to determine what limit switch to check and what direction to set the motor in question
	{

	case BASE:

		if(LIMIT_SWITCH_BASE == 0) // if the limit switch on the base is not at home
		{
			hold_value = 127 - BASE_SPEED; // set the hold value to a backwards direction
			direction_base = 1; // set the base encoder direction to positive
		}
		else // if the base limit switch is engaged
		{
		return 127; // return neutral
		}

	break;

	case ELBOW:

		if(LIMIT_SWITCH_ELBOW == 0) // if the limit switch on the elbow is not at home
		{
		hold_value = 127 - ELBOW_SPEED; // set the hold value to a backwards direction
		direction_elbow = 1; // set the elbow encoder direction to positive
		}
		else // if the elbow limit switch is engaged
		{
		return 127; // return neutral
		}

	break;

	case WRIST:

		if(LIMIT_SWITCH_WRIST == 0) // if the limit switch on the wrist is not at home
		{
		hold_value = 127 - WRIST_SPEED; // set the hold value to a backwards direction
		direction_wrist = 1; // set the wrist encoder direction to positive
		}
		else // if the wrist limit switch is engaged
		{
		return 127; // return neutral
		}


	break;

	case GRIPPER:

		if(LIMIT_SWITCH_GRIPPER == 0) // if the limit switch on the gripper is not at home
		{
		hold_value = 127 - GRIPPER_SPEED_BACKWARD; // set the hold value to a backwards direction
		direction_gripper = -1; // set the gripper encoder direction to negative
		}
		else // if the gripper limit switch is engaged
		{
		return 127; // return neutral
		}


	break;

	}

return hold_value; // return the value determined by the mathematical logic

}

