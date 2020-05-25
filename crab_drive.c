#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "crab_drive.h"


//#define DRIVE_DEBUG
//#define ACL_DEBUG


// Function: crab_drive
// Parameters:
//	unsigned char input_x: left-right movement. 0 is full speed left and 255 is full speed right unless invert_x_axis is true. 127 or 128 is neutral.
//	unsigned char input_y: front-back movement. 0 is full speed back and 255 is full speed forward unless invert_y_axis is true. 127 or 128 is neutral.
//	unsigned char input_z: rotational movement. 0 is full speed twist left and 255 is full speed twist right unless invert_z_axis is true. 127 or 128 is neutral.
//Returns: void
//Function:
//	crab_drive, given 3 input axes, calculates the desired motor speeds assuming mechanum wheels are attached to 4 motors.
void crab_drive(unsigned char input_x, unsigned char input_y, unsigned char input_z)
{

	//We want to make the joysticks partially negative and do integer calculations with them, so we cast to int.
	int joy_x = (int)input_x;
	int joy_y = (int)input_y;
	int joy_z = (int)input_z;
	//These are the working motor values. The motors are set to these after all calculations and conversions are done.
	int front_left;
	int front_right;
	int back_left;
	int back_right;
	
	//Center joysticks at 0. newbase is defined in crab_drive.h
	newbase(joy_x);
	newbase(joy_y);
	newbase(joy_z);

	//Invert an axis if we need to (if our joysticks are weird or flipped around). The invert_*_axis defines are defined in crab_drive.h.
	if(invert_x_axis)
	{
		joy_x *= -1;
	}
	if(invert_y_axis)
	{
		joy_y *= -1;
	}
	if(invert_z_axis)
	{
		joy_z *= -1;
	}
	
	//Set joysticks to 0 if in the dead zone, DEAD_ZONE is defined in crab_drive.h
	if(joy_x < DEAD_ZONE && joy_x > -DEAD_ZONE)
	{
		joy_x = 0;
	}
	if(joy_y < DEAD_ZONE && joy_y > -DEAD_ZONE)
	{
		joy_y = 0;
	}
	if(joy_z < DEAD_ZONE && joy_z > -DEAD_ZONE)
	{
		joy_z = 0;
	}
	
	//Calculate desired motor speeds.
	front_left	= joy_x + joy_y + joy_z;
	front_right = -joy_x + joy_y - joy_z;
	back_left	= -joy_x + joy_y + joy_z;
	back_right	= joy_x + joy_y - joy_z;
	
	//Invert right side if we need to (if the motors are mounted the opposite direction, as they usually are). invert_right is defined in crab_drive.h.
	if(invert_right)
	{
		front_right *= -1;
		back_right *= -1;
	}
	
	//We want to go half speed normally when driving because our robot is very fast, the trigger allows us to go full speed.
	//When in autonomous mode, we want to go full speed because we like to run into stuff and autonomous mode can't press the trigger.
	if((!p1_sw_trig) && (!autonomous_mode))
	{
		front_left /= 2;
		front_right /= 2;
		back_left /= 2;
		back_right /= 2;
	}
	
	//Ensure the motor values are within -127 to +127. limit is defined in crab_drive.h.
	limit(front_left)
	limit(front_right)
	limit(back_left)
	limit(back_right)	
	
	//Center motors at 127-128. oldbase is defined in crab_drive.h.
	oldbase(front_left)
	oldbase(front_right)
	oldbase(back_left)
	oldbase(back_right)
	
	//Here is where we set the motor speeds. We only take the lower byte.
	motor_fl = front_left & 0xff;
	motor_fr = front_right & 0xff;
	motor_bl = back_left & 0xff;
	motor_br = back_right & 0xff;
	return;
}
