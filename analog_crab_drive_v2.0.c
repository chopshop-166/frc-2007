#include "ifi_aliases.h"

//newbase: rebases an unsigned char (0 - 255) to be centered at 0 (-127 to 127)
#define newbase(num)	if(num <= 127){num -= 127;}else if(num >= 128){num -= 128;}
//limit: limits an int to have a minimum value of -127 and a maximum value of 127
#define limit(num) if(num < -127) { num = -127; } else if(num > 127) { num = -127; }
//oldbase: rebases a signed number (-127 to 127) to be centered at 127-128
#define oldbase(num) if(num <= 0) {num += 127; } else { num += 128; }

//f: front, b: back, l: left, r: right
#define motor_fl pwm01
#define motor_fr pwm02
#define motor_bl pwm03
#define motor_br pwm04

unsigned int fl_percent_fwd = 100;
unsigned int fr_percent_fwd = 100;
unsigned int bl_percent_fwd = 100;
unsigned int br_percent_fwd = 100;
unsigned int fl_percent_rev = 100;
unsigned int fr_percent_rev = 100;
unsigned int bl_percent_rev = 100;
unsigned int br_percent_rev = 100;
char invert_left;

void crab_drive(unsigned char input_x, unsigned char input_y, unsigned char input_z)
{
	int joy_x = (int)input_x;
	int joy_y = (int)input_y;
	int joy_z = (int)input_z;
	int front_left;
	int front_right;
	int back_left;
	int back_right;

	//initialize to 1, so we don't get a divide by 0 error
	int fl_percent = 1, fr_percent = 1, bl_percent = 1, br_percent = 1;
	//center joysticks at 0
	newbase(joy_x);
	newbase(joy_y);
	newbase(joy_z);
	//calculate desired motor speeds
	front_left	= (joy_x + joy_y + joy_z);
	front_right = (-joy_x + joy_y - joy_z);
	back_left	= (-joy_x + joy_y + joy_z);
	back_right	= (joy_x + joy_y - joy_z);
	
	//limit to -127 -> 127
	limit(front_left)
	limit(front_right)
	limit(back_left)
	limit(back_right)
	
	//what direction are motors going?
	if(front_left > 0)
	{
		fl_percent = (int)fl_percent_fwd;
	}
	else if(front_left < 0)
	{
		fl_percent = (int)fl_percent_rev;
	}
	if(front_right > 0)
	{
		fr_percent = (int)fr_percent_fwd;
	}
	else if(front_right < 0)
	{
		fr_percent = (int)fr_percent_rev;
	}
	if(back_left > 0)
	{
		bl_percent = (int)bl_percent_fwd;
	}
	else if(back_left < 0)
	{
		bl_percent = (int)bl_percent_rev;
	}
	if(back_right > 0)
	{
		br_percent = (int)br_percent_fwd;
	}
	else if(back_right < 0)
	{
		br_percent = (int)br_percent_rev;
	}
	
	//invert left side if we need to.
	if(invert_left)
	{
		front_left *= -1;
		back_left *= -1;
	}
	
	//reduce motors to actual values
	front_left *= fl_percent;
	front_left /= 100;
	front_right *= fr_percent;
	front_right /= 100;
	back_left *= bl_percent;
	back_left /= 100;
	back_right *= br_percent;
	back_right /= 100;

	//center motors at 127-128
	oldbase(front_left)
	oldbase(front_right)
	oldbase(back_left)
	oldbase(back_right)
	//fast int -> unsigned char casting (bitwise and, lower byte)
	motor_fl = front_left & 255;
	motor_fr = front_right & 255;
	motor_bl = back_left & 255;
	motor_br = back_right & 255;
	return;
}
