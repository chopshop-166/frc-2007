//joystick dead zone, on each side of center of axis
#define DEAD_ZONE 8

//f: front, b: back, l: left, r: right These are what crab_drive outputs to.
//Set these to the PWMs for the corresponding motors if you want to use crab_drive to drive the robot.
#define motor_fl pwm01
#define motor_fr pwm02
#define motor_bl pwm03
#define motor_br pwm04


//newbase: rebases an unsigned char (0 - 255) to be centered at 0 (-127 to 127). 
#define newbase(num)	if(num <= 127){num -= 127;}else if(num >= 128){num -= 128;}
//limit: limits an int to have a minimum value of -127 and a maximum value of 127.
#define limit(num) if(num < -127) { num = -127; } else if(num > 127) { num = 127; }
//oldbase: rebases a signed number (-127 to 127) to be centered at 127-128.
#define oldbase(num) if(num <= 0) {num += 127; } else { num += 128; }
//aclreduce: reduces the power based on a predefined acceleration rate. The drivers didn't like this because it gives them less of an ability to react quickly.
#define aclreduce(num,oldnum) if(num < 0) { if(num < oldnum - aclrate) { num = oldnum - aclrate; } } else if(num > 0) { if(num > oldnum + aclrate) { num = oldnum + aclrate; } }


//invert motors on right side. Use this if the motors on the right are oriented so that driving them forward would drive the robot backwards.
#define invert_right 1

//invert joystick axes for drive input. Use these for joysticks where an axis is the opposite of what you want.
//Note: Be advised that this will affect function calls while in autonomous mode, so changing these will screw up any autonomous mode that uses crab_drive.
#define invert_x_axis			0
#define invert_y_axis			0
#define invert_z_axis			0

void crab_drive(unsigned char,unsigned char,unsigned char);
