//motor state defines
#define MOTOR_STATE_FRONT		0
#define MOTOR_STATE_BACK		1
#define MOTOR_STATE_FINAL		2


//how far to shift for EEPROM addressing
#define FB_SHIFTAMOUNT			4
#define LR_SHIFTAMOUNT			3
#define SPEED_SHIFTAMOUNT		0

//********
//array index #defines
//********
//left-right
#define ARRAYINDEX_LEFT			0
#define ARRAYINDEX_RIGHT		1

//front-back
#define ARRAYINDEX_FRONT		0
#define ARRAYINDEX_BACK			1


//which speed
#define ARRAYINDEX_SPEEDONE		0
#define ARRAYINDEX_SPEEDTWO		1
#define ARRAYINDEX_SPEEDTHREE	2
#define ARRAYINDEX_SPEEDFOUR	3
#define ARRAYINDEX_SPEEDFIVE	4
#define ARRAYINDEX_SPEEDSIX		5
#define ARRAYINDEX_SPEEDSEVEN	6
#define ARRAYINDEX_SPEEDEIGHT	7
//********
//End array index #defines
//********

#define CAL_SWITCHONE		p1_sw_aux1
#define CAL_SWITCHTWO		p1_sw_aux2

//cal state defines
#define CAL_START				-1
#define CAL_SPEEDONE			0
#define CAL_SPEEDTWO			1
#define CAL_SPEEDTHREE			2
#define CAL_SPEEDFOUR			3
#define CAL_SPEEDFIVE			4
#define CAL_SPEEDSIX			5
#define CAL_SPEEDSEVEN			6
#define CAL_SPEEDEIGHT			7
#define CAL_END					8

//switches between motors
#define MOTOR_SWITCH			p1_sw_trig

//does the calibration
#define GO_BUTTON				p1_sw_top

//#defines for clicks
#define FL_CLICK_COUNT			Get_Encoder_3_Count()
#define FR_CLICK_COUNT			Get_Encoder_4_Count()
#define BL_CLICK_COUNT			Get_Encoder_5_Count()
#define BR_CLICK_COUNT			Get_Encoder_6_Count()

//#defines for click resetting
#define RESET_FL_CLICKS			Reset_Encoder_3_Count()
#define RESET_FR_CLICKS			Reset_Encoder_4_Count()
#define RESET_BL_CLICKS			Reset_Encoder_5_Count()
#define RESET_BR_CLICKS			Reset_Encoder_6_Count()

//bits for front and back (for "done" variable)
#define FRONT_BIT			1
#define BACK_BIT			2


//change rate, how fast the motor speed changes
#define CHANGE_RATE				0x10

//motor pwm outputs
#define MOTOR_FL				pwm01
#define MOTOR_FR				pwm02
#define MOTOR_BL				pwm03
#define MOTOR_BR				pwm04

//invert motors?
#define INVERT_LEFT				0
#define INVERT_RIGHT			0

//how long we wait, in loops
#define WAIT_TIME				32

//percent of full speed each speed is
#define SPEEDEIGHT_PERCENT		100
#define SPEEDSEVEN_PERCENT		88
#define SPEEDSIX_PERCENT		75
#define SPEEDFIVE_PERCENT		63
#define SPEEDFOUR_PERCENT		50
#define SPEEDTHREE_PERCENT		38
#define SPEEDTWO_PERCENT		25
#define SPEEDONE_PERCENT		13


//user feedback
#define USER_FEEDBACK_BYTE		User_Mode_byte

//calibration debug
#define CAL_DEBUG

void calibrate_two_at_a_time(void);
