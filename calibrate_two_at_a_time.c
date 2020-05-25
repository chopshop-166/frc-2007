//Calibrate two at a time (forward, backward)
//Use user input to select forward or backward, use another input to start

//motor state defines
#define MOTOR_STATE_FRONT		0
#define MOTOR_STATE_REAR		1
#define MOTOR_STATE_FINAL		2



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



//cal state defines
#define CAL_START				0
#define CAL_SPEEDONE			1
#define CAL_SPEEDTWO			2
#define CAL_SPEEDTHREE			3
#define CAL_SPEEDFOUR			4
#define CAL_SPEEDFIVE			5
#define CAL_SPEEDSIX			6
#define CAL_SPEEDSEVEN			7
#define CAL_SPEEDEIGHT			8
#define CAL_END					9

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

//bit masks for forward and reverse calibrations (for "done" variable)
#define FORWARD_BIT_MASK		1
#define REVERSE_BIT_MASK		2

//debug
#define TWOATATIME_DEBUG

void calibrate_two_at_a_time(void)
{
	//maximum clicks per given period of time for each motor
	//[front-back][left-right][speed]
	static long maxclicks[2][2][8];
	
	//clicks left and right, only need to do 2 at a time
	long clicks_l, clicks_r;
	
	//are calibrations done?
	static unsigned char done = 0;
	
	//which motors are we doing?
	static unsigned char motor_state = MOTOR_STATE_FRONT;
	
	//what stage of calibration are we in?
	static unsigned char cal_state = CAL_STATE_START;
	
	//is MOTOR_SWITCH being held?
	static unsigned char motor_switch_hold = 0;
	
	
	//do we do anything or wait to be told to do something?
	static unsigned char go = 0;
	
	//count the loops
	static unsigned int loopcount = 0;
	
	//loop counter for debug
#ifdef TWOATATIME_DEBUG
	static unsigned int debug_loopcount = 0;
	debug_loopcount++;
#endif
	loopcount++;
	
	//If the switch is being pressed, but not held...
	if((MOTOR_SWITCH) && !(motor_switch_hold))
	{
		//change motor state
		motor_state++;
		
		//make sure we're not above FINAL
		if(motor_state > MOTOR_STATE_FINAL)
		{
			motor_state = MOTOR_STATE_FRONT;
		}
		//it's being held now.
		motor_switch_hold = 1;
#ifdef TWOATATIME_DEBUG
		printf("Switched modes. New mode: %d\r",motor_state);
#endif
	}
	//If the switch isn't being pressed, but we think it's being held...
	else if(!(MOTOR_SWITCH) && (motor_switch_hold))
	{
		//it's not being held.
		motor_switch_hold = 0;
	}
	
	//if go is 0, and GO_BUTTON is being pressed...
	if(!(go) && (GO_BUTTON))
	{
		//go!
		go = 1;
		
		//reset loop count
		loopcount = 0;
#ifdef TWOATATIME_DEBUG
		printf("We're going now.\r");
#endif
	}
	
	//if we're going...
	if(go)
	{
		//if we're doing a motor
		if((motor_state == MOTOR_STATE_FRONT) || (motor_state == MOTOR_STATE_REVERSE))
		{
			
		}
	}
}