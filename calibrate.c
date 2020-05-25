#include <stdio.h>
#include "calibrate.h"
#include "ifi_aliases.h"
#include "ifi_utilities.h"
#include "ifi_default.h"
#include "user_routines.h"
#include "encoder.h"
#include "eeprom.h"




//absolute value for long
long abslong(long input)
{
	if(input < 0)
	{
		return (input * -1);
	}
	return input;
}


void calibrate_two_at_a_time(void)
{
	//maximum clicks per given period of time for each motor
	//[front-back][left-right][speed]
	static long clicks[2][2][8];
	
	//compensations, we store this in EEPROM
	char offset[2][2][8] = {
		{
			{0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}
		},
		{
			{0,0,0,0,0,0,0,0}, {0,0,0,0,0,0,0,0}
		} };
	
	//are calibrations done?
	static unsigned char done = 0;
	
	//which motors are we doing?
	static unsigned char motor_state = MOTOR_STATE_FRONT;
	
	//what stage of calibration are we in?
	static char cal_state = CAL_START;
	
	//is MOTOR_SWITCH being held?
	static unsigned char motor_switch_hold = 0;
	
	//left and right motor outputs
	static unsigned char motor_left = 127, motor_right = 127;
	
	//do we do anything or wait to be told to do something?
	static unsigned char go = 0;
	
	//slowest motor encoder count
	long lowest_count;
	
	//are we done calculating percentages?
	static unsigned char percentcalculations_done = 0;
	
	//count the loops
	static unsigned int loopcount = 0;
	
	
	//loop counter for debug
#ifdef CAL_DEBUG
	static unsigned int debug_loopcount = 0;
	debug_loopcount++;
#endif
	loopcount++;

	USER_FEEDBACK_BYTE = motor_state;


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
	}
	//If the switch isn't being pressed, but we think it's being held...
	else if(!(MOTOR_SWITCH) && (motor_switch_hold))
	{
		//it's not being held.
		motor_switch_hold = 0;
	}
	
	//if go is 0, and GO_BUTTON is being pressed...
	if((go == 0) && (GO_BUTTON))
	{
		//go!
		go = 1;
		
		//reset loop count
		loopcount = 0;
	}

	//if we're not going
	if(!go)
	{
		//make sure all motors are netural
		MOTOR_FL = MOTOR_FR = MOTOR_BL = MOTOR_BR = 127;
		motor_left = motor_right = 127;
	}
	//if we're going...
	else
	{
		//if we're doing the front motors
		if(motor_state == MOTOR_STATE_FRONT)
		{
			switch(cal_state)
			{
				case CAL_START:
					//go to speedone, fall through
					cal_state++;
			    case CAL_SPEEDONE:
				case CAL_SPEEDTWO:
				case CAL_SPEEDTHREE:
				case CAL_SPEEDFOUR:
				case CAL_SPEEDFIVE:
				case CAL_SPEEDSIX:
				case CAL_SPEEDSEVEN:
				case CAL_SPEEDEIGHT:

					//left side is not inverted, we need to accellerate
					if(cal_state == 0)
					{
						if(motor_left <= 127)
						{
							motor_left = 127 +
								CHANGE_RATE +
								offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state];
						}
						if(motor_right <= 127)
						{
							motor_right = 127 +
								CHANGE_RATE +
								offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state];
							break;
						}
					}
					else
					{
						if(motor_left <= (127 + (((cal_state + 1) * CHANGE_RATE) + (offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state - 1]))))
						{
							//accellerate
						
							motor_left = 127 +
								((cal_state + 1) * CHANGE_RATE) +
								(offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state - 1]) +
								(offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state]);
						
						}
						//same thing, but with right
						if(motor_right <= (127 + ((cal_state * CHANGE_RATE) + (offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state - 1]))))
						{
							motor_right = 127 +
								((cal_state + 1) * CHANGE_RATE) +
								(offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state - 1]) +
								(offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state]);
						
							//break now, no point in looking at loopcount
							break;
						}
					}
					//now, wait a bit for the clicks to accumulate

					//we need to wait more...
					if(loopcount < WAIT_TIME)
					{
						break;
					}


					//we've waited long enough
					//store clicks in array
					clicks[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state] = FL_CLICK_COUNT;
					clicks[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state] = FR_CLICK_COUNT;
					
					//left is faster than right. speed up right, slow down left
					if(clicks[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state] > clicks[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state])
					{
						offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state] = 
							-(((((clicks[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state] * 100) /
							clicks[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state]) /
							2)) % 100);
						
						offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state] = 
							((((clicks[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state] * 100) /
							clicks[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state]) /
							2) % 100);
					}
					//right is faster than left, speed up left, slow down right
					if(clicks[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state] > clicks[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state])
					{
						offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state] = 
							((((clicks[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state] * 100) /
							clicks[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state]) /
							2) % 100);
						
						offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state] = 
							-(((((clicks[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][cal_state] * 100) /
							clicks[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][cal_state]) /
							2)) % 100);
					}

					
					//rest clicks
					RESET_FL_CLICKS;
					RESET_FR_CLICKS;

					//rest loops
					loopcount = 0;

					//change state, fall through
					cal_state++;
					break;
				case CAL_END:
				//if the front hasn't been done before, say it has been done
					if(!(done & FRONT_BIT))
					{
						//forward is now done
						done += 1;
					}
					//don't go again
					go = 0;
					cal_state = CAL_START;
					motor_left = 127;
					motor_right = 127;
					break;
			}

		MOTOR_FL = motor_left;
		MOTOR_FR = motor_right;
		}
		//if we're doing the back motors
		else if(motor_state == MOTOR_STATE_BACK)
		{
			switch(cal_state)
			{
				case CAL_START:
					cal_state++;
			    case CAL_SPEEDONE:
				case CAL_SPEEDTWO:
`				case CAL_SPEEDTHREE:
				case CAL_SPEEDFOUR:
				case CAL_SPEEDFIVE:
				case CAL_SPEEDSIX:
				case CAL_SPEEDSEVEN:
				case CAL_SPEEDEIGHT:
					//left side is not inverted, we need to accellerate
					if(cal_state == 0)
					{
						if(motor_left <= 127)
						{
							motor_left = 127 +
								CHANGE_RATE +
								offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state];
						}
						if(motor_right <= 127)
						{
							motor_right = 127 +
								CHANGE_RATE +
								offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state];
							break;
						}
					}
					else
					{
						if(motor_left <= (127 + (((cal_state + 1) * CHANGE_RATE) + (offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state - 1]))))
						{
							//accellerate
						
							motor_left = 127 +
								((cal_state + 1) * CHANGE_RATE) +
								(offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state - 1]) +
								(offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state]);
						
						}
						//same thing, but with right
						if(motor_right <= (127 + ((cal_state * CHANGE_RATE) + (offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state - 1]))))
						{
							motor_right = 127 +
								((cal_state + 1) * CHANGE_RATE) +
								(offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state - 1]) +
								(offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state]);
						
							//break now, no point in looking at loopcount
							break;
						}
					}
					//now, wait a bit for the clicks to accumulate

					//we need to wait more...
					if(loopcount < WAIT_TIME)
					{
						break;
					}


					//we've waited long enough
					//store clicks in array
					clicks[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state] = FL_CLICK_COUNT;
					clicks[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state] = FR_CLICK_COUNT;

					//left is faster than right. speed up right, slow down left
					if(clicks[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state] > clicks[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state])
					{
						offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state] = 
							-(((((clicks[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state] * 100) /
							clicks[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state]) /
							2)) % 100);
						
						offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state] = 
							((((clicks[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state] * 100) /
							clicks[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state]) /
							2) % 100);
					}
					//right is faster than left, speed up left, slow down right
					if(clicks[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state] > clicks[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state])
					{
						offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state] = 
							((((clicks[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state] * 100) /
							clicks[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state]) /
							2) % 100);
						
						offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state] = 
							-(((((clicks[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][cal_state] * 100) /
							clicks[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][cal_state]) /
							2)) % 100);
					}
					
					
					//rest clicks
					RESET_BL_CLICKS;
					RESET_BR_CLICKS;

					//rest loops
					loopcount = 0;

					//change state, fall through
					cal_state++;
					break;
				case CAL_END:
				//if the forward hasn't been done before, say it has been done before
					if(!(done & BACK_BIT))
					{
						//forward is now done
						done += BACK_BIT;
					}
					//don't go again until user hits button
					go = 0;
					//reset cal_state
					cal_state = CAL_START;
					RESET_BL_CLICKS;
					RESET_BR_CLICKS;
					break;
			}

		MOTOR_BL = motor_left;
		MOTOR_BR = motor_right;
		}
		//make sure we've done the other two states, then do calculations
		else if((motor_state == MOTOR_STATE_FINAL) && ((done & (FRONT_BIT | BACK_BIT)) == (FRONT_BIT | BACK_BIT)))
		{
			//find lowest encoder count (at max power)
			lowest_count = abslong(clicks[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDEIGHT]);
			if(abslong(clicks[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDEIGHT] < lowest_count))
			{
				lowest_count = abslong(clicks[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDEIGHT]);
			}
			if(abslong(clicks[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDEIGHT]) < lowest_count)
			{
				lowest_count = abslong(clicks[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDEIGHT]);
			}
			if(abslong(clicks[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDEIGHT]) < lowest_count)
			{
				lowest_count = abslong(clicks[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDEIGHT]);
			}
			//all percentages have been calculated, now store in EEPROM

			//slowest speed
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDONE)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDONE]);
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDONE)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDONE]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDONE)), offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDONE]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDONE)), offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDONE]);
			
			//2/8
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDTWO)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDTWO]);
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDTWO)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDTWO]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDTWO)), offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDTWO]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDTWO)), offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDTWO]);
			
			//3/8
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDTHREE)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDTHREE]);
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDTHREE)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDTHREE]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDTHREE)), offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDTHREE]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDTHREE)), offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDTHREE]);
			
			//4/8
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDFOUR)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDFOUR]);
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDFOUR)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDFOUR]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDFOUR)), offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDFOUR]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDFOUR)), offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDFOUR]);
			
			//5/8
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDFIVE)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDFIVE]);
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDFIVE)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDFIVE]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDFIVE)), offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDFIVE]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDFIVE)), offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDFIVE]);
			
			//6/8
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDSIX)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDSIX]);
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDSIX)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDSIX]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDSIX)), offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDSIX]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDSIX)), offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDSIX]);
			
			//7/8
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDSEVEN)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDSEVEN]);
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDSEVEN)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDSEVEN]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDSEVEN)), offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDSEVEN]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDSEVEN)), offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDSEVEN]);
			
			//100%
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDEIGHT)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDEIGHT]);
			EEPROM_prep(((ARRAYINDEX_FRONT << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDEIGHT)), offset[ARRAYINDEX_FRONT][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDEIGHT]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_LEFT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDEIGHT)), offset[ARRAYINDEX_BACK][ARRAYINDEX_LEFT][ARRAYINDEX_SPEEDEIGHT]);
			EEPROM_prep(((ARRAYINDEX_BACK << FB_SHIFTAMOUNT) | (ARRAYINDEX_RIGHT << LR_SHIFTAMOUNT) | (ARRAYINDEX_SPEEDEIGHT)), offset[ARRAYINDEX_BACK][ARRAYINDEX_RIGHT][ARRAYINDEX_SPEEDEIGHT]);
			
			//done, don't go anymore
			go = 0;
			return;
	}

	}
#ifdef CAL_DEBUG
	if(!(debug_loopcount % 32))
	{
		printf("motor_left = %d motor_right = %d\r",motor_left,motor_right);
	}
#endif
#ifdef CAL_DEBUG
	if(!(debug_loopcount % 32))
	{
		printf("clicks_fl = %ld clicks_fr = %ld clicks_bl = %ld clicks_br = %ld\r",FL_CLICK_COUNT, FR_CLICK_COUNT, BL_CLICK_COUNT, BR_CLICK_COUNT);
	}
#endif
}

