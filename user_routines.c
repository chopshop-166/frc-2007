 /*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
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
#include "autonomous_166.h"
#include "user_byte_166.h"
#include "manipulator.h"
#include "calibrate.h"
#include "crab_drive.h"
#include "adc.h"

#define COPILOT_BOX_1	1
#define COPILOT_BOX_2	0


/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
/* EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
unsigned char wheel_revolutions = 0; (can vary from 0 to 255)
unsigned int  delay_count = 7;       (can vary from 0 to 65,535)
int           angle_deviation = 142; (can vary from -32,768 to 32,767)
unsigned long very_big_counter = 0;  (can vary from 0 to 4,294,967,295)
*/

unsigned char rotary_dial = 1;
char rocker_3_position_switch = 0;
unsigned char mini_joystick_1 = 127;
unsigned char mini_joystick_2 = 127;
unsigned char mini_joystick_3 = 127;

int Base_Position_Box		= 0;
int Base_Position_Floor		= 0;
int Base_Position_Low		= 0;
int Base_Position_Middle	= 0;
int Base_Position_High		= 0;
int Elbow_Position_Box		= 0;
int Elbow_Position_Floor	= 0;
int Elbow_Position_Low		= 0;
int Elbow_Position_Middle	= 0;
int Elbow_Position_High		= 0;
int Wrist_Position_Box		= 0;
int Wrist_Position_Floor	= 0;
int Wrist_Position_Low		= 0;
int Wrist_Position_Middle	= 0;
int Wrist_Position_High		= 0;
int	Gripper_Open			= 0;
int Gripper_Closed			= 0;





/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Max
* PURPOSE:       Sets a PWM value to neutral (127) if it exceeds 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value > 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Min
* PURPOSE:       Sets a PWM value to neutral (127) if it's less than 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value < 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}


/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = OUTPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
  rc_dig_out17 = 0;

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */

  Initialize_Encoders();
 
  // Kevin's code for serial ports to support his 07 camera code
  Init_Serial_Port_One();
  Init_Serial_Port_Two();

#ifdef TERMINAL_SERIAL_PORT_1    
  stdout_serial_port = SERIAL_PORT_ONE;
#endif

#ifdef TERMINAL_SERIAL_PORT_2    
  stdout_serial_port = SERIAL_PORT_TWO;
#endif

	manipulator_initialization();

  Putdata(&txdata);             /* DO NOT CHANGE! */


  printf("IFI 2006 User Processor Initialized ...\r");  /* Optional - Print initialization message. */

  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
  static unsigned char i;
  static unsigned char loops = 0;

  Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */
  EEPROM_write();
  Default_Routine();  /* Optional.  See below. */


  /* Add your own code here. (a printf will not be displayed when connected to 
     the breaker panel unless a Y cable is used) */
#if 0
loops++;
if(loops % 20 == 0)
{
  /* printf EXAMPLE */
  printf("Y %3d, X %3d, Wheel %3d, Aux %3d, Trig %1d, Top %1d, Aux1 %1d, Aux2 %1d\r\r",
		p1_y,p1_x, p1_wheel,p1_aux, p1_sw_trig, p1_sw_top, p1_sw_aux1, p1_sw_aux2);  
//  printf("Port2 Y %3d, X %3d\r",p2_y,p2_x);
}
#endif
  
  // This function is responsible for camera initialization 
  // and camera serial data interpretation. Once the camera
  // is initialized and starts sending tracking data, this 
  // function will continuously update the global T_Packet_Data 
  // structure with the received tracking information.
  Camera_Handler();	
  
  //if there is data in the queue, write to the data EEPROM
  //EEPROM_write();		
  //enable digital input 1 to be an interrupt (from 06 code)
  //remove comments if want to use
  //INTCON3bits.INT2IE = 1;		

  Generate_Pwms(pwm13,pwm14,pwm15,pwm16);


  Putdata(&txdata);             /* DO NOT CHANGE! */
}

/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{
  /*---------- ROBOT FEEDBACK LEDs------------------------------------------------
  *------------------------------------------------------------------------------
  *   This section drives the "ROBOT FEEDBACK" lights on the Operator Interface.
  *   The lights are green for joystick forward and red for joystick reverse.
  *   Both red and green are on when the joystick is centered.  Use the
  *   trim tabs on the joystick to adjust the center.     
  *   These may be changed for any use that the user desires.                       
  */	
	
	static unsigned char click_handler = 0;

	// for calibration
	static unsigned char do_cal = 0;
	static unsigned char loopcount = 0;
	loopcount ++;
	if(!(loopcount % 128))
	{
		printf("Autonomous Switch: %u\r",Get_Analog_Value(rc_ana_in01));
	}
	//dummy variables for limit switch values and copilot box controls
	manipulator_initialization();
	teleoperated_manipulator_master_control();
	manipulator_calibration();

	if(p1_sw_aux2 == 0)
	click_handler = 0;


#if COPILOT_BOX_1

// rotary dial
	
	if(p3_wheel == 127)
		rotary_dial = 0;

	if((p3_wheel >= 6) && (p3_wheel <= 12))
		rotary_dial = 1;

	if((p3_wheel >= 31) && (p3_wheel <= 37))
		rotary_dial = 2;

	if((p3_wheel >= 86) && (p3_wheel <= 92))
		rotary_dial = 3;

	if((p3_wheel >= 141) && (p3_wheel <= 147))
		rotary_dial = 4;

	if((p3_wheel >= 196) && (p3_wheel <= 202))
		rotary_dial = 5;

// rocker switch

	if((p3_sw_trig == 1) && (p3_sw_top == 0))
		rocker_3_position_switch = -1;

	if((p3_sw_trig == 0) && (p3_sw_top == 1))
		rocker_3_position_switch = 1;

	if((p3_sw_trig == 0) && (p3_sw_top == 0))
		rocker_3_position_switch = 0;

// mini joystick 1

mini_joystick_3 = make_a_5_position_switch(p3_y , 110, 150 , 68, 175, 1);

/*
	if(p3_y > 150)
	{
		mini_joystick_1 = linear_regression_creator(150, 127, 200, 254, p3_y);	
	}
	else if(p3_y < 110)
	{
		mini_joystick_1 = linear_regression_creator(110, 127, 25, 0, p3_y);
	}
	else
	{
		mini_joystick_1 = 127;
	}

*/

// mini joystick 2

mini_joystick_1 = make_a_5_position_switch(p3_x , 112, 152 , 69, 182, 0);

/*
	if(p3_x > 154)
	{
		mini_joystick_2 = linear_regression_creator(154, 127, 210, 254, p3_x);	
	}
	else if(p3_x < 114)
	{
		mini_joystick_2 = linear_regression_creator(114, 127, 50, 0, p3_x);
	}
	else
	{
		mini_joystick_2 = 127;
	}
*/

// mini joystick 3

mini_joystick_2 = make_a_5_position_switch(p3_aux , 120, 160 , 95, 185, 0);

/*
	if(p3_aux > 160)
	{
		mini_joystick_3 = linear_regression_creator(160, 127, 215, 254, p3_aux);	
	}
	else if(p3_aux < 120)
	{
		mini_joystick_3 = linear_regression_creator(120, 127, 55, 0, p3_aux);
	}
	else
	{
		mini_joystick_3 = 127;
	}
*/

//  printf("outputs: rotary_dial, rocker_switch, mini joy 1, mini joy 2, mini joy 3 \r%d\r%d\r%d,%d,%d\r\r", rotary_dial,rocker_3_position_switch,mini_joystick_1,mini_joystick_2,mini_joystick_3);
//  printf("outputs: mini joy 1, mini joy 2, mini joy 3 \r%d,%d,%d\r\r", mini_joystick_1,mini_joystick_2,mini_joystick_3);


#endif
	
#if COPILOT_BOX_2

// rotary dial
	
	if(p3_wheel == 127)
		rotary_dial = 0;

	if((p3_wheel >= 31) && (p3_wheel <= 37))
		rotary_dial = 1;

	if((p3_wheel >= 55) && (p3_wheel <= 61))
		rotary_dial = 2;

	if((p3_wheel >= 104) && (p3_wheel <= 110))
		rotary_dial = 3;

	if((p3_wheel >= 152) && (p3_wheel <= 158))
		rotary_dial = 4;

	if((p3_wheel >= 202) && (p3_wheel <= 208))
		rotary_dial = 5;

// rocker switch

	if((p3_sw_trig == 1) && (p3_sw_top == 0))
		rocker_3_position_switch = -1;

	if((p3_sw_trig == 0) && (p3_sw_top == 1))
		rocker_3_position_switch = 1;

	if((p3_sw_trig == 0) && (p3_sw_top == 0))
		rocker_3_position_switch = 0;

// mini joystick 1

mini_joystick_1 = make_a_5_position_switch(p3_x , 69, 109 , 35, 157, 1);

/*
	if(p3_y > 150)
	{
		mini_joystick_1 = linear_regression_creator(150, 127, 200, 254, p3_y);	
	}
	else if(p3_y < 110)
	{
		mini_joystick_1 = linear_regression_creator(110, 127, 25, 0, p3_y);
	}
	else
	{
		mini_joystick_1 = 127;
	}

*/

// mini joystick 2

mini_joystick_2 = make_a_5_position_switch(p3_y , 102, 142 , 69, 191, 1);

/*
	if(p3_x > 154)
	{
		mini_joystick_2 = linear_regression_creator(154, 127, 210, 254, p3_x);	
	}
	else if(p3_x < 114)
	{
		mini_joystick_2 = linear_regression_creator(114, 127, 50, 0, p3_x);
	}
	else
	{
		mini_joystick_2 = 127;
	}
*/

// mini joystick 3

mini_joystick_3 = make_a_5_position_switch(p3_aux , 104, 144 , 60, 170, 1);

/*
	if(p3_aux > 160)
	{
		mini_joystick_3 = linear_regression_creator(160, 127, 215, 254, p3_aux);	
	}
	else if(p3_aux < 120)
	{
		mini_joystick_3 = linear_regression_creator(120, 127, 55, 0, p3_aux);
	}
	else
	{
		mini_joystick_3 = 127;
	}
*/

//  printf("outputs: rotary_dial, rocker_switch, mini joy 1, mini joy 2, mini joy 3 \r%d\r%d\r%d,%d,%d\r\r", rotary_dial,rocker_3_position_switch,mini_joystick_1,mini_joystick_2,mini_joystick_3);
//  printf("outputs: mini joy 1, mini joy 2, mini joy 3 \r%d,%d,%d\r\r", mini_joystick_1,mini_joystick_2,mini_joystick_3);


#endif



	
#if 0
//	printf("joystick 1, joystick 2, joystick 3, %d,%d,%d\r\r",p3_y,p3_x,p3_aux);
//	printf("rotary_dial \r%d\r",p3_wheel);
//	printf("3 Position Switch: Backwards,Forwards \r%d,%d\r\r",p3_sw_top,p3_sw_trig);
#endif


//////////////////////////////////


	//If buttons 3 and 4 are being pressed simultaneously, put in calibration mode
	if((CAL_SWITCHONE && CAL_SWITCHTWO) && !(do_cal))
	{
		do_cal = 1;
#ifdef CAL_DEBUG
		printf("do_cal is now 1\r");
#endif
	}
	
	//we want to use this to display other info
	
	//Calibration
	if(do_cal)
	{
		calibrate_two_at_a_time();
		EEPROM_write();
	}
	//drive
	else
	{
	  	crab_drive(JOY_X, JOY_Y, JOY_Z);
	}

	// pack user bytes for dashboard display
	Store_User_Bytes();
	
	//  camera_debug();

} /* END Default_Routine(); */


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

// ROBERT CAN THIS CODE BE DELETED???
/*
int linear_regression_creator(int x1, int y1, int x2, int y2, int input)

{

long hold_value_1; // variable to hold temporary values
long hold_value_2; // variable to hold temporary values
long m; // m as in y = mx + b
long b; // b as in y = mx + b
int output;

// find slope of linear relationship

	hold_value_1 = y1 - y2; // set the first hold value to the top part of slope equation
	hold_value_2 = x1 - x2; // set the second hold value to the bottom part of slope equation
	hold_value_1 = hold_value_1 * 50; // multiply the top part by 50 to avoid calculations with floats
	m = hold_value_1 / hold_value_2; // division part of slope equation, yielding an m = to 100* actual m

// find y-intercept of linear relationship	

	hold_value_2 = x1; // work around type casting
	hold_value_1 = -1 * hold_value_2; // invert x1 for equation b = -m*x1 + y
	hold_value_1 = m * hold_value_2; // multiply x1 by the slope, value is 100x actual
	hold_value_1 = hold_value_1 / 50; // revert from 50x actual value
	b = hold_value_1 + y1; // complete equation

// run data through equation

	hold_value_2 = input; // work around type casting
	hold_value_1 = hold_value_2 * m; //multiply by slope, value is 100x actual
	hold_value_2 = hold_value_1 / 50; // revert from 50x actual value
	hold_value_1 = hold_value_2 + b;

// check to make sure it is a 0 - 254 value

printf("hold_value_1, %d\r", hold_value_1);

	if(hold_value_1 < 0) // if less than 0
	{
		output = 0; // set to 0
	}
	else if(hold_value_1 > 254) // if greater than 254
	{
		output = 254; // set to 254
	}
	else
	{
		output = hold_value_1;
	}

return output;
} 
*/

unsigned char make_a_5_position_switch(unsigned char input, unsigned char lower_deadzone, unsigned char upper_deadzone, unsigned char lower_midpoint, unsigned char upper_midpoint, unsigned char invert)
{
	if(invert)
	{
		if((input >= lower_deadzone) && (input <= upper_deadzone))
		{
			return 127;
		}
		if((input >= lower_midpoint) && (input < lower_deadzone))
		{
			return 191;
		}	
		if(input < lower_midpoint)
		{
			return 254;
		}	
		if((input <= upper_midpoint) && (input > upper_deadzone))
		{
			return 163;
		}	
		if(input > upper_midpoint)
		{
			return 0;
		}
	}

	else
	{
		if((input >= lower_deadzone) && (input <= upper_deadzone))
		{
			return 127;
		}
		if((input >= lower_midpoint) && (input < lower_deadzone))
		{
			return 63;
		}	
		if(input < lower_midpoint)
		{
			return 0;
		}	
		if((input <= upper_midpoint) && (input > upper_deadzone))
		{
			return 191;
		}	
		if(input > upper_midpoint)
		{
			return 254;
		}
	}
}
