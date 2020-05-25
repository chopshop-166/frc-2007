/*******************************************************************************
* FILE NAME: user_routines.h
*
* DESCRIPTION: 
*  This is the include file which corresponds to user_routines.c and
*  user_routines_fast.c
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __user_program_h_
#define __user_program_h_


/*******************************************************************************
                            MACRO DECLARATIONS
*******************************************************************************/
/* Add your macros (aliases and constants) here.                              */
/* Do not edit the ones in ifi_aliases.h                                      */
/* Macros are substituted in at compile time and make your code more readable */
/* as well as making it easy to change a constant value in one place, rather  */
/* than at every place it is used in your code.                               */
/*
 EXAMPLE CONSTANTS:
#define MAXIMUM_LOOPS   5
#define THE_ANSWER      42
#define TRUE            1
#define FALSE           0
#define PI_VAL          3.1415

 EXAMPLE ALIASES:
#define LIMIT_SWITCH_1  rc_dig_int1  (Points to another macro in ifi_aliases.h)
#define MAIN_SOLENOID   solenoid1    (Points to another macro in ifi_aliases.h)
*/

/* Used in limit switch routines in user_routines.c */
#define OPEN        1     /* Limit switch is open (input is floating high). */
#define CLOSED      0     /* Limit switch is closed (input connected to ground). */


//#defines for drive
#define JOY_X	p1_x
#define JOY_Y	p1_y
#define JOY_Z 	p1_wheel	//p1_aux for xbox360, p2_x for 2 joysticks, p1_wheel for 3-axis joystick


/*******************************************************************************
                            TYPEDEF DECLARATIONS
*******************************************************************************/
/* EXAMPLE DATA STRUCTURE */
/*
typedef struct
{
  unsigned int  NEW_CAPTURE_DATA:1;
  unsigned int  LAST_IN1:1;
  unsigned int  LAST_IN2:1;
  unsigned int  WHEEL_COUNTER_UP:1;
  unsigned int  :4;
  unsigned int wheel_left_counter;
  unsigned int wheel_right_counter;
} user_struct;
*/


/*******************************************************************************
                           FUNCTION PROTOTYPES
*******************************************************************************/

/* These routines reside in user_routines.c */
void User_Initialization(void);
void Process_Data_From_Master_uP(void);
void Default_Routine(void);
//int linear_regression_creator(int x1, int y1, int x2, int y2, int input);
unsigned char make_a_5_position_switch(unsigned char input, unsigned char lower_deadzone, unsigned char upper_deadzone, unsigned char lower_midpoint, unsigned char upper_midpoint, unsigned char invert);


/* These routines reside in user_routines_fast.c */
void InterruptHandlerLow (void);  /* DO NOT CHANGE! */
void User_Autonomous_Code(void);  /* Only in full-siz FRC system. */
void Process_Data_From_Local_IO(void);


#endif
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/*custom declarations*/
