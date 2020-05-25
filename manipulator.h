/*******************************************************************************
* FILE NAME: manipulator.h
*
* DESCRIPTION: 
*  This is the include file which corresponds to manipulator.c 
*  It contains some aliases and function prototypes used in those files.
*
*******************************************************************************/

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

//encoders, currently plugged into digital inputs 3 through 6
#define ENCODER_BASE					Get_Encoder_3_Count()
#define ENCODER_ELBOW					Get_Encoder_4_Count()
#define ENCODER_WRIST					Get_Encoder_5_Count()
#define ENCODER_GRIPPER					Get_Encoder_6_Count()
//motors, currently plugged into pwms 1 through 4
#define MOTOR_BASE						pwm05
#define MOTOR_ELBOW						pwm06
#define MOTOR_WRIST						pwm07
#define MOTOR_GRIPPER					pwm08
//limit switches, currently plugged into digital inputs 7 through 10
#define LIMIT_SWITCH_BASE				rc_dig_in07
#define LIMIT_SWITCH_ELBOW				rc_dig_in08
#define LIMIT_SWITCH_WRIST				rc_dig_in09
#define LIMIT_SWITCH_GRIPPER			rc_dig_in10


#define ENCODER_BASE_DEAD_ZONE			2
#define ENCODER_ELBOW_DEAD_ZONE			5
#define ENCODER_WRIST_DEAD_ZONE			5
#define ENCODER_GRIPPER_DEAD_ZONE		5
#define BASE_DEAD_ZONE	 			7
#define ELBOW_DEAD_ZONE	 			7
#define WRIST_DEAD_ZONE	 			7
#define ROTARY_SWITCH_DEAD_ZONE		3
#define BASE_POSITION_BOX	    	Base_Position_Box
#define BASE_POSITION_FLOOR			Base_Position_Floor
#define BASE_POSITION_LOW			Base_Position_Low
#define BASE_POSITION_MIDDLE		Base_Position_Middle
#define BASE_POSITION_HIGH			Base_Position_High
#define BASE_SPEED					30
#define BASE_MANUAL_REDUCTION		30
#define ELBOW_POSITION_BOX			Elbow_Position_Box
#define ELBOW_POSITION_FLOOR		Elbow_Position_Floor
#define ELBOW_POSITION_LOW			Elbow_Position_Low
#define ELBOW_POSITION_MIDDLE		Elbow_Position_Middle
#define ELBOW_POSITION_HIGH			Elbow_Position_High
#define ELBOW_SPEED					30
#define ELBOW_MANUAL_REDUCTION		30
#define WRIST_POSITION_BOX			Wrist_Position_Box
#define WRIST_POSITION_FLOOR		Wrist_Position_Floor
#define WRIST_POSITION_LOW			Wrist_Position_Low
#define WRIST_POSITION_MIDDLE		Wrist_Position_Middle
#define WRIST_POSITION_HIGH			Wrist_Position_High
#define WRIST_SPEED					40
#define WRIST_MANUAL_REDUCTION		40
#define GRIPPER_SPEED_FORWARD		50
#define GRIPPER_SPEED_BACKWARD		35
#define GRIPPER_OPEN				Gripper_Open
#define GRIPPER_CLOSED				Gripper_Closed
#define BOX							1
#define FLOOR						2
#define LOW							3
#define MIDDLE						4
#define HIGH						5
#define BASE						5
#define ELBOW						6
#define WRIST						7
#define GRIPPER						8
#define BASE_UPPER_LIMIT			-29
#define BASE_LOWER_LIMIT			-29
#define ELBOW_UPPER_LIMIT			-130
#define ELBOW_LOWER_LIMIT			-130
#define WRIST_UPPER_LIMIT			-115
#define WRIST_LOWER_LIMIT			-115
#define GRIPPER_UPPER_LIMIT			-40
#define GRIPPER_LOWER_LIMIT			-40

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

/* These routines reside in manipulator.c */

unsigned char manipulator_automatic_adjustment(long goal_joint_position, unsigned char joint_to_be_moved, unsigned char desired_speed_of_movement, long upper_soft_stop, long lower_soft_stop);
unsigned char manipulator_manual_adjustment(int input_value, unsigned char reduction_factor, unsigned char joint_to_be_moved);
void teleoperated_manipulator_master_control(void);
void autonomous_manipulator_master_control(unsigned char desired_position, unsigned char gripper_open_or_close);
void encoder_reset_battery(void);
int debug(void);
void manipulator_calibration(void);
void manipulator_initialization(void);
unsigned char goto_box(unsigned char joint_to_be_moved);


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
