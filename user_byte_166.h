/***********************************************************************
Header:			User_Byte_166.h 
Created by: 	Beth - Mentor
Date modified:  1-28-07
Function name:  Store_User_Bytes
Used in:		User_Routines.c, user_byte_166.c
**********************************************************************/

// Default parameters 
#define NUMBER_TYPES			6
#define CAMERA_MASS				1
#define CAMERA_RECTANGLE		2
#define ENCODER_3				3
#define ENCODER_4				4
#define ENCODER_5				5
#define ENCODER_6				6

// global variables
extern unsigned int camera_t_packets;
extern T_Packet_Data_Type T_Packet_Data;

extern volatile long Encoder_3_Count;
extern volatile long Encoder_4_Count;
extern volatile long Encoder_5_Count;
extern volatile long Encoder_6_Count;

extern unsigned char Encoder_3_State;
extern unsigned char Encoder_4_State;
extern unsigned char Encoder_5_State;
extern unsigned char Encoder_6_State;

// function prototypes
void Store_User_Bytes(void);

