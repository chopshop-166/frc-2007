#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "camera.h"
#include "encoder.h"
#include "user_byte_166.h"

/***********************************************************************
Function name:  Store_User_Bytes
Created by: 	Beth - Mentor
Date modified:  1-28-07
Called From:	User_Routines.Default_Routine
Parameters:     
Returns: 		void
Purpose: 		To fill userbytes with a series of user data for display on dashboard
Notes: 
**********************************************************************/

/******************** USER BYTE ASSIGNING  ***********
These are bytes that are sent back to the operator controller. By using a drain program,
you are able to pull these numbers out of the radio packets and use excel to graph them. 
It also gives you the ability to look at data recorded during a match at a later date so 
that having the robot can be unnecessary. The below code will assign the values to the
user bytes. 
************************************************************/
static unsigned char ub_counter_slow = 1; 	// use for overloading user bytes
static unsigned char ub_counter_fast = 0;

// treated as a constant; not a #define because only used
// in this file and #define makes it an integer
static unsigned char FAST_COUNTER_MAX =	10;		//# repeats  

void Store_User_Bytes(void)
{ 	
	
  // this will make data change every 10 times
  if (ub_counter_fast >= FAST_COUNTER_MAX)
  {
		ub_counter_slow++;
		if (ub_counter_slow > NUMBER_TYPES)
		{
			ub_counter_slow = 1;
		}
		ub_counter_fast = 0;

		// Temporarily set type of data to "invalid" because data type is changing
		User_Byte1 = 0; 		
  }
  // the slow counter starts at 1 
  switch(ub_counter_slow)
  {	 

		case CAMERA_MASS:	// camera T Packet  mx, my, pixels, confidence
	
		  // Load user bytes with camera data
          // The T_Packet_Data structure is continually updated by Camera_Handler() 
          // which is called from Process_Data_From_Master_uP in user_routines.c
   		  User_Byte2 = T_Packet_Data.mx;			// Middle of mass x value
     	  User_Byte3 = T_Packet_Data.my;			// Middle of mass y value
          User_Byte4 = T_Packet_Data.pixels;		// # pixes in tracked region, scaled & capped at 255
          User_Byte5 = T_Packet_Data.confidence;	// (#pixels/area)*256 of bounded rectangle (capped at 255)
          User_Byte6 = 0xFF;						// not used

    	  // set ub1 last so mixed data won't be picked up
		  User_Byte1 = ub_counter_slow;					// tell user what type of data

		  ub_counter_fast++;
		  break;
	
		case CAMERA_RECTANGLE: // camera T Packet  x1, y1, x2, y2		  

		  // Load user bytes with camera data
   		  User_Byte2 = T_Packet_Data.x1;			// left corner x value
     	  User_Byte3 = T_Packet_Data.y1;			// left corner y value
          User_Byte4 = T_Packet_Data.x2;			// right corner x value
          User_Byte5 = T_Packet_Data.y2;			// right corner y value
          User_Byte6 = 0xFF;						// not used

    	  // set ub1 last so mixed data won't be picked up
		  User_Byte1 = ub_counter_slow;				// tell user what type of data
		  ub_counter_fast++;
		  break;
	
		case ENCODER_3: 
	  
		  // Load user bytes with encoder data
          Encoder_3_Count = Get_Encoder_3_Count();
		  User_Byte2 = (Encoder_3_Count & 0xFF000000) >> 24;	// encoder highest byte
     	  User_Byte3 = (Encoder_3_Count & 0x00FF0000) >> 16;	// encoder next hi byte
          User_Byte4 = (Encoder_3_Count & 0x0000FF00) >> 8;		// encoder next to lo byte
          User_Byte5 = (Encoder_3_Count & 0x000000FF);			// encoder lowest byte
          User_Byte6 = Encoder_3_State;				// direction

    	  // set ub1 last so mixed data won't be picked up
		  User_Byte1 = ub_counter_slow;				// tell user what type of data
		  ub_counter_fast++;
		  break;
	
		case ENCODER_4: 

		  // Load user bytes with encoder data
          Encoder_4_Count = Get_Encoder_4_Count();
		  User_Byte2 = (Encoder_4_Count & 0xFF000000) >> 24;	// encoder highest byte
     	  User_Byte3 = (Encoder_4_Count & 0x00FF0000) >> 16;	// encoder next hi byte
          User_Byte4 = (Encoder_4_Count & 0x0000FF00) >> 8;		// encoder next to lo byte
          User_Byte5 = (Encoder_4_Count & 0x000000FF);			// encoder lowest byte
          User_Byte6 = Encoder_4_State;				// direction						
	  
    	  // set ub1 last so mixed data won't be picked up
		  User_Byte1 = ub_counter_slow;					// tell user what type of data
		  ub_counter_fast++;							// start over again
		  break;

		case ENCODER_5: 	  

		  // Load user bytes with encoder data
          Encoder_5_Count = Get_Encoder_5_Count();
		  User_Byte2 = (Encoder_5_Count & 0xFF000000) >> 24;	// encoder highest byte
     	  User_Byte3 = (Encoder_5_Count & 0x00FF0000) >> 16;	// encoder next hi byte
          User_Byte4 = (Encoder_5_Count & 0x0000FF00) >> 8;		// encoder next to lo byte
          User_Byte5 = (Encoder_5_Count & 0x000000FF);			// encoder lowest byte
          User_Byte6 = Encoder_5_State;				// direction

    	  // set ub1 last so mixed data won't be picked up
		  User_Byte1 = ub_counter_slow;				// tell user what type of data
		  ub_counter_fast++;
		  break;
	
		case ENCODER_6: 	

		  // Load user bytes with encoder data
          Encoder_6_Count = Get_Encoder_6_Count();
		  User_Byte2 = (Encoder_6_Count & 0xFF000000) >> 24;	// encoder highest byte
     	  User_Byte3 = (Encoder_6_Count & 0x00FF0000) >> 16;	// encoder next hi byte
          User_Byte4 = (Encoder_6_Count & 0x0000FF00) >> 8;		// encoder next to lo byte
          User_Byte5 = (Encoder_6_Count & 0x000000FF);			// encoder lowest byte
          User_Byte6 = Encoder_6_State;				// direction

    	  // set ub1 last so mixed data won't be picked up
		  User_Byte1 = ub_counter_slow;					// tell user what type of data
		  ub_counter_fast++;							// start over again
		  break;
	}
}
