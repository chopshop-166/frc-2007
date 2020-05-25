#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "camera.h"
#include "encoder.h"
#include "autonomous_166.h"
#include "camera_166.h"

// how often to print out mid_x values
unsigned char LOOP_COUNTER_MAX = 100;

/***********************************************************************
Function name:  camera_debug
Created by: 	Sid
Date modified:  1-28-07
Called From:	User_Routines.Default_Routine
Parameters:     
Returns: 		void
Purpose: 		Autonomous mode processing
Notes: 
**********************************************************************/

void camera_debug()
{
	printf("The Middle of the blob is %d,%d\r", T_Packet_Data.mx, T_Packet_Data.my);
	printf("The left corner of the rectangle is %d, %d\r", T_Packet_Data.x1, T_Packet_Data.y1);
	printf("The right corner of the rectanlge is %d, %d\r", T_Packet_Data.x2, T_Packet_Data.y2);
	printf("Number of pixels is %d\r", T_Packet_Data.pixels);
	printf("Confidence is %d\r\r", T_Packet_Data.confidence);
}


/***********************************************************************
Function name:  target_at_center
Created by:		Sid
Date modified:  1-28-07
Called From:	autonomous_166.c
Parameters:     nsigned int (LEFT, CENTER, RIGHT) see autonomous_166.h
Returns: 		unsigned int (TRUE or FALSE)
Purpose: 		Autonomous mode processing to determine if we can move 
				directly toward the target or need to turn bot
Notes: 			May need to tweak tolerances
**********************************************************************/
unsigned char target_at_center(unsigned char mid_x)
{
	unsigned char target;
	unsigned char loop_counter=0;
	loop_counter++;
	if(loop_counter==LOOP_COUNTER_MAX)
	{
		printf("The mid_x value in target_at_center = %d",mid_x);
		loop_counter = 0;
    }
	if((mid_x>=(MIDPOINT-TOLERANCE)) && (mid_x<=(TOLERANCE+MIDPOINT)))
	{
		target='t';
    }
	else if(mid_x<(MIDPOINT-TOLERANCE))
	{
		target='l';
	}
	else //  if(mid_x>(MIDPOINT+TOLERANCE))
	{
		target='r';
	}
	
	
	return target;
}




/***********************************************************************
Function name:  single_light
Created by: 	Sid
Date modified:  1-28-07
Called From:	autonomous_166.c
Parameters:     unsigned int (CLOSENESS) see autonomous_166.h
Returns: 		unsigned int (TRUE or FALSE)
Purpose: 		Autonomous mode processing to determine if the
				camera is targeted on a single light
Notes: 			May need to tweak tolerances
**********************************************************************/
unsigned char single_light(unsigned char closeness, unsigned char confidence)
{
	unsigned char lights;	
	if(T_Packet_Data.pixels<6)
	{
		lights=0;
	}


else	if(determine_closeness(T_Packet_Data.my)<=FAR_STATE_3)
	{
		if(confidence>=NUM_OF_TARGET_TOLERANCE)
		{
			lights=1;
		}
		else
		{
			lights=2;
		}
	}
	else if(determine_closeness(T_Packet_Data.my)<=MIDRANGE_STATE_3 && determine_closeness(T_Packet_Data.my)>FAR_STATE_3 )
	{
		if(confidence>=NUM_OF_TARGET_TOLERANCE)
		{
			lights=1;
		}
		else
		{
			lights=2;
		}
	}
	else if(determine_closeness(T_Packet_Data.my)<=THERE && determine_closeness(T_Packet_Data.my)>MIDRANGE_STATE_3 )
	{
		if(confidence>=NUM_OF_TARGET_TOLERANCE)
		{
			lights=1;
		}
		else
		{
			lights=2;
		}
	}
	// update this code to return estimated number of targets in view
	// based on distance from rack
	// if vertical angle = ? then size of rectangle (pixel size)
	// should be < ?
	return lights;
}

/***********************************************************************
Function name:  determine_closeness
Created by:		Sid
Date modified:  1-28-07
Called From:	autonomous_166.c
Parameters:     none
Returns: 		unsigned int (closeness) see autonomous.h
Purpose: 		Autonomous mode processing to determine how
				far from target so auto knows how to steer
Notes: 			
**********************************************************************/
unsigned char determine_closeness(unsigned char mid_of_blob)
{
	unsigned char CLOSENESS;		
	if(mid_of_blob >= CLOSENESS_FAR_1)
	{
		CLOSENESS=0;
    }
   else if(mid_of_blob >= CLOSENESS_FAR_2 && mid_of_blob < CLOSENESS_FAR_1 )
	{
		CLOSENESS=1;
    }
    else if(mid_of_blob >= CLOSENESS_FAR_3 && mid_of_blob < CLOSENESS_FAR_2 )
	{
		CLOSENESS=2;
    }
    else if(mid_of_blob >= CLOSENESS_MID_1 && mid_of_blob < CLOSENESS_FAR_3 )
	{
		CLOSENESS=3;
    }
	else if(mid_of_blob >= CLOSENESS_MID_2 && mid_of_blob < CLOSENESS_MID_1 )
	{
		CLOSENESS=4;
    }
	else if(mid_of_blob >= CLOSENESS_MID_3 && mid_of_blob < CLOSENESS_MID_2 )
	{
		CLOSENESS=5;
    }
	else if(mid_of_blob >= CLOSENESS_NEAR_1 && mid_of_blob < CLOSENESS_MID_3 )
	{
		CLOSENESS=6;
    }
	else if(mid_of_blob >= CLOSENESS_NEAR_2 && mid_of_blob < CLOSENESS_NEAR_1 )
	{
		CLOSENESS=7;
    }
	else if(mid_of_blob >= CLOSENESS_NEAR_3 && mid_of_blob < CLOSENESS_NEAR_2 )
	{
		CLOSENESS=8;
    }
	else if(mid_of_blob >= AT_TARGET && mid_of_blob < CLOSENESS_NEAR_3 )
	{
		CLOSENESS=9;
    }
	
	
	
	return CLOSENESS;
}


/***********************************************************************
Function name:  adjust_window
Created by:		Sid
Date modified:  2-03-07
Called From:	autonomous_166.c
Parameters:     unsigned char 0 or 1 ; 0=normal window; 1=adjust to have
				 only one target in the window
Returns: 		void
Purpose: 		To target only one target
Notes: 			Virtual_window() is in camera.c/h 
**********************************************************************/

unsigned char adjust_window(unsigned char adjust_mode)
{
	unsigned char left_x_value, right_x_value, adjust_x_value;
	if(adjust_mode==0)
	{
		Virtual_Window(1,1,159,239);
	}
	else
	{
		left_x_value=T_Packet_Data.x1;
		right_x_value=T_Packet_Data.x2;
		adjust_x_value=((left_x_value+right_x_value)/2);
		Virtual_Window(1,1,adjust_x_value,239);
	}
	return adjust_x_value;	
}

