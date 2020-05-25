#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "camera.h"
#include "encoder.h"

/***********************************************************************
Header:			Camera_166.h 
Date modified:  1-28-07
**********************************************************************/

// Teh Defines

//These values are found by using the camera's window coordonate plane // found in determine_closeness()
#define CLOSENESS_NEAR_3 27
#define CLOSENESS_NEAR_2 53
#define CLOSENESS_NEAR_1 80
#define CLOSENESS_MID_3 107
#define CLOSENESS_MID_2 133
#define CLOSENESS_MID_1 160
#define CLOSENESS_FAR_3 180
#define CLOSENESS_FAR_2 200
#define CLOSENESS_FAR_1 230
#define AT_TARGET 2
#define TOLERANCE 10
#define MIDPOINT 80
#define NUM_OF_TARGET_TOLERANCE 180




// Default parameters 


// global variables


// function prototypes
void camera_debug(void);
unsigned char single_light(unsigned char, unsigned char);
unsigned char target_at_center(unsigned char);
unsigned char determine_closeness(unsigned char);

//0=normal, 1=make 1 target visible
unsigned char adjust_window(unsigned char); 
