/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "calibrate.h"
#include "encoder.h"
#include "autonomous_166.h"
#include "manipulator.h"



/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
unsigned char autonomous_goal = GOAL_CENTER;  	// default
unsigned char auto_manip_position = 4; 			// middle position

/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD /* You may want to save additional symbols. */

void InterruptHandlerLow ()     
{
#if 0 // psh: disable this for support of encoder interrupts
  unsigned char int_byte;       
  if (INTCON3bits.INT2IF && INTCON3bits.INT2IE)       /* The INT2 pin is RB2/DIG I/O 1. */
  { 
    INTCON3bits.INT2IF = 0;
  }
  else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE)  /* The INT3 pin is RB3/DIG I/O 2. */
  {
    INTCON3bits.INT3IF = 0;
  }
  else if (INTCONbits.RBIF && INTCONbits.RBIE)  /* DIG I/O 3-6 (RB4, RB5, RB6, or RB7) changed. */
  {
    int_byte = PORTB;          /* You must read or write to PORTB */
    INTCONbits.RBIF = 0;     /*     and clear the interrupt flag         */
  }                                        /*     to clear the interrupt condition.  */
  else
  { 
    CheckUartInts();    /* For Dynamic Debug Tool or buffered printf features. */
  }
#else
	unsigned char Port_B;
	unsigned char Port_B_Delta;

	if (PIR1bits.RC1IF && PIE1bits.RC1IE) // rx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_RX
		Rx_1_Int_Handler(); // call the rx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_RX
		Rx_2_Int_Handler(); // call the rx2 interrupt handler (in serial_ports.c)
		#endif
	} 
	else if (PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_TX
		Tx_1_Int_Handler(); // call the tx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_TX
		Tx_2_Int_Handler(); // call the tx2 interrupt handler (in serial_ports.c)
		#endif
	}
	else if (INTCON3bits.INT2IF && INTCON3bits.INT2IE) // encoder 1 interrupt?
	{ 
		INTCON3bits.INT2IF = 0; // clear the interrupt flag
		#ifdef ENABLE_ENCODER_1
		Encoder_1_Int_Handler(); // call the left encoder interrupt handler (in encoder.c)
		#endif
	}
	else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE) // encoder 2 interrupt?
	{
		INTCON3bits.INT3IF = 0; // clear the interrupt flag
		#ifdef ENABLE_ENCODER_2
		Encoder_2_Int_Handler(); // call right encoder interrupt handler (in encoder.c)
		#endif
	}
	else if (INTCONbits.RBIF && INTCONbits.RBIE) // encoder 3-6 interrupt?
	{
		Port_B = PORTB; // remove the "mismatch condition" by reading port b            
		INTCONbits.RBIF = 0; // clear the interrupt flag
		Port_B_Delta = Port_B ^ Old_Port_B; // determine which bits have changed
		Old_Port_B = Port_B; // save a copy of port b for next time around
	 
		if(Port_B_Delta & 0x10) // did external interrupt 3 change state?
		{
			#ifdef ENABLE_ENCODER_3
			Encoder_3_Int_Handler(Port_B & 0x10 ? 1 : 0); // call the encoder 3 interrupt handler (in encoder.c)
			#endif
		}
		if(Port_B_Delta & 0x20) // did external interrupt 4 change state?
		{
			#ifdef ENABLE_ENCODER_4
			Encoder_4_Int_Handler(Port_B & 0x20 ? 1 : 0); // call the encoder 4 interrupt handler (in encoder.c)
			#endif
		}
		if(Port_B_Delta & 0x40) // did external interrupt 5 change state?
		{
			#ifdef ENABLE_ENCODER_5
			Encoder_5_Int_Handler(Port_B & 0x40 ? 1 : 0); // call the encoder 5 interrupt handler (in encoder.c)
			#endif
		}
		if(Port_B_Delta & 0x80) // did external interrupt 6 change state?
		{
			#ifdef ENABLE_ENCODER_6
			Encoder_6_Int_Handler(Port_B & 0x80 ? 1 : 0); // call the encoder 6 interrupt handler (in encoder.c)
			#endif
		}
	}
#endif // psh: end new code
}


/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Autonomous_Code(void)
{
  /* Initialize all PWMs and Relays when entering Autonomous mode, or else it
     will be stuck with the last values mapped from the joysticks.  Remember, 
     even when Disabled it is reading inputs from the Operator Interface. 
  */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
  relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
  relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
  relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
  relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;

  while (autonomous_mode)   /* DO NOT CHANGE! */
  {
	unsigned char contcal = 1;
	unsigned char count_print = 5;
	static unsigned int autonomous_switch = AUTO_DO_NOTHING;
	int calreturn;
	unsigned char print_counter = 10;
	static unsigned char switch_set=FALSE;
	

	print_counter++;
    if (print_counter >= 20)
	{
		print_counter = 0;
		printf("   statusflag.NEW_SPI_DATA = %d\r", statusflag.NEW_SPI_DATA);
	}      

    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {
        Getdata(&rxdata);   /* DO NOT DELETE, or you will be stuck here forever! */

        /* Add your own autonomous code here. */

        Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */

		encoder_reset_battery();
#if 1
		
		// test code
		
		if(switch_set==FALSE)
		{
			autonomous_switch=auto_switch_find();
			switch_set=TRUE;		
			printf("U-R-F autonomous switch: %d/r", autonomous_switch);
		}		
		
		
		switch(autonomous_switch)
		{	
		
			case AUTO_DO_NOTHING: //switch position 1
    			break;

			case AUTO_ARM_LOW:  //switch position 2
//				printf("U-R-F switchstatement: AUTO_ARM_LOW/r");
				auto_manip_position = LOW;
				autonomous_goal = GOAL_CENTER;
				// go to floor position
				do_auto_arm(auto_manip_position);
				break;
	
			case AUTO_FWD_FLOOR:  //switch position 3
//				printf("U-R-F switchstatement: AUTO_FWD_LOW/r");
				go_and_move_arm(0);
				break;

			case AUTO_SCORE_MID:   //switch position 4				
			//	printf("U-R-F switchstatement: AUTO_SCORE/r");
				auto_manip_position = 4;
				autonomous_goal = GOAL_CENTER;
				do_auto();
				break;
			
			case AUTO_REV_MOVE_ARM:   //switch position 5
//				printf("U-R-F switchstatement: AUTO_REVERSE_LOW/r");
				go_and_move_arm(1);

			case AUTO_ARM_FLOOR:  //switch position 6
//				printf("U-R-F switchstatement: AUTO_ARM_FLOOR/r");
				auto_manip_position = FLOOR;
				autonomous_goal = GOAL_CENTER;
				// go to LOW first to avoid collision with chassis
				do_auto_arm(LOW);
				// go to floor position
				do_auto_arm(auto_manip_position);

			default:
				printf("autonomous mode not supported at present/r");
				break;
		}
#endif
    }
  }
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void)
{
  /* Add code here that you want to be executed every program loop. */

}

/*******************************************************************************
* FUNCTION NAME: Serial_Char_Callback
* PURPOSE:       Interrupt handler for the TTL_PORT.
* CALLED FROM:   user_SerialDrv.c
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     data        unsigned char    I    Data received from the TTL_PORT
* RETURNS:       void
*******************************************************************************/

void Serial_Char_Callback(unsigned char data)
{
  /* Add code to handle incomming data (remember, interrupts are still active) */
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
