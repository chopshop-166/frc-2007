;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; FILE NAME: Util_lib.asm
;
; DESCRIPTION: 
;  This file contains important setup info and initialization 
;  for the SPI buffers.  It also contains the Hex_output routine.
;
; USAGE:
;  Users should not modify this file at all.
;  DO NOT MODIFY THIS FILE!
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

#include    p18cxxx.inc

#define   DATA_SIZE          30
#define   SPI_TRAILER_SIZE   2
#define   SPI_XFER_SIZE      DATA_SIZE + SPI_TRAILER_SIZE

 ifdef _SNOOP_ON_COM1
#define	RCSTA		        RCSTA2
#define TXSTA		        TXSTA2
#define TXREG		        TXREG2
#define RCREG		        RCREG2
#define SPBRG		        SPBRG2
 else
 ifdef __18F8520
#define TXREG		        TXREG1
 endif
 endif

;*****************************************************************************
 ifdef __18F8520
	__CONFIG	_CONFIG1H, _OSC_HSPLL_1H  
 	__CONFIG	_CONFIG2L, _BOR_ON_2L & _PWRT_ON_2L
	__CONFIG	_CONFIG2H, _WDT_OFF_2H & _WDTPS_64_2H
        __CONFIG        _CONFIG3H, _CCP2MUX_OFF_3H

	__CONFIG	_CONFIG4L, _STVR_ON_4L & _LVP_OFF_4L & _DEBUG_OFF_4L

	__CONFIG	_CONFIG5L, _CP0_OFF_5L & _CP1_OFF_5L & _CP2_OFF_5L & _CP3_OFF_5L 
	__CONFIG	_CONFIG5H, _CPB_OFF_5H & _CPD_OFF_5H
	__CONFIG	_CONFIG6L, _WRT0_OFF_6L & _WRT1_OFF_6L & _WRT2_OFF_6L & _WRT3_OFF_6L 
	__CONFIG	_CONFIG6H, _WRTC_OFF_6H & _WRTB_OFF_6H & _WRTD_OFF_6H
	__CONFIG	_CONFIG7L, _EBTR0_OFF_7L & _EBTR1_OFF_7L & _EBTR2_OFF_7L & _EBTR3_OFF_7L
	__CONFIG	_CONFIG7H, _EBTRB_OFF_7H
 endif
 ifdef __18F8722
	__CONFIG _CONFIG1H, _OSC_HSPLL_1H & _FCMEN_OFF_1H & _IESO_OFF_1H
	__CONFIG _CONFIG2L, _BOREN_SBORDIS_2L & _BORV_28_2L & _PWRT_ON_2L
	__CONFIG _CONFIG2H, _WDT_OFF_2H & _WDTPS_32768_2H
	__CONFIG _CONFIG3L, _MODE_MC_3L & _WAIT_OFF_3L & _ADDRBW_ADDR20BIT_3L & _DATABW_DATA16BIT_3L
	;__CONFIG _CONFIG3H, _MCLRE_ON_3H & _LPT1OSC_OFF_3H & _ECCPMX_PORTE_3H & _CCP2MX_PORTBE_3H
	__CONFIG _CONFIG4L, _LVP_OFF_4L & _DEBUG_OFF_4L & _BBSIZ_BB2K_4L & _XINST_OFF_4L
	__CONFIG _CONFIG5L, _CP0_OFF_5L & _CP1_OFF_5L & _CP2_OFF_5L & _CP3_OFF_5L & _CP4_OFF_5L & _CP5_OFF_5L & _CP6_OFF_5L & _CP7_OFF_5L
	__CONFIG _CONFIG5H, _CPB_ON_5H & _CPD_OFF_5H
	__CONFIG _CONFIG6L, _WRT0_OFF_6L & _WRT1_OFF_6L & _WRT2_OFF_6L & _WRT3_OFF_6L & _WRT4_OFF_6L & _WRT5_OFF_6L & _WRT6_OFF_6L & _WRT7_OFF_6L
	__CONFIG _CONFIG6H, _WRTB_ON_6H & _WRTC_ON_6H & _WRTD_OFF_6H
	__CONFIG _CONFIG7L, _EBTR0_OFF_7L & _EBTR1_OFF_7L & _EBTR2_OFF_7L & _EBTR3_OFF_7L & _EBTR4_OFF_7L & _EBTR5_OFF_7L & _EBTR6_OFF_7L & _EBTR7_OFF_7L
	__CONFIG _CONFIG7H, _EBTRB_ON_7H
 endif
;*****************************************************************************

#define dPWM01                  PWMdisableMask,0
#define dPWM02                  PWMdisableMask,1
#define dPWM03                  PWMdisableMask,2
#define dPWM04                  PWMdisableMask,3

  ifdef _FRC_NC1
#define	sPWM01	                LATJ,3     ;User Processor driven PWM Signal Pins
#define	sPWM02	                LATJ,2
#define	sPWM03	                LATJ,1
#define	sPWM04	                LATJ,0     ;User Processor driven PWM Signal Pins
  else
#define	sPWM01	                LATE,7     ;PWM Signal Pins
#define	sPWM02	                LATG,0
#define	sPWM03	                LATG,3
#define	sPWM04	                LATG,4     ;PWM Signal Pins
  endif
#define	sPWM05	                PORT_TRASH,0
#define	sPWM06	                PORT_TRASH,1
#define	sPWM07	                PORT_TRASH,2    
#define	sPWM08	                PORT_TRASH,3
#define	sPWM09	                PORT_TRASH,4

#define NEW_SPI_DATA			statusflag,0	;definition in ifi_default.c	
#define TX_UPDATED				statusflag,1
#define FIRST_TIME				statusflag,2
#define TX_BUFFSELECT			statusflag,3
#define RX_BUFFSELECT			statusflag,4

    EXTERN  Wait4TXEmpty,statusflag

    GLOBAL  Clear_Memory,Hex_output,Generate_Pwms,UpdateLocalPWMDirection
    GLOBAL  gSPI_RCV_PTR,gSPI_XMT_PTR,PWMdisableMask
    GLOBAL  gTX_BUFF0,gTX_BUFF1,gRX_BUFF0,gRX_BUFF1
	GLOBAL	GetDataFromMaster,SendDataToMaster

COPY_VAR	        UDATA  0x80
gTX_BUFF0           RES     .32         ;spi transmit buffer0
gTX_BUFF1           RES     .32         ;spi transmit buffer1
gRX_BUFF0           RES     .32         ;spi receive buffer0
gRX_BUFF1           RES     .32         ;spi receive buffer1

SPI_VAR	            UDATA_ACS
gTX_PTRH	        RES		1
gTX_PTRL	        RES		1
gRX_PTRH	        RES		1
gRX_PTRL	        RES		1
FSRH_temp           RES     1
FSRL_temp           RES     1

gSPICNT	            RES     1
gSPI_RCV_PTR        RES     1           ;pointers for double buffering scheme
gSPI_XMT_PTR        RES     1
tmp					RES		1			;used in the Hex conversion routine

gPWM_DATA1          RES     1
gPWM_DATA2          RES     1
gPWM_DATA3          RES     1
gPWM_DATA4          RES     1
gPWM_DATA5          RES     1
gPWM_DATA6          RES     1
gPWM_DATA7          RES     1
gPWM_DATA8          RES     1
PORT_TRASH          RES     1
Cnt                 RES     1
Cnt1                RES     1
txPWM_MASK          RES     1
PWMdisableMask      RES     1

UTIL_LIB	CODE


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; FUNCTION NAME: Clear_Memory
; PURPOSE:       Clear SPI transmit and receive buffers.
; CALLED FROM:   ifi_startup.c
; ARGUMENTS:     none
; RETURNS:       none
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Clear_Memory
    movlw   0x00
    movwf   FSR0L
    movwf   FSR0H
Clear_Loop
    clrwdt                  
    call    Clear_Bank
    incf    FSR0H,f
    movlw   .8
    subwf   FSR0H,w
    btfss   STATUS,Z
    goto    Clear_Loop
    return


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; FUNCTION NAME: Clear_Bank
; PURPOSE:       Loop for routine above.
; CALLED FROM:   this file, Clear_Memory routine
; ARGUMENTS:     none
; RETURNS:       none
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Clear_Bank
    clrf    INDF0
    incfsz  FSR0L,f
    goto    Clear_Bank
    return


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; FUNCTION NAME: Hex_output
; PURPOSE:       Convert a byte to the the hex value, 
;                represent it in ASCII, and stuff it over the USART
;                to view in a serial terminal on a PC.
; CALLED FROM:   anywhere
; ARGUMENTS:     
;     Argument       Type             IO   Description
;     --------       -------------    --   -----------
;                    unsigned char    I    byte to be transmitted
; RETURNS:       none
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Hex_output

    movlw	0xff		;two's complement representation of -1
    movff	PLUSW1,tmp	;indirectly address 1 below where FSR1 points

    swapf   tmp,w       ;get high byte
    andlw   0x0f
    addlw   0x06
    btfsc   STATUS,DC
    addlw   0x07
    addlw   0x30-0x06

    movwf	TXREG		;transmit over USART
    call	Wait4TXEmpty
	
    movf    tmp,w       ;get low byte
    andlw   0x0f
    addlw   0x06
    btfsc   STATUS,DC
    addlw   0x07
    addlw   0x30-0x06

    movwf	TXREG		;transmit over USART
    call	Wait4TXEmpty

    retlw   0x00

;*****************************************************************************
UpdateLocalPWMDirection ;(txdata.pwm_mask)
    return

;*****************************************************************************
SendDataToMaster
    movff   FSR0H,FSRH_temp   
    movff   FSR0L,FSRL_temp
	MOVLW	0xff		    ;two's complement representation of -1
	MOVFF	PLUSW1,FSR0H	;get upper address from stack  
	MOVLW	0xfe		    ;two's complement representation of -2
	MOVFF	PLUSW1,FSR0L	;get lower address from stack  

	btfss	TX_BUFFSELECT
	bra		MoveData4Buff0
    movff   POSTINC0,gTX_BUFF1
    movff   POSTINC0,gTX_BUFF1+.1
    movff   POSTINC0,gTX_BUFF1+.2
    movff   POSTINC0,gTX_BUFF1+.3
    movff   POSTINC0,gTX_BUFF1+.4
    movff   POSTINC0,gTX_BUFF1+.5
    movff   POSTINC0,gTX_BUFF1+.6
    movff   POSTINC0,gTX_BUFF1+.7
    movff   POSTINC0,gTX_BUFF1+.8
    movff   POSTINC0,gTX_BUFF1+.9
    movff   POSTINC0,gTX_BUFF1+.10
    movff   POSTINC0,gTX_BUFF1+.11
    movff   POSTINC0,gTX_BUFF1+.12
    movff   POSTINC0,gTX_BUFF1+.13
    movff   POSTINC0,gTX_BUFF1+.14
    movff   POSTINC0,gTX_BUFF1+.15
    movff   POSTINC0,gTX_BUFF1+.16
    movff   POSTINC0,gTX_BUFF1+.17
    movff   POSTINC0,gTX_BUFF1+.18
    movff   POSTINC0,gTX_BUFF1+.19
    movff   POSTINC0,gTX_BUFF1+.20
    movff   POSTINC0,gTX_BUFF1+.21
    movff   POSTINC0,gTX_BUFF1+.22
    movff   POSTINC0,gTX_BUFF1+.23
    movff   POSTINC0,gTX_BUFF1+.24
    movff   POSTINC0,gTX_BUFF1+.25
    movff   POSTINC0,gTX_BUFF1+.26
    movff   POSTINC0,gTX_BUFF1+.27
    movff   POSTINC0,gTX_BUFF1+.28
    movff   POSTINC0,gTX_BUFF1+.29
    movff   POSTINC0,gTX_BUFF1+.30
    movff   POSTINC0,gTX_BUFF1+.31
	bra		TerminateSend

MoveData4Buff0
    movff   POSTINC0,gTX_BUFF0
    movff   POSTINC0,gTX_BUFF0+.1
    movff   POSTINC0,gTX_BUFF0+.2
    movff   POSTINC0,gTX_BUFF0+.3
    movff   POSTINC0,gTX_BUFF0+.4
    movff   POSTINC0,gTX_BUFF0+.5
    movff   POSTINC0,gTX_BUFF0+.6
    movff   POSTINC0,gTX_BUFF0+.7
    movff   POSTINC0,gTX_BUFF0+.8
    movff   POSTINC0,gTX_BUFF0+.9
    movff   POSTINC0,gTX_BUFF0+.10
    movff   POSTINC0,gTX_BUFF0+.11
    movff   POSTINC0,gTX_BUFF0+.12
    movff   POSTINC0,gTX_BUFF0+.13
    movff   POSTINC0,gTX_BUFF0+.14
    movff   POSTINC0,gTX_BUFF0+.15
    movff   POSTINC0,gTX_BUFF0+.16
    movff   POSTINC0,gTX_BUFF0+.17
    movff   POSTINC0,gTX_BUFF0+.18
    movff   POSTINC0,gTX_BUFF0+.19
    movff   POSTINC0,gTX_BUFF0+.20
    movff   POSTINC0,gTX_BUFF0+.21
    movff   POSTINC0,gTX_BUFF0+.22
    movff   POSTINC0,gTX_BUFF0+.23
    movff   POSTINC0,gTX_BUFF0+.24
    movff   POSTINC0,gTX_BUFF0+.25
    movff   POSTINC0,gTX_BUFF0+.26
    movff   POSTINC0,gTX_BUFF0+.27
    movff   POSTINC0,gTX_BUFF0+.28
    movff   POSTINC0,gTX_BUFF0+.29
    movff   POSTINC0,gTX_BUFF0+.30
    movff   POSTINC0,gTX_BUFF0+.31

TerminateSend
    movff   FSRH_temp,FSR0H         ;Restore FSR0
    movff   FSRL_temp,FSR0L
	return

;*****************************************************************************
GetDataFromMaster				;GetDataFromMaster(unsigned char *ptr)
	btfss	NEW_SPI_DATA	;if (statusflag.NEW_SPI_DATA == 0)  return
	return
    movff   FSR0H,FSRH_temp   
    movff   FSR0L,FSRL_temp
	MOVLW	0xff		    ;two's complement representation of -1
	MOVFF	PLUSW1,FSR0H	;get upper address from stack  
	MOVLW	0xfe		    ;two's complement representation of -2
	MOVFF	PLUSW1,FSR0L	;get lower address from stack  

	btfss	RX_BUFFSELECT
	bra		MoveDataFromBuff1
    movff   gRX_BUFF0,POSTINC0
    movff   gRX_BUFF0+.1,POSTINC0
    movff   gRX_BUFF0+.2,POSTINC0
    movff   gRX_BUFF0+.3,POSTINC0
    movff   gRX_BUFF0+.4,POSTINC0
    movff   gRX_BUFF0+.5,POSTINC0
    movff   gRX_BUFF0+.6,POSTINC0
    movff   gRX_BUFF0+.7,POSTINC0
    movff   gRX_BUFF0+.8,POSTINC0
    movff   gRX_BUFF0+.9,POSTINC0
    movff   gRX_BUFF0+.10,POSTINC0
    movff   gRX_BUFF0+.11,POSTINC0
    movff   gRX_BUFF0+.12,POSTINC0
    movff   gRX_BUFF0+.13,POSTINC0
    movff   gRX_BUFF0+.14,POSTINC0
    movff   gRX_BUFF0+.15,POSTINC0
    movff   gRX_BUFF0+.16,POSTINC0
    movff   gRX_BUFF0+.17,POSTINC0
    movff   gRX_BUFF0+.18,POSTINC0
    movff   gRX_BUFF0+.19,POSTINC0
    movff   gRX_BUFF0+.20,POSTINC0
    movff   gRX_BUFF0+.21,POSTINC0
    movff   gRX_BUFF0+.22,POSTINC0
    movff   gRX_BUFF0+.23,POSTINC0
    movff   gRX_BUFF0+.24,POSTINC0
    movff   gRX_BUFF0+.25,POSTINC0
    movff   gRX_BUFF0+.26,POSTINC0
    movff   gRX_BUFF0+.27,POSTINC0
    movff   gRX_BUFF0+.28,POSTINC0
    movff   gRX_BUFF0+.29,POSTINC0
    movff   gRX_BUFF0+.30,POSTINC0
    movff   gRX_BUFF0+.31,POSTINC0
	bra		TerminateGet

MoveDataFromBuff1
    movff   gRX_BUFF1,POSTINC0
    movff   gRX_BUFF1+.1,POSTINC0
    movff   gRX_BUFF1+.2,POSTINC0
    movff   gRX_BUFF1+.3,POSTINC0
    movff   gRX_BUFF1+.4,POSTINC0
    movff   gRX_BUFF1+.5,POSTINC0
    movff   gRX_BUFF1+.6,POSTINC0
    movff   gRX_BUFF1+.7,POSTINC0
    movff   gRX_BUFF1+.8,POSTINC0
    movff   gRX_BUFF1+.9,POSTINC0
    movff   gRX_BUFF1+.10,POSTINC0
    movff   gRX_BUFF1+.11,POSTINC0
    movff   gRX_BUFF1+.12,POSTINC0
    movff   gRX_BUFF1+.13,POSTINC0
    movff   gRX_BUFF1+.14,POSTINC0
    movff   gRX_BUFF1+.15,POSTINC0
    movff   gRX_BUFF1+.16,POSTINC0
    movff   gRX_BUFF1+.17,POSTINC0
    movff   gRX_BUFF1+.18,POSTINC0
    movff   gRX_BUFF1+.19,POSTINC0
    movff   gRX_BUFF1+.20,POSTINC0
    movff   gRX_BUFF1+.21,POSTINC0
    movff   gRX_BUFF1+.22,POSTINC0
    movff   gRX_BUFF1+.23,POSTINC0
    movff   gRX_BUFF1+.24,POSTINC0
    movff   gRX_BUFF1+.25,POSTINC0
    movff   gRX_BUFF1+.26,POSTINC0
    movff   gRX_BUFF1+.27,POSTINC0
    movff   gRX_BUFF1+.28,POSTINC0
    movff   gRX_BUFF1+.29,POSTINC0
    movff   gRX_BUFF1+.30,POSTINC0
    movff   gRX_BUFF1+.31,POSTINC0
	
TerminateGet
    movff   FSRH_temp,FSR0H         ;Restore FSR0
    movff   FSRL_temp,FSR0L
	return


;*****************************************************************************
Generate_Pwms 
 ifndef _DONT_USE_TMR0
    btfsc   INTCON,TMR0IF
    return
 endif
 
    movf	PWMdisableMask,w   ;If all PWMs are disabled then bail     
    xorlw   b'00001111'                      
    btfsc   STATUS,Z
    return  
    
    MOVLW   0xff		    ;two's complement representation of -1
    MOVFF   PLUSW1,gPWM_DATA1	 
    MOVLW   0xfe		    ;two's complement representation of -2
    MOVFF   PLUSW1,gPWM_DATA2	 
    MOVLW   0xfd		    ;two's complement representation of -3
    MOVFF   PLUSW1,gPWM_DATA3	 
    MOVLW   0xfc		    ;two's complement representation of -4
    MOVFF   PLUSW1,gPWM_DATA4	 

    incf    gPWM_DATA1,f
    btfss   dPWM01
    bsf	    sPWM01
    incf    gPWM_DATA2,f
    btfss   dPWM02
    bsf	    sPWM02
    incf    gPWM_DATA3,f
    btfss   dPWM03
    bsf	    sPWM03
    incf    gPWM_DATA4,f
    btfss   dPWM04
    bsf	    sPWM04
 ifdef _FRC_BOARD           ;Generate_Pwms(pwm01,pwm02,pwm03,pwm04)
    clrf    gPWM_DATA5
    clrf    gPWM_DATA6
    clrf    gPWM_DATA7
    clrf    gPWM_DATA8
 endif

Generate_Pwms_Cont
    call    Dead_Space_Loop

LP4	decfsz	gPWM_DATA1,f        ;Complete 1st PWM delay cycle
	goto	LP4_Cont1
    btfss   dPWM01
	bcf	    sPWM01
LP4_Cont1
	decfsz	gPWM_DATA2,f        ;Complete 2nd PWM delay cycle
	goto	LP4_Cont2
    btfss   dPWM02
	bcf	    sPWM02
LP4_Cont2
	decfsz	gPWM_DATA3,f        ;Complete 3rd PWM delay cycle
	goto	LP4_Cont3
    btfss   dPWM03
	bcf	    sPWM03
LP4_Cont3
    nop
	decfsz	gPWM_DATA4,f        ;Complete 4th PWM delay cycle
	goto	LP4_Cont4
    btfss   dPWM04
	bcf	    sPWM04

LP4_Cont4
    nop
	decfsz	gPWM_DATA5,f        ;Complete 5th PWM delay cycle
	goto	LP4_Cont5
	bcf	    sPWM05
LP4_Cont5
    nop
	decfsz	gPWM_DATA6,f        ;Complete 6th PWM delay cycle
	goto	LP4_Cont6
	bcf	    sPWM06
LP4_Cont6
    nop
	decfsz	gPWM_DATA7,f        ;Complete 7th PWM delay cycle
	goto	LP4_Cont7
	bcf	    sPWM07
LP4_Cont7
    nop
	decfsz	gPWM_DATA8,f        ;Complete 8th PWM delay cycle
	goto	LP4_Cont8
	bcf	    sPWM08
LP4_Cont8

    call    Adjust4
	incfsz	Cnt,f
	goto	LP4
    return

;*****************************************************************************
Dead_Space_Loop
	movlw	.215                ;Allow dead space (860us)
	movwf	Cnt
DL1	nop
	nop
	nop
	nop
	call	Pwm_Delay
	decfsz	Cnt,f
	goto	DL1
	movlw	.215
	movwf	Cnt
DL2	nop
	nop
	nop
	nop
	call	Pwm_Delay
	decfsz	Cnt,f
	goto	DL2                 
    nop                         ;Extra padding for 40Mhz 
    return

;*****************************************************************************
Pwm_Delay			    ;takes 11 + 2 cyles to get here
	nop
	goto	s_delay		;no byte, so delay before return
s_delay
	nop
	nop
	nop
	nop
	nop
	nop
	return

;*****************************************************************************
Adjust4                  ;40MHz adjustment to mirror 20MHz loop
    movlw   .3
	movwf	Cnt1
Adj_lp4
    nop
	decfsz	Cnt1,f
	goto	Adj_lp4
	return


    END

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
