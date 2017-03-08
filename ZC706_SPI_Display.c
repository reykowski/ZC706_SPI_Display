//*****************************************************************************
//
// ZC706_SPI_Display.c - Display data retrieved from ZC706 via SPI port.
//
// Copyright (c) 2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 1.0 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

//*****************************************************************************
//
//!
//! This code configures the SSI1 as SPI Slave on an
//! TM4C123G launchpad evaluation board.  RX timeout interrupt is configured for SSI1.
//! Four 16bit words are sent from ZC706 FPGA.
//! The SSI1 RX timeout interrupt is enabled.
//! The code then waits for the interrupt to fire.
//! Once the interrupt is fired, the data from slave RX FIFO is read, checked and assembled to a 32 bit word
//! The 32bit words are stored in an array for further processing and disply
//! UART0 is used to send information to host at 115200 baud and 8-n-1
//! mode.
//!
//! This example uses the following peripherals and I/O signals on EK-TM4C123GXL.
//!
//! - SSI1 peripheral
//! - GPIO Port D peripheral (for SSI1 pins)
//! - SSI1CLK - PD0  - Clock
//! - SSI1Fss - PD1	 - Enable
//! - SSI1Rx  - PD2  - MOSI
//! - SSI1Tx  - PD3  - MISO (currently not used)
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of SSI0.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - SSI1IntHandler.
//!
//
//*****************************************************************************



#include "stdint.h"
#include "stdbool.h"
#include <stdarg.h>
#include <math.h>
#include "time.h"
#include "touch.h"
#include "images.h"

#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"

#include "driverlib/fpu.h"
#include "driverlib/mpu.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/flash.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/rom.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"
#include "grlib/checkbox.h"
#include "grlib/container.h"
#include "grlib/pushbutton.h"
#include "grlib/radiobutton.h"
#include "grlib/slider.h"

#include "utils/ustdlib.h"
#include "utils/uartstdio.h"

#include "Kentec320x240x16_ssd2119_8bit.h"

#include "drivers/rgb.h"

#define	timeout 4e5

#define Display_Width	320
#define Display_Height  240
#define P11_x (Display_Width / 10)+45
#define P21_x (Display_Width / 10)+45
#define P31_x (Display_Width / 10)+45
#define P41_x (Display_Width / 10)+45
#define P51_x (Display_Width / 10)+45
#define P61_x (Display_Width / 10)+45

#define P12_x (Display_Width / 2)+80
#define P22_x (Display_Width / 2)+80
#define P32_x (Display_Width / 2)+80
#define P42_x (Display_Width / 2)+80
#define P52_x (Display_Width / 2)+80
#define P62_x (Display_Width / 2)+80

#define P11_y 30
#define P21_y 60
#define P31_y 90
#define P41_y 120
#define P51_y 150
#define P61_y 180

#define P12_y 30
#define P22_y 60
#define P32_y 90
#define P42_y 120
#define P52_y 150
#define P62_y 180

#define S11_x (Display_Width/ 10)+45
#define S21_x (Display_Width/ 10)+45
#define S31_x (Display_Width/ 10)+45
#define S41_x (Display_Width/ 10)+45
#define S51_x (Display_Width/ 10)+45
#define S61_x (Display_Width/ 10)+45

#define S12_x (Display_Width/ 2)+80
#define S22_x (Display_Width/ 2)+80
#define S32_x (Display_Width/ 2)+80
#define S42_x (Display_Width/ 2)+80
#define S52_x (Display_Width/ 2)+80
#define S62_x (Display_Width/ 2)+80

#define S11_y 30
#define S21_y 60
#define S31_y 90
#define S41_y 120
#define S51_y 150
#define S61_y 180

#define S12_y 30
#define S22_y 60
#define S32_y 90
#define S42_y 120
#define S52_y 150
#define S62_y 180

// Define String names for data display:
#define str_AGC		"AGC: "
#define str_LOCK	"LOCK: "
#define str_HOLD	"HOLD: "
#define str_EVM		"EVM: "
#define str_FRAME	"FRAME: "
#define str_CRC		"CRC: "
#define str_BER		"BER: "
#define str_NA		"---: "


// Define names for LED display:
#define str_LED1	"AGC"
#define str_LED2	"LOCK"
#define str_LED3	"HOLD"
#define str_LED4	"EVM"
#define str_LED5	"FRAME"
#define str_LED6	"CRC"
#define str_LED7	"BER"
#define str_LED8	"---"
#define str_LED9	"---"
#define str_LED10	"---"
//*****************************************************************************
//
// Number of bytes to send and receive.
//
//*****************************************************************************
#define NUM_SSI_DATA 128

//*****************************************************************************
// Define global variables

    float fBER=0, fTemp2=0, fTempInt=0;
    float fAGC=0, fLock=0, fHold=0, fEVM=0;
    float fFrame=0, fCRC=0;
    float fBTemp1[3] = {0,0,0}, fBTemp2[3] = {0,0,0}, fBTempInt[3] = {0,0,0};
    float fBVin0[3] = {0,0,0}, fBVin1[3] = {0,0,0}, fBVin2[3] = {0,0,0}, fBVin3[3] = {0,0,0};
    float fBIsense1[3] = {0,0,0}, fBIsense2[3] = {0,0,0};
    float fADC_Val=0;
    float fRsense = 0.010;
    float fVref0 = 2.5, fVref1 = 2.5, fVref2 = 2.5, fVref3 = 2.5;


    uint16_t i_counter=0;
    uint8_t iAGC_Curve[320]={ [ 0 ... 319 ] = (186-25)};
    uint8_t iCRC_Curve[320]={ [ 0 ... 319 ] = 186};



    float my_noise;

    static char val_AGC_Str[20];
    static char val_LOCK_Str[20];
    static char val_HOLD_Str[20];
    static char val_EVM_Str[20];
    static char val_FRAME_Str[20];
    static char val_CRC_Str[20];
    static char val_BER_Str[20];
    static char val8_string[20];
    static char val9_string[20];
    static char val10_string[20];
    static char val11_string[20];
    static char val12_string[20];

    //*****************************************************************************
    //
    // Global variables used in interrupt handler and the main loop.
    //
    //*****************************************************************************

    uint16_t i16average = 16;
    uint8_t	SysTick_Semafore = 1;
    volatile uint32_t g_ulSSI1RXFF = 0;
    int32_t g_ulDataRx2[NUM_SSI_DATA];


    //*****************************************************************************
    // Global variables used by widgets
    // Forward declarations for the globals required to define the widgets at
    // compile-time.
    //
    //*****************************************************************************
    void OnPrevious(tWidget *pWidget);
    void OnNext(tWidget *pWidget);
    void OnButtonPress2(tWidget *pWidget);
    void OnRxButtonPress(tWidget *pWidget);
    void OnPLLButtonPress(tWidget *pWidget);
    void OnSYNCButtonPress(tWidget *pWidget);
    void OnRXMSGButtonPress(tWidget *pWidget);
    void OnBlockDiagramPaint(tWidget *pWidget, tContext *pContext);
    void OnLEDStatusPaint(tWidget *pWidget, tContext *pContext);
    void OnDataPaint(tWidget *pWidget, tContext *pContext);
    void OnRxStatusPaint(tWidget *pWidget, tContext *pContext);
    void OnPLLStatusPaint(tWidget *pWidget, tContext *pContext);
    void OnSYNCStatusPaint(tWidget *pWidget, tContext *pContext);
    void OnRXMSGStatusPaint(tWidget *pWidget, tContext *pContext);
    void OnPrimitivePaint(tWidget *pWidget, tContext *pContext);
    void OnRadioChange(tWidget *pWidget, uint32_t bSelected);

    extern tCanvasWidget g_psPanels[10];
    tContext *p1Context;



//*****************************************************************************
//
// The DMA control structure table.
//
//*****************************************************************************

    #ifdef ewarm
    #pragma data_alignment=1024
    tDMAControlTable sDMAControlTable[64];
    #elif defined(ccs)
    #pragma DATA_ALIGN(sDMAControlTable, 1024)
    tDMAControlTable sDMAControlTable[64];
    #else
    tDMAControlTable sDMAControlTable[64] __attribute__ ((aligned(1024)));
    #endif



//*****************************************************************************
//
// Global array for holding the color values for the RGB.
//
//*****************************************************************************
uint32_t g_pui32Colors[3];


//*****************************************************************************
//
// Number of bytes to send and receive.
//
//*****************************************************************************
#define NUM_SSI_DATA 128



//*****************************************************************************
//
// Interrupt handler for SSI1 peripheral in slave mode.  It reads the interrupt
// status and if the interrupt is fired by a RX time out interrupt it reads the
// SSI1 RX FIFO and increments a counter to tell the main loop that RX timeout
// interrupt was fired.
//
//*****************************************************************************
void
SSI1IntHandler(void)
{
	uint8_t B3_counter, B2_counter, B1_counter, B0_counter;
	uint16_t B3_val, B2_val, B1_val, B0_val;
	uint32_t Byte3, Byte2, Byte1, Byte0;
	uint32_t ulStatus;
	int32_t val;

	//
	// Read interrupt status.
	//
	ulStatus = SSIIntStatus(SSI1_BASE, 1);

	//
	// Check the reason for the interrupt.
	//

	if(ulStatus & SSI_RXTO)
	{
		//
		// Interrupt is because of RX tidata received.
		// Read one data word and
		// increment counter to tell
		// main loop that RX data has been received.
		//



		SSIDataGet(SSI1_BASE, &Byte3);

		if ((Byte3 & 0x0300)!=0x0300)			// Check if we are properly sync'ed
		{
			SSIIntClear(SSI1_BASE, ulStatus);
			return;
		}


		SSIDataGet(SSI1_BASE, &Byte2);

		if ((Byte2 & 0x0300)!=0x0200)			// Check if we are properly sync'ed
		{
			SSIIntClear(SSI1_BASE, ulStatus);
			return;
		}


		SSIDataGet(SSI1_BASE, &Byte1);

		if ((Byte1 & 0x0300)!=0x0100)			// Check if we are properly sync'ed
		{
			SSIIntClear(SSI1_BASE, ulStatus);
			return;
		}


		SSIDataGet(SSI1_BASE, &Byte0);

		if ((Byte0 & 0x0300)!=0x0000)			// Check if we are properly sync'ed
		{
			SSIIntClear(SSI1_BASE, ulStatus);
			return;
		}

		B3_val = Byte3 & 0xffff;
		B2_val = Byte2 & 0xffff;
		B1_val = Byte1 & 0xffff;
		B0_val = Byte0 & 0xffff;

		B3_counter = B3_val>>10;
		B2_counter = B2_val>>10;
		B1_counter = B1_val>>10;
		B0_counter = B0_val>>10;

		val = (B0_val&0xff) + ((B1_val&0xff)<<8)+((B2_val&0xff)<<16)+((B3_val&0xff)<<24);
		if ((B0_counter==B1_counter)& (B0_counter==B1_counter)& (B0_counter==B3_counter))
		{
			g_ulDataRx2[B0_counter-1] = val;
			g_ulSSI1RXFF++;
		}
	}

	if(ulStatus & SSI_RXFF)
	{
		//
		// Interrupt is because of RX time out.
		// Don't do nothing
		// Just clear interrupt
		//
	}

	//
	// Clear interrupts.
	//
	SSIIntClear(SSI1_BASE, ulStatus);
}

//*****************************************************************************
//
// Called by the NVIC as a SysTick interrupt, which is used to generate the
// sample interval
//
//*****************************************************************************
uint8_t SysTickIntHandler()
{
	uint8_t status = 0;
	if (SysTick_Semafore == 1)
	{
		status = 1;
		return status;
	}
	ROM_IntMasterDisable();

	if (status)
	{
		//return status;
	}

    ROM_IntMasterEnable();
    SysTick_Semafore = 1;		// Indicate to Main Routine that Sys Tick occured

    return status;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//	Define Panels and Widgets
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//*****************************************************************************
//
// The first panel, which contains the Block Diagram with Push Buttons
//
//*****************************************************************************
//
// Start placing some Blocks for Block Diagram
//
#define rect_x 	45
#define rect_y	45
#define rect_x1 20
#define rect_x2 95
#define rect_x3 170
#define rect_x4 245
#define rect_y1 45
#define rect_y2 120
uint32_t g_ulButtonState;
uint32_t g_ulButtonState2;
tPushButtonWidget g_psPushButtons2[] =
{
    RectangularButtonStruct(g_psPanels, g_psPushButtons2 + 1, 0,
                            &g_sKentec320x240x16_SSD2119, rect_x1, rect_y1, rect_x, rect_y,
                            PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY ,
							ClrLightGreen, ClrGold,  ClrGray, ClrBlack,
                            &g_sFontCm14, "RX", 0, 0, 0, 0, OnRxButtonPress),
    RectangularButtonStruct(g_psPanels, g_psPushButtons2 + 2, 0,
    					    &g_sKentec320x240x16_SSD2119, rect_x2, rect_y1, rect_x, rect_y,
                            PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY ,
							ClrLightGreen, ClrGold, ClrGray, ClrBlack,
                            &g_sFontCm14, "PLL", 0, 0, 0, 0, OnPLLButtonPress),
    RectangularButtonStruct(g_psPanels, g_psPushButtons2 + 3, 0,
    						&g_sKentec320x240x16_SSD2119, rect_x3, rect_y1, rect_x, rect_y,
	                        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY ,
							ClrLightGreen, ClrGold, ClrGray, ClrBlack,
	                        &g_sFontCm14, "SYNC", 0, 0, 0, 0, OnSYNCButtonPress),
	RectangularButtonStruct(g_psPanels,g_psPushButtons2 + 4 , 0,
						    &g_sKentec320x240x16_SSD2119, rect_x4, rect_y1, rect_x, rect_y,
							PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY ,
							ClrLightGreen, ClrGold, ClrGray, ClrBlack,
							&g_sFontCm14, "MSG", 0, 0, 0, 0, OnRXMSGButtonPress),
    RectangularButtonStruct(g_psPanels, g_psPushButtons2 + 5, 0,
                            &g_sKentec320x240x16_SSD2119, rect_x1, rect_y2, rect_x, rect_y,
                            PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY ,
                            ClrLightGreen, ClrGold, ClrGray, ClrBlack,
                            &g_sFontCm14, "TX", 0, 0, 0, 0, OnButtonPress2),
    RectangularButtonStruct(g_psPanels, g_psPushButtons2 + 6, 0,
    					    &g_sKentec320x240x16_SSD2119, rect_x2, rect_y2, rect_x, rect_y,
                            PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY ,
                            ClrLightGreen, ClrGold, ClrGray, ClrBlack,
                            &g_sFontCm14, "RRC", 0, 0, 0, 0, OnButtonPress2),
    RectangularButtonStruct(g_psPanels, g_psPushButtons2 + 7, 0,
    						&g_sKentec320x240x16_SSD2119, rect_x3, rect_y2, rect_x, rect_y,
	                        PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY ,
	                        ClrLightGreen, ClrGold, ClrGray, ClrBlack,
	                        &g_sFontCm14, "BPSK", 0, 0, 0, 0, OnButtonPress2),
	RectangularButtonStruct(g_psPanels, 0, 0,
						    &g_sKentec320x240x16_SSD2119, rect_x4, rect_y2, rect_x, rect_y,
							PB_STYLE_FILL | PB_STYLE_OUTLINE | PB_STYLE_TEXT | PB_STYLE_RELEASE_NOTIFY ,
							ClrLightGreen, ClrGold, ClrGray, ClrBlack,
							&g_sFontCm14, "MSG", 0, 0, 0, 0, OnButtonPress2),
};


tCanvasWidget g_psButtonCanvas[] =
{
    CanvasStruct(g_psPanels, g_psButtonCanvas+1, 0,
                 &g_sKentec320x240x16_SSD2119, 0, 24, 320, 166,
                 CANVAS_STYLE_FILL, ClrSeashell, 0, 0, 0, 0, 0, 0),
    CanvasStruct(g_psPanels, g_psPushButtons2, 0,
                 &g_sKentec320x240x16_SSD2119, 0, 24, 320, 166,
                 CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnBlockDiagramPaint),
};


#define NUM_PUSH_BUTTONS2        (sizeof(g_psPushButtons2) /   \
                                 sizeof(g_psPushButtons2[0]))


//*****************************************************************************
//
// The second panel, which contains the Status LEDs
//
//*****************************************************************************
Canvas(g_sLED_Display, g_psPanels+1, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 24,
       320, 166, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnLEDStatusPaint);

//*****************************************************************************
//
// The third panel, which contains Status Data in string form
//
//*****************************************************************************
Canvas(g_sStatusData, g_psPanels + 2, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 24,
       320, 166, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnDataPaint);
//*****************************************************************************
//
// The fourth panel, which contains Rx Status Info
//
//*****************************************************************************
Canvas(g_sRxStatus, g_psPanels + 3, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 24,
       320, 166, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnRxStatusPaint);

//*****************************************************************************
//
// The fifth panel, which contains PLL Status Info
//
//*****************************************************************************
Canvas(g_sPLLStatus, g_psPanels + 4, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 24,
       320, 166, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnPLLStatusPaint);
//*****************************************************************************
//
// The sixth panel, which contains SYNC Status Info
//
//*****************************************************************************
Canvas(g_sSYNCStatus, g_psPanels + 5, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 24,
       320, 166, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnSYNCStatusPaint);
//*****************************************************************************
//
// The seventh panel, which contains PLL Status Info
//
//*****************************************************************************
Canvas(g_sRXMSGStatus, g_psPanels + 6, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 24,
       320, 166, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnRXMSGStatusPaint);
//*****************************************************************************
//
// The eigth panel, which draws some Primitives
//
//*****************************************************************************
Canvas(g_sPrimitives, g_psPanels + 7, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 24,
	   320, 166, CANVAS_STYLE_APP_DRAWN, 0, 0, 0, 0, 0, 0, OnPrimitivePaint);
//*****************************************************************************
//
// An array of canvas widgets, one per panel.  Each canvas is filled with
// black, overwriting the contents of the previous panel.
//
//*****************************************************************************
tCanvasWidget g_psPanels[] =
{
	CanvasStruct(0, 0, &g_psButtonCanvas, &g_sKentec320x240x16_SSD2119, 0, 24,
	             320, 166, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
	CanvasStruct(0, 0, &g_sLED_Display , &g_sKentec320x240x16_SSD2119, 0, 24,
			     320, 166, CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0),
    CanvasStruct(0, 0, &g_sStatusData, &g_sKentec320x240x16_SSD2119, 0, 24,
                 320, 166, CANVAS_STYLE_FILL, ClrLime, 0, 0, 0, 0, 0, 0),
    CanvasStruct(0, 0, &g_sRxStatus, &g_sKentec320x240x16_SSD2119, 0, 24,
	             320, 166, CANVAS_STYLE_FILL, ClrLime, 0, 0, 0, 0, 0, 0),
    CanvasStruct(0, 0, &g_sPLLStatus, &g_sKentec320x240x16_SSD2119, 0, 24,
			     320, 166, CANVAS_STYLE_FILL, ClrLime, 0, 0, 0, 0, 0, 0),
	CanvasStruct(0, 0, &g_sSYNCStatus, &g_sKentec320x240x16_SSD2119, 0, 24,
			     320, 166, CANVAS_STYLE_FILL, ClrLime, 0, 0, 0, 0, 0, 0),
    CanvasStruct(0, 0, &g_sRXMSGStatus, &g_sKentec320x240x16_SSD2119, 0, 24,
			     320, 166, CANVAS_STYLE_FILL, ClrLime, 0, 0, 0, 0, 0, 0),
};
/*

	             	CanvasStruct(0, 0, &g_sPrimitives  , &g_sKentec320x240x16_SSD2119, 0, 24,
			     320, 166, CANVAS_STYLE_FILL, ClrSeashell, 0, 0, 0, 0, 0, 0),
};
*/
//*****************************************************************************
//
// The number of panels.
//
//*****************************************************************************
#define NUM_PANELS              (sizeof(g_psPanels) / sizeof(g_psPanels[0]))

//*****************************************************************************
//
// The names for each of the panels, which is displayed at the bottom of the
// screen.
//
//*****************************************************************************
char *g_pcPanelNames[] =
{
	"     Block Diagram  ",
    "     Status LEDs    ",
    "     Status Data    ",
    "     Rx Status      ",
    "     PLL Status     ",
    "     SYNC Status    ",
    "     Rx MSG Status  ",
    "     Sliders     ",
    "     S/W Update    "
};
#define Block_Diagram  0
#define LED_Panel      1
#define Data_Panel 	   2
#define RX_Panel       3
#define PLL_Panel      4
#define SYNC_Panel     5
#define RXMSG_Panel    6
//*****************************************************************************
//
// The buttons and text across the bottom of the screen.
//
//*****************************************************************************
RectangularButton(g_sPrevious, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 190,
                  50, 50, PB_STYLE_FILL, ClrBlack, ClrBlack, 0, ClrSilver,
                  &g_sFontCm20, "-", g_pucBlue50x50, g_pucBlue50x50Press, 0, 0,
                  OnPrevious);

Canvas(g_sTitle, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 50, 190, 220, 50,
       CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_OPAQUE, 0, 0, ClrSilver,
       &g_sFontCm20, 0, 0, 0);

RectangularButton(g_sNext, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 270, 190,
                  50, 50, PB_STYLE_IMG | PB_STYLE_TEXT, ClrBlack, ClrBlack, 0,
                  ClrSilver, &g_sFontCm20, "+", g_pucBlue50x50,
                  g_pucBlue50x50Press, 0, 0, OnNext);


//*****************************************************************************
//
// The panel that is currently being displayed.
//
//*****************************************************************************
uint32_t g_ulPanel;
uint32_t g_ulPreviousPanel;
//*****************************************************************************
//
// Draw a string right side adjusted
//
//*****************************************************************************
void  GrStringDrawRight(const tContext *pContext, const char *pcString, int32_t i32Length,
        int32_t i32X, int32_t i32Y, uint32_t bOpaque)
{

long w;

w = GrStringWidthGet(pContext, pcString, i32Length);
GrStringDraw(pContext, pcString, i32Length, i32X-w, i32Y, bOpaque);
}
//*****************************************************************************
//
// Handles presses of the previous panel button.
//
//*****************************************************************************
void
OnPrevious(tWidget *pWidget)
{
	g_ulPreviousPanel = g_ulPanel;
    //
    // There is nothing to be done if the first panel is already being
    // displayed.
    //
    if(g_ulPanel == 0)
    {
        return;
    }

    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ulPanel));

    if (g_ulPanel>2)
    {
    	g_ulPanel = 0;	// Go back to first panel
    }
    else
    {
    	//
    	// Decrement the panel index.
    	//
    	g_ulPanel--;
    }

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ulPanel));
    WidgetPaint((tWidget *)(g_psPanels + g_ulPanel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanelNames[g_ulPanel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // See if this is the first panel.
    //
    if(g_ulPanel == 0)
    {
        //
        // Clear the previous button from the display since the first panel is
        // being displayed.
        //
        PushButtonImageOff(&g_sPrevious);
        PushButtonTextOff(&g_sPrevious);
        PushButtonFillOn(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }

    //
    // See if the previous panel was the last panel.
    //
    //if(g_ulPanel == (NUM_PANELS - 2))
      if(g_ulPreviousPanel >= ( Data_Panel ))
    {
        //
        // Display the next button.
        //
        PushButtonImageOn(&g_sNext);
        PushButtonTextOn(&g_sNext);
        PushButtonFillOff(&g_sNext);
        WidgetPaint((tWidget *)&g_sNext);
    }

}

//*****************************************************************************
//
// Handles presses of the next panel button.
//
//*****************************************************************************
void
OnNext(tWidget *pWidget)
{
    //
    // There is nothing to be done if the last panel is already being
    // displayed.
    //

    if(g_ulPanel >= Data_Panel)
    {
        return;
    }

    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ulPanel));

    //
    // Increment the panel index.
    //
    g_ulPanel++;

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ulPanel));
    WidgetPaint((tWidget *)(g_psPanels + g_ulPanel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanelNames[g_ulPanel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // See if the previous panel was the first panel.
    //
    if(g_ulPanel == 1)
    {
        //
        // Display the previous button.
        //
        PushButtonImageOn(&g_sPrevious);
        PushButtonTextOn(&g_sPrevious);
        PushButtonFillOff(&g_sPrevious);
        WidgetPaint((tWidget *)&g_sPrevious);
    }

    //
    // See if this is the last panel.
    //
    if(g_ulPanel == Data_Panel)
    {
        //
        // Clear the next button from the display since the last panel is being
        // displayed.
        //
        PushButtonImageOff(&g_sNext);
        PushButtonTextOff(&g_sNext);
        PushButtonFillOn(&g_sNext);
        WidgetPaint((tWidget *)&g_sNext);
    }

}


//*****************************************************************************
//
// Handles paint requests for the primitives g_psButtonCanvas widget.
//
//*****************************************************************************
void
OnBlockDiagramPaint(tWidget *pWidget, tContext *pContext)
{

#define rect_x 		45
#define rect_y		45
#define rect_x1 	20
#define rect_x2		95
#define rect_x3 	170
#define rect_x4		245
#define rect_y1 	45
#define rect_y2		120

    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, rect_x1+rect_x, rect_y1+rect_y/2, rect_x2, rect_y1+rect_y/2);
    GrLineDraw(pContext, rect_x2-10, rect_y1+rect_y/2-5, rect_x2, rect_y1+rect_y/2);
    GrLineDraw(pContext, rect_x2-10, rect_y1+rect_y/2+5, rect_x2, rect_y1+rect_y/2);

    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, rect_x2+rect_x, rect_y1+rect_y/2, rect_x3, rect_y1+rect_y/2);
    GrLineDraw(pContext, rect_x3-10, rect_y1+rect_y/2-5, rect_x3, rect_y1+rect_y/2);
    GrLineDraw(pContext, rect_x3-10, rect_y1+rect_y/2+5, rect_x3, rect_y1+rect_y/2);

    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, rect_x3+rect_x, rect_y1+rect_y/2, rect_x4, rect_y1+rect_y/2);
    GrLineDraw(pContext, rect_x4-10, rect_y1+rect_y/2-5, rect_x4, rect_y1+rect_y/2);
    GrLineDraw(pContext, rect_x4-10, rect_y1+rect_y/2+5, rect_x4, rect_y1+rect_y/2);

    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, rect_x4+rect_x/2, rect_y1+rect_y, rect_x4+rect_x/2, rect_y2);
    GrLineDraw(pContext, rect_x4+rect_x/2+5, rect_y2-10, rect_x4+rect_x/2, rect_y2);
    GrLineDraw(pContext, rect_x4+rect_x/2-5, rect_y2-10, rect_x4+rect_x/2, rect_y2);

    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, rect_x1+rect_x, rect_y2+rect_y/2, rect_x2, rect_y2+rect_y/2);
    GrLineDraw(pContext, rect_x1+rect_x, rect_y2+rect_y/2, rect_x1+rect_x+10, rect_y2+rect_y/2-5);
    GrLineDraw(pContext, rect_x1+rect_x, rect_y2+rect_y/2, rect_x1+rect_x+10, rect_y2+rect_y/2+5);

    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, rect_x2+rect_x, rect_y2+rect_y/2, rect_x3, rect_y2+rect_y/2);
    GrLineDraw(pContext, rect_x2+rect_x, rect_y2+rect_y/2, rect_x2+rect_x+10, rect_y2+rect_y/2-5);
    GrLineDraw(pContext, rect_x2+rect_x, rect_y2+rect_y/2, rect_x2+rect_x+10, rect_y2+rect_y/2+5);

    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, rect_x3+rect_x, rect_y2+rect_y/2, rect_x4, rect_y2+rect_y/2);
    GrLineDraw(pContext, rect_x3+rect_x, rect_y2+rect_y/2, rect_x3+rect_x+10, rect_y2+rect_y/2-5);
    GrLineDraw(pContext, rect_x3+rect_x, rect_y2+rect_y/2, rect_x3+rect_x+10, rect_y2+rect_y/2+5);


}
//*****************************************************************************
//
// Handles presses of the RX Status button.
//
//*****************************************************************************
void
OnRxButtonPress(tWidget *pWidget)
{
	//return;

    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ulPanel));

    //
    // Set panel index to Rx Panel.
    //
    g_ulPanel = RX_Panel;

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ulPanel));
    WidgetPaint((tWidget *)(g_psPanels + g_ulPanel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanelNames[g_ulPanel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // Display the previous button.
    //
    PushButtonImageOn(&g_sPrevious);
    PushButtonTextOn(&g_sPrevious);
    PushButtonFillOff(&g_sPrevious);
    WidgetPaint((tWidget *)&g_sPrevious);

    //
    // Clear the next button
    //
    PushButtonImageOff(&g_sNext);
    PushButtonTextOff(&g_sNext);
    PushButtonFillOn(&g_sNext);
    WidgetPaint((tWidget *)&g_sNext);

}
//*****************************************************************************
//
// Handles presses of the PLL Status button.
//
//*****************************************************************************
void
OnPLLButtonPress(tWidget *pWidget)
{
    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ulPanel));

    //
    // Set panel index to PLL Panel.
    //
    g_ulPanel = PLL_Panel;

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ulPanel));
    WidgetPaint((tWidget *)(g_psPanels + g_ulPanel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanelNames[g_ulPanel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // Display the previous button.
    //
    PushButtonImageOn(&g_sPrevious);
    PushButtonTextOn(&g_sPrevious);
    PushButtonFillOff(&g_sPrevious);
    WidgetPaint((tWidget *)&g_sPrevious);

    //
    // Clear the next button
    //
    PushButtonImageOff(&g_sNext);
    PushButtonTextOff(&g_sNext);
    PushButtonFillOn(&g_sNext);
    WidgetPaint((tWidget *)&g_sNext);
}
//*****************************************************************************
//
// Handles presses of the SYNC Status button.
//
//*****************************************************************************
void
OnSYNCButtonPress(tWidget *pWidget)
{
    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ulPanel));

    //
    // Set panel index to SYNC Panel.
    //
    g_ulPanel = SYNC_Panel;

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ulPanel));
    WidgetPaint((tWidget *)(g_psPanels + g_ulPanel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanelNames[g_ulPanel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // Display the previous button.
    //
    PushButtonImageOn(&g_sPrevious);
    PushButtonTextOn(&g_sPrevious);
    PushButtonFillOff(&g_sPrevious);
    WidgetPaint((tWidget *)&g_sPrevious);

    //
    // Clear the next button
    //
    PushButtonImageOff(&g_sNext);
    PushButtonTextOff(&g_sNext);
    PushButtonFillOn(&g_sNext);
    WidgetPaint((tWidget *)&g_sNext);
}
//*****************************************************************************
//
// Handles presses of the RX MSG Status button.
//
//*****************************************************************************
void
OnRXMSGButtonPress(tWidget *pWidget)
{
    //
    // Remove the current panel.
    //
    WidgetRemove((tWidget *)(g_psPanels + g_ulPanel));

    //
    // Set panel index to Rx MSG Panel.
    //
    g_ulPanel = RXMSG_Panel;

    //
    // Add and draw the new panel.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)(g_psPanels + g_ulPanel));
    WidgetPaint((tWidget *)(g_psPanels + g_ulPanel));

    //
    // Set the title of this panel.
    //
    CanvasTextSet(&g_sTitle, g_pcPanelNames[g_ulPanel]);
    WidgetPaint((tWidget *)&g_sTitle);

    //
    // Display the previous button.
    //
    PushButtonImageOn(&g_sPrevious);
    PushButtonTextOn(&g_sPrevious);
    PushButtonFillOff(&g_sPrevious);
    WidgetPaint((tWidget *)&g_sPrevious);

    //
    // Clear the next button
    //
    PushButtonImageOff(&g_sNext);
    PushButtonTextOff(&g_sNext);
    PushButtonFillOn(&g_sNext);
    WidgetPaint((tWidget *)&g_sNext);
}

//*****************************************************************************
//
// Handles press notifications for the push button widgets 2.
//
//*****************************************************************************
void
OnButtonPress2(tWidget *pWidget)
{
    uint32_t ulIdx;
#define rect_x 	45
#define rect_y	45


    //
    // Find the index of this push button.
    //
    for(ulIdx = 0; ulIdx < NUM_PUSH_BUTTONS2; ulIdx++)
    {
        if(pWidget == (tWidget *)(g_psPushButtons2 + ulIdx))
        {
            break;
        }
    }

    //
    // Return if the push button could not be found.
    //
    if(ulIdx == NUM_PUSH_BUTTONS2)
    {
        return;
    }

    //
    // Toggle the state of this push button.
    //
    g_ulButtonState2 ^= 1 << ulIdx;

    //
    // Set the matching indicator based on the selected state of the check box.
    //

    PushButtonFillColorSet(g_psPushButtons2 + ulIdx, (g_ulButtonState2 & (1 << ulIdx)) ? ClrRed : ClrLightGreen);

}
//*****************************************************************************
//
// Handles paint requests for the LED Status canvas widget.
//
//*****************************************************************************
void
OnLEDStatusPaint(tWidget *pWidget, tContext *pContext)
{



    //
    // Start placing some LEDs
    //

    #define LED_x0 40
	#define LED_y0 80
	#define LED_xd 56
	#define LED_yd 90
    #define LED_size 10

    GrContextForegroundSet(pContext, ClrLime);
    GrCircleFill(pContext, (LED_x0 + 0*LED_xd), (LED_y0 + 0*LED_yd), LED_size);
    GrCircleFill(pContext, (LED_x0 + 1*LED_xd), (LED_y0 + 0*LED_yd), LED_size);
    GrCircleFill(pContext, (LED_x0 + 2*LED_xd), (LED_y0 + 0*LED_yd), LED_size);
    GrCircleFill(pContext, (LED_x0 + 3*LED_xd), (LED_y0 + 0*LED_yd), LED_size);
    GrCircleFill(pContext, (LED_x0 + 4*LED_xd), (LED_y0 + 0*LED_yd), LED_size);


    GrCircleFill(pContext, (LED_x0 + 0*LED_xd), (LED_y0 + 1*LED_yd), LED_size);
    GrCircleFill(pContext, (LED_x0 + 1*LED_xd), (LED_y0 + 1*LED_yd), LED_size);
    GrCircleFill(pContext, (LED_x0 + 2*LED_xd), (LED_y0 + 1*LED_yd), LED_size);
    GrCircleFill(pContext, (LED_x0 + 3*LED_xd), (LED_y0 + 1*LED_yd), LED_size);
    GrCircleFill(pContext, (LED_x0 + 4*LED_xd), (LED_y0 + 1*LED_yd), LED_size);

    GrContextFontSet(pContext, &g_sFontCm14 );
    GrContextForegroundSet(pContext, ClrLimeGreen);

    GrStringDrawCentered(pContext, str_LED1,  9,(LED_x0 + 0*LED_xd), (LED_y0 + 0*LED_yd -30) , 0);
    GrStringDrawCentered(pContext, str_LED2,  9,(LED_x0 + 1*LED_xd), (LED_y0 + 0*LED_yd -30) , 0);
    GrStringDrawCentered(pContext, str_LED3,  9,(LED_x0 + 2*LED_xd), (LED_y0 + 0*LED_yd -30) , 0);
    GrStringDrawCentered(pContext, str_LED4,  9,(LED_x0 + 3*LED_xd), (LED_y0 + 0*LED_yd -30) , 0);
    GrStringDrawCentered(pContext, str_LED5,  9,(LED_x0 + 4*LED_xd), (LED_y0 + 0*LED_yd -30) , 0);

    GrStringDrawCentered(pContext, str_LED6,  9,(LED_x0 + 0*LED_xd), (LED_y0 + 1*LED_yd -30) , 0);
    GrStringDrawCentered(pContext, str_LED7,  9,(LED_x0 + 1*LED_xd), (LED_y0 + 1*LED_yd -30) , 0);
    GrStringDrawCentered(pContext, str_LED8,  9,(LED_x0 + 2*LED_xd), (LED_y0 + 1*LED_yd -30) , 0);
    GrStringDrawCentered(pContext, str_LED9,  9,(LED_x0 + 3*LED_xd), (LED_y0 + 1*LED_yd -30) , 0);
    GrStringDrawCentered(pContext, str_LED10, 9,(LED_x0 + 4*LED_xd), (LED_y0 + 1*LED_yd -30) , 0);


}

//*****************************************************************************
//
// Handles paint requests for the data canvas widget.
//
//*****************************************************************************
void
OnDataPaint(tWidget *pWidget, tContext *pContext)
{


    //
    // Start placing some strings
    //
    GrContextFontSet(pContext, &g_sFontCm20);
    GrContextForegroundSet(pContext, ClrBlack);

    GrStringDrawRight(pContext, str_AGC,  9, P11_x+10, P11_y, 0);
    GrStringDrawRight(pContext, str_LOCK,  9, P12_x+10, P12_y, 0);
    GrStringDrawRight(pContext, str_HOLD,  9, P21_x+10, P21_y, 0);
    GrStringDrawRight(pContext, str_EVM,  9, P22_x+10, P22_y, 0);
    GrStringDrawRight(pContext, str_FRAME,  9, P31_x+10, P31_y, 0);
    GrStringDrawRight(pContext, str_CRC,  9, P32_x+10, P32_y, 0);
    GrStringDrawRight(pContext, str_BER,  9, P41_x+10, P41_y, 0);
    GrStringDrawRight(pContext, str_NA,  9, P42_x+10, P42_y, 0);
    GrStringDrawRight(pContext, str_NA,  9, P51_x+10, P51_y, 0);
    GrStringDrawRight(pContext, str_NA, 9, P52_x+10, P52_y, 0);

}
//*****************************************************************************
//
// Handles paint requests for the Rx Status canvas widget.
//
//*****************************************************************************
void
OnRxStatusPaint(tWidget *pWidget, tContext *pContext)
{
    //
    // Start placing some strings
    //
    GrContextFontSet(pContext, &g_sFontCm20);
    GrContextForegroundSet(pContext, ClrBlack);
    GrStringDrawRight(pContext, str_AGC,  9, P11_x+10, P11_y, 0);

}
//*****************************************************************************
//
// Handles paint requests for the PLL Status canvas widget.
//
//*****************************************************************************
void
OnPLLStatusPaint(tWidget *pWidget, tContext *pContext)
{


    //
    // Start placing some strings
    //
    GrContextFontSet(pContext, &g_sFontCm20);
    GrContextForegroundSet(pContext, ClrBlack);

    GrStringDrawRight(pContext, str_LOCK,  9, P11_x+10, P11_y, 0);
    GrStringDrawRight(pContext, str_HOLD,  9, P12_x+10, P12_y, 0);


}
//*****************************************************************************
//
// Handles paint requests for the SYNC Status canvas widget.
//
//*****************************************************************************
void
OnSYNCStatusPaint(tWidget *pWidget, tContext *pContext)
{

    //
    // Start placing some strings
    //
    GrContextFontSet(pContext, &g_sFontCm20);
    GrContextForegroundSet(pContext, ClrBlack);
    GrContextBackgroundSet(pContext, ClrLime);
    GrStringDraw(pContext, str_FRAME, -1, 32, S11_y, 1);
    //GrStringDrawRight(pContext, str_FRAME,  9, P11_x+10, P11_y, 0);

}
//*****************************************************************************
//
// Handles paint requests for the Rx MSG Status canvas widget.
//
//*****************************************************************************
void
OnRXMSGStatusPaint(tWidget *pWidget, tContext *pContext)
{

    //
    // Start placing some strings
    //
    GrContextFontSet(pContext, &g_sFontCm20);
    GrContextForegroundSet(pContext, ClrBlack);

    GrStringDrawRight(pContext, str_CRC,  9, P11_x+10, P11_y, 0);

}
//*****************************************************************************
//
// Handles paint requests for the primitives canvas widget.
//
//*****************************************************************************
void
OnPrimitivePaint(tWidget *pWidget, tContext *pContext)
{

    tRectangle sRect;

#define rect_x 	45
#define rect_y	45

    //
    // Draw a filled rectangle with an overlapping rectangle.
    //
    GrContextForegroundSet(pContext, ClrLightGreen);
    sRect.i16XMin = 20;
    sRect.i16YMin = 45;
    sRect.i16XMax = 20 +  rect_x;
    sRect.i16YMax = 45 + rect_y;
    GrRectFill(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrSlateBlue);
    GrRectDraw(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrBlack);
    GrContextFontSet(pContext, &g_sFontCm14);
    GrStringDrawCentered(pContext, "RX", -1, 20+rect_x/2, 45+rect_y/2, 0);
    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, 20+rect_x, 45+rect_y/2, 95, 45+rect_y/2);

    GrContextForegroundSet(pContext, ClrLightGreen);
    sRect.i16XMin = 95;
    sRect.i16YMin = 45;
    sRect.i16XMax = 95 +  rect_x;
    sRect.i16YMax = 45 + rect_y;
    GrRectFill(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrSlateBlue);
    GrRectDraw(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrBlack);
    GrContextFontSet(pContext, &g_sFontCm14);
    GrStringDrawCentered(pContext, "PLL", -1, 95+rect_x/2, 45+rect_y/2, 0);
    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, 95+rect_x, 45+rect_y/2, 170, 45+rect_y/2);

    GrContextForegroundSet(pContext, ClrLightGreen);
    sRect.i16XMin = 170;
    sRect.i16YMin = 45;
    sRect.i16XMax = 170 +  rect_x;
    sRect.i16YMax = 45 + rect_y;
    GrRectFill(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrSlateBlue);
    GrRectDraw(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrBlack);
    GrContextFontSet(pContext, &g_sFontCm14);
    GrStringDrawCentered(pContext, "SYNC", -1, 170+rect_x/2, 45+rect_y/2, 0);
    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, 170+rect_x, 45+rect_y/2, 245, 45+rect_y/2);


    GrContextForegroundSet(pContext, ClrLightGreen);
    sRect.i16XMin = 245;
    sRect.i16YMin = 45;
    sRect.i16XMax = 245 +  rect_x;
    sRect.i16YMax = 45 + rect_y;
    GrRectFill(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrSlateBlue);
    GrRectDraw(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrBlack);
    GrContextFontSet(pContext, &g_sFontCm14);
    GrStringDrawCentered(pContext, "MSG", -1, 245+rect_x/2, 45+rect_y/2, 0);
    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, 245+rect_x/2, 45+rect_y, 245+rect_x/2, 120);



    GrContextForegroundSet(pContext, ClrLightGreen);
    sRect.i16XMin = 20;
    sRect.i16YMin = 120;
    sRect.i16XMax = 20 +  rect_x;
    sRect.i16YMax = 120 + rect_y;
    GrRectFill(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrSlateBlue);
    GrRectDraw(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrBlack);
    GrContextFontSet(pContext, &g_sFontCm14);
    GrStringDrawCentered(pContext, "TX", -1, 20+rect_x/2, 120+rect_y/2, 0);
    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, 20+rect_x, 120+rect_y/2, 95, 120+rect_y/2);


    GrContextForegroundSet(pContext, ClrLightGreen);
    sRect.i16XMin = 95;
    sRect.i16YMin = 120;
    sRect.i16XMax = 95 +  rect_x;
    sRect.i16YMax = 120 + rect_y;
    GrRectFill(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrSlateBlue);
    GrRectDraw(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrBlack);
    GrContextFontSet(pContext, &g_sFontCm14);
    GrStringDrawCentered(pContext, "RRC", -1, 95+rect_x/2, 120+rect_y/2, 0);
    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, 95+rect_x, 120+rect_y/2, 170, 120+rect_y/2);


    GrContextForegroundSet(pContext, ClrLightGreen);
    sRect.i16XMin = 170;
    sRect.i16YMin = 120;
    sRect.i16XMax = 170 +  rect_x;
    sRect.i16YMax = 120 + rect_y;
    GrRectFill(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrSlateBlue);
    GrRectDraw(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrBlack);
    GrContextFontSet(pContext, &g_sFontCm14);
    GrStringDrawCentered(pContext, "BPSK", -1, 170+rect_x/2, 120+rect_y/2, 0);
    GrContextForegroundSet(pContext, ClrBrown);
    GrLineDraw(pContext, 170+rect_x, 120+rect_y/2, 245, 120+rect_y/2);


    GrContextForegroundSet(pContext, ClrLightGreen);
    sRect.i16XMin = 245;
    sRect.i16YMin = 120;
    sRect.i16XMax = 245 +  rect_x;
    sRect.i16YMax = 120 + rect_y;
    GrRectFill(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrSlateBlue);
    GrRectDraw(pContext, &sRect);
    GrContextForegroundSet(pContext, ClrBlack);
    GrContextFontSet(pContext, &g_sFontCm14);
    GrStringDrawCentered(pContext, "MSG", -1, 245+rect_x/2, 120+rect_y/2, 0);

}


//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}




//*****************************************************************************
//
// This function sets up SPI1 to be used as slave in freescale mode.
//
//*****************************************************************************
void
InitSPI1(void)
{
    //
    // The SSI1 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    //
    // For this example SSI1 is used with PortD[3:0].  GPIO port D needs to be
    // enabled so these pins can be used.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for SSI1 functions on port D0, D1, D2 and D3.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    GPIOPinConfigure(GPIO_PD1_SSI1FSS);
    GPIOPinConfigure(GPIO_PD2_SSI1RX);
    GPIOPinConfigure(GPIO_PD3_SSI1TX);

    //
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PD0 - SSI1Tx
    //      PD1 - SSI1Rx
    //      PD2 - SSI1Fss
    //      PD3 - SSI1CLK
    //
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 |
                   GPIO_PIN_0);

    //
    // Configure and enable the SSI1 port for SPI slave mode.
    //
    //SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_2,
    //				   SSI_MODE_SLAVE, 660000, 8);
    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
    				   SSI_MODE_SLAVE, 660000, 16);

    //
    // Enable the SSI1 module.
    //
    SSIEnable(SSI1_BASE);
}





//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


//*****************************************************************************
void	Median_Filter_for_3(float *data_in)
{
	uint8_t	i=0;
	float	data_sort[3];
	float	x=0;
	for (i=0;i<3;i++) data_sort[i]=data_in[i];
	//Bubble sort the data by size :
	if (data_sort[2]<data_sort[1])
	{
		x=data_sort[2];
		data_sort[2] = data_sort[1];
		data_sort[1] = x;
	}

	if (data_sort[1]<data_sort[0])
	{
		x=data_sort[1];
		data_sort[1] = data_sort[0];
		data_sort[0] = x;
	}
	if (data_sort[2]<data_sort[1])
	{
		x=data_sort[2];
		data_sort[2] = data_sort[1];
		data_sort[1] = x;
	}


	data_in[1] = data_sort[1];	//Median filtering on center value

}


//*****************************************************************************
void float_to_int_and_fract(float float_value, int32_t *i32IntegerPart, int32_t *i32FractionPart)
//
// Convert the floats to an integer part and fraction part for easy
// print.
//
{
*i32IntegerPart = (int32_t) float_value;
*i32FractionPart =(int32_t) (float_value * 1000.0f);
*i32FractionPart = *i32FractionPart - (*i32IntegerPart * 1000);
if(*i32FractionPart < 0)
	{
    *i32FractionPart *= -1;
	}
}

void delay_cycles(long cycles)
{
	long k = 0;

	while(k<cycles)
		{
		k = k+1;
		}
	k = k+5;
}

//*****************************************************************************
//
// Main 'C' Language entry point.
//
//*****************************************************************************
int
main(void)

{

    int32_t i32IntegerPart;
    int32_t i32FractionPart;
    uint8_t i8Count=0;

    uint32_t ulindex, ulindex_max = 0;
    int16_t val;
    uint16_t LSB_val, MSB_val;

    float float_val;

    tContext sContext;
    tRectangle sRect;


//***************************************************************************************

#define	pos_AGC		12
#define	pos_Lock	10
#define	pos_Hold	22
#define	pos_EVM		3
#define	pos_Frame	24
#define	pos_CRC		14
#define	pos_BER		6

    g_ulDataRx2[0] = 0b1111000011110000;
    g_ulDataRx2[1] = 0b1100110011110000;
    g_ulDataRx2[2] = 0b1111000011001100;
    g_ulDataRx2[3] = 0b1010101011110000;
    g_ulDataRx2[4] = 0b1010101011001100;
    g_ulDataRx2[5] = 0b1111000011110000;
    g_ulDataRx2[6] = 0b1100110011110000;
    g_ulDataRx2[7] = 0b1111000011001100;
    g_ulDataRx2[8] = 0b1010101011110000;
    g_ulDataRx2[9] = 0b1010101011001100;
    g_ulDataRx2[10] = 0b1111000011110000;
    g_ulDataRx2[11] = 0b1100110011110000;

    g_ulDataRx2[13] = 0b1010101011110000;

    g_ulDataRx2[pos_AGC] =	 0b100001111000011001100;
    g_ulDataRx2[pos_Lock] =  0b1000000000000000;
    g_ulDataRx2[pos_Hold] =  0b0000000000000000;
    g_ulDataRx2[pos_CRC] =   0b0000000000000000;
    g_ulDataRx2[pos_Frame] = 0b1000000000000000;


    LSB_val = 128;
    MSB_val = 0b11110000;
	val = LSB_val + (MSB_val<<8);
	float_val = val;
	float_val =  g_ulDataRx2[0]/(16384.0);
	float_val = float_val/(16384.0);




//***************************************************************************************
    //
    // The FPU should be enabled because some compilers will use floating-
    // point registers, even for non-floating-point code.  If the FPU is not
    // enabled this will cause a fault.  This also ensures that floating-
    // point operations could be added to this application and would work
    // correctly and use the hardware floating-point unit.  Finally, lazy
    // stacking is enabled for interrupt handlers.  This allows floating-
    // point instructions to be used within interrupt handlers, but at the
    // expense of extra stack usage.
    //
    FPUEnable();
    FPULazyStackingEnable();

    MPUEnable(MPU_CONFIG_PRIV_DEFAULT);


    //
    // Set the clocking to run directly from the external crystal/oscillator.
    //
    //  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN );

    //
    // Set the clock to 40Mhz derived from the PLL and the external oscillator
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN );

    //
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init();


    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    //
    // Fill the top 24 rows of the screen with blue to create the banner.
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = Display_Width- 1;
    sRect.i16YMax = 23;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);

    //
    // Put a white box around the banner.
    //
    GrContextForegroundSet(&sContext, ClrWhite);
    GrRectDraw(&sContext, &sRect);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&sContext, &g_sFontCm20);
    GrStringDrawCentered(&sContext, "ZC706 SDR Test v0.1", -1,
                             Display_Width/ 2, 10, 0);


    //
    // Configure and enable uDMA
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlDelay(10);
    uDMAControlBaseSet(&sDMAControlTable[0]);
    uDMAEnable();



    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    //
    // Initialize the touch screen driver and have it route its messages to the
    // widget tree.
    //
    TouchScreenInit();
    TouchScreenCallbackSet(WidgetPointerMessage);

    //
    // Add the title block and the previous and next buttons to the widget
    // tree.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sPrevious);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sTitle);
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sNext);

    //
    // Add the first panel to the widget tree.
    //
    g_ulPanel = 0;
    WidgetAdd(WIDGET_ROOT, (tWidget *)g_psPanels);
    CanvasTextSet(&g_sTitle, g_pcPanelNames[0]);

    //
    // Issue the initial paint request to the widgets.
    //
    WidgetPaint(WIDGET_ROOT);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Display the setup on the console.
    //
    UARTprintf("\n Read data from ZC706 via SPI Interface: \n");
    UARTprintf("  SPI Mode = 0 with SPO=0 and SPH=0\n");
    UARTprintf("  Data: 2 x 16-bit\n");
    UARTprintf("  First 8 bits are identifier, second 8 bit are data MSB/LSB \n");



    //
    // After all the init and config we start blink the LED
    //
    //RGBBlinkRateSet(0.5f);


    //
    // Init SPI1 as slave.
    //
    InitSPI1();

    //
    // Enable RX timeout interrupt.
    //
    SSIIntEnable(SSI1_BASE, SSI_RXTO);
	//
	// Read any residual data from the SSI port.  This makes sure the receive
	// FIFOs are empty, so we don't read any unwanted junk.  This is done here
	// because the SPI SSI mode is full-duplex, which allows you to send and
	// receive at the same time.  The SSIDataGetNonBlocking function returns
	// "true" when data was returned, and "false" when no data was returned.
	// The "non-blocking" function checks if there is any data in the receive
	// FIFO and does not "hang" if there isn't.
	//
    //      while(SSIDataGetNonBlocking(SSI1_BASE, &g_ulDataRx2[0]))
    //      {
    //      }

       //
       // Clear any pending interrupt
       //
       SSIIntClear(SSI1_BASE, SSI_RXTO);

       //
       // Enable the SSI1 interrupts to ARM core.  This has to be done here,
       // otherwise the RX timeout interrupt will fire before all the data has
       // been transferred.  This is specific to this example as both the SSI
       // master and slave are on the same microcontroller.
       //
       IntEnable(INT_SSI1);


       //
       // Set the color to a white approximation.
       //
       g_pui32Colors[RED] = 0x8000;
       g_pui32Colors[BLUE] = 0x8000;
       g_pui32Colors[GREEN] = 0x8000;

       //
       // Initialize RGB driver. Use a default intensity and blink rate.
       //
/*       RGBInit(0);
       RGBColorSet(g_pui32Colors);
       RGBIntensitySet(0.5f);
       RGBEnable();
*/


       //
       //Initial the GPIO for LED
       //
       ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
       ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
       ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);

       //
       // Enable interrupts to the processor.
       //
       ROM_IntMasterEnable();

       // Enable the system ticks at 100 hz. - 100 measurements per second
       //
       uint32_t a = ROM_SysCtlClockGet();		// Current Clock Rate (40MHz)
       ROM_SysTickPeriodSet(a/3);			//Argument has to be <= 16777216
       ROM_SysTickIntEnable();
       ROM_SysTickEnable();


    //
    // Begin the data collection and printing.  Loop Forever.
    //
    while(1)
    {

        //
        // Process any messages in the widget message queue.
        //
        WidgetMessageQueueProcess();

		//SysTick_Semafore = 1;
    	i8Count++;

    	if (SysTick_Semafore==1) 	 // Only print after Sys Tick occured
   		{

        	IntDisable(INT_SSI1);
        	//
        	// Display the data that were read from RX FIFO.
        	//
        	ulindex_max = g_ulSSI1RXFF;
        	g_ulSSI1RXFF = 0;

        	//
        	// Display indication that we have received data.
        	//


    		// Convert the 32bit integer to floats
            // Convert the floats to integer part and fraction part for easy
            // print.

            // AGC Value
    		fAGC =  (g_ulDataRx2[pos_AGC]/32768.0);
    		float_to_int_and_fract(fAGC, &i32IntegerPart, &i32FractionPart);
            usprintf(val_AGC_Str, " %4d.%1d     ", i32IntegerPart, i32FractionPart);

            // Lock Status
    		fLock = (g_ulDataRx2[pos_Lock]/32768.0);
    		float_to_int_and_fract(fLock, &i32IntegerPart, &i32FractionPart);
            usprintf(val_LOCK_Str, " %2d.%02d    ", i32IntegerPart, i32FractionPart);

            // Hold Over
    		fHold = (g_ulDataRx2[pos_Hold]/32768.0);
    		float_to_int_and_fract(fHold, &i32IntegerPart, &i32FractionPart);
            usprintf(val_HOLD_Str, " %2d.%02d    ", i32IntegerPart, i32FractionPart);

            // EVM
    		fEVM = (g_ulDataRx2[pos_EVM]/(16384.0))+my_noise/10;
    		float_to_int_and_fract(fEVM, &i32IntegerPart, &i32FractionPart);
            usprintf(val_EVM_Str, " %2d.%02d    ", i32IntegerPart, i32FractionPart);

            // Frame
            fFrame = (g_ulDataRx2[pos_Frame]/32768.0);
    		float_to_int_and_fract(fFrame, &i32IntegerPart, &i32FractionPart);
            usprintf(val_FRAME_Str, " %2d.%02d    ", i32IntegerPart, i32FractionPart);

            // Message (CRC Error)
            fCRC = (g_ulDataRx2[pos_CRC]/32768.0);
    		float_to_int_and_fract(fCRC, &i32IntegerPart, &i32FractionPart);
            usprintf(val_CRC_Str, " %2d.%01d    ", i32IntegerPart, i32FractionPart);

            // BER
            fBER = (g_ulDataRx2[pos_BER]/(16384.0))+my_noise;
    		float_to_int_and_fract(fBER, &i32IntegerPart, &i32FractionPart);
            usprintf(val_BER_Str, " %3d.%01d    ", i32IntegerPart, i32FractionPart);


            fTemp2 = (g_ulDataRx2[7]/(16384.0))+my_noise;
    		float_to_int_and_fract(fTemp2, &i32IntegerPart, &i32FractionPart);
            usprintf(val8_string, " %2d.%02d    ", i32IntegerPart, i32FractionPart);

            fTempInt = (g_ulDataRx2[8]/(16384.0))+my_noise;
    		float_to_int_and_fract(fTempInt, &i32IntegerPart, &i32FractionPart);
            usprintf(val9_string, " %2d.%02d    ", i32IntegerPart, i32FractionPart);

            //
            // Check which panel.
            //
            switch(g_ulPanel) {

               case Block_Diagram  :
           	        PushButtonFillColorSet( g_psPushButtons2 + 0, ClrLightGreen );
           	        PushButtonFillColorSet( g_psPushButtons2 + 1, ClrLightGreen );
           	        PushButtonFillColorSet( g_psPushButtons2 + 2, ClrLightGreen );
           	        PushButtonFillColorSet( g_psPushButtons2 + 3, ClrLightGreen );
           	        g_ulButtonState=0;
           	        if (fAGC>1000.0)
           	        {
                	    //
                	    // Change color of AGC button to Red
                	    //
                	    PushButtonFillColorSet( g_psPushButtons2 + 0, ClrRed );
               	        g_ulButtonState=1;
           	        }

           	        else if ((fLock<1)|(fHold>0))
           	        {
                	    //
                	    // Change color of PLL button to Red
                	    //
                 	    PushButtonFillColorSet( g_psPushButtons2 + 1, ClrRed );
               	        g_ulButtonState=2;
           	        }

           	        else if (fFrame<1)
           	        {
                	    //
                	    // Change color of SYNC button to Red
                	    //
                	    PushButtonFillColorSet( g_psPushButtons2 + 2, ClrRed );
               	        g_ulButtonState=4;
           	        }

           	        else if (fCRC>0)
           	        {
                	    //
                	    // Change color of MSG button to Red
                	    //
                	    PushButtonFillColorSet( g_psPushButtons2 + 3, ClrRed );
               	        g_ulButtonState=8;
           	        }
                    WidgetPaint((tWidget *)(g_psPushButtons2+0) );
                    WidgetPaint((tWidget *)(g_psPushButtons2+1) );
                    WidgetPaint((tWidget *)(g_psPushButtons2+2) );
                    WidgetPaint((tWidget *)(g_psPushButtons2+3) );
                    break;

               case LED_Panel  :
            	    GrContextForegroundSet(&sContext, ClrLime);
            	    if (fAGC>1000.0)
            	    {
            	    	GrContextForegroundSet(&sContext, ClrRed);
            	    }
            	    GrCircleFill(&sContext, (LED_x0 + 0*LED_xd), (LED_y0 + 0*LED_yd), 10);	//AGC
            	    GrContextForegroundSet(&sContext, ClrLime);

            	    if (fLock<1)
            	    {
            	    	GrContextForegroundSet(&sContext, ClrRed);
            	    }
            	    GrCircleFill(&sContext, (LED_x0 + 1*LED_xd), (LED_y0 + 0*LED_yd), 10);  // LOCK
            	    GrContextForegroundSet(&sContext, ClrLime);

            	    if (fHold>0)
            	    {
            	    	GrContextForegroundSet(&sContext, ClrRed);
            	    }
            	    GrCircleFill(&sContext, (LED_x0 + 2*LED_xd), (LED_y0 + 0*LED_yd), 10);  // Hold Over
            	    GrContextForegroundSet(&sContext, ClrLime);

            	    GrCircleFill(&sContext, (LED_x0 + 3*LED_xd), (LED_y0 + 0*LED_yd), 10);	// EVM

            	    if (fFrame<1)
            	    {
            	    	GrContextForegroundSet(&sContext, ClrRed);
            	    }
            	    GrCircleFill(&sContext, (LED_x0 + 4*LED_xd), (LED_y0 + 0*LED_yd), 10);	// Frame
            	    GrContextForegroundSet(&sContext, ClrLime);

            	    if (fCRC>0)
            	    {
            	    	GrContextForegroundSet(&sContext, ClrRed);
            	    }
            	    GrCircleFill(&sContext, (LED_x0 + 0*LED_xd), (LED_y0 + 1*LED_yd), 10);	// CRC (CRC)
            	    GrContextForegroundSet(&sContext, ClrLime);

            	    GrCircleFill(&sContext, (LED_x0 + 1*LED_xd), (LED_y0 + 1*LED_yd), 10);	// BER
            	    GrCircleFill(&sContext, (LED_x0 + 2*LED_xd), (LED_y0 + 1*LED_yd), 10);
            	    GrCircleFill(&sContext, (LED_x0 + 3*LED_xd), (LED_y0 + 1*LED_yd), 10);
            	    GrCircleFill(&sContext, (LED_x0 + 4*LED_xd), (LED_y0 + 1*LED_yd), 10);

                   break;

               case Data_Panel  :
                   GrContextForegroundSet(&sContext, ClrBlack);
                   GrContextBackgroundSet(&sContext, ClrLime);
                   GrStringDrawRight(&sContext, val_AGC_Str, 8, S11_x+70, S11_y, 1);	// AGC
                   GrStringDrawRight(&sContext, val_LOCK_Str, 8, S12_x+70, S12_y, 1);	// Lock

                   GrStringDrawRight(&sContext, val_HOLD_Str, 8, S21_x+70, S21_y, 1);	// Holder Over
                   GrStringDrawRight(&sContext, val_EVM_Str, 8, S22_x+70, S22_y, 1);	// EVM

                   GrStringDrawRight(&sContext, val_FRAME_Str, 8, S31_x+70, S31_y, 1);	// Frame Status
                   GrStringDrawRight(&sContext, val_CRC_Str, 8, S32_x+70, S32_y, 1);	// Message Status (CRC Error)

                   GrStringDrawRight(&sContext, val_BER_Str, 8, S41_x+70, S41_y, 1);	// BER
                   GrStringDrawRight(&sContext, val8_string, 8, S42_x+70, S42_y, 1);

                   GrStringDrawRight(&sContext, val9_string, 8, S51_x+70, S51_y, 1);
                   GrStringDrawRight(&sContext, val10_string, 8, S52_x+70, S52_y, 1);

                  break;

               case RX_Panel  :
            	   GrContextBackgroundSet(&sContext, ClrLime);
                   GrContextForegroundSet(&sContext, ClrBlack);
           	       if (fAGC>1000.0)
           	       {
           	    	   GrContextBackgroundSet(&sContext, ClrRed);
           	    	   GrStringDraw(&sContext, "Check input to RX1A ", -1, 32, S21_y, 1);
           	       }
           	       else
           	       {
           	        GrStringDraw(&sContext,    "AGC is within range    ", -1, 32, S21_y, 1);
           	       }
                   GrStringDrawRight(&sContext, val_AGC_Str, 10, S11_x+100, S11_y, 1);	// AGC
                   GrContextBackgroundSet(&sContext, ClrLime);

                   GrContextForegroundSet(&sContext, ClrLime);
                   GrLineDraw(&sContext, 0,  iAGC_Curve[0], 1, iAGC_Curve[1]);  // Remove First Line Segment (old)
                   iAGC_Curve[0] = iAGC_Curve[1];	// Move 1 pixel to the left

                   for (i_counter = 1; i_counter < 319; i_counter++)
                   {
                	   GrContextForegroundSet(&sContext, ClrLime);
                	   GrLineDraw(&sContext, i_counter,  iAGC_Curve[i_counter], i_counter+1, iAGC_Curve[i_counter+1]);
                       iAGC_Curve[i_counter] = iAGC_Curve[i_counter+1];	// Move 1 pixel to the left
                	   GrContextForegroundSet(&sContext, ClrBlack);
                	   GrLineDraw(&sContext, i_counter-1,  iAGC_Curve[i_counter-1], i_counter, iAGC_Curve[i_counter]);
                   }
                   iAGC_Curve[319] = (186 - fAGC/40.0);
                   GrContextForegroundSet(&sContext, ClrBlack);
                   GrLineDraw(&sContext, 318,  iAGC_Curve[318], 319, iAGC_Curve[319]);
                   GrContextForegroundSet(&sContext, ClrForestGreen);
                   GrLineDrawH(&sContext, 0, 319, 133);
                   GrLineDrawH(&sContext, 0, 319, 134);
                   GrLineDrawH(&sContext, 0, 319, 188);
                   GrLineDrawH(&sContext, 0, 319, 189);
                   GrContextForegroundSet(&sContext, ClrBlack);

            	  break;

               case PLL_Panel  :
                   GrContextForegroundSet(&sContext, ClrBlack);
                   GrContextBackgroundSet(&sContext, ClrLime);
                   GrStringDrawRight(&sContext, val_LOCK_Str, 8, S11_x+70, S11_y, 1);	// LOCK
                   GrStringDrawRight(&sContext, val_HOLD_Str, 8, S12_x+70, S12_y, 1);	// Hold Over

            	  break;

               case SYNC_Panel  :
                   GrContextForegroundSet(&sContext, ClrBlack);
                   GrContextBackgroundSet(&sContext, ClrLime);
                   if (fFrame<1.0)
                  {
                      GrContextBackgroundSet(&sContext, ClrRed);
                      GrStringDraw(&sContext, "Synchronization Error ", -1, 32, S21_y, 1);
                  }
                  else
                  {
                   GrStringDraw(&sContext,    "Synchronization Ok    ", -1, 32, S21_y, 1);
                  }

                   GrStringDrawRight(&sContext, val_FRAME_Str, 8, S11_x+90, S11_y, 1);	// Frame Status
                   GrContextBackgroundSet(&sContext, ClrLime);

            	  break;

               case RXMSG_Panel  :
                   GrContextForegroundSet(&sContext, ClrBlack);
                   GrContextBackgroundSet(&sContext, ClrLime);
                   if (fCRC>0.0)
                  {
                      GrContextBackgroundSet(&sContext, ClrRed);
                      GrStringDraw(&sContext, "Check Sum Error ", -1, 32, S21_y, 1);
                  }
                  else
                  {
                   GrStringDraw(&sContext,    "Check Sum Ok    ", -1, 32, S21_y, 1);
                  }
                   GrStringDrawRight(&sContext, val_CRC_Str, 8, S11_x+70, S11_y, 1); // Message Status (CRC Error)
                   GrContextBackgroundSet(&sContext, ClrLime);

                   GrContextForegroundSet(&sContext, ClrLime);
                   GrLineDraw(&sContext, 0,  iCRC_Curve[0], 1, iCRC_Curve[1]);  // Remove First Line Segment (old)
                   iCRC_Curve[0] = iCRC_Curve[1];   // Move 1 pixel to the left

                   for (i_counter = 1; i_counter < 319; i_counter++)
                   {
                       GrContextForegroundSet(&sContext, ClrLime);
                       GrLineDraw(&sContext, i_counter,  iCRC_Curve[i_counter], i_counter+1, iCRC_Curve[i_counter+1]);
                       iCRC_Curve[i_counter] = iCRC_Curve[i_counter+1]; // Move 1 pixel to the left
                       GrContextForegroundSet(&sContext, ClrBlack);
                       GrLineDraw(&sContext, i_counter-1,  iCRC_Curve[i_counter-1], i_counter, iCRC_Curve[i_counter]);
                   }
                   iCRC_Curve[319] = (186 - fCRC*50.0);
                   GrContextForegroundSet(&sContext, ClrBlack);
                   GrLineDraw(&sContext, 318,  iCRC_Curve[318], 319, iCRC_Curve[319]);
                   GrContextForegroundSet(&sContext, ClrForestGreen);
                   GrLineDrawH(&sContext, 0, 319, 133);
                   GrLineDrawH(&sContext, 0, 319, 134);
                   GrLineDrawH(&sContext, 0, 319, 188);
                   GrLineDrawH(&sContext, 0, 319, 189);
                   GrContextForegroundSet(&sContext, ClrBlack);

            	  break;

            }


//           UARTprintf("Re(S): %10s   ", val_AGC_Str);
/*            UARTprintf("Vin1: %10s   ", vin1_string);
            UARTprintf("Vin2: %10s   ", vin2_string);
            UARTprintf("Vin3: %10s   ",vin3_string);
            UARTprintf("Isense1: %10s   ", isense1_string );
            UARTprintf("Isense2: %10s   ", isense2_string );
            UARTprintf("Temp1: %10s   ", temp1_string );
            UARTprintf("Temp2: %10s   ", temp2_string );
            UARTprintf("TempInt: %10s   ", tempint_string );
            UARTprintf("\n");
*/

    		SysTick_Semafore =0;
    		i8Count=0;
    	    IntEnable(INT_SSI1);

    	}


    }//while end


    while(1)
    {
    //
    // Process any messages in the widget message queue.
    //
    WidgetMessageQueueProcess();
    }


}

