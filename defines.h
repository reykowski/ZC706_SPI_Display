
#define timeout 4e5

#define Display_Width   320                         //!< Display Width
#define Display_Height  240                         //!< Display Height

///
///         Defines for Data Display Panel
///         Data is organized in 6 rows and 2 columns
///

#define P11_x (Display_Width / 10)+45               //!< Row 1 Column 1 x-value for Text Display
#define P21_x (Display_Width / 10)+45               //!< Row 2 Column 1 x-value for Text Display
#define P31_x (Display_Width / 10)+45               //!< Row 3 Column 1 x-value for Text Display
#define P41_x (Display_Width / 10)+45               //!< Row 4 Column 1 x-value for Text Display
#define P51_x (Display_Width / 10)+45               //!< Row 5 Column 1 x-value for Text Display
#define P61_x (Display_Width / 10)+45               //!< Row 6 Column 1 x-value for Text Display

#define P12_x (Display_Width / 2)+80                //!< Row 1 Column 2 x-value for Text Display
#define P22_x (Display_Width / 2)+80                //!< Row 2 Column 2 x-value for Text Display
#define P32_x (Display_Width / 2)+80                //!< Row 3 Column 2 x-value for Text Display
#define P42_x (Display_Width / 2)+80                //!< Row 4 Column 2 x-value for Text Display
#define P52_x (Display_Width / 2)+80                //!< Row 5 Column 2 x-value for Text Display
#define P62_x (Display_Width / 2)+80                //!< Row 6 Column 2 x-value for Text Display

#define P11_y 30                                    //!< Row 1 Column 1 y-value for Text Display
#define P21_y 60                                    //!< Row 2 Column 1 y-value for Text Display
#define P31_y 90                                    //!< Row 3 Column 1 y-value for Text Display
#define P41_y 120                                   //!< Row 4 Column 1 y-value for Text Display
#define P51_y 150                                   //!< Row 5 Column 1 y-value for Text Display
#define P61_y 180                                   //!< Row 6 Column 1 y-value for Text Display

#define P12_y 30                                    //!< Row 1 Column 2 y-value for Text Display
#define P22_y 60                                    //!< Row 2 Column 2 y-value for Text Display
#define P32_y 90                                    //!< Row 3 Column 2 y-value for Text Display
#define P42_y 120                                   //!< Row 4 Column 2 y-value for Text Display
#define P52_y 150                                   //!< Row 5 Column 2 y-value for Text Display
#define P62_y 180                                   //!< Row 6 Column 2 y-value for Text Display

#define S11_x (Display_Width/ 10)+45                //!< Row 1 Column 1 x-value for Data Display
#define S21_x (Display_Width/ 10)+45                //!< Row 2 Column 1 x-value for Data Display
#define S31_x (Display_Width/ 10)+45                //!< Row 3 Column 1 x-value for Data Display
#define S41_x (Display_Width/ 10)+45                //!< Row 4 Column 1 x-value for Data Display
#define S51_x (Display_Width/ 10)+45                //!< Row 5 Column 1 x-value for Data Display
#define S61_x (Display_Width/ 10)+45                //!< Row 6 Column 1 x-value for Data Display

#define S12_x (Display_Width/ 2)+80                 //!< Row 1 Column 2 x-value for Data Display
#define S22_x (Display_Width/ 2)+80                 //!< Row 2 Column 2 x-value for Data Display
#define S32_x (Display_Width/ 2)+80                 //!< Row 3 Column 2 x-value for Data Display
#define S42_x (Display_Width/ 2)+80                 //!< Row 4 Column 2 x-value for Data Display
#define S52_x (Display_Width/ 2)+80                 //!< Row 5 Column 2 x-value for Data Display
#define S62_x (Display_Width/ 2)+80                 //!< Row 6 Column 2 x-value for Data Display

#define S11_y 30                                    //!< Row 1 Column 1 y-value for Data Display
#define S21_y 60                                    //!< Row 2 Column 1 y-value for Data Display
#define S31_y 90                                    //!< Row 3 Column 1 y-value for Data Display
#define S41_y 120                                   //!< Row 4 Column 1 y-value for Data Display
#define S51_y 150                                   //!< Row 5 Column 1 y-value for Data Display
#define S61_y 180                                   //!< Row 6 Column 1 y-value for Data Display

#define S12_y 30                                    //!< Row 1 Column 2 y-value for Data Display
#define S22_y 60                                    //!< Row 2 Column 2 y-value for Data Display
#define S32_y 90                                    //!< Row 3 Column 2 y-value for Data Display
#define S42_y 120                                   //!< Row 4 Column 2 y-value for Data Display
#define S52_y 150                                   //!< Row 5 Column 2 y-value for Data Display
#define S62_y 180                                   //!< Row 6 Column 2 y-value for Data Display


// Define String names for data display:
#define str_AGC     "AGC: "
#define str_Lock    "Lock: "
#define str_HOLD    "HOLD: "
#define str_EVM     "EVM: "
#define str_FRAME   "FRAME: "
#define str_CRC     "CRC: "
#define str_BER     "BER: "
#define str_NA      "---: "


// Define names for LED display:
#define str_LED1    "AGC"
#define str_LED2    "Lock"
#define str_LED3    "HOLD"
#define str_LED4    "EVM"
#define str_LED5    "FRAME"
#define str_LED6    "CRC"
#define str_LED7    "BER"
#define str_LED8    "---"
#define str_LED9    "---"
#define str_LED10   "---"


// Define LED colors
#define light_is_red        2
#define light_is_yellow     1
#define light_is_green      0

//
// Start placing some Blocks for Block Diagram
//

#define rect_x  45
#define rect_y  45
#define rect_x1 20
#define rect_x2 95
#define rect_x3 170
#define rect_x4 245
#define rect_y1 45
#define rect_y2 120

#define First_Panel    0
#define Block_Diagram  1
#define LED_Panel      2
#define Data_Panel     3
#define RX_Panel       4
#define PLL_Panel      5
#define SYNC_Panel     6
#define RXMSG_Panel    7

    #define LED_x0 40
    #define LED_y0 80
    #define LED_xd 56
    #define LED_yd 90
    #define LED_size 10

#define tick_half 3

#define x_tick1 40
#define x_tick2 80
#define x_tick3 120
#define x_tick4 160
#define x_tick5 200
#define x_tick6 240
#define x_tick7 280

#define pos_Lock_Avg1     20
#define pos_Lock_Avg2     21
#define pos_AGC           12
#define pos_Lock          10
#define pos_Hold          22
#define pos_EVM            3
#define pos_Frame         24
#define pos_CRC           14
#define pos_BER            6


