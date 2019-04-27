/* Copyright 2019, Facundo Adrian Lucianna <facundolucianna@gmail.com>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Date: 2019-04-06 */

#ifndef _CAMERA_OV7670_H_
#define _CAMERA_OV7670_H_

/*PinOut
    CAMERA  |     EDU-CIAA

    3V3	  	<-		+3.3V
    GND	  	<-		GND
    SCL	  	<-		I2C_SCL
    SDA	  	<->		I2C_SDA
    VS	  	->		GPIO3
    HS		  ->		GPIO1
    PCLK	 	->		GPIO5
    MCLK		<-		REF_CLK
    D7	  	->		GPIO8
    D6		  ->		LCD3
    D5	  	->		LCD2
    D4	  	->		LCD1
    D3	  	->		T_FIL3
    D2	  	->		T_FIL2
    D1	  	->		T_FIL1
    D0	   	->		T_FIL0
    RESET		<-		+3.3V
    PWDN		<-		GND
*/

/*==================[inclusions]=============================================*/

#include "sapi.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
// Use the i2c sapi library (1) or use a native library (0)
#define SAPI_MODE                 1

// Pins name
#define OV7670_PIN_VS             GPIO3    // Port 3 pin 15
#define OV7670_PIN_HS             GPIO1    // Port 3 pin 03
#define OV7670_PIN_RESET          GPIO7    // Port 3 pin 07
#define OV7670_PIN_PCLK           GPIO5    // Port 3 pin 05

#define OV7670_PIN_D7             GPIO8    // Port 2 pin 08
#define OV7670_PIN_D6             LCD3     // Port 2 pin 06
#define OV7670_PIN_D5             LCD2     // Port 2 pin 05
#define OV7670_PIN_D4             LCD1     // Port 2 pin 03
#define OV7670_PIN_D3             T_FIL3   // Port 2 pin 02
#define OV7670_PIN_D2             T_FIL2   // Port 2 pin 02
#define OV7670_PIN_D1             T_FIL1   // Port 2 pin 01
#define OV7670_PIN_D0             T_FIL0   // Port 2 pin 00

//Pins for interruption (Not used)
#define OV7670_PORT_GPIO_VS       3
#define OV7670_PORT_GPIO_HS       3
#define OV7670_PORT_GPIO_PCLK     3

#define OV7670_PIN_GPIO_VS        15
#define OV7670_PIN_GPIO_HS        3
#define OV7670_PIN_GPIO_PCLK      5

#define OV7670_IRQ_VS             0
#define OV7670_IRQ_HS             1
#define OV7670_IRQ_PCLK           2

// I2C baudrate
#define OV7670_I2C_RATE           100000 // 100 kHz

 // 7-bit i2c camera address
#define OV7670_ADDR               0x21

// Frame per second
#define OV7670_15FPS              15
#define OV7670_SLOW               1

// Resolution
#define OV7670_VGA                0    //VGA (640 x 480)
#define OV7670_CIF                1    //QVGA (320 x 240)
#define OV7670_QVGA               2    //CIF (352 x 240)
#define OV7670_QCIF               3    //QCIF (176 x 144)

//Color Mode
#define OV7670_COLOR_RAWRGB_BLUE           0    //Raw Bayer Blue (8-bit)
#define OV7670_COLOR_RAWRGB_RED            1    //Raw Bayer Red (8-bit)
#define OV7670_COLOR_RAWRGB_GREEN          2    //Raw Bayer Green (8-bit)
#define OV7670_COLOR_BAYERRGB_BLUE         3    //Proccesed Bayer Blue (8-bit)
#define OV7670_COLOR_BAYERRGB_RED          4    //Proccesed Bayer Red (8-bit)
#define OV7670_COLOR_BAYERRGB_GREEN        5    //Proccesed Bayer Green (8-bit)
#define OV7670_COLOR_YUV                   6    //YUV color model (8-bit Y and 8-bit U)
#define OV7670_COLOR_YCBCR                 7    //YCbCr color model (8-bit U and 8-bit V)
#define OV7670_COLOR_GR                    8    //GRB (8-bit Green and 8-bit Red)
#define OV7670_COLOR_GB                    9    //GRB (8-bit Green and 8-bit Blue)
#define OV7670_RGB565                      10    //5-bit Red, 6-bit Green and 5-bit Blue
#define OV7670_RGB555                      11    //5-bit Red, 5-bit Green and 5-bit Blue

// OV7670 registers (based from https://github.com/desaster/ov7670test)
#define OV7670_GAIN          0x00    // Gain lower 8 bits (rest in vref)
#define OV7670_BLUE          0x01    // blue gain
#define OV7670_RED           0x02    // red gain
#define OV7670_VREF          0x03    // Pieces of GAIN, VSTART, VSTOP
#define OV7670_COM1          0x04    // Control 1
#define OV7670_BAVE          0x05    // U/B Average level
#define OV7670_GBAVE         0x06    // Y/Gb Average level
#define OV7670_AECHH         0x07    // AEC MS 5 bits
#define OV7670_RAVE          0x08    // V/R Average level
#define OV7670_COM2          0x09    // Control 2
#define OV7670_PID           0x0A    // Product ID MSB
#define OV7670_VER           0x0B    // Product ID LSB
#define OV7670_COM3          0x0C    // Control 3
#define OV7670_COM4          0x0D    // Control 4
#define OV7670_COM5          0x0E    // All "reserved"
#define OV7670_COM6          0x0F    // Control 6
#define OV7670_AECH          0x10    // More bits of AEC value
#define OV7670_CLKRC         0x11    // Clock control
#define OV7670_COM7          0x12    // Control 7
#define OV7670_COM8          0x13    // Control 8
#define OV7670_COM9          0x14    // Control 9  - gain ceiling
#define OV7670_COM10         0x15    // Control 10
#define OV7670_HSTART        0x17    // Horiz start high bits
#define OV7670_HSTOP         0x18    // Horiz stop high bits
#define OV7670_VSTART        0x19    // Vert start high bits
#define OV7670_VSTOP         0x1A    // Vert stop high bits
#define OV7670_PSHFT         0x1B    // Pixel delay after HREF
#define OV7670_MIDH          0x1C    // Manuf. ID high
#define OV7670_MIDL          0x1D    // Manuf. ID low
#define OV7670_MVFP          0x1E   // Mirror / vflip
#define OV7670_AEW           0x24    // AGC upper limit
#define OV7670_AEB           0x25    // AGC lower limit
#define OV7670_VPT           0x26    // AGC/AEC fast mode op region
#define OV7670_HSYST         0x30    // HSYNC rising edge delay
#define OV7670_HSYEN         0x31    // HSYNC falling edge delay
#define OV7670_HREF          0x32    // HREF pieces
#define OV7670_TSLB          0x3A    // lots of stuff
#define OV7670_COM11         0x3B    // Control 11
#define OV7670_COM12         0x3C    // Control 12
#define OV7670_COM13         0x3D    // Control 13
#define OV7670_COM14         0x3E    // Control 14
#define OV7670_EDGE          0x3F    // Edge enhancement factor
#define OV7670_COM15         0x40    // Control 15
#define OV7670_COM16         0x41    // Control 16
#define OV7670_COM17         0x42    // Control 17
#define OV7670_CMATRIX_BASE  0x4F
#define OV7670_BRIGHT        0x55    // Brightness
#define OV7670_CONTRAS       0x56    // Contrast control
#define OV7670_CMATRIX_SIGN  0x58
#define OV7670_GFIX          0x69    // Fix gain control
#define OV7670_DBLV          0x6B    // Fix gain control

#define OV7670_REG76         0x76    // OV's name
#define OV7670_RGB444        0x8C    // RGB 444 control
#define OV7670_HAECC1        0x9F    // Hist AEC/AGC control 1
#define OV7670_HAECC2        0xA0    // Hist AEC/AGC control 2
#define OV7670_BD50MAX       0xA5    // 50hz banding step limit
#define OV7670_HAECC3        0xA6    // Hist AEC/AGC control 3
#define OV7670_HAECC4        0xA7    // Hist AEC/AGC control 4
#define OV7670_HAECC5        0xA8    // Hist AEC/AGC control 5
#define OV7670_HAECC6        0xA9    // Hist AEC/AGC control 6
#define OV7670_HAECC7        0xAA    // Hist AEC/AGC control 7
#define OV7670_BD60MAX       0xAB    // 60hz banding step limit

// Registers configurations
#define CLKRC_DIS_PREESCALER      0xC0    // Disable prescaler on input clock (Well, really it enables the PLL and set it as zero)
#define CLKRC_ENA_PREESCALER      0x80    // Enable prescaler on input clock
#define CLKRC_SET_PREESCALER_3    0x02    // Divide by 3

#define CLKRC_SET_PREESCALER_64   0x3F


#define COM1_CCIR656         0x40    // CCIR656 enable
#define COM2_SSLEEP          0x10    // Soft sleep mode
#define COM3_SWAP            0x40    // Byte swap
#define COM3_SCALEEN         0x08    // Enable scaling
#define COM3_DCWEN           0x04    // Enable downsamp/crop/window
#define CLK_EXT              0x40    // Use external clock directly
#define CLK_SCALE            0x3F    // Mask for internal clock scale
#define COM7_RESET           0x80    // Register reset
#define COM7_FMT_MASK        0x38
#define COM7_FMT_VGA         0x00
#define COM7_FMT_CIF         0x20    // CIF format
#define COM7_FMT_QVGA        0x10    // QVGA format
#define COM7_FMT_QCIF        0x08    // QCIF format
#define COM7_RGB             0x04    // bits 0 and 2 - RGB format
#define COM7_YUV             0x00    // YUV
#define COM7_BAYER           0x01    // Bayer format
#define COM7_PBAYER          0x05    // "Processed bayer"
#define COM8_FASTAEC         0x80    // Enable fast AGC/AEC
#define COM8_AECSTEP         0x40    // Unlimited AEC step size
#define COM8_BFILT           0x20    // Band filter enable
#define COM8_AGC             0x04    // Auto gain enable
#define COM8_AWB             0x02    // White balance enable
#define COM8_AEC             0x01    // Auto exposure enable
#define COM10_HSYNC          0x40    // HSYNC instead of HREF
#define COM10_PCLK_HB        0x20    // Suppress PCLK on horiz blank
#define COM10_HREF_REV       0x08    // Reverse HREF
#define COM10_VS_LEAD        0x04    // VSYNC on clock leading edge
#define COM10_VS_NEG         0x02    // VSYNC negative
#define COM10_HS_NEG         0x01    // HSYNC negative
#define MVFP_MIRROR          0x20    // Mirror image
#define MVFP_FLIP            0x10    // Vertical flip
#define TSLB_YLAST           0x04    // UYVY or VYUY - see com13
#define COM11_NIGHT          0x80    // NIght mode enable
#define COM11_NMFR           0x60    // Two bit NM frame rate
#define COM11_HZAUTO         0x10    // Auto detect 50/60 Hz
#define COM11_50HZ           0x08    // Manual 50Hz select
#define COM11_EXP            0x02
#define COM12_HREF           0x80    // HREF always
#define COM13_GAMMA          0x80    // Gamma enable
#define COM13_UVSAT          0x40    // UV saturation auto adjustment
#define COM13_UVSWAP         0x01    // V before U - w/TSLB
#define COM14_DCWEN          0x10    // DCW/PCLK-scale enable
#define COM15_R10F0          0x00    // Data range 10 to F0
#define COM15_R01FE          0x80    //            01 to FE
#define COM15_R00FF          0xc0    //            00 to FF
#define COM15_RGB565         0x10    // RGB565 output
#define COM15_RGB555         0x30    // RGB555 output
#define COM16_AWBGAIN        0x08    // AWB gain enable
#define COM17_AECWIN         0xc0    // AEC window - must match COM4
#define COM17_CBAR           0x08    // DSP Color bar
#define CMATRIX_LEN          0x06
#define R76_BLKPCOR          0x80    // Black pixel correction enable
#define R76_WHTPCOR          0x40    // White pixel correction enable
#define R444_ENABLE          0x02    // Turn on RGB444, overrides 5x5
#define R444_RGBX            0x01    // Empty nibble at end

#define XLCKOUT_PORT         0x1    // Port of the pin REF_CLK (XCLK camera input)
#define XLCKOUT_PIN          19    // Pin number of the pin REF_CLK (XCLK camera input)

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

// Initialize ov7670
int8_t ov7670Init(void);

//Change Frame Rate
/*
	+ framerate: frame rate desired (two modes were implemented)
    OV7670_15FPS (15 fps at 640x480 res)
    OV7670_SLOW  (pclk approx 125 kHz)
*/
void ov7670ChangeFrameRate(uint8_t framerate);

//Set resolution
/*
	+ resolution: Camera resolution
    OV7670_VGA  (640 x 480)
    OV7670_CIF  (320 x 240)
    OV7670_QVGA (352 x 240)
    OV7670_QCIF (176 x 144)
*/
void ov7670ChangeResolution(uint8_t resolution);

//Set color mode
/*
	+ color: Color mode
    Most important:
    OV7670_COLOR_YUV  - YUV color model (8-bit Y and 8-bit U)
    OV7670_RGB565     - 5-bit Red, 6-bit Green and 5-bit Blue
    OV7670_RGB555     - 5-bit Red, 5-bit Green and 5-bit Blue
*/
void ov7670ChangeColorMode(uint8_t color);

//Set color mode
/*
	+ lineStart: Starting line to save (for example, QCIF has 144 lines, read a image from the 20th line)
	+ linEnd:   Ending line to save (for example, QCIF has 144 lines, read a image to the 60th line)
  + evenLine: Indicate if we want to save the odd bytes o even byte from a line
  + output:   Pointer to the image buffer
  + bytePerLine:   Pointer to the buffer where it saved the number of byte per line
*/
void ov7670TakePhoto(uint8_t lineStart, uint8_t linEnd, uint8_t evenLine, uint8_t* output, uint16_t* bytePerLine);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _CAMERA_OV7670_H_ */
