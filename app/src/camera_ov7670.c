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

/*==================[inclusions]=============================================*/

#include "camera_ov7670.h"

/*==================[macros and definitions]=================================*/
#define SCCB_3_PHASE_WRITE    3		//SCCB defines
#define SCCB_2_PHASE_WRITE    2
#define SCCB_ERROR_START     -1
#define SCCB_WRONG_ADDR      -2
#define SCCB_INVALID_DATA    -3

#define RAISING_EDGE 					0	  /* IRQs defines */
#define FALLING_EDGE 					1
#define BOTH_EDGES 						2

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
static void enableGPIOIrq(uint8_t irqChannel, uint8_t port, uint8_t pin, uint8_t edge);
static void ov7670I2cInit(void);
static int8_t SCCBWrite(uint8_t address, uint8_t subAddress, uint8_t data, uint8_t phase);
static int8_t SCCBRead(uint8_t address, uint8_t* output);

static int8_t ov7670ReadRegister(uint8_t reg, uint8_t* output);
static int8_t ov7670WriteRegister(uint8_t reg, uint8_t data);
static void ov7670ResetCamera(void);

static int8_t ov7670EdgeDetect( uint8_t signal, uint8_t *statusSignalPrev);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/* Init i2c */
static void ov7670I2cInit(void)
{

// Init i2c using sapi library
#if( SAPI_MODE == 1 )

			i2cInit( I2C0, OV7670_I2C_RATE);

// Init i2c without sapi library (lpcopen needed)
#else

			Chip_I2C_Init(I2C0);
			Chip_SCU_I2C0PinConfig(I2C0_STANDARD_FAST_MODE);
			Chip_I2C_SetClockRate(I2C0, OV7670_I2C_RATE);

			Chip_I2C_SetMasterEventHandler( I2C0, Chip_I2C_EventHandlerPolling );

#endif

}

// i2c routines without sapi library (lpcopen needed)

#if( SAPI_MODE == 0 )

// 2-phase or 2-phase writing routine SCCB
/*
	+ address: Device Address (7-bit)
	+ subAddress: register Address (8-bit)
	+ data: data to write in the register (8-bit)
	+ phase: SCCB_3_PHASE_WRITE: 3-phase writing.
					 SCCB_2_PHASE_WRITE: 2-phase writing.
*/
static int8_t SCCBWrite(uint8_t address, uint8_t subAddress, uint8_t data, uint8_t phase)
{

    int8_t status = 0;

    LPC_I2C0->CONSET = I2C_I2CONSET_I2EN;		// Enable i2c

    LPC_I2C0->CONCLR = I2C_I2CONCLR_SIC;  // Reset I2C interrupt flag
    LPC_I2C0->CONSET = I2C_I2CONSET_STA;  // Send a START condition
    while(!(LPC_I2C0->CONSET & I2C_I2CONSET_SI )); //Waiting for the I2C interrupt flag to be set

    if (LPC_I2C0->STAT == 0x08) {		// A START condition has been transmitted

        LPC_I2C0->DAT = (address << 1);  // Address + Write flag (0)
        LPC_I2C0->CONCLR = I2C_I2CONCLR_SIC;
        while(!(LPC_I2C0->CONSET & I2C_I2CONSET_SI ));
        LPC_I2C0->CONCLR = I2C_I2CONCLR_STAC;  // Clear START condition

				if (LPC_I2C0->STAT == 0x18) {		// SLA+W has been transmitted; ACK has been received.

	    	    LPC_I2C0->DAT = subAddress;  // Load sub-address
	    	    LPC_I2C0->CONCLR = I2C_I2CONCLR_SIC;
	    	    while(!(LPC_I2C0->CONSET & I2C_I2CONSET_SI ));

					 	if (LPC_I2C0->STAT == 0x28) {		// Data byte in DAT has	been transmitted; ACK has been	received

								status = status + 1;   // increase number of bytes sent.

								if (phase == SCCB_3_PHASE_WRITE) {   //If the user choose a 3-phase write (e.g. the user wants to save a data to the camera)

					 	        LPC_I2C0->DAT = data;  // Send the data to load
					 	        LPC_I2C0->CONCLR = I2C_I2CONCLR_SIC;
					 	        while(!(LPC_I2C0->CONSET & I2C_I2CONSET_SI ));

					 	        if (LPC_I2C0->STAT == 0x28) {

					 	            status = status + 1;

					 	        } else {
					 	            status = SCCB_INVALID_DATA;
										}
								}

					 	} else {
					 	        status = SCCB_INVALID_DATA;
						}

				} else {
	    	    status = SCCB_WRONG_ADDR;
				}

    } else {
        status = SCCB_ERROR_START;
    }

		LPC_I2C0->CONSET = I2C_I2CONSET_STO;  // Send a STOP condition
		LPC_I2C0->CONCLR = I2C_I2CONCLR_SIC;

    return status;

}

// 2-phase reading routine SCCB
/*
	+ address: Device Address (7-bit)
	+ data: Pointer to memory where the reading is going to  be saved
*/
static int8_t SCCBRead(uint8_t address, uint8_t* output)
{

    int8_t status = 0;

    LPC_I2C0->CONSET = I2C_I2CONSET_I2EN;  // Enable i2c

    LPC_I2C0->CONCLR = I2C_I2CONCLR_SIC;  // Reset I2C interrupt flag
    LPC_I2C0->CONSET = I2C_I2CONSET_STA;  // Send a START condition
    while(!(LPC_I2C0->CONSET & I2C_I2CONSET_SI )); //Waiting for the i2c interrupt flag to be set

    if (LPC_I2C0->STAT == 0x08) { // A START condition has been transmitted

        LPC_I2C0->DAT = (address << 1) | 0x01;  // Address + Read flag (1)
        LPC_I2C0->CONCLR = I2C_I2CONCLR_SIC;
        while(!(LPC_I2C0->CONSET & I2C_I2CONSET_SI ));
        LPC_I2C0->CONCLR = I2C_I2CONCLR_STAC;  // Clear START condition

				if (LPC_I2C0->STAT == 0x40) { // SLA+R has been transmitted; ACK has been received.

						// Receive the data from the SCCB device
            LPC_I2C0->CONCLR = I2C_I2CONCLR_SIC;
            while(!(LPC_I2C0->CONSET & I2C_I2CONSET_SI ));

				    *output = LPC_I2C0->DAT;
						status = 0;

				} else {
						status = SCCB_WRONG_ADDR;
				}

    } else {
        status = SCCB_ERROR_START;
    }

		LPC_I2C0->CONSET = I2C_I2CONSET_STO;  // Send a STOP condition
		LPC_I2C0->CONCLR = I2C_I2CONCLR_SIC;  // Reset I2C interrupt flag

    return status;

}

#endif

// OV7670 writing in a register routine
/*
	+ address: Device Address (7-bit)
	+ data: data to write in the register (8-bit)
*/
static int8_t ov7670WriteRegister(uint8_t reg, uint8_t data)
{

// Using the sapi library
#if( SAPI_MODE == 1 )

    uint8_t transmitDataBuffer[2];
    transmitDataBuffer[0] = reg;
    transmitDataBuffer[1] = data;

    int8_t status = i2cWrite(I2C0, OV7670_ADDR, transmitDataBuffer, 2, TRUE);

// Without the sapi library
#else

    int8_t status = SCCBWrite(OV7670_ADDR, reg, data, SCCB_3_PHASE_WRITE);

#endif


		return status;
}

// OV7670 reading in a register routine
/*
	+ address: Device Address (7-bit)
	+ data: Pointer to memory where the reading is going to  be saved
*/
static int8_t ov7670ReadRegister(uint8_t reg, uint8_t* output)
{

    uint8_t readed;

// Using the sapi library
#if( SAPI_MODE == 1 )

	  int8_t status = i2cWrite(I2C0, OV7670_ADDR, &reg, (SCCB_2_PHASE_WRITE - 1), TRUE);
		if (status == 1) {// If the write phase pass
        status = i2cRead(I2C0, OV7670_ADDR, 0, 0, FALSE, &readed, (SCCB_2_PHASE_WRITE - 1), TRUE);
		} else {
			status = SCCB_WRONG_ADDR;
		}
// Without the sapi library
#else

    int8_t status = SCCBWrite(OV7670_ADDR, reg, 0,  SCCB_2_PHASE_WRITE);  // We tell the camera what register we want to read

		if (status == 1) { // If the write phase pass

         status = SCCBRead(OV7670_ADDR, &readed ); // We read the register
		}

#endif

		*output = readed;

		return status;
}

// OV7670 reset routine
static void ov7670ResetCamera(void)
{
    gpioWrite( OV7670_PIN_RESET, ON );
		delay(10);
		gpioWrite( OV7670_PIN_RESET, OFF );
		delay(100);
		gpioWrite( OV7670_PIN_RESET, ON );
		delay(10);

}

// OV7670 edge detect routine
/*
	+ signal: GPIO pin
	+ statusSignalPrev: Pointer to memory where the prevoius status is going to be readed and saved
*/
static int8_t ov7670EdgeDetect( uint8_t signal, uint8_t *statusSignalPrev) {

    uint8_t status = -1;
    uint8_t statusSignal = gpioRead(signal);

    if(statusSignal != *statusSignalPrev) {

      status = statusSignal;   // Si es 0 es flanco de descenso, si es 1 es flanco de ascenso

    }

    *statusSignalPrev = statusSignal;

  return status;

}

/*==================[external functions definition]==========================*/

//ov7670 routine
int8_t ov7670Init(void)
{
    int8_t status = 0;
    uint8_t output = 0;

    // Set IRC clock (12MHz) as output in pin REF_CLK
    Chip_Clock_SetBaseClock(CLK_BASE_OUT, CLKIN_IRC, TRUE, FALSE);
    /* CLK_BASE_OUT -> CLKOUT (REF_CLK)
    CLKIN_IRC -> IRC clock
    AUTOBLOCK -> TRUE (Block clock automatically during frequency change)
    POWERDOWN -> FALSE (Enabled)	*/

    //Configure the pin REF_CLK (P1_19) as CLKOUT
    Chip_SCU_PinMuxSet(XLCKOUT_PORT, XLCKOUT_PIN, (SCU_PINIO_FAST | SCU_MODE_FUNC4));
    /*	SCU_PINIO_FAST | SCU_MODE_FUNC4 ->
    + Disabled pull-down and pull-up resistor at resistor at pad
        + Enabled high-speed slew
        + Enabled Input buffer
        + Disabled input glitch filter
        + Selected pin function 4 (CLKOUT)

		More info at UM10503 - User Manual for LPC43xx ARM Cortex-M4/M0 Dual-Core Microcontroller
		(http://www.keil.com/dd/docs/datashts/nxp/lpc43xx/um10503.pdf)	*/

		//Set Pins
		gpioInit( OV7670_PIN_RESET, GPIO_OUTPUT );
		gpioInit( OV7670_PIN_VS, GPIO_INPUT );
		gpioInit( OV7670_PIN_HS, GPIO_INPUT );
		gpioInit( OV7670_PIN_PCLK, GPIO_INPUT );
		gpioInit( OV7670_PIN_D7, GPIO_INPUT );
		gpioInit( OV7670_PIN_D6, GPIO_INPUT );
		gpioInit( OV7670_PIN_D5, GPIO_INPUT );
		gpioInit( OV7670_PIN_D4, GPIO_INPUT );
		gpioInit( OV7670_PIN_D3, GPIO_INPUT );
		gpioInit( OV7670_PIN_D2, GPIO_INPUT );
		gpioInit( OV7670_PIN_D1, GPIO_INPUT );
		gpioInit( OV7670_PIN_D0, GPIO_INPUT );

		gpioInit( LCD4, GPIO_OUTPUT );
		gpioWrite( LCD4, OFF );

		// Init the i2c communication
		ov7670I2cInit();

		//Reset the Camera
		ov7670ResetCamera();

		// Check if the camera is working
	  status = 	ov7670ReadRegister(OV7670_PID, &output);

		if (output != 0x76) // The Camera PID
		{
			return -1;
		}

		//Reset the Camera
		ov7670ResetCamera();

    // PCLK does not toggle during horizontal blank
    ov7670ReadRegister(OV7670_COM10, &status);
    ov7670WriteRegister(OV7670_COM10, status | 0x20);

		// Set frame-rate as slow mode (approx.125 kHz)
		ov7670ChangeFrameRate(OV7670_SLOW);

		// Set resolution as QCIF
		ov7670ChangeResolution(OV7670_QCIF);

		// Set mode RGB555
		ov7670ChangeColorMode(OV7670_RGB555);

		// Check if the camera is working, again
	  status = 	ov7670ReadRegister(OV7670_PID, &output);

		if (output != 0x76) // The Camera PID
		{
			return -1;
		}

    delay(100);

		// successful init, return 1
		return 1;
}

//Change Frame Rate
void ov7670ChangeFrameRate(uint8_t framerate)
{
    uint8_t output = 0;

    if (framerate == OV7670_SLOW){
        // Set frame-rate PCLK = 187.5 KHz.
        ov7670ReadRegister(OV7670_CLKRC, &output);
        ov7670WriteRegister(OV7670_CLKRC, (output & CLKRC_ENA_PREESCALER) | CLKRC_SET_PREESCALER_64 ); //Divide the clock by 64 (12MHz/3)
        ov7670ReadRegister(OV7670_DBLV, &output);
        ov7670WriteRegister(OV7670_DBLV, output & 0x3F); //Bypass the PLL

    } else {
			// Set frame-rate PCLK = 12 MHz.
				ov7670ReadRegister(OV7670_CLKRC, &output);
				ov7670WriteRegister(OV7670_CLKRC, output & CLKRC_DIS_PREESCALER); //Disable the preescaler
        ov7670ReadRegister(OV7670_DBLV, &output);
        ov7670WriteRegister(OV7670_DBLV, output & 0x3F); //Bypass the PLL
    }

}

//Set resolution
// TODO: Implement manual scaling
void ov7670ChangeResolution(uint8_t resolution)
{
	   uint8_t output = 0;

	   switch(resolution) {

	   	   case OV7670_QCIF:
	   	   	   ov7670ReadRegister(OV7670_COM3, &output);
						 ov7670WriteRegister(OV7670_COM3, output | 0x08);  //Enable format scaling

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, (output & 0xC7) | 0x08);  //Select QCIF format
	   	   	   break;

	   	   case OV7670_QVGA:
						 ov7670ReadRegister(OV7670_COM3, &output);
						 ov7670WriteRegister(OV7670_COM3, output | 0x08);  //Enable format scaling

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, (output & 0xC7) | 0x10);  //Select QVGA format
	   	   	   break;

				 case OV7670_CIF:
						 ov7670ReadRegister(OV7670_COM3, &output);
						 ov7670WriteRegister(OV7670_COM3, output | 0x08);  //Enable format scaling

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, (output & 0xC7) | 0x20);  //Select CIF format
	   	   	   break;

	   	   default:		//VGA
						 ov7670ReadRegister(OV7670_COM3, &output);
						 ov7670WriteRegister(OV7670_COM3, output & 0xF7);  //Disable format scaling

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, output & 0xC7);  //Do not select a format
	   	   	   break;

	   }

}

//Set color mode
void ov7670ChangeColorMode(uint8_t color)
{
	   uint8_t output = 0;

	   switch(color) {

	   	   case OV7670_COLOR_RAWRGB_BLUE: //Raw Bayer Blue (8-bit)

	   	   	   ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, (output & 0xFB)  | 0x01);  //Set Raw RGB

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, output & 0xEF);  				//Set Raw RGB

	   	   	   break;

				 case OV7670_COLOR_RAWRGB_RED: //Raw Bayer Red (8-bit)

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, (output & 0xFB)  | 0x01);  //Set Raw RGB

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, output & 0xEF);  				//Set Raw RGB

	   	   	   break;

				 case OV7670_COLOR_RAWRGB_GREEN: //Raw Bayer Green (8-bit)

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, (output & 0xFB)  | 0x01);  //Set Raw RGB

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, output & 0xEF);  				//Set Raw RGB

	   	   	   break;

				 case OV7670_COLOR_BAYERRGB_BLUE: //Proccesed Bayer Blue (8-bit)

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, output | 0x05);  //Set Proccesed Bayer RGB

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, output & 0xEF); //Set Proccesed Bayer RGB

	   	   	   break;

				 case OV7670_COLOR_BAYERRGB_RED: //Proccesed Bayer Red (8-bit)

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, output | 0x05);  //Set Proccesed Bayer RGB

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, output & 0xEF); //Set Proccesed Bayer RGB

	   	   	   break;

				 case OV7670_COLOR_BAYERRGB_GREEN: //Proccesed Bayer Green (8-bit)

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, output | 0x05);  //Set Proccesed Bayer RGB

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, output & 0xEF); //Set Proccesed Bayer RGB

	   	   	   break;

				 case OV7670_COLOR_YUV: //YUV color model (8-bit Y and 8-bit U)

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, output & 0xFA);  		//Set YUV color model

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, output & 0xEF);  //Set YUV color model

	   	   	   break;

				 case OV7670_COLOR_YCBCR: //YCbCr color model (8-bit U and 8-bit V)

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, output & 0xFA);  		//Set YUV color model

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, output & 0xEF);  //Set YUV color model

	   	   	   break;

				 case OV7670_COLOR_GR: //GRB (8-bit Green and 8-bit Red)

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, (output & 0xFE)  | 0x04);  //Set GRB

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, output & 0xEF);  				//Set GRB

	   	   	   break;

				 case OV7670_COLOR_GB: //GRB (8-bit Green and 8-bit Blue)

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, (output & 0xFE)  | 0x04);  //Set GRB

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, output & 0xEF);  				//Set GRB

	   	   	   break;

				 case OV7670_RGB565: //5-bit Red, 6-bit Green and 5-bit Blue

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, (output & 0xFE)  | 0x04);  //Set RGB

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, (output & 0xDF)  | 0x10);  	//Set RGB565

	   	   	   break;

	   	   default:		//5-bit Red, 5-bit Green and 5-bit Blue

						 ov7670ReadRegister(OV7670_COM7, &output);
						 ov7670WriteRegister(OV7670_COM7, (output & 0xFE)  | 0x04);  //Set RGB

						 ov7670ReadRegister(OV7670_COM15, &output);
						 ov7670WriteRegister(OV7670_COM15, output | 0x30);  	//Set RGB565

	   	   	   break;

	   }

}

// Take a photo
void ov7670TakePhoto(uint8_t lineStart, uint8_t lineEnd, uint8_t evenLine, uint8_t* output, uint16_t* bytePerLine)
{

     Chip_GPIO_SetPortMask(LPC_GPIO_PORT, 2, 0x80);

		 uint32_t buffer_count = 0;   		// Buffer counter
     uint32_t line_count = 0;	  			// line counter
     uint16_t pixel_line_count = 0;		// Pixel per line counter
     uint8_t even = 1;								// Detect if the byte is even or odd

		 uint8_t d7_d0[2];								// Data output

		 // Falling edge flags
     int8_t fallEdgeVS = -1;

		 // Rising edge flags
     int8_t riseEdgeVS = -1;
     int8_t riseEdgeHS = -1;
     int8_t riseEdgePCLK = -1;

		 // Signal prevoius state
     uint8_t HSPrev = 0;
     uint8_t PCLKPrev = 0;
     uint8_t VSPrev = gpioRead(OV7670_PIN_VS);

		 // Wait for a rising edge in VS to sycnronize
     while(riseEdgeVS != 1) {
       riseEdgeVS = ov7670EdgeDetect(OV7670_PIN_VS, &VSPrev);
     }

     VSPrev = gpioRead(OV7670_PIN_VS);
     riseEdgeVS = -1;

		 // Wait for a fall edge in VS to start the image scanning
     while(fallEdgeVS != 0) {
       fallEdgeVS = ov7670EdgeDetect(OV7670_PIN_VS, &VSPrev);
     }

     fallEdgeVS = -1;

     HSPrev = gpioRead(OV7670_PIN_HS);
     VSPrev = gpioRead(OV7670_PIN_VS);

		 // A rise edge in VS means the end of the image
     while(riseEdgeVS != 1) {

       riseEdgeHS = ov7670EdgeDetect(OV7670_PIN_HS, &HSPrev);

			 // Wait for a rise edge in HS to start a line scanning
       if(riseEdgeHS == 1){

				 // If we are between the specified lines
         if (line_count >= lineStart) {

           if(line_count == lineEnd) {
             break;
           }

           HSPrev = gpioRead(OV7670_PIN_HS);
           PCLKPrev = gpioRead(OV7670_PIN_PCLK);

					 // While HS is ON
           while(HSPrev == 1){


             riseEdgePCLK = ov7670EdgeDetect(OV7670_PIN_PCLK, &PCLKPrev);

						  // We wait for a rising edge in PCLK
             if(riseEdgePCLK == 1){

               if (evenLine == FALSE) {  // If we set to save odd bytes number

                 if (even == 0) {		// If the bytes number is odd
                   d7_d0[0] = Chip_GPIO_GetMaskedPortValue(LPC_GPIO_PORT, 2);
                   d7_d0[1] = (LPC_GPIO_PORT->W[2][8]) & 0x80;                 // We read the GPIO port
                   output[buffer_count] = d7_d0[0] | d7_d0[1];								 // And save it in the output
                   buffer_count = buffer_count + 1;
                   pixel_line_count = pixel_line_count + 1;
                   even = 1;
                 } else
                 {
                   even = 0;
                 }

               } else { // If we set to save even bytes number

                 if (even == 1) {  	// If the bytes number is even

                   d7_d0[0] = Chip_GPIO_GetMaskedPortValue(LPC_GPIO_PORT, 2);  // We read the GPIO port
                   d7_d0[1] = (LPC_GPIO_PORT->W[2][8]) & 0x80;
                   output[buffer_count] = d7_d0[0] | d7_d0[1];                 // And save it in the output
                   buffer_count = buffer_count + 1;
                   pixel_line_count = pixel_line_count + 1;
                   even = 0;

                 } else
                 {
                   even = 1;
                 }


               }

               riseEdgePCLK = -1;

             }

             HSPrev = gpioRead(OV7670_PIN_HS);

          }

          bytePerLine[line_count - lineStart] = pixel_line_count;   // Save the number of bytes readed per line (We needed this information if a byte is lost)
          pixel_line_count = 0;

        }

         line_count = line_count + 1;
       }

       riseEdgeVS = ov7670EdgeDetect(OV7670_PIN_VS, &VSPrev);

     }
}

/*==================[end of file]============================================*/
