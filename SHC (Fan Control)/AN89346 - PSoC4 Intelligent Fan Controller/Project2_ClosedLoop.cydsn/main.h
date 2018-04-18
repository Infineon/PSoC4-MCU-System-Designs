/*******************************************************************************
* File Name: main.h  
* Version:   1.0
*
* Description:
*  This file contains the function prototypes and constants used in
*  the Project2_ClosedLoop
*
********************************************************************************
* Copyright (2012), Cypress Semiconductor Corporation.
********************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("High Risk Product"). By including Cypress's 
* product in a High Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*******************************************************************************/

/* I2C Commands */
#define RESET_SETTINGS_COMMAND 		0x01
#define SPEED_UPDATE_COMMAND   		0x02
#define SPEED_INC_COMMAND			0x03
#define SPEED_DEC_COMMAND			0x04
#define PID_UPDATE					0x05

/* Speed points */
#define MIN_RPM  2500
#define MAX_RPM  9500
#define INIT_RPM 4500
#define RPM_STEP 500

/* Fan Identifier */
#define FAN_1   1
#define FAN_2   2

/* Pointer to parameters in the buffer */
#define COMMAND_INPUT 		0x00
#define FAN_NUMBER 			0x01
#define SPEED_INPUT			0x02
#define FAN1_DUTY_CYCLE		0x03
#define FAN2_DUTY_CYCLE		0x04
#define FAN1_SPD_REQUEST	0x05
#define FAN2_SPD_REQUEST	0x06
#define FAN1_ACT_SPEED		0x07
#define FAN2_ACT_SPEED		0x08

/* Read write boundary for I2C slave */
#define READ_WRITE_BOUNDARY 0x06 // Only 6 bytes of I2CBuffer is writable 

/* Clear the first two variables of I2CBuffer */
#define CLEAR_COMMAND I2CBuffer[COMMAND_INPUT] = 0; I2CBuffer[FAN_NUMBER] = 0

/* Cortex-M0 hard vector */
#define SYSTICK_INTERRUPT_VECTOR_NUMBER 15u 

/* ARM Cortex M0 has a system timer (SysTick) which has a clock input same as the CPU Clock.
* The interrupt frequency from the SysTick timer can be set by configuring its period using 
* the API - SysTick_Config. Use the below macros CLOCK_FREQ and INTERRUPT_FREQ to generate 
* the required period. CLOCK_FREQ/INTERRUPT_FREQ gives the required timer period. In this
* project, the interrupt frequency is set to 10Hz. 
*/
#define CLOCK_FREQ     48000000u //system clock frequency
#define INTERRUPT_FREQ 10u //interrupts every 100ms (10Hz)

/* LED Update */
/* Active low LED connection */
#define LED_ON	0
#define LED_OFF	1

/* Function to scan the switches and update the speed */
extern void ScanSwitchesAndTakeAction(void);

/* Function to execute the I2C commands */
extern void ReadI2CCommandAndTakeAction(void);


/* [] END OF FILE */
