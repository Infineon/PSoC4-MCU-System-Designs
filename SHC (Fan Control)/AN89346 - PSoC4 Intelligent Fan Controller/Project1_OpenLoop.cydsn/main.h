/*******************************************************************************
* File Name: main.h  
* Version:   1.0
*
* Description:
*  This file contains the function prototypes and constants used in
*  Project1_OpenLoop.
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

/* Duty cycles expressed in hundreths percent */
#define MIN_DUTY               		0		//0%	
#define MAX_DUTY               		10000	//100%
#define DEFAULT_DUTY_CYCLE     		2500	//25%
#define DUTY_STEP					100		//1%


/* I2C Commands */
#define RESET_SETTINGS_COMMAND 		0x01
#define DUTY_UPDATE_COMMAND   		0x02
#define DUTY_INC_COMMAND   			0x03
#define DUTY_DEC_COMMAND   			0x04


/* Fan Identifier */
#define FAN_1   1
#define FAN_2   2

/* I2C buffer reference */
#define COMMAND_INPUT 		0x00
#define FAN_NUMBER			0x01
#define DUTY_CYCLE_INPUT	0x02
#define FAN1_DUTY_CYCLE		0x03
#define FAN2_DUTY_CYCLE		0x04
#define FAN1_ACT_SPEED		0x05
#define FAN2_ACT_SPEED		0x06


/* read write boundary for I2C slave */
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
#define CLOCK_FREQ     48000000u //system clock frequency - same as CPU clock
#define INTERRUPT_FREQ 10u //interrupts at 10Hz rate (every 100ms)

/* Function to scan the switches and update the PWM duty cycle */
extern void ScanSwitchesAndTakeAction(void);

/* Function to execute the I2C commands */
extern void ReadI2CCommandAndTakeAction(void);

/* [] END OF FILE */
