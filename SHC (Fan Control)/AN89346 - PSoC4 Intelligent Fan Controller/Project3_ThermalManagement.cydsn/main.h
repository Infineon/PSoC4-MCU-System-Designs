/*******************************************************************************
* File Name: main.h  
* Version:   1.0
*
* Description:
*  This file contains the function prototypes and constants used in
*  the Project3_ThermalManagement.
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

/* I2C buffer contents */
#define ZONE1_TEMP 		0x00
#define ZONE2_TEMP		0x01
#define I2C_TEMP		0x02
#define ANALOG_TEMP		0x03
#define FAN1_SET_SPEED	0x04
#define FAN2_SET_SPEED	0x05	
#define FAN1_ACT_SPEED	0x06
#define FAN2_ACT_SPEED	0x07
#define FAN1_DUTY_CYCLE	0x08
#define FAN2_DUTY_CYCLE	0x09

/* Read write boundary for I2C slave */
#define READ_WRITE_BOUNDARY 0x00 //all bytes are only readable 

/* LED Update */
/* Active low LED connection */
#define LED_ON	0
#define LED_OFF	1

/* Cortex-M0 hard vector */
#define SYSTICK_INTERRUPT_VECTOR_NUMBER 15u 

/* ARM Cortex M0 has a system timer (SysTick) which has a clock input same as the CPU Clock.
* The interrupt frequency from the SysTick timer can be set by configuring its period using 
* the API - SysTick_Config. Use the below macros CLOCK_FREQ and INTERRUPT_FREQ to generate 
* the required period. CLOCK_FREQ/INTERRUPT_FREQ gives the required timer period. In this
* project, the interrupt frequency is set to 10Hz. 
*/
#define CLOCK_FREQ     48000000u //system clock frequency
#define INTERRUPT_FREQ 10u //interrupts every 100ms

/* [] END OF FILE */
