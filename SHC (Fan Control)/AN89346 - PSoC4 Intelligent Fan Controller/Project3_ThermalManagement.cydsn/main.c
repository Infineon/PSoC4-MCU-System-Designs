/*******************************************************************************
* Project Name:      Project3_ThermalManagement
* Version:           1.1
* Device Used:       PSoC 4 CY8C4245AXI-483
* Software Used:     PSoC Creator v4.0 or higher
* Compiler Used:     ARM GNU CC
* Related Hardware:  CY8CKIT-042 PSoC 4 Pioneer Kit, CY8CKIT-036 PSoC Thermal Management EBK,
*					 CY8CKIT-019 PSoC 4 Shield EBK
********************************************************************************
* Theory of Operation:
*
*  The FanController Component is configured into closed loop speed control. 
*  I2C temperature sensor and a potentiometer (which emulates analog output sensor)are 
*  read in firmware and fan speed is controlled. The fan speed depends on the weighted 
*  average of the temperature readings from the two sensors.
*
*********************************************************************************
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
*/

#include <project.h>
#include "ThermalManager.h"
#include "main.h"

/* Buffer for I2C slave, look at main.h file for the parameters */
uint16 I2CBuffer[10];

/* Cortex M0 systick timer flag. This flag is set every 100ms in SysTick_ISR */
uint8 SysTickFlag=0;

/* This flag is set in the ISR of eoc when a new fan speed data is available */
uint8 eoc_flag=0;

/* This is used to define 500ms delay using systick timer interrupts */
uint8 SoftCounter=0;

/*****************************************************************************
* Function Name: SysTick_ISR
******************************************************************************
* Summary: Interrupt on Cortex-M0 SysTick
*
* Parameters: none; it's an ISR function
*
* Return: none; it's an ISR function
*
*****************************************************************************/
CY_ISR(SysTick_ISR)
{
	/* Set the flag */	
	SysTickFlag = 1;
	
	/* Increment the software counter */
	SoftCounter++;
	
	if(SoftCounter > 4)
	{
		/* 500ms+ delay elapsed */
		
		/* Reset the software counter */
		SoftCounter = 0;
	
		/* Check the fan speed regulation status */
		if(FanController_GetFanSpeedStatus())
		{
			/* Speed regulation fault, blink the LED */
			Pin_SpdFail_Write(~Pin_SpdFail_Read());
		}
		else
			Pin_SpdFail_Write(LED_OFF); //no error, turn off LED
		
		/* Check the fan stall status */	
		if(FanController_GetFanStallStatus())
		{
			/* Fan stuck, blink the LED */
			Pin_FanStall_Write(~Pin_FanStall_Read());
		}
		else
			Pin_FanStall_Write(LED_OFF); //no error, turn off LED
	}		
}


/*****************************************************************************
* Function Name: eoc_ISR
******************************************************************************
* Summary: Interrupt on eoc signal of the FanController Component
*
* Parameters: none; it's an ISR function
*
* Return: none; it's an ISR function
*
*****************************************************************************/
CY_ISR(eoc_ISR)
{
	/* Set the flag */	
	eoc_flag = 1;
}


int main()
{    	
	/* Enable global interrupt */
    CyGlobalIntEnable; 
	
	/* Configure I2C */
	EZI2C_EzI2CSetBuffer1(sizeof(I2CBuffer),READ_WRITE_BOUNDARY, (uint8 *)I2CBuffer);
	EZI2C_Start();
	
	 /* Start the FanController and other Components of the Thermal Manager */
   	ThermalManager_Start();
	
	/* Point the Systick vector to the ISR in this file. This function is called every 100ms */
	CyIntSetSysVector(SYSTICK_INTERRUPT_VECTOR_NUMBER, SysTick_ISR);
	
	/* Set the number of ticks between the interrupts. Ignore the function success/fail return value */
	/* defined in auto-generated core_cm0.h */
    (void)SysTick_Config((uint32)CLOCK_FREQ / (uint32)INTERRUPT_FREQ); 
	
	/* Set eoc_ISR as the interrupt service routine for the eoc interrupt */
	isr_eoc_StartEx(eoc_ISR);
	
    for(;;)
    {		
		/* Executes every 100ms */
		if(SysTickFlag)
		{
			/* Clear flag */
			SysTickFlag = 0;
			
			/* Read the temperature sensors, apply weights and set the fan speed */
			ServiceThermalManager();				
			
			/* Disable the EzI2C interrupt temporarily */
			EZI2C_DisableInt();		
				
			/* Check if there is no active I2C activity */
			if((EZI2C_EzI2CGetActivity() & EZI2C_EZI2C_STATUS_BUSY)==0) 
			{				
				/* Update the I2C buffer */	
				
				/* Get the calculated zone temperature */					
				I2CBuffer[ZONE1_TEMP] = zoneTemperature[ZONE1];
				I2CBuffer[ZONE2_TEMP] = zoneTemperature[ZONE2];
				
				/* Get the sensor readings */
				I2CBuffer[I2C_TEMP] = temperatureReadings[I2C_TEMP_SENSOR];
				I2CBuffer[ANALOG_TEMP] = temperatureReadings[ANALOG_TEMP_SENSOR];				
				
				/* Get the current set fan speed */
				I2CBuffer[FAN1_SET_SPEED] = zoneConfig[ZONE1].fanCurrentSetSpeed;
				I2CBuffer[FAN2_SET_SPEED] = zoneConfig[ZONE2].fanCurrentSetSpeed;
				
				/* Get the PWM Duty cycle of the fans */
				I2CBuffer[FAN1_DUTY_CYCLE] = FanController_GetDutyCycle(FAN_1);
				I2CBuffer[FAN2_DUTY_CYCLE] = FanController_GetDutyCycle(FAN_2);			
				
				/* Check if a new speed value is generated */
				if(eoc_flag)
				{
					/* Reset the flag */
					eoc_flag = 0;		
		
					/* Update the I2C buffer with actual fan speed */
					I2CBuffer[FAN1_ACT_SPEED] = FanController_GetActualSpeed(FAN_1);
					I2CBuffer[FAN2_ACT_SPEED] = FanController_GetActualSpeed(FAN_2);
				}
			}
			
			/* Enable the I2C Interrupt */
			EZI2C_EnableInt();
		}	
    }
}

/* [] END OF FILE */
