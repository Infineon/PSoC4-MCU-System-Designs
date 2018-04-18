/*******************************************************************************
* Project Name:      Project1_OpenLoop
* Version:           1.1
* Device Used:       PSoC 4 CY8C4245AXI-483
* Software Used:     PSoC Creator v4.0 or higher
* Compiler Used:     ARM GNU CC
* Related Hardware:  CY8CKIT-042 PSoC 4 Pioneer Kit, CY8CKIT-036 PSoC Thermal Management EBK,
*					 CY8CKIT-019 PSoC 4 Shield EBK
********************************************************************************
* Theory of Operation:
*
*  The FanController Component is configured into Manual fan control mode to
*  drive 2 Fans. Fan Speed is set by feeding the PWM duty cycle from an I2C Master 
*  or using the switches
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
#include "main.h"

/* Buffer for I2C slave, look at main.h file for the parameters */
uint16 I2CBuffer[7];

/* Cortex M0 systick timer flag. This flag is set every 100ms in SysTick_ISR */
uint8 SysTickFlag=0;

/* This flag is set in eoc_ISR when a new fan speed data is available */
uint8 eoc_flag=0;

/*****************************************************************************
* Function Name: SysTick_ISR
******************************************************************************
* Summary: Interrupt from Cortex-M0 timer - SysTick
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
	
	/* Initialize the I2C buffer to default values */
	I2CBuffer[COMMAND_INPUT]=0;	
	I2CBuffer[FAN1_DUTY_CYCLE] = DEFAULT_DUTY_CYCLE;
	I2CBuffer[FAN2_DUTY_CYCLE] = DEFAULT_DUTY_CYCLE;
	
	/* Set the buffer for I2C */
	EZI2C_EzI2CSetBuffer1(sizeof(I2CBuffer), READ_WRITE_BOUNDARY, (uint8 *)I2CBuffer);
	
	/* Enable I2C */
	EZI2C_Start();		
	
	/* Start the fan controller component */
	FanController_Start();	
	
	/* Set the duty cycle of both the fans */
	FanController_SetDutyCycle(FAN_1, I2CBuffer[FAN1_DUTY_CYCLE]);
	FanController_SetDutyCycle(FAN_2, I2CBuffer[FAN2_DUTY_CYCLE]);
	
	/* Point the Systick vector to the ISR in this file */
	CyIntSetSysVector(SYSTICK_INTERRUPT_VECTOR_NUMBER, SysTick_ISR);
	
	/* Set the number of ticks between interrupts. Ignore the function success/fail return value */
    /* defined in auto-generated core_cm0.h */
	(void)SysTick_Config((uint32)CLOCK_FREQ / (uint32)INTERRUPT_FREQ); 
		
	/* Set eoc_ISR as the interrupt service routine for eoc interrupt */
	isr_eoc_StartEx(eoc_ISR);
	
	for(;;)
    {	
		/* Check if new speed value is generated */
		if(eoc_flag)
		{
			/* Reset the flag */
			eoc_flag = 0;
			
			/* Disable I2C slave interrupt */
			EZI2C_DisableInt();
				
			/* Check if there is no active I2C activity */
			if((EZI2C_EzI2CGetActivity() & EZI2C_EZI2C_STATUS_BUSY)== 0)
			{					
				/* Update the I2C buffer with the current speed */
				I2CBuffer[FAN1_ACT_SPEED] =  FanController_GetActualSpeed(FAN_1);
				I2CBuffer[FAN2_ACT_SPEED] =  FanController_GetActualSpeed(FAN_2);						
			}
			
			/* Enable I2C slave interrupt */
			EZI2C_EnableInt();
		}
		
		/* Executes every 100ms */
		if(SysTickFlag)
		{
			/* Clear flag */
			SysTickFlag = 0;
			
			/* Disable I2C slave interrupt */
			EZI2C_DisableInt();
				
			/* Check if there is no active I2C activity */
			if((EZI2C_EzI2CGetActivity() & EZI2C_EZI2C_STATUS_BUSY) == 0)
			{	
				/* Scan the switches SW1 and SW2 and update the PWM duty cycle of both 
				 the fans*/
				ScanSwitchesAndTakeAction();
			}			
			
			/* Enable I2C slave interrupt */
			EZI2C_EnableInt();
		}
		
		/* Disable I2C slave interrupt */
		EZI2C_DisableInt();
			
		/* Check if there is no active I2C activity */
		if((EZI2C_EzI2CGetActivity() & EZI2C_EZI2C_STATUS_BUSY) == 0)
		{	
			/* If there is any pending command, execute it */
			if(I2CBuffer[COMMAND_INPUT]!=0)
				ReadI2CCommandAndTakeAction();
		}			
		
		/* Enable I2C slave interrupt */
		EZI2C_EnableInt();
    }
}

/******************************************************************************* 
* Function Name: ScanSwitchesAndTakeAction()
********************************************************************************
*
* Summary:
* Scans switches SW1 and SW2. If SW1 is pressed, PWM duty cycle for both the fans 
* is increased by DUTY_STEP. If SW2 is pressed, PWM duty cycle for both the fans is 
* decreased by DUTY_STEP.
*
* Parameters: None 
*
* Return: None
*  
******************************************************************************/

void ScanSwitchesAndTakeAction()
{					
	if(Pin_SW1_Read()==0 && Pin_SW2_Read()==0)
	{
		/* Both switches pressed - don't do anything */
	}
	else
	if(Pin_SW1_Read()== 0)
	{			
		/* Switch SW1 is pressed - Increment PWM duty cycle for both the fans */
		
		/* Check if current duty cycle of FAN1 is within the limits */
		if((I2CBuffer[FAN1_DUTY_CYCLE] + DUTY_STEP) < MAX_DUTY)				
			I2CBuffer[FAN1_DUTY_CYCLE] = I2CBuffer[FAN1_DUTY_CYCLE] + DUTY_STEP;
		else
			I2CBuffer[FAN1_DUTY_CYCLE] = MAX_DUTY;
			
		/* Update the PWM duty cycle of FAN1 */	
		FanController_SetDutyCycle(FAN_1, I2CBuffer[FAN1_DUTY_CYCLE] );				
		
		/* Check if current duty cycle of FAN2 is within the limits */
		if((I2CBuffer[FAN2_DUTY_CYCLE] + DUTY_STEP) < MAX_DUTY)				
			I2CBuffer[FAN2_DUTY_CYCLE] = I2CBuffer[FAN2_DUTY_CYCLE] + DUTY_STEP;
		else
			I2CBuffer[FAN2_DUTY_CYCLE] = MAX_DUTY;
		
		/* Update the PWM duty cycle of FAN2 */	
		FanController_SetDutyCycle(FAN_2, I2CBuffer[FAN2_DUTY_CYCLE] );	
		
	}
	else
	if(Pin_SW2_Read() == 0)
	{				
		/* Switch SW2 is pressed - Decrement PWM duty cycle for both the fans */
		
		/* Check if the current duty cycle of FAN1 is within the limits */
		if((I2CBuffer[FAN1_DUTY_CYCLE] - DUTY_STEP) > MIN_DUTY)				
			I2CBuffer[FAN1_DUTY_CYCLE] = I2CBuffer[FAN1_DUTY_CYCLE] - DUTY_STEP;
		else
			I2CBuffer[FAN1_DUTY_CYCLE] = MIN_DUTY;
			
		/* Update the PWM duty cycle of FAN1 */		
		FanController_SetDutyCycle(FAN_1, I2CBuffer[FAN1_DUTY_CYCLE] );	
		
		/* Check if the current duty cycle of FAN2 is within the limits */
		if((I2CBuffer[FAN2_DUTY_CYCLE] - DUTY_STEP) > MIN_DUTY)				
			I2CBuffer[FAN2_DUTY_CYCLE] = I2CBuffer[FAN2_DUTY_CYCLE] - DUTY_STEP;
		else
			I2CBuffer[FAN2_DUTY_CYCLE] = MIN_DUTY;
		
		/* Update the PWM duty cycle of FAN2 */		
		FanController_SetDutyCycle(FAN_2, I2CBuffer[FAN2_DUTY_CYCLE] );						
		
	}			
}

/******************************************************************************* 
* Function Name: ReadI2CCommandAndTakeAction()
********************************************************************************
*
* Summary:
* Executes reset, duty cycle update, increment/decrement command from an external
* I2C master
*
* Parameters: None 
*
* Return: None
*  
******************************************************************************/

void ReadI2CCommandAndTakeAction(void)
{
	/* Flags for updating the PWM duty cycle */
	uint8 UpdateDutyFan1_flag = 0;	
	uint8 UpdateDutyFan2_flag = 0;	
			
	/* Check for valid command */
	switch(I2CBuffer[COMMAND_INPUT])
	{
		/* Reset command */ 
		case RESET_SETTINGS_COMMAND:					
		
			/* Load the default values for the duty cycle */
			I2CBuffer[FAN1_DUTY_CYCLE] = DEFAULT_DUTY_CYCLE;
			I2CBuffer[FAN2_DUTY_CYCLE] = DEFAULT_DUTY_CYCLE;
			
			/* Clear command bytes */
			CLEAR_COMMAND;					
		
			/* Set the flag */
			UpdateDutyFan1_flag = 1;
			UpdateDutyFan2_flag = 1;
			
		break;
			
		/* Duty cycle update command */
		case DUTY_UPDATE_COMMAND:
			
			/* Check whether the duty cycle value provided is within the limits */
			if(I2CBuffer[DUTY_CYCLE_INPUT] < MAX_DUTY)
			{
				/* Check which fan, the command is for */
				if(I2CBuffer[FAN_NUMBER] == FAN_1)
				{
					I2CBuffer[FAN1_DUTY_CYCLE] = I2CBuffer[DUTY_CYCLE_INPUT];
					UpdateDutyFan1_flag = 1;
				}
				else
				if(I2CBuffer[FAN_NUMBER] == FAN_2)
				{
					I2CBuffer[FAN2_DUTY_CYCLE] = I2CBuffer[DUTY_CYCLE_INPUT];							
					UpdateDutyFan2_flag = 1;
				}				
			}						
		
			/* Clear command bytes */
			CLEAR_COMMAND;			
			
			break;				
			
		/* Duty cycle increment command */		
		case DUTY_INC_COMMAND:
				
			/* Check which fan, the command is for */
			if(I2CBuffer[FAN_NUMBER] == FAN_1)
			{
				/* Check if the current duty cycle of FAN1 is within the limits */
				if((I2CBuffer[FAN1_DUTY_CYCLE] + DUTY_STEP) < MAX_DUTY)				
					I2CBuffer[FAN1_DUTY_CYCLE] = I2CBuffer[FAN1_DUTY_CYCLE] + DUTY_STEP;
				else
					I2CBuffer[FAN1_DUTY_CYCLE] = MAX_DUTY;
				
				/* Set the flag */	
				UpdateDutyFan1_flag = 1;					
			}
			else
			if(I2CBuffer[FAN_NUMBER] == FAN_2)
			{
				/* Check if the current duty cycle of FAN2 is within the limits */
				if((I2CBuffer[FAN2_DUTY_CYCLE] + DUTY_STEP) < MAX_DUTY)				
					I2CBuffer[FAN2_DUTY_CYCLE] = I2CBuffer[FAN2_DUTY_CYCLE] + DUTY_STEP;
				else
					I2CBuffer[FAN2_DUTY_CYCLE] = MAX_DUTY;
				
				/* Set the flag */	
				UpdateDutyFan2_flag = 1;							
			}						
			
			/* Clear command bytes */
			CLEAR_COMMAND;
			
			break;
		
		/* Duty cycle decrement command */		
		case DUTY_DEC_COMMAND:
				
			/* Check which fan, the command is for */
			if(I2CBuffer[FAN_NUMBER] == FAN_1)
			{			
				/* Check if the current duty cycle of FAN1 is within the limits */
				if((I2CBuffer[FAN1_DUTY_CYCLE] - DUTY_STEP) > MIN_DUTY)				
					I2CBuffer[FAN1_DUTY_CYCLE] = I2CBuffer[FAN1_DUTY_CYCLE] - DUTY_STEP;
				else
					I2CBuffer[FAN1_DUTY_CYCLE] = MIN_DUTY;
					
				/* Set the flag */	
				UpdateDutyFan1_flag = 1;
			}
			else
			if(I2CBuffer[FAN_NUMBER] == FAN_2)
			{	
				/* Check if the current duty cycle of FAN2 is within the limits */
				if((I2CBuffer[FAN2_DUTY_CYCLE] - DUTY_STEP) > MIN_DUTY)				
					I2CBuffer[FAN2_DUTY_CYCLE] = I2CBuffer[FAN2_DUTY_CYCLE] - DUTY_STEP;
				else
					I2CBuffer[FAN2_DUTY_CYCLE] = MIN_DUTY;		
				
				/* Set the flag */	
				UpdateDutyFan2_flag = 1;	
			}
			
			/* Clear command bytes */
			CLEAR_COMMAND;					
			
			break;	
		
		default:
			/* Clear command bytes */
			CLEAR_COMMAND;	
			break;
				
	}	
	
	/* If the flag is set, then update the duty cycle of FAN1 */
	if(UpdateDutyFan1_flag)
	{
		/* Update the PWM duty cycle of FAN1 */
		FanController_SetDutyCycle(FAN_1, I2CBuffer[FAN1_DUTY_CYCLE] );
		
		/* Clear Flag */
		UpdateDutyFan1_flag = 0;
	}
	
	/* If the flag is set, then update the duty cycle of FAN2 */
	if(UpdateDutyFan2_flag)
	{
		/* Update the PWM duty cycle of FAN2 */
		FanController_SetDutyCycle(FAN_2, I2CBuffer[FAN2_DUTY_CYCLE] );	
		
		/* Clear Flag */
		UpdateDutyFan2_flag = 0;
	}	
}

/* [] END OF FILE */

