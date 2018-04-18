/*******************************************************************************
* Project Name:      Project2_ClosedLoop
* Version:           1.1
* Device Used:       PSoC 4 CY8C4245AXI-483
* Software Used:     PSoC Creator v4.0 or higher
* Compiler Used:     ARM GNU CC
* Related Hardware:  CY8CKIT-042 PSoC 4 Pioneer Kit, CY8CKIT-036 PSoC Thermal Management EBK,
*					 CY8CKIT-019 PSoC 4 Shield EBK
********************************************************************************
* Theory of Operation:
*
*  The FanController Component is configured in closed loop speed control mode to drive
*  two fans. The Fan speed is set from an I2C master or using the switches.
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
uint16 I2CBuffer[9];

/* Cortex M0 systick timer flag. This flag is set every 100ms in SysTick_ISR */
uint8 SysTickFlag=0;

/* This flag is set in eoc_ISR when a new fan speed data is available */
uint8 eoc_flag=0;

/* This is used to define 500ms delay using systick timer interrupts */
uint8 SoftCounter=0;

/*****************************************************************************
* Function Name: SysTick_ISR
******************************************************************************
* Summary: Interrupt on Cortex-M0 SysTick. Sets the SysTickFlag flag and checks 
*          for fan faults
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
			/* Fan stalled, blink the LED */
			Pin_FanStall_Write(~Pin_FanStall_Read());
		}
		else
			Pin_FanStall_Write(LED_OFF); //no error, turn off LED
	}		
}

/*****************************************************************************
* Function Name: eoc_ISR
******************************************************************************
* Summary: Interrupt on eoc signal of the FanController Component. Sets the 
*          flag eoc_flag.
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
	
	/* Initialize I2C buffer to default values */
	I2CBuffer[COMMAND_INPUT]=0;	
	I2CBuffer[SPEED_INPUT] = INIT_RPM;
	I2CBuffer[FAN1_SPD_REQUEST] = INIT_RPM;
	I2CBuffer[FAN2_SPD_REQUEST] = INIT_RPM;
	
	/* Set the buffer for I2C */
	EZI2C_EzI2CSetBuffer1(sizeof(I2CBuffer), READ_WRITE_BOUNDARY, (uint8 *)(&I2CBuffer[COMMAND_INPUT]));
	
	/* Enable I2C */
	EZI2C_Start();
	
	/* Start the fan controller component */
	FanController_Start();	
	
	/* Set the speed of both fans */
	FanController_SetDesiredSpeed(FAN_1,I2CBuffer[FAN1_SPD_REQUEST]);
	FanController_SetDesiredSpeed(FAN_2,I2CBuffer[FAN2_SPD_REQUEST]);	
	
	/* Point the Systick vector to the ISR in this file. This function is 
	called every 100ms */
	CyIntSetSysVector(SYSTICK_INTERRUPT_VECTOR_NUMBER, SysTick_ISR);
	
	/* Set the interrupt frequency. Ignore the function success/fail return value */
    (void)SysTick_Config((uint32)CLOCK_FREQ / (uint32)INTERRUPT_FREQ); /* defined in auto-generated core_cm0.h */
	
	/* Set eoc_ISR as the interrupt service routine for eoc interrupt */
	isr_eoc_StartEx(eoc_ISR);
	
    for(;;)
    {	
		/* Check if the new speed value is generated */
		if(eoc_flag)
		{
			/* Reset the flag */
			eoc_flag = 0;			
			
			/* Disable EzI2C interrupt temporarily */
			EZI2C_DisableInt();
				
			/* Check if there is no active I2C activity */
			if((EZI2C_EzI2CGetActivity() & EZI2C_EZI2C_STATUS_BUSY)==0) 
			{				
				/* Update the I2C buffer with current fan speed */
				I2CBuffer[FAN1_ACT_SPEED] =  FanController_GetActualSpeed(FAN_1);
				I2CBuffer[FAN2_ACT_SPEED] =  FanController_GetActualSpeed(FAN_2);
				
				/* Update the I2C buffer with current PWM duty cycle */
				I2CBuffer[FAN1_DUTY_CYCLE] = FanController_GetDutyCycle(FAN_1);
				I2CBuffer[FAN2_DUTY_CYCLE] = FanController_GetDutyCycle(FAN_2);				
			}				
			
			/* Enable EzI2C Interrupt */
			EZI2C_EnableInt();
		}		
	
		/* Executes every 100ms */
		if(SysTickFlag)
		{
			/* Clear flag */
			SysTickFlag = 0;
			
			/* Disable EzI2C interrupt temporarily */
			EZI2C_DisableInt();
				
			/* Check if there is no active I2C activity */
			if((EZI2C_EzI2CGetActivity() & EZI2C_EZI2C_STATUS_BUSY)==0) 
			{			
				/* Scan switches SW1 and SW2 and update the speed of both fans */
				ScanSwitchesAndTakeAction();				
			}			
			
			/* Enable EzI2C Interrupt */
			EZI2C_EnableInt();
		}
		
		/* Disable EzI2C interrupt temporarily */
		EZI2C_DisableInt();
			
		/* Check if there is no active I2C activity */
		if((EZI2C_EzI2CGetActivity() & EZI2C_EZI2C_STATUS_BUSY)==0) 
		{	
			/* If there is any active command, execute it */
			if(I2CBuffer[COMMAND_INPUT]!=0)
				ReadI2CCommandAndTakeAction();
		}	
		
		/* Enable EzI2C Interrupt */	
		EZI2C_EnableInt();	
    }
}

/******************************************************************************* 
* Function Name: ScanSwitchesAndTakeAction()
********************************************************************************
*
* Summary:
*  Scans SW1 and SW2. If SW1 is pressed, speed of both the fans is increased
*  by RPM_STEP. If SW2 is pressed, speed of both the fans is decreased by RPM_STEP. 
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
		/* Both switches pressed, don't do anything */
	}
	else
	if(Pin_SW1_Read()== 0)
	{		
		/* Switch SW1 is pressed, increment speed of both the fans */
		
		/* Check if current FAN1 speed is within the limits */
		if((I2CBuffer[FAN1_SPD_REQUEST] + RPM_STEP) > MAX_RPM)
			I2CBuffer[FAN1_SPD_REQUEST] = MAX_RPM;
		else
			I2CBuffer[FAN1_SPD_REQUEST] = I2CBuffer[FAN1_SPD_REQUEST] + RPM_STEP;

		/* Update the speed of FAN1 */	
		FanController_SetDesiredSpeed(FAN_1, I2CBuffer[FAN1_SPD_REQUEST] );				
		
		/* Check if current FAN2 speed is within the limits */
		if((I2CBuffer[FAN2_SPD_REQUEST] + RPM_STEP) > MAX_RPM)
			I2CBuffer[FAN2_SPD_REQUEST] = MAX_RPM;
		else
			I2CBuffer[FAN2_SPD_REQUEST] = I2CBuffer[FAN2_SPD_REQUEST] + RPM_STEP;

		/* Update the speed of FAN2 */	
		FanController_SetDesiredSpeed(FAN_2, I2CBuffer[FAN2_SPD_REQUEST] );					
		
	}
	else
	if(Pin_SW2_Read() == 0)
	{	
		/* Switch SW2 is pressed, decrement speed of both the fans */
				
		/* Check if the current FAN1 speed is within the limits */								
		if((I2CBuffer[FAN1_SPD_REQUEST]- RPM_STEP) < MIN_RPM )
			I2CBuffer[FAN1_SPD_REQUEST] = MIN_RPM;			
		else
			I2CBuffer[FAN1_SPD_REQUEST] = I2CBuffer[FAN1_SPD_REQUEST] - RPM_STEP;

		/* Update the speed of FAN1 */			
		FanController_SetDesiredSpeed(FAN_1,I2CBuffer[FAN1_SPD_REQUEST]);
		
		/* Check if the current FAN2 speed is within the limits */								
		if((I2CBuffer[FAN2_SPD_REQUEST]- RPM_STEP) < MIN_RPM )
			I2CBuffer[FAN2_SPD_REQUEST] = MIN_RPM;			
		else
			I2CBuffer[FAN2_SPD_REQUEST] = I2CBuffer[FAN2_SPD_REQUEST] - RPM_STEP;

		/* Update the speed of FAN2 */		
		FanController_SetDesiredSpeed(FAN_2,I2CBuffer[FAN2_SPD_REQUEST]);		
		
	}		
}

/******************************************************************************* 
* Function Name: ReadI2CCommandAndTakeAction()
********************************************************************************
*
* Summary:
* Executes reset, speed update, increment/decrement speed command from external
* I2C master
*
* Parameters: None 
*
* Return: None
*  
******************************************************************************/

void ReadI2CCommandAndTakeAction(void)
{
	/* Flag for updating the PWM duty cycle */
	uint8 UpdateSpeedFan1_flag = 0;	
	uint8 UpdateSpeedFan2_flag = 0;	
	
	/* Check for valid command */
	switch(I2CBuffer[COMMAND_INPUT])
	{
		/* Reset command */ 
		case RESET_SETTINGS_COMMAND:
			
			/* Load the default values for fans speed */
			I2CBuffer[FAN1_SPD_REQUEST] = INIT_RPM;
			I2CBuffer[FAN2_SPD_REQUEST] = INIT_RPM;
			
			/* Clear command bytes */
			CLEAR_COMMAND;
			
			/* Set the flag */
			UpdateSpeedFan1_flag = 1;
			UpdateSpeedFan2_flag = 1;
			
		break;
			
		/* Duty update command */
		case SPEED_UPDATE_COMMAND:
			
			/* Check whether the speed value provided is within the limits */
			if((I2CBuffer[SPEED_INPUT] <= MAX_RPM) && (I2CBuffer[SPEED_INPUT] >= MIN_RPM)) 
			{
				/* Check which fan, command is for */
				if(I2CBuffer[FAN_NUMBER] == FAN_1)
				{
					I2CBuffer[FAN1_SPD_REQUEST] = I2CBuffer[SPEED_INPUT];
					UpdateSpeedFan1_flag = 1;
				}
				else
				if(I2CBuffer[FAN_NUMBER] == FAN_2)
				{
					I2CBuffer[FAN2_SPD_REQUEST] = I2CBuffer[SPEED_INPUT];
					UpdateSpeedFan2_flag = 1;
				}
				
			}						
		
			/* Clear command data */
			CLEAR_COMMAND;
		
		break;				
			
		/* Speed increment command */		
		case SPEED_INC_COMMAND:
				
			/* Check which fan, command is for */
			if(I2CBuffer[FAN_NUMBER] == FAN_1)
			{		
				/* Check if present speed is within the limits */
				if((I2CBuffer[FAN1_SPD_REQUEST] + RPM_STEP) > MAX_RPM)
					I2CBuffer[FAN1_SPD_REQUEST] = MAX_RPM;
				else
					I2CBuffer[FAN1_SPD_REQUEST] = I2CBuffer[FAN1_SPD_REQUEST] + RPM_STEP;
						
				/* Set the flag */	
				UpdateSpeedFan1_flag = 1;
			
			}
			else
			if(I2CBuffer[FAN_NUMBER] == FAN_2)
			{
				/* Check if present speed is within the limits */
				if((I2CBuffer[FAN2_SPD_REQUEST] + RPM_STEP) > MAX_RPM)
					I2CBuffer[FAN2_SPD_REQUEST] = MAX_RPM;
				else
					I2CBuffer[FAN2_SPD_REQUEST] = I2CBuffer[FAN2_SPD_REQUEST] + RPM_STEP;
						
				/* Set the flag */	
				UpdateSpeedFan2_flag = 1;			
					
			}						
			
			/* Clear command data */
			CLEAR_COMMAND;
			
		break;
		
		/* Speed decrement command */		
		case SPEED_DEC_COMMAND:
				
			/* Check which fan, command is for */
			if(I2CBuffer[FAN_NUMBER] == FAN_1)
			{						
				/* Check if the present speed is within the limits */								
				if((I2CBuffer[FAN1_SPD_REQUEST]- RPM_STEP) < MIN_RPM )
					I2CBuffer[FAN1_SPD_REQUEST] = MIN_RPM;			
				else
					I2CBuffer[FAN1_SPD_REQUEST] = I2CBuffer[FAN1_SPD_REQUEST] - RPM_STEP;
					
				/* Set the flag */	
				UpdateSpeedFan1_flag = 1;
				
			}
			else
			if(I2CBuffer[FAN_NUMBER] == FAN_2)
			{	
				/* Check if the present speed is within the limits */								
				if((I2CBuffer[FAN2_SPD_REQUEST]- RPM_STEP) < MIN_RPM )
					I2CBuffer[FAN2_SPD_REQUEST] = MIN_RPM;			
				else
					I2CBuffer[FAN2_SPD_REQUEST] = I2CBuffer[FAN2_SPD_REQUEST] - RPM_STEP;
					
				/* Set the flag */	
				UpdateSpeedFan2_flag = 1;
			}
			
			/* Clear command data */
			CLEAR_COMMAND;
			
		break;			
	}				
	
	/* Check if the flag is set */
	if(UpdateSpeedFan1_flag)
	{
		/* Update the desired speed for fan1 */
		FanController_SetDesiredSpeed(FAN_1, I2CBuffer[FAN1_SPD_REQUEST] );
						
		/* Clear Flag */
		UpdateSpeedFan1_flag = 0;
	}			
	
	if(UpdateSpeedFan2_flag)
	{
		/* Update the desired speed for fan2 */
		FanController_SetDesiredSpeed(FAN_2, I2CBuffer[FAN2_SPD_REQUEST] );
		
		/* Clear Flag */
		UpdateSpeedFan2_flag = 0;
	}		
}

/* [] END OF FILE */
