/*******************************************************************************
* Project Name:      FW_Fan_Control_with_Alert
*
* Version:           1.0
*
* Theory of Operation:
*
*  Fan Controller is configured into Automatic Firmware (CPU) fan control mode.
*  Speed control algorithm is implemented in firmware using PID control
*  algorithm.
*  2 Fans are supported (individual PWMs - no banks).
*
*  1st line of LCD displays Desired Speed and Actual Speed and PWM Duty Cycle
*  of Fan 1
*  2nd line of LCD displays Actual Speed and PWM Duty Cycle of Fan 2
*  SW1 decreases desired speed in RPM
*  SW2 increases desired speed in RPM
*  Fan stall/speed alert monitored - pulses GPIO, triggers interrupt which
*  sets flag
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation. All rights reserved.
* This software is owned by Cypress Semiconductor Corporation and is protected
* by and subject to worldwide patent and copyright laws and treaties.
* Therefore, you may use this software only as provided in the license agreement
* accompanying the software package from which you obtained this software.
* CYPRESS AND ITS SUPPLIERS MAKE NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* WITH REGARD TO THIS SOFTWARE, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT,
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/
#include <project.h>
#include <stdio.h>

#if defined (__GNUC__)
    /* Add an explicit reference to the floating point printf library */
    /* to allow the usage of floating point conversion specifiers. */
    /* This is not linked in by default with the newlib-nano library. */
    asm (".global _printf_float");
#endif


/***************************************
*       Constants
***************************************/

#define MIN_RPM     (1000u)
#define MAX_RPM     (20000u)
#define INIT_RPM    (4500u)
#define RPM_STEP    (500u)
#define FAN_1       (1u)
#define FAN_2       (2u)


/***************************************
*        Function Prototypes
***************************************/

CY_ISR_PROTO(AlertInt_Isr);


/***************************************
*   Global variable declarations
***************************************/

uint16  stallStatus = 0u;
uint16  speedStatus = 0u;


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  main() performs following functions:
*  1: Starts the components
*  2: Checks for Fan Stall and Speed Failure.
*  3: Displays Fan actual speeds when there are no errors.
*  4: Displays PWM duty cycles for each fan.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
int main()
{
    uint16 desiredSpeed = INIT_RPM;
    uint8  fanNumber;
    char   string[6];

    /* Enable interrupts to the CPU core */
    CyGlobalIntEnable;

    /* Start alert interrupt with a custom handler */
    AlertInt_StartEx(&AlertInt_Isr);

    /* Start components */
    LCD_Start();
    FanController_Start();

    /* Set desired fan speed (fan number, rpm) */
    FanController_SetDesiredSpeed(FAN_1, desiredSpeed);
    FanController_SetDesiredSpeed(FAN_2, desiredSpeed);

    FanController_EnableAlert();

    LCD_Position(0u, 0u);
    LCD_PrintNumber(desiredSpeed);
    LCD_Position(1u, 0u);
    LCD_PrintString("F/W");

    while(1u)
    {
        /* If the tachometer block has read speeds of all fans in system,
         * only then check speed/stall conditions.
         */
        if(EOC_SR_Read())
        {
            for(fanNumber = FAN_1; fanNumber <= FAN_2; fanNumber++)
            {
                LCD_Position(fanNumber - 1u, 5u);

                /* Check for Fan Stall and Speed Failure (flags set in AlertInt ISR) */
                if(stallStatus & fanNumber)
                {
                    LCD_PrintString("STALL");
                    stallStatus &= ~fanNumber;
                }
                else if(speedStatus & fanNumber)
                {
                    LCD_PrintString("SPEED");
                    speedStatus &= ~fanNumber;
                }
                else
                {
                    /* Display Fan actual speeds when there are no errors */
                    LCD_PrintNumber(FanController_GetActualSpeed(fanNumber));
                    LCD_PrintString(" ");
                }

                /* Display PWM duty cycles for each fan */
                LCD_Position(fanNumber - 1u, 11u);
                sprintf(string, "%4.1f", ((float)(FanController_GetDutyCycle(fanNumber))) / 100u);
                LCD_PrintString(string);
                LCD_PrintString("%");
            }

        }

        /* Check for Button Press to Change Speed */
        if((!SW1_Read()) || (!SW2_Read()))
        {
            /* Decrease Speed */
            if(!SW1_Read())
            {
                if(desiredSpeed > MIN_RPM)
                {
                    desiredSpeed -= RPM_STEP;
                }
            }

            /* Increase Speed */
            else
            {
                desiredSpeed += RPM_STEP;
                if(desiredSpeed > MAX_RPM)
                {
                    desiredSpeed = MAX_RPM;
                }
            }

            FanController_SetDesiredSpeed(1u, desiredSpeed);
            FanController_SetDesiredSpeed(2u, desiredSpeed);

            /* Display Updated Desired Speed */
            LCD_Position(0u, 0u);
            LCD_PrintNumber(desiredSpeed);
            LCD_PrintString(" ");

            /* Switch Debounce */
            CyDelay(250u);
        }
    }
}


/*******************************************************************************
* Function Name: AlertInt_Isr
********************************************************************************
* Summary:
*  Informs main() routine about occured alerts.
*
* Parameters:  
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
CY_ISR(AlertInt_Isr)
{
    uint8 alertStatus;
    
    /* Determine alert source: stall or speed regulation failure (could be both) */
    alertStatus = FanController_GetAlertSource();
    
    /* If stall alert, determine which fan(s) */
    if (alertStatus & FanController_STALL_ALERT)
	{
		stallStatus = FanController_GetFanStallStatus();
	}
		
    /* If speed regulation failure alert, determine which fan(s) */
    if (alertStatus & FanController_SPEED_ALERT)
    {
		speedStatus = FanController_GetFanSpeedStatus();
	}
}


/* [] END OF FILE */
