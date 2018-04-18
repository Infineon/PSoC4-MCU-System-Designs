/*********************************************************************************
* Project Name:      Firmware Fan Control 
* Version:           1.20
*
* Device Used:       PSoC 3 CY8C3866AXI-040 or PSoC 5 CY8C5588AXI-060ES1
* Software Used:     PSoC Creator v2.1
* Compiler Used:     Keil(C51), ARM GNU CC
* Related Hardware:  CY8CKIT-001 PSoC DVK
*********************************************************************************
* Theory of Operation:
*
*  The Fan Controller is configured into the firmware (CPU) fan control mode
*  The speed control algorithm is implemented in the firmware
*  The firmware synchornizes to hardware using the end-of-cycle (eoc) pulse
*  2 Fans are supported (individual PWMs - no banks)
*
*  1st line of LCD displays Desired Speed and Actual Speed and PWM Duty Cycle of Fan 1
*  2nd line of LCD displays Actual Speed and PWM Duty Cycle of Fan 2
*  SW1 decreases desired speed in RPM
*  SW2 increases desired speed in RPM
*
********************************************************************************
* Copyright 2013, Cypress Semiconductor Corporation. All rights reserved.
* This software is owned by Cypress Semiconductor Corporation and is protected
* by and subject to worldwide patent and copyright laws and treaties.
* Therefore, you may use this software only as provided in the license agreement
* accompanying the software package from which you obtained this software.
* CYPRESS AND ITS SUPPLIERS MAKE NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* WITH REGARD TO THIS SOFTWARE, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT,
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/

#include <device.h>
#include <stdio.h>

#if defined (__GNUC__)
    /* Add an explicit reference to the floating point printf library */
    /* to allow the usage of floating point conversion specifiers. */
    /* This is not linked in by default with the newlib-nano library. */
    asm (".global _printf_float");
#endif


/* PWM duty cycle controls - units are hundredths of a percent */
#define MIN_DUTY            (50u)
#define MAX_DUTY            (9950u)
#define DUTY_STEP_COARSE    (100u)
#define DUTY_STEP_FINE      (2u)

/* Speed controls - units are RPM */
#define MIN_RPM             (2500u)
#define MAX_RPM             (9500u)
#define INIT_RPM            (4500u)
#define RPM_STEP            (500u)
#define RPM_DELTA_LARGE     (500u)
#define RPM_TOLERANCE       (100u)

void main()
{
    uint16  desiredSpeed = INIT_RPM;
    uint16  dutyCycle[2];
    uint8   fanNumber;
    uint8   dutyCycle_int;
    uint8   dutyCycle_dec;
    char    displayString[6];
    
    /* Globally Enable Interrupts to the CPU Core */
    CyGlobalIntEnable;
    
    FanController_Start();
    FanController_SetDesiredSpeed(1u, desiredSpeed);
    FanController_SetDesiredSpeed(2u, desiredSpeed);
    dutyCycle[0] = FanController_GetDutyCycle(1u);
    dutyCycle[1] = FanController_GetDutyCycle(2u);
    
    LCD_Start();
    LCD_Position(0u, 0u);
    LCD_PrintDecUint16(desiredSpeed);
    LCD_Position(1u, 0u);
    LCD_PrintString("F/W");
    
    while(1u)
    {
        /* Synchronize firmware to end-of-cycle pulse from FanController */
        if(EOC_SR_Read())
        {
            for(fanNumber = 1u; fanNumber <= 2u; fanNumber++)
            {
                /* Display Fan Actual Speeds */
                LCD_Position(fanNumber-1,5u);
                LCD_PrintDecUint16(FanController_GetActualSpeed(fanNumber));
                LCD_PrintString("   ");
    
                /* Firmware Speed Regulation */
                LCD_Position(fanNumber-1,9u);
                
                /* Fan Below Desired Speed */
                if(FanController_GetActualSpeed(fanNumber) < desiredSpeed)
                {
                    if((desiredSpeed - FanController_GetActualSpeed(fanNumber)) > RPM_DELTA_LARGE)
                    {
                        dutyCycle[fanNumber-1] += DUTY_STEP_COARSE;
                    }
                    else
                    {
                        dutyCycle[fanNumber-1] += DUTY_STEP_FINE;
                    }
                    if(dutyCycle[fanNumber-1] > MAX_DUTY)
                    {
                        dutyCycle[fanNumber-1] = MAX_DUTY;
                    }
                }
                /* Fan Above Desired Speed */
                else if(FanController_GetActualSpeed(fanNumber) > desiredSpeed)
                {
                    if((FanController_GetActualSpeed(fanNumber) - desiredSpeed) > RPM_DELTA_LARGE)
                    {
                        if(dutyCycle[fanNumber-1] > (MIN_DUTY+DUTY_STEP_COARSE))
                        {
                            dutyCycle[fanNumber-1] -= DUTY_STEP_COARSE;
                        }
                    }
                    else if((FanController_GetActualSpeed(fanNumber) - desiredSpeed) > RPM_TOLERANCE)
                    {
                        if(dutyCycle[fanNumber-1] > MIN_DUTY)
                        {
                            dutyCycle[fanNumber-1] -= DUTY_STEP_FINE;
                        }
                    }
                }
                FanController_SetDutyCycle(fanNumber, dutyCycle[fanNumber-1]);
    
                /* Display Current Duty Cycle Settings (in 100ths of a percent) */
                LCD_Position(fanNumber-1,10u);
                dutyCycle_int=dutyCycle[fanNumber-1]/100;
                dutyCycle_dec=((float)dutyCycle[fanNumber-1]/100.0 - dutyCycle_int)*100;
                sprintf(displayString, "%3d.%02d",dutyCycle_int,dutyCycle_dec);
                LCD_PrintString(displayString);
                LCD_PrintString("%    ");
            }
            CyDelay(250u);
            
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
    
                /* Display Updated Desired Speed */
                LCD_Position(0u, 0u);
                LCD_PrintDecUint16(desiredSpeed);
                FanController_SetDesiredSpeed(1u, desiredSpeed);
                FanController_SetDesiredSpeed(2u, desiredSpeed);
                dutyCycle[0] = FanController_GetDutyCycle(1u);
                dutyCycle[1] = FanController_GetDutyCycle(2u);
                
                /* Switch Debounce */
                CyDelay(250u);
            }
        }
    }
}


/* [] END OF FILE */
