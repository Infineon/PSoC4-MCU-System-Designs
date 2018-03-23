/*******************************************************************************
* Project Name:      PMBus Slave Example Project
* Version:           1.00
*
* Related Hardware:  PSoC4 Pioneer kit
********************************************************************************
* Theory of Operation:
*
* This example project demonstrates usage of the PMBus Slave component in a
* simulated Thermal Management application. 
* In place of an external temperature sensor, the on-chip temperature sensor is
* used. It is combined with the ADC SAR Sequencer components to create a simple
* thermal manager that can run on a PSoC 4 Pioneer kit without any special
* hardware. The example supports a small subset of the PMBus command set to
* provide the voltage and temperature reading.
*
* Refer to the example project datasheet for more information.
*
* Hardware Dependency:
*  PSoC4 Pioneer kit
*
********************************************************************************
* Copyright 2014, Cypress Semiconductor Corporation. All rights reserved.
* This software is owned by Cypress Semiconductor Corporation and is protected
* by and subject to worldwide patent and copyright laws and treaties.
* Therefore, you may use this software only as provided in the license agreement
* accompanying the software package from which you obtained this software.
* CYPRESS AND ITS SUPPLIERS MAKE NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* WITH REGARD TO THIS SOFTWARE, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT,
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/

#include <project.h>

#define CH0_N           (0x00u)
#define TEMP_CH         (0x01u)
#define COUNTS_IN_1SEC  (2000u) /* 1 Tick is 500 us */
#define TIMER_PERIOD    (COUNTS_IN_1SEC - 1u) /* Count down from period to 0 */

volatile int16  result[ADC_SAR_SEQ_TOTAL_CHANNELS_NUM];
volatile uint32 timer = TIMER_PERIOD;

/* Function prototypes */
static void  HandlePMBusCommand(void);

/* Interrupt handler */
CY_ISR_PROTO(ADC_SAR_SEQ_ISR_LOC);


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  Starts all the components and initializes the ADC interrupt. Checks for a
*  pending PMBus transaction and handle it in the main loop.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
int main()
{
    /* Start components */
    PMBus_Start();
    ADC_SAR_SEQ_Start();
    ADC_SAR_SEQ_StartConvert();

    /* Enable interrupt and set interrupt handler to local routine */
    ADC_SAR_SEQ_IRQ_StartEx(ADC_SAR_SEQ_ISR_LOC);

    CyGlobalIntEnable;

    for(;;)
    {
        /* Check for a pending PMBus transaction and handle it */
        if(0u != PMBus_GetTransactionCount())
        {
            HandlePMBusCommand();
        }
    }
}


/******************************************************************************
* Function Name: ADC_SAR_SEQ_ISR_LOC
*******************************************************************************
*
* Summary:
*  Interrupt Service Routine for the ADC SAR Seq. Invoked when the ADC
*  conversion is complete (every 500 us in this example). This routine also
*  implements the software timer to start the temperature measurement with an
*  interval
*  of one second between the measurements.
*
* Parameters:
*   None
*
* Return:
*   None
*
******************************************************************************/
CY_ISR(ADC_SAR_SEQ_ISR_LOC)
{
    uint32 intr_status;

    if(0u == timer)
    {
        /* Enable injection channel for next scan */
        ADC_SAR_SEQ_EnableInjection();
        timer = TIMER_PERIOD;
    }
    else
    {
        --timer;
    }

    /* Read interrupt status registers */
    intr_status = ADC_SAR_SEQ_SAR_INTR_MASKED_REG;

    /* Check for End of Scan interrupt */
    if((intr_status & ADC_SAR_SEQ_EOS_MASK) != 0u)
    {
        /* Read conversion result */
        result[CH0_N] = ADC_SAR_SEQ_GetResult16(CH0_N);
    }    
    /* Check for Injection End of Conversion */
    if((intr_status & ADC_SAR_SEQ_INJ_EOC_MASK) != 0u)
    {
        result[TEMP_CH] = ADC_SAR_SEQ_GetResult16(TEMP_CH);
    }    

    /* Clear handled interrupt */
    ADC_SAR_SEQ_SAR_INTR_REG = intr_status;
}


/*******************************************************************************
* Function Name: HandlePMBusCommand
********************************************************************************
*
* Summary:
*   This function fetches and processes a pending PMBus transaction. Any PMBus
*   command that is designated as "Manual" in the customizer will result in
*   a call to this routine when received.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
static void HandlePMBusCommand(void)
{
    uint8   commandCode;

    PMBus_TRANSACTION_STRUCT *transact;
    
    transact = PMBus_GetNextTransaction();

    if (transact != NULL)
    {
        commandCode   = transact->commandCode;

        switch (commandCode)
        {
            case PMBus_READ_TEMPERATURE_1:
                PMBus_regs.READ_TEMPERATURE_1 = DieTemp_CountsTo_Celsius(result[TEMP_CH]);
                break;

            case PMBus_READ_VOUT:
                PMBus_regs.READ_VOUT = ADC_SAR_SEQ_CountsTo_mVolts(CH0_N, result[CH0_N]);

            default:
                break;
        }

        /* Always call CompleteTransaction for Manual commands */
        PMBus_CompleteTransaction();
    }            
}

/* [] END OF FILE */
