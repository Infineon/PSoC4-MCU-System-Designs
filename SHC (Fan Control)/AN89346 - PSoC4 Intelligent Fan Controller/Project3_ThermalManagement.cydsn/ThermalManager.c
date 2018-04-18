/*******************************************************************************
* File Name: ThermalManager.c  
* Version:   2.10
*
* Description:
*  This file provides the source code for Thermal Manager.
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

#include <project.h>
#include "ThermalManager.h"
#include "math.h"

/******************************************************************************* 
* Global Variables
********************************************************************************/
uint8 temperatureReadings[TEMP_SENSOR_MAX];
uint16 zoneTemperature[ZONE_COUNT];

ThermalLookupTableEntry thermalLookup[ZONE_COUNT][MAX_LOOKUP_TABLE_ENTRIES] = 
    {
        /* Zone1 Temp/Speed Lookup */
        {
		{0, 4000},
		{15,4500},
        {35,5500},
        {55,7500},
        {75,9500},
        },

        /* Zone2 Temp/Speed Lookup */
        {
		{0, 4500},
        {23,5000},
        {25,7000},
        {27,9000},
        {29,10000}
		}
    };
     
ZoneConfigEntry zoneConfig[ZONE_COUNT] =
    {
        /* Zone1 */
        {1,                                 /* fanCount           */
        FAN_1,                              /* fanSelect          */
        FAN_SPEED_DEFAULT,                  /* fanCurrentSetSpeed */
        TEMP_SPEED_FUNC_LOOKUP_TABLE,       /* fanTempSpeedFunc   */
        4,                                  /* tempHysteresis     */
        2,                                  /* tempSensorCount    */
        0x0003,                             /* tempSensorSelect   */
        TEMP_COMPOSITE_WEIGHTED_AVG,        /* tempCompositeFunc  */
        {1,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0}   /* tempSensorWeight   */
        },

        /* Zone2 */
        {1,                                 /* fanCount           */
        FAN_2,                              /* fanSelect          */
        FAN_SPEED_DEFAULT,                  /* fanCurrentSetSpeed */
        TEMP_SPEED_FUNC_LOOKUP_TABLE,       /* fanTempSpeedFunc   */
        1,                                  /* tempHysteresis     */
        2,                                  /* tempSensorCount    */
        0x0003,                             /* tempSensorSelect   */
        TEMP_COMPOSITE_WEIGHTED_AVG,        /* tempCompositeFunc  */
        {99,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  /* tempSensorWeight   */
        }
    };

/******************************************************************************* 
* Public API's
********************************************************************************/

/******************************************************************************* 
* Function Name: ThermalManager_Start()
********************************************************************************
*
* Summary:
* Initialize the various Thermal Manager function blocks
*
* Parameters: None 
*
* Return: None
*  
******************************************************************************/
	
void ThermalManager_Start()
{
    /* Start the fan controller component and set the fans to the default starting speed */
    FanController_Start();
    FanController_SetDesiredSpeed(FAN_1, FAN_SPEED_DEFAULT);
    FanController_SetDesiredSpeed(FAN_2, FAN_SPEED_DEFAULT);

    /* Start the TMP175_I2C temperature sensor*/
    TMP175I2C_Start();
	
    /* Start the ADC */
    ADC_Start();

	/* Clear temperatureReadings array */
	memset(temperatureReadings,0, TEMP_SENSOR_MAX);
	
	/* Launch the temp sensing hardware to prepare for first call the ServiceThermalManager */
    LaunchTempMeasure();
}

/******************************************************************************* 
* Function Name: ServiceThermalManager
********************************************************************************
*
* Summary:
*  Determines fan speeds based on termperature sensor inputs.
*  Application firmware should call this API periodically at a rate such that
*  it meets the temperature response time requirements.
*
* Parameters: None
*
* Return: None
*******************************************************************************/
void ServiceThermalManager(void)
{    
    uint16  speed;
    uint8   zoneNum;
    
    for (zoneNum = 0; zoneNum < ZONE_COUNT; zoneNum++)
    {   
		/* Get the zone temperature */
		zoneTemperature[zoneNum] = GetZoneTemp(zoneNum);
		
		/* Get the required fan speed for the zone */
        speed = CalcFanSpeed(zoneNum, zoneTemperature[zoneNum]);
		
		/* Set the fan speed for the zone */
        SetFanSpeed(zoneNum, speed);
    }

	/* Done with current thermal zone composite temperature measurements.
	   Re-launch the temp sensing hardware for next call of the ServiceThermalManager
	*/
    LaunchTempMeasure();
}

/******************************************************************************* 
* Function Name: GetZoneTemp
********************************************************************************
*
* Summary:
*  Calculates the temperature for the specified zone.  The actual temperature
*  measurement is handled elsewhere.  This function simply determines
*  which temperature sensors (possibly a single sensor) in the system are associated
*  with this zone and then combines those sensor measurements according to a user
*  specified, per-zone, temperature compositing algorithm.
*  
*
* Parameters:  
*  zoneNum	: Which zone number to return the temperature for
*
*
*  Return	: Calculated temperature for the zone 
*
*******************************************************************************/
uint8 GetZoneTemp(uint8 zoneNum)
{
    uint8 zoneTemp;

    switch (zoneConfig[zoneNum].tempCompositeFunc)
    {
        case TEMP_COMPOSITE_STRAIGHT_AVG:
        {
            zoneTemp = CalcAverageZoneTemp(zoneNum);
            break;
        }
        case TEMP_COMPOSITE_WEIGHTED_AVG:
        {
            zoneTemp = CalcWeightedAverageZoneTemp(zoneNum);
            break;
        }
        case TEMP_COMPOSITE_MAX_TEMP:
        {
            zoneTemp = CalcMaxTempZoneTemp(zoneNum);
            break;
        }
        case TEMP_COMPOSITE_CUSTOM:
        default:
        {
            zoneTemp = TEMP_INVALID_TEMP;
            break;
        }
    }
    
    return(zoneTemp);
}

/******************************************************************************* 
* Function Name: CalcFanSpeed
********************************************************************************
*
* Summary:
*  Provides the required fan speed for the specified zone based on the temperature    
*
* Parameters:  
*  zoneNum     	: The zone number that the fan(s) is (are) in.  This is used to look up
*                 which thermal zone control algorithm (linear, table based, etc.)
*                 to use for calculating the fan speed.
*
*  temperature  : Current zone temperature
*
*  Return		: Calculated fan speed in RPM 
*
*******************************************************************************/
uint16 CalcFanSpeed(uint8 zoneNum, uint8 temperature)
{
    uint16 newSpeed;

    switch (zoneConfig[zoneNum].fanTempSpeedFunc)
    {
        case TEMP_SPEED_FUNC_LOOKUP_TABLE:
        {
            newSpeed = TemperatureToSpeedLookupTable(zoneNum, temperature);
            break;
        }
        case TEMP_SPEED_FUNC_LINEAR:
        case TEMP_COMPOSITE_CUSTOM:
        default:
        {
            /* Future feature : Currently only table look-up transfer function is implemented. */
            newSpeed = 4000;
            break;
        }
    }
    
    return(newSpeed);
}

/******************************************************************************* 
* Function Name: SetFanSpeed
********************************************************************************
*
* Summary: Sets the required fan speed
*  
* Parameters:  
*  zoneNum          : The zone number that the fan(s) are in  
*  speed            : This is the requested speed value for the fan decided by zoneNum
*
* Return			: None
*******************************************************************************/
void SetFanSpeed(uint8 zoneNum, uint16 speed)
{
    uint8 fanIndex;
    
	/* Check if the requested speed value is different from the current set speed */
	if (!(speed == zoneConfig[zoneNum].fanCurrentSetSpeed))
    {		
		/* Loop through fan selects, to get the index for fans present in the requested zoneNum */
		for (fanIndex = 0; fanIndex < FAN_MAX; fanIndex++)
	    {
	        if ((zoneConfig[zoneNum].fanSelect & (1 << fanIndex)) != 0)
	        {
				/* Update the desired fan speed */
				FanController_SetDesiredSpeed(fanIndex + 1,speed);
	            zoneConfig[zoneNum].fanCurrentSetSpeed = speed;
	        }
	    }
    }
}

/******************************************************************************* 
* Function Name: LaunchTempMeasure
********************************************************************************
*
* Summary:
* Read the various temperature sensors defined in the system and store the results in a
* global data structure.
*
* Parameters	: None 
*
* Return		: None
*
*******************************************************************************/
void LaunchTempMeasure()
{
    /*
       Read the TMP175/I2C temp sensor.  Note: this is a blocking call.
    */
    temperatureReadings[0] = GetI2CTemp(0);    

    /*
       Read from the emulated analog Temp Sensor
    */
   	temperatureReadings[1] = GetAnalogTemp(0);
}

/******************************************************************************* 
* Function Name: GetSensorTemp
********************************************************************************
*
* Summary:
* Gets the latest value for the specified sensor number.  This is not the live
* temperature, but rather the most recent temperature "snapshot" for the temperature
* sensor stored by the ThermalManager Service.
*
* Parameters:  sensorNum : Temperature sensor to read (I2C Sensor - 0x00, 
*							Analog Sensor - 0x01)
*
* Return	: Temperature value from the sensor
*******************************************************************************/
uint8 GetSensorTemp(uint8 sensorNum)
{
    uint8 temperature;
    
	if (temperatureReadings[sensorNum] == TEMP_INVALID_TEMP)
    {
        temperature = 0;
    }    
	else	
    {
        temperature = temperatureReadings[sensorNum];
    }
    return (temperature);
}

/******************************************************************************* 
* Function Name: GetFanCurrentSetSpeed
********************************************************************************
*
* Summary:
* Gets the desired speed set for the specified zone
* 
* Parameters:  zoneNum - Thermal profile zone number
*
* Return	: Desired fan speed in RPM
*******************************************************************************/
uint16 GetFanCurrentSetSpeed(uint8 zoneNum)
{
    return (zoneConfig[zoneNum].fanCurrentSetSpeed);
}

/******************************************************************************* 
* Private/Internal API's
********************************************************************************/
/****************************************************************************** 
*    Function: CalcWeightedAverageZoneTemp
*    
*    Description:Calculate the temperature weighted average for the specified zone
*
*	 Parameters: zone number
*    
*    Returns: weighted average of the values from temperature sensors of the zone
*    
********************************************************************************/
uint8 CalcWeightedAverageZoneTemp(uint8 zone)
{
    uint8   tempSensorIndex;
    uint16  weightSum = 0;
    uint16  weightedSum = 0;
    uint8   zoneTemp;
    
    for (tempSensorIndex = 0; tempSensorIndex < TEMP_SENSOR_MAX; tempSensorIndex++)
    {
        if ((zoneConfig[zone].tempSensorSelect & (1 << tempSensorIndex)) != 0)
        {
            weightedSum += GetSensorTemp(tempSensorIndex) * zoneConfig[zone].tempSensorWeight[tempSensorIndex];
            weightSum += zoneConfig[zone].tempSensorWeight[tempSensorIndex];
        }
    }
    
    /* Calculate the weighted average.  Check for divide by zero. */
    if (!(weightSum == 0))
    {
        zoneTemp = weightedSum / weightSum;
    }
    else
    {
        zoneTemp = TEMP_INVALID_TEMP;
    }
    return (zoneTemp);
}

/****************************************************************************** 
*    Function: CalcAverageZoneTemp
*    
*    Description: Calculates the temperature average for the specified zone
*
*	 Parameters	: zone number
*    
*    Returns	: simple average of the values from temperature sensors of the zone
*    
********************************************************************************/
uint8 CalcAverageZoneTemp(uint8 zoneNum)
{
    uint8   tempSensorIndex;
    uint8   sensorCount = 0;
    uint16  tempSum = 0;
    uint8   zoneTemp;
    
    for (tempSensorIndex = 0; tempSensorIndex < TEMP_SENSOR_MAX; tempSensorIndex++)
    {
        if ((zoneConfig[zoneNum].tempSensorSelect & (1 << tempSensorIndex)) != 0)
        {
            tempSum += GetSensorTemp(tempSensorIndex);
            sensorCount++;
        }
    }
    
    /* Calculate the average.  Check for divide by zero. */
    if (!(sensorCount == 0))
    {
        zoneTemp = tempSum / sensorCount;
    }
    else
    {
        zoneTemp = TEMP_INVALID_TEMP;
    }
    return (zoneTemp);
}

/****************************************************************************** 
*    Function: CalcMaxTempZoneTemp
*    
*    Description	: Picks the max temperature from a sensor in the specified zone
*
*	 Parameters		: zone number
*    
*    Returns		: returns the highest temperature reading from a sensor in the zone
*    
********************************************************************************/
uint8 CalcMaxTempZoneTemp(uint8 zoneNum)
{
    uint8   tempSensorIndex;
    uint8   zoneTemp = 0;
    
    for (tempSensorIndex = 0; tempSensorIndex < TEMP_SENSOR_MAX; tempSensorIndex++)
    {
        if ((zoneConfig[zoneNum].tempSensorSelect & (1 << tempSensorIndex)) != 0)
        {
            if (GetSensorTemp(tempSensorIndex) > zoneTemp)
            {
                zoneTemp = GetSensorTemp(tempSensorIndex);
            }
        }
    }

    return (zoneTemp);
}

/****************************************************************************** 
*    Function: TemperatureToSpeedLookupTable
*    
*    Description: Translates the zone temperature to a fan speed based on look up table
*    thermalLookup
*
*	 Parameters	: zone number and zone temperature
*    
*    Returns	: returns the required fan speed
*    
********************************************************************************/

uint16 TemperatureToSpeedLookupTable(uint8 zoneNum, uint8 temperature)
{
    uint16  newSpeed = 0;
    uint8   tableIndex;

    for (tableIndex = 1; tableIndex < MAX_LOOKUP_TABLE_ENTRIES; tableIndex++)
    {
        if (temperature < thermalLookup[zoneNum][tableIndex].temperature)
        {
            newSpeed = thermalLookup[zoneNum][tableIndex - 1].fanSpeed;

			/* Fan slowing down - need hysteresis check */
            if (newSpeed < zoneConfig[zoneNum].fanCurrentSetSpeed) 
            {
                /* If the temperature has not fallen below the hysteresis level, do */
                /* not reduce the speed                                             */
                if (!(temperature < (thermalLookup[zoneNum][tableIndex].temperature - zoneConfig[zoneNum].tempHysteresis)))
                {
                    newSpeed = thermalLookup[zoneNum][tableIndex].fanSpeed;
                }
            }
            break;
        }
    }

    /* If the speed was not set in the preceeding loop, then the temperature */
    /* is higher than the last table entry.  Set newSpeed to max (the last   */
    /* entry) */
    if (newSpeed == 0)
    {
        newSpeed = thermalLookup[zoneNum][MAX_LOOKUP_TABLE_ENTRIES - 1].fanSpeed;
    }

    return (newSpeed);
}

/****************************************************************************** 
*    Function: GetI2CTemp
*    
*    Description: Reads the temperature measured by TMP175
*
*	 Parameters	: Sensor index
*    
*    Returns	: returns the temperature value
*    
********************************************************************************/
uint8 GetI2CTemp(uint8 sensorIndex)
{
    uint8 temp1 = TEMP_INVALID_TEMP;
    uint8 status = 0;
    uint8 i2c_addr = 0;

    if (sensorIndex == 0)
    {
        i2c_addr = TM_EBK_TMP175_ADDR;
    }

    /* Read from the temperature sensor if there is a valid I2C address */
    if (!(i2c_addr == 0))
    {
        status = TMP175I2C_I2CMasterSendStart(i2c_addr, TMP175I2C_I2C_READ_XFER_MODE);
        if(status == TMP175I2C_I2C_MSTR_NO_ERROR) /* Check if transfer completed without errors */
        {
            /* Read the first temp byte */
            temp1 = TMP175I2C_I2CMasterReadByte(TMP175I2C_I2C_ACK_DATA);
            /* Read the second temp byte */
            (void) TMP175I2C_I2CMasterReadByte(TMP175I2C_I2C_NAK_DATA);
        }
        TMP175I2C_I2CMasterSendStop(); /* Send Stop */
    }

    return(temp1);
}

/*******************************************************************************
* Function Name: uint8 GetAnalogTemp(uint8 sensorIndex)
********************************************************************************
*
* Summary	: Potentiometer connected to ADC emulates the analog output
*			  temperature sensor
*
* Parameters: sensor index
*
* Return	: temperature value
*******************************************************************************/
uint8 GetAnalogTemp(uint8 sensorIndex)
{
    uint16 adcCounts;
    uint8  temperature;
    
    if (sensorIndex == 0)
    {
		/* Start the ADC conversion */
        ADC_StartConvert();   
		
		/* Wait till the result is available */
        ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);
		
		/* Read the ADC */
        adcCounts = ADC_GetResult16(0);  
		
		/* Stop the ADC conversion */
		ADC_StopConvert();    
		
		if(adcCounts & 0x8000)
		{
			adcCounts = 0;
		}
		
		/* Scale the result. Note that this is being done only for the demo */
        temperature = adcCounts>>4;
    }
    else
    {
        temperature = TEMP_INVALID_TEMP;
    }
    return (temperature);
}

/* [] END OF FILE */
