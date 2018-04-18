/*******************************************************************************
* File Name: ThermalManager.h  
* Version:   2.10
*
* Description:
*  This file contains the function prototypes and constants used in
*  the Thermal Management project.
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

#if !defined(THERMALMANAGER_H)
#define THERMALMANAGER_H

#define TEMP_SENSOR_MAX                     16	
#define FAN_MAX                             4
#define ZONE_COUNT              			2
#define TM_EBK_TMP175_ADDR                  0x48

/* Sensors*/	
#define I2C_TEMP_SENSOR 	0x00
#define ANALOG_TEMP_SENSOR	0x01	
	
/* Algorithms to calcuate zone temperature */	
#define TEMP_COMPOSITE_STRAIGHT_AVG         0x01u
#define TEMP_COMPOSITE_WEIGHTED_AVG         0x02u
#define TEMP_COMPOSITE_MAX_TEMP             0x03u
#define TEMP_COMPOSITE_CUSTOM               0x04u
	
#define TEMP_SPEED_FUNC_LINEAR              0x01u
#define TEMP_SPEED_FUNC_LOOKUP_TABLE        0x02u
#define TEMP_SPEED_FUNC_CUSTOM              0x03u
#define MAX_LOOKUP_TABLE_ENTRIES            5

#define TEMP_INVALID_TEMP                   255
	
	
/* Fan Identifier */
#define FAN_1   1
#define FAN_2   2
	
/* zone identifier */
#define ZONE1 	0
#define ZONE2	1	

/* FAN RPM presets */	
#define MIN_RPM         	1000
#define MAX_RPM         	9500
#define INIT_RPM        	5000
#define RPM_STEP        	500
#define	FAN_SPEED_DEFAULT	4500
	

typedef struct
{
    uint16 temperature;
    uint16 fanSpeed;
} ThermalLookupTableEntry;

typedef struct
{
    uint8   fanCount;               /* Number of fans in this zone                                     */
    uint16  fanSelect;              /* Fan select bitmask - which fans are assigned to this            */
                                    /* zone. The number of bits set == fanCount                        */
    uint16  fanCurrentSetSpeed;     /* The current "desired" set speed of the fan(s). is is not the    */
                                    /* live speed, but rather, the speed that the fans should be       */
    uint8   fanTempSpeedFunc;       /* Temperature to Speed transfer funtion for this zone (enum type) */
    uint8   tempHysteresis;         /* Hysteresis value (degrees) to apply to temps for falling speeds */

    uint8   tempSensorCount;        /* Number of temperature Sensors assigned to this thermal zone     */
    uint16  tempSensorSelect;       /* Temp sensor select bitmask - which sensors are assigned to this */
                                    /* zone. The number of bits set = tempSensorCount                 */
    uint8   tempCompositeFunc;      /* Multi-sensor temperature composite function to use (enum type)  */
    uint8   tempSensorWeight[16];   /* Weight factor for each temperature sensor.  The weight is       */
                                    /* is indexed by the temp sensor index                             */
} ZoneConfigEntry;
extern ZoneConfigEntry zoneConfig[ZONE_COUNT];

extern uint8 temperatureReadings[TEMP_SENSOR_MAX];
extern uint16 zoneTemperature[ZONE_COUNT];
/******************************************************************************* 
* Public API's
********************************************************************************/
void    ThermalManager_Start(void);
void    ServiceThermalManager(void);
uint8   GetZoneTemp(uint8 zoneNum);
uint16  CalcFanSpeed(uint8 zone, uint8 temperature);
void    SetFanSpeed(uint8 zone, uint16 speed);
void    LaunchTempMeasure(void);
uint8   GetSensorTemp(uint8 sensorNum);
uint16  GetFanCurrentSetSpeed(uint8 zoneNum);

/******************************************************************************* 
* Private/Internal API's
********************************************************************************/
uint16  TemperatureToSpeedLookupTable(uint8 zone, uint8 temperature);
uint8   GetI2CTemp(uint8 sensorIndex);
uint8   GetAnalogTemp(uint8 sensorIndex);
uint8   CalcWeightedAverageZoneTemp(uint8 zone);
uint8   CalcAverageZoneTemp(uint8 zone);
uint8   CalcMaxTempZoneTemp(uint8 zoneNum);

#endif /* THERMALMANAGER_H header sentry */

/* [] END OF FILE */
