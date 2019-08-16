/*
  *****************************************************************************
  * @title  bms_MTR_FRoST.h
  *
  * @author Alex Kingston & Max Triller
  *
  * @date   Created on: 15.07.2019
  *
  * @brief  Add battery monitoring on our FRoST rover
  * 		It's for six channels aka 6s lipo battery and one channel for current
  * 		Note: 	only one ADC channel is used and an external multiplexer
  * 				see also hardware and chip markings
  *
  * 		Note: Steps to save ADC values in global data structure
  * 		- TX task sends data periodical to GS and set Semaphore
  * 		- Read task triggered by semaphore from TX task
  * 		- Read task and ADC interrupt triggers themselves alternately to get all seven readouts complete
  * 		- Read task stores data into global data structure
  * 		- Read task goes to blocked mode until TX task sends data
  *
  * @edit	 Max Triller 		15.07.2019	MTR
  *******************************************************************************
  */


#ifndef BMS_MTR_FROST_
#define BMS_MTR_FROST_


/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* defines ------------------------------------------------------------------*/
#define BMS_MUX_PIN_1 GPIO_PIN_6
#define BMS_MUX_PIN_2 GPIO_PIN_7
#define BMS_MUX_PIN_3 GPIO_PIN_8
#define BMS_MUX_PIN_4 GPIO_PIN_9

#define BMS_MUX_PORT_1 GPIOF
#define BMS_MUX_PORT_2 GPIOF
#define BMS_MUX_PORT_3 GPIOF
#define BMS_MUX_PORT_4 GPIOF

#define BMS_VDIVIDER_RATIO (2/3)


/* typedefs  ------------------------------------------------------------------*/
typedef enum
{
	bmsCell1,
	bmsCell2,
	bmsCell3,
	bmsCell4,
	bmsCell5,
	bmsCell6,
	bmsCurrent
}bms_typeDef_sourceSelect;

/* functions ------------------------------------------------------------------*/

void initOsImplementation(void);
void bmsSetSource(bms_typeDef_sourceSelect); //select source pin on external multiplexer
uint16_t bmsGetAdcData(void); 				 //read data from adc
float bmsCalcVoltage(uint16_t); 			 //give adc data and get real voltage
float bmsCalcCurrent(uint16_t); 			 //give adc data and get real current

#endif /*BMS */

