/*
  *****************************************************************************
  * @title  bms_MTR_FRoST.c
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

/* Includes ------------------------------------------------------------------*/
#include "bms_MTR_FRoST.h"
#include "cmsis_os.h"



/* functions ------------------------------------------------------------------*/

void bmsSetSource(bms_typeDef_sourceSelect source)
{
	switch(source)
	{
	case bmsCell1:

		//Set to decimal 0
		HAL_GPIO_WritePin(BMS_MUX_PORT_1,BMS_MUX_PIN_1,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_2,BMS_MUX_PIN_2,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_3,BMS_MUX_PIN_3,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_4,BMS_MUX_PIN_4,GPIO_PIN_RESET); //0
		break;

	case bmsCell2:

		//Set to decimal 1
		HAL_GPIO_WritePin(BMS_MUX_PORT_1,BMS_MUX_PIN_1,GPIO_PIN_SET); 	//1
		HAL_GPIO_WritePin(BMS_MUX_PORT_2,BMS_MUX_PIN_2,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_3,BMS_MUX_PIN_3,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_4,BMS_MUX_PIN_4,GPIO_PIN_RESET); //0
		break;

	case bmsCell3:

		//Set to decimal 2
		HAL_GPIO_WritePin(BMS_MUX_PORT_1,BMS_MUX_PIN_1,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_2,BMS_MUX_PIN_2,GPIO_PIN_SET); 	//1
		HAL_GPIO_WritePin(BMS_MUX_PORT_3,BMS_MUX_PIN_3,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_4,BMS_MUX_PIN_4,GPIO_PIN_RESET); //0
		break;

	case bmsCell4:

		//Set to decimal 3
		HAL_GPIO_WritePin(BMS_MUX_PORT_1,BMS_MUX_PIN_1,GPIO_PIN_SET); 	//1
		HAL_GPIO_WritePin(BMS_MUX_PORT_2,BMS_MUX_PIN_2,GPIO_PIN_SET); 	//1
		HAL_GPIO_WritePin(BMS_MUX_PORT_3,BMS_MUX_PIN_3,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_4,BMS_MUX_PIN_4,GPIO_PIN_RESET); //0
		break;

	case bmsCell5:

		//Set to decimal 4
		HAL_GPIO_WritePin(BMS_MUX_PORT_1,BMS_MUX_PIN_1,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_2,BMS_MUX_PIN_2,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_3,BMS_MUX_PIN_3,GPIO_PIN_SET); 	//1
		HAL_GPIO_WritePin(BMS_MUX_PORT_4,BMS_MUX_PIN_4,GPIO_PIN_RESET); //0
		break;

	case bmsCell6:

		//Set to decimal 5
		HAL_GPIO_WritePin(BMS_MUX_PORT_1,BMS_MUX_PIN_1,GPIO_PIN_SET); 	//1
		HAL_GPIO_WritePin(BMS_MUX_PORT_2,BMS_MUX_PIN_2,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_3,BMS_MUX_PIN_3,GPIO_PIN_SET);  	//1
		HAL_GPIO_WritePin(BMS_MUX_PORT_4,BMS_MUX_PIN_4,GPIO_PIN_RESET); //0
		break;

	case bmsCurrent:

		//Set to decimal 6
		HAL_GPIO_WritePin(BMS_MUX_PORT_1,BMS_MUX_PIN_1,GPIO_PIN_RESET); //0
		HAL_GPIO_WritePin(BMS_MUX_PORT_2,BMS_MUX_PIN_2,GPIO_PIN_SET); 	//1
		HAL_GPIO_WritePin(BMS_MUX_PORT_3,BMS_MUX_PIN_3,GPIO_PIN_SET); 	//1
		HAL_GPIO_WritePin(BMS_MUX_PORT_4,BMS_MUX_PIN_4,GPIO_PIN_RESET); //0
		break;
	}
}

//give adc data and get real voltage
float bmsCalcVoltage(uint16_t adcData)
{
	float voltage;
	voltage = (adcData/4095.0)*3300; //voltage on voltage divider
	voltage = voltage*3/2;//BMS_VDIVIDER_RATIO; // voltage before divider
	voltage = voltage /1000;

	return voltage;
}

//give adc data and get real current
float bmsCalcCurrent(uint16_t adcData)
{
	float current = 0;
	current  = (adcData/4095.0)*3300; //voltage from hal sensor
	current = (current - (3300.0/2.0));///90.0; 	//
	current = current /90;
	//current = current / 1000;

	/*
	if (current <0){current = 0;}
	current = current /0.09;
	*/

	return current;
}




