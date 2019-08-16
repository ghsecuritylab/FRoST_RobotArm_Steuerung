/*
  *****************************************************************************
  * @title  rgb_indicator_MTR_FRoST.h
  *
  * @author Max Triller
  *
  * @date   Created on: 19.07.2019
  *
  * @brief  Added control functionality for the RGB Status indicator
  * 		Note: 	The indicator is connected over UART to the CPU, by sending numbers the light will change
  * 				Currently USART3 interface is used.
  *
  * @edit	 Max Triller 		19.07.2019	MTR
  *******************************************************************************
  */

#ifndef RGB_INDICATOR_MTR_FROST_
#define RGB_INDICATOR_MTR_FROST_

/*Includes*/
#include "main.h"
#include "globalDataStructures_CM_MTR_FRoST.h"

/*Prototypes*/
void rgb_setState(globalData_enumTypeDef_systemState); //set state in Global Data Structure
void rgb_displayState(void); 			//send state via USART

#endif // RGB_INDICATOR_MTR_FROST_
