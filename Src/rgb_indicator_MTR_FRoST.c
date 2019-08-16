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


/*Includes*/
#include "rgb_indicator_MTR_FRoST.h"
#include "usart.h"

/*Functions*/
void rgb_setState(globalData_enumTypeDef_systemState mode)
{

	//create storage variable
	static globalData_typeDef_systemState_IO_GS d_systemState;

	//read data from Structure
	d_systemState = globalDataStructures_getSystemState_IO_GS();

	//check read was a success
	if(d_systemState.dataID == GLOBALDATA_ID_SYSTEMSTATE)
	{
		//store mode
		d_systemState.systemState = mode;
	}

	//store data back to global data structure
	if(globalDataStructures_setSystemState_IO_GS(d_systemState)!=GLOBAL_DATA_STRUCT_SET_OK)
	{
		Error_Handler();
	}
}

void rgb_displayState(void)
{

	//create storage variable
	static globalData_typeDef_systemState_IO_GS d_systemState;

	//read data from Structure
	d_systemState = globalDataStructures_getSystemState_IO_GS();

	//check read was a success
	if(d_systemState.dataID == GLOBALDATA_ID_SYSTEMSTATE)
	{
		//store mode
		HAL_UART_Transmit(&huart2,&d_systemState.systemState,sizeof(d_systemState.systemState),100);
	}



}
