/*
  *****************************************************************************
  * @title  globalDataStructures_CM_MTR.c
  *
  * @author Carsten Mueller & Max Triller
  *
  * @date   Created on: 14.03.2019
  *
  * @brief  Typedef of all data structures for data exchange between ground station and rover !
  * 		Also Mutex and Semaphore definitions.
  * 		- MODULE_typeDef_SUBMODULE_producer_consumer:
  * 			IO 		= I/O Core
  * 			GS 		= ground station
  * 			ARM 	= arm Core
  * 			DRILL	= drill Core
  * 			BMS		= battery management system
  * 		- Use get() and set() functions for save access to data structures
  *
  * @edit	 Max Triller 		24.06.2019	MTR
  *******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <globalDataStructures_CM_MTR_FRoST.h>

/*Global variables ---------------------------------------------------------*/

//volatile static globalData_typeDef_allData 							globalData_allData					;
//volatile static globalData_typeDef_DataToGS							globalData_allDataToGs				;
volatile static globalData_typeDef_drill_DRILL_GS						globalData_drill_drillToGs			;
volatile static globalData_typeDef_drill_GS_DRILL						globalData_drill_gsToDrill			;
volatile static globalData_typeDef_robotArm_ARM_GS						globalData_arm_armToGs				;
volatile static globalData_typeDef_robotArm_GS_ARM						globalData_arm_gsToArm				;
volatile static globalData_typeDef_BMS_BMS_GS							globalData_bms_bmsToGs				;
volatile static globalData_typeDef_security_GS_IO						globalData_security_gsToIo			;
volatile static globalData_typeDef_systemState_IO_GS					globalData_systemState_ioToGs		;
volatile static globalData_typeDef_weightCell_IO_GS						globalData_weightCell_ioToGs		;
volatile static globalData_typeDef_environmentSensor_IO_GS				globalData_environmentSensor_ioToGs	;
volatile static globalData_typeDef_powerTrain_GS_IO						globalData_powerTrain_gsToIO		;
volatile static globalData_typeDef_powerTrain_IO_GS						globalData_powerTrain_ioToGs		;
volatile static globalData_typeDef_boschIMU								globalData_boschImu					;

/*Mutexes -----------------------------------------------------------------*/
/*Mutexes for Tx data to Gs*/ /*MTR 18.06.2019*/
//osMutexId m_globalDataStructureHandle_allData;
//osMutexId m_globalDataStructureHandle_setAllData_allDatatoGs;
osMutexId m_globalDataStructureHandle_PowerTrain_ioToGs;
osMutexId m_globalDataStructureHandle_EnvironmentSensor_ioToGs; 		//only Tx
osMutexId m_globalDataStructureHandle_WeightCell_ioToGs; 				//only Tx
osMutexId m_globalDataStructureHandle_SystemState_ioToGs;
osMutexId m_globalDataStructureHandle_Security_ioToGs;
osMutexId m_globalDataStructureHandle_BMS_bmsToGs; 						//only Tx
osMutexId m_globalDataStructureHandle_Arm_armToGs;
osMutexId m_globalDataStructureHandle_Drill_DrillToGs;
osMutexId m_globalDataStructureHandle_boschImu;

/*Mutexes for Rx data to Gs*/ /*MTR 18.06.2019*/
//osMutexId m_globalDataStructureHandle_allData;
//osMutexId m_globalDataStructureHandle_setAllData_allData_GsToIo;
osMutexId m_globalDataStructureHandle_PowerTrain_GsToIo;
osMutexId m_globalDataStructureHandle_SystemState_GsToIo;
osMutexId m_globalDataStructureHandle_Security_GsToIO;
osMutexId m_globalDataStructureHandle_Arm_GsToAm;
osMutexId m_globalDataStructureHandle_Drill_GsToDrill;

/*Semaphore -----------------------------------------------------------------*/
/*Semaphore for signaling new Tx data to Gs*/ /*MTR 18.06.2019*/
osSemaphoreId s_globalDataStructureHandle_newPowerTrain_ioToGs;
osSemaphoreId s_globalDataStructureHandle_newEnvironmentSensor_ioToGs;
osSemaphoreId s_globalDataStructureHandle_newWeightCell_ioToGs;
osSemaphoreId s_globalDataStructureHandle_newSystemState_ioToGs;
osSemaphoreId s_globalDataStructureHandle_newSecurity_ioToGs;
osSemaphoreId s_globalDataStructureHandle_newBMS_bmsToGs;
osSemaphoreId s_globalDataStructureHandle_newArm_armToGs;
osSemaphoreId s_globalDataStructureHandle_newDrill_DrillToGs;

/*Semaphore for signaling new Rx data from Gs*/ /*MTR 18.06.2019*/
osSemaphoreId s_globalDataStructureHandle_newPowerTrain_GsToIo;
osSemaphoreId s_globalDataStructureHandle_newSystemState_GsToIo;
osSemaphoreId s_globalDataStructureHandle_newSecurity_GsToIO;
osSemaphoreId s_globalDataStructureHandle_newArm_GsToAm;
osSemaphoreId s_globalDataStructureHandle_newDrill_GsToDrill;

/*Semaphore for trigger new Sensor readout*/
osSemaphoreId s_globalDataStructureHandle_readNewSensorData_BNO055; //Added MTR 09.07.2019 for BOSCH IMU





/* Functions ------------------------------------------------------------*/
void globalDataStructures_initOsImplementation_MutexSemaphore (void)
{
	/* Create all mutexes -----------------------------------------------------------------*/
	/*CHANGE MTR 18.06.2019 */ //add Mutexes
	/*Mutexes for Tx data to Gs*//*MTR 18.06.2019*/
	osMutexDef(m_globalDataStructureHandle_PowerTrain_ioToGs);
	m_globalDataStructureHandle_PowerTrain_ioToGs = osMutexCreate(osMutex(m_globalDataStructureHandle_PowerTrain_ioToGs));

	osMutexDef(m_globalDataStructure_EnvironmentSensor_ioToGs);
	m_globalDataStructureHandle_EnvironmentSensor_ioToGs = osMutexCreate(osMutex(m_globalDataStructure_EnvironmentSensor_ioToGs));

	osMutexDef(m_globalDataStructure_WeightCell_ioToGs);
	m_globalDataStructureHandle_WeightCell_ioToGs = osMutexCreate(osMutex(m_globalDataStructure_WeightCell_ioToGs));

	osMutexDef(m_globalDataStructure_SystemState_ioToGs);
	m_globalDataStructureHandle_SystemState_ioToGs = osMutexCreate(osMutex(m_globalDataStructure_SystemState_ioToGs));

	osMutexDef(m_globalDataStructure_Security_ioToGs);
	m_globalDataStructureHandle_Security_ioToGs = osMutexCreate(osMutex(m_globalDataStructure_Security_ioToGs));

	osMutexDef(m_globalDataStructure_BMS_bmsToGs);
	m_globalDataStructureHandle_BMS_bmsToGs = osMutexCreate(osMutex(m_globalDataStructure_BMS_bmsToGs));

	osMutexDef(m_globalDataStructure_Arm_armToGs);
	m_globalDataStructureHandle_Arm_armToGs = osMutexCreate(osMutex(m_globalDataStructure_Arm_armToGs));

	osMutexDef(m_globalDataStructure_Drill_DrillToGs);
	m_globalDataStructureHandle_Drill_DrillToGs = osMutexCreate(osMutex(m_globalDataStructure_Drill_DrillToGs));


	/*Mutexes for Rx data to Gs*//*MTR 18.06.2019*/
	osMutexDef(m_globalDataStructure_PowerTrain_GsToIo);
	m_globalDataStructureHandle_PowerTrain_GsToIo = osMutexCreate(osMutex(m_globalDataStructure_PowerTrain_GsToIo));

	osMutexDef(m_globalDataStructure_SystemState_GsToIo);
	m_globalDataStructureHandle_SystemState_GsToIo = osMutexCreate(osMutex(m_globalDataStructure_SystemState_GsToIo));

	osMutexDef(m_globalDataStructure_Security_GsToIO);
	m_globalDataStructureHandle_Security_GsToIO = osMutexCreate(osMutex(m_globalDataStructure_Security_GsToIO));

	osMutexDef(m_globalDataStructuree_Arm_GsToAm);
	m_globalDataStructureHandle_Arm_GsToAm = osMutexCreate(osMutex(m_globalDataStructuree_Arm_GsToAm));

	osMutexDef(m_globalDataStructure_Drill_GsToDrill);
	m_globalDataStructureHandle_Drill_GsToDrill = osMutexCreate(osMutex(m_globalDataStructure_Drill_GsToDrill));

	osMutexDef(m_globalDataStructur_boschImu);
	m_globalDataStructureHandle_boschImu = osMutexCreate(osMutex(m_globalDataStructur_boschImu));

	/* Create all Semaphores -----------------------------------------------------------------*/

	/*Semaphore for signaling new Tx data to Gs*/ /*MTR 18.06.2019*/
	osSemaphoreDef( s_globalDataStructure_newPowerTrain_ioToGs);
	s_globalDataStructureHandle_newPowerTrain_ioToGs = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newPowerTrain_ioToGs), 1);

	osSemaphoreDef( s_globalDataStructure_newEnvironmentSensor_ioToGs);
	s_globalDataStructureHandle_newEnvironmentSensor_ioToGs = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newEnvironmentSensor_ioToGs), 1);

	osSemaphoreDef( s_globalDataStructure_newWeightCell_ioToGs);
	s_globalDataStructureHandle_newWeightCell_ioToGs = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newWeightCell_ioToGs), 1);

	osSemaphoreDef( s_globalDataStructure_newSystemState_ioToGs);
	s_globalDataStructureHandle_newSystemState_ioToGs = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newSystemState_ioToGs), 1);

	osSemaphoreDef( s_globalDataStructure_newSecurity_ioToGs);
	s_globalDataStructureHandle_newSecurity_ioToGs = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newSecurity_ioToGs), 1);

	osSemaphoreDef( s_globalDataStructure_newBMS_bmsToGs);
	s_globalDataStructureHandle_newBMS_bmsToGs = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newBMS_bmsToGs), 1);

	osSemaphoreDef( s_globalDataStructure_newArm_armToGs);
	s_globalDataStructureHandle_newArm_armToGs = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newArm_armToGs), 1);

	osSemaphoreDef( s_globalDataStructure_newDrill_DrillToGs);
	s_globalDataStructureHandle_newDrill_DrillToGs = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newDrill_DrillToGs), 1);


	/*Semaphore for signaling new Rx data from Gs*/ /*MTR 18.06.2019*/
	osSemaphoreDef (s_globalDataStructure_newPowerTrain_GsToIo);
	s_globalDataStructureHandle_newPowerTrain_GsToIo = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newPowerTrain_GsToIo), 1);

	osSemaphoreDef (s_globalDataStructure_newSystemState_GsToIo);
	s_globalDataStructureHandle_newSystemState_GsToIo = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newSystemState_GsToIo), 1);

	osSemaphoreDef (s_globalDataStructure_newSecurity_GsToIO);
	s_globalDataStructureHandle_newSecurity_GsToIO = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newSecurity_GsToIO), 1);

	osSemaphoreDef (s_globalDataStructure_newArm_GsToAm);
	s_globalDataStructureHandle_newArm_GsToAm = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newArm_GsToAm), 1);

	osSemaphoreDef (s_globalDataStructure_newDrill_GsToDrill);
	s_globalDataStructureHandle_newDrill_GsToDrill = osSemaphoreCreate(osSemaphore(s_globalDataStructure_newDrill_GsToDrill), 1);

	osSemaphoreDef (s_globalDataStructure_readNewSensorData_BNO055);
	s_globalDataStructureHandle_readNewSensorData_BNO055 = osSemaphoreCreate(osSemaphore(s_globalDataStructure_readNewSensorData_BNO055), 1);

	/*IMPORTANT!
   * since osSemaphoreCreate calls vSemaphoreCreateBinary (deprecated!)
   * and not xSemaphoreCreateBinary the semaphores have to be blocked
   * at startup! This may change in a later version of cmsis where
   * xSemaphoreCreateBinary may be used which made blocking semaphores at
   * startup unnecessary.
   * MTR 24.06.2019
   */

	  osSemaphoreWait(s_globalDataStructureHandle_newPowerTrain_ioToGs,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newEnvironmentSensor_ioToGs,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newWeightCell_ioToGs,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newSystemState_ioToGs,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newSecurity_ioToGs,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newBMS_bmsToGs,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newArm_armToGs,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newDrill_DrillToGs,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newPowerTrain_GsToIo,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newSystemState_GsToIo,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newSecurity_GsToIO,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newArm_GsToAm,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_newDrill_GsToDrill,osWaitForever);
	  osSemaphoreWait(s_globalDataStructureHandle_readNewSensorData_BNO055,osWaitForever);


}


/*MTR 20.06.2019*/ // set data validation byte and system status correctly
/*Note: IS NOT PROTECTET BY MUTEXES - SHOUD BE CALLED BEFORE FREERTOS START*/
void globalDataStrucutres_initData(void)
{
	globalData_drill_drillToGs			.dataID = 		GLOBALDATA_ID_DRILL; 	//TX
	globalData_drill_drillToGs			.enable = 		GLOBAL_DATA_STRUCT_DISABLE;
	globalData_drill_gsToDrill			.dataID = 		GLOBALDATA_ID_DRILL; 	//RX

	globalData_arm_armToGs				.dataID = 		GLOBALDATA_ID_ARM; 		//TX
	globalData_arm_armToGs				.enable = 		GLOBAL_DATA_STRUCT_DISABLE;
	globalData_arm_gsToArm				.dataID = 		GLOBALDATA_ID_ARM; 		//RX

	globalData_bms_bmsToGs				.dataID = 		GLOBALDATA_ID_BMS; 		//TX

	globalData_security_gsToIo			.dataID = 		GLOBALDATA_ID_SECURITY;	//RX

	globalData_systemState_ioToGs		.dataID 	 = 	GLOBALDATA_ID_SYSTEMSTATE;	//TX
	globalData_systemState_ioToGs		.systemState = 	SYSTEMSTATE_ON;

	globalData_weightCell_ioToGs		.dataID = 		GLOBALDATA_ID_WEIGHTCELL;			//TX

	globalData_environmentSensor_ioToGs	.dataID = 		GLOBALDATA_ID_ENVIROMENTSENSOR;		//TX

	globalData_powerTrain_gsToIO		.dataID = 		GLOBALDATA_ID_POWERTRAIN;				//RX
	globalData_powerTrain_ioToGs		.dataID = 		GLOBALDATA_ID_POWERTRAIN ;				//TX
	globalData_powerTrain_ioToGs		.actualState =  GLOBAL_DATA_STRUCT_DISABLE;

	globalData_boschImu					.dataID = 		GLOBALDATA_ID_IMU;
}


/*Set data functions */ /*CHANGE MTR 18.06.2019 */
/*Functions give DATA_SET_OK or DATA_SET_FAIL as return value to check success of function call --------------------------------------------*/

globalData_enumTypeDef_functionReturn globalDataStructures_setPowerTrain_GS_IO		 (globalData_typeDef_powerTrain_GS_IO data)
{
	if(osMutexWait(m_globalDataStructureHandle_PowerTrain_GsToIo, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_powerTrain_gsToIO = data;
		osMutexRelease(m_globalDataStructureHandle_PowerTrain_GsToIo);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}

globalData_enumTypeDef_functionReturn globalDataStructures_setPowerTrain_IO_GS		 (globalData_typeDef_powerTrain_IO_GS data)
{
	if(osMutexWait(m_globalDataStructureHandle_PowerTrain_ioToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_powerTrain_ioToGs = data;
		osMutexRelease(m_globalDataStructureHandle_PowerTrain_ioToGs);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}
globalData_enumTypeDef_functionReturn globalDataStructures_setEnvironmentSensor_IO_GS (globalData_typeDef_environmentSensor_IO_GS data)
{
	if(osMutexWait(m_globalDataStructureHandle_EnvironmentSensor_ioToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_environmentSensor_ioToGs = data;
		osMutexRelease(m_globalDataStructureHandle_EnvironmentSensor_ioToGs);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}
globalData_enumTypeDef_functionReturn globalDataStructures_setWeightCell_IO_GS		 (globalData_typeDef_weightCell_IO_GS data)
{
	if(osMutexWait(m_globalDataStructureHandle_WeightCell_ioToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_weightCell_ioToGs = data;
		osMutexRelease(m_globalDataStructureHandle_WeightCell_ioToGs);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}
globalData_enumTypeDef_functionReturn globalDataStructures_setSystemState_IO_GS		 (globalData_typeDef_systemState_IO_GS data)
{
	if(osMutexWait(m_globalDataStructureHandle_SystemState_ioToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_systemState_ioToGs = data;
		osMutexRelease(m_globalDataStructureHandle_SystemState_ioToGs);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}
globalData_enumTypeDef_functionReturn globalDataStructures_setSecurity_GS_IO			 (globalData_typeDef_security_GS_IO data)
{
	if(osMutexWait(m_globalDataStructureHandle_Security_GsToIO, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_security_gsToIo = data;
		osMutexRelease(m_globalDataStructureHandle_Security_GsToIO);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}
globalData_enumTypeDef_functionReturn globalDataStructures_setBMS_BMS_GS				 (globalData_typeDef_BMS_BMS_GS data)
{
	if(osMutexWait(m_globalDataStructureHandle_BMS_bmsToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_bms_bmsToGs = data;
		osMutexRelease(m_globalDataStructureHandle_BMS_bmsToGs);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}
globalData_enumTypeDef_functionReturn globalDataStructures_setRobotArm_GS_ARM		 (globalData_typeDef_robotArm_GS_ARM data)
{
	if(osMutexWait(m_globalDataStructureHandle_Arm_GsToAm, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_arm_gsToArm = data;
		osMutexRelease(m_globalDataStructureHandle_Arm_GsToAm);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}
globalData_enumTypeDef_functionReturn globalDataStructures_setRobotArm_ARM_GS		 (globalData_typeDef_robotArm_ARM_GS data)
{
	if(osMutexWait(m_globalDataStructureHandle_Arm_armToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_arm_armToGs = data;
		osMutexRelease(m_globalDataStructureHandle_Arm_armToGs);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}
globalData_enumTypeDef_functionReturn globalDataStructures_setDrill_GS_DRILL			 (globalData_typeDef_drill_GS_DRILL data)
{
	if(osMutexWait(m_globalDataStructureHandle_Drill_GsToDrill, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_drill_gsToDrill = data;
		osMutexRelease(m_globalDataStructureHandle_Drill_GsToDrill);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}
globalData_enumTypeDef_functionReturn globalDataStructures_setDrill_DRILL_GS			 (globalData_typeDef_drill_DRILL_GS data)
{
	if(osMutexWait(m_globalDataStructureHandle_Drill_DrillToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_drill_drillToGs = data;
		osMutexRelease(m_globalDataStructureHandle_Drill_DrillToGs);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}

globalData_enumTypeDef_functionReturn globalDataStructures_setImuData_Gs		 (globalData_typeDef_boschIMU data)
{
	if(osMutexWait(m_globalDataStructureHandle_boschImu, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		globalData_boschImu = data;
		osMutexRelease(m_globalDataStructureHandle_boschImu);
		return GLOBAL_DATA_STRUCT_SET_OK;
	}else {
		return GLOBAL_DATA_STRUCT_SET_FAIL;
	}
}


/*CHANGE MTR 18.06.2019
 *All data can't be set anymore, because of change in system design. Each processing-mcu
 *is directly connected to Rovers Ethernet network.
 * */
			/*
			void globalDataStructures_setAllData(globalData_typeDef_allData data)
			{
				if(osMutexWait(m_globalDataStructureHandle, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
				{
					globalDataStructures_allData = data;
					osMutexRelease(m_globalDataStructureHandle);
				}
			}

			void globalDataStructures_setDataToGS				 (globalData_typeDef_DataToGS data)
			{
				if(osMutexWait(m_globalDataStructureHandle, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
				{
					globalDataStructures_allData = data;
					osMutexRelease(m_globalDataStructureHandle);
				}
			}
*/


/*Get data functions*/ /*CHANGE MTR 18.06.2019 */
/*Functions give a empty structure as return value, if data reading fails -----------------------------------------------------*/

globalData_typeDef_powerTrain_GS_IO 			globalDataStructures_getPowerTrain_GS_IO()
{
	globalData_typeDef_powerTrain_GS_IO dataReturn = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	if(osMutexWait(m_globalDataStructureHandle_PowerTrain_GsToIo, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_powerTrain_gsToIO;
		osMutexRelease(m_globalDataStructureHandle_PowerTrain_GsToIo);
	}
	return dataReturn;
}

globalData_typeDef_powerTrain_IO_GS 			globalDataStructures_getPowerTrain_IO_GS		()
{
	 globalData_typeDef_powerTrain_IO_GS	dataReturn = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	if(osMutexWait(m_globalDataStructureHandle_PowerTrain_ioToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_powerTrain_ioToGs;
		osMutexRelease(m_globalDataStructureHandle_PowerTrain_ioToGs);
	}
	return dataReturn;
}
globalData_typeDef_environmentSensor_IO_GS 	    globalDataStructures_getEnvironmentSensor_IO_GS ()
{
	globalData_typeDef_environmentSensor_IO_GS	dataReturn = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	if(osMutexWait(m_globalDataStructureHandle_EnvironmentSensor_ioToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_environmentSensor_ioToGs;
		osMutexRelease(m_globalDataStructureHandle_EnvironmentSensor_ioToGs);
	}
	return dataReturn;
}

globalData_typeDef_weightCell_IO_GS 		    globalDataStructures_getWeightCell_IO_GS		()
{
	globalData_typeDef_weightCell_IO_GS dataReturn = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	if(osMutexWait(m_globalDataStructureHandle_WeightCell_ioToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_weightCell_ioToGs;
		osMutexRelease(m_globalDataStructureHandle_WeightCell_ioToGs);
	}
	return dataReturn;
}

globalData_typeDef_systemState_IO_GS		    globalDataStructures_getSystemState_IO_GS		()
{
	globalData_typeDef_systemState_IO_GS dataReturn;
	dataReturn.systemState = SYSTEMSTATE_READ_ERROR;

	if(osMutexWait(m_globalDataStructureHandle_SystemState_ioToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_systemState_ioToGs;
		osMutexRelease(m_globalDataStructureHandle_SystemState_ioToGs);
	}
	return dataReturn;
}

globalData_typeDef_security_GS_IO 			    globalDataStructures_getSecurity_GS_IO			()
{
	globalData_typeDef_security_GS_IO dataReturn = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	if(osMutexWait(m_globalDataStructureHandle_Security_GsToIO, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_security_gsToIo;
		osMutexRelease(m_globalDataStructureHandle_Security_GsToIO);
	}
	return dataReturn;
}

globalData_typeDef_BMS_BMS_GS 				    globalDataStructures_getBMS_BMS_GS				()
{
	globalData_typeDef_BMS_BMS_GS dataReturn = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	if(osMutexWait(m_globalDataStructureHandle_BMS_bmsToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_bms_bmsToGs;
		osMutexRelease(m_globalDataStructureHandle_BMS_bmsToGs);
	}
	return dataReturn;
}

globalData_typeDef_robotArm_GS_ARM 			    globalDataStructures_getRobotArm_GS_ARM			()
{
	globalData_typeDef_robotArm_GS_ARM dataReturn = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	if(osMutexWait(m_globalDataStructureHandle_Arm_GsToAm, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_arm_gsToArm;
		osMutexRelease(m_globalDataStructureHandle_Arm_GsToAm);
	}
	return dataReturn;
}

globalData_typeDef_robotArm_ARM_GS 			    globalDataStructures_getRobotArm_ARM_GS			()
{
	globalData_typeDef_robotArm_ARM_GS dataReturn = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	if(osMutexWait(m_globalDataStructureHandle_Arm_armToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_arm_armToGs;
		osMutexRelease(m_globalDataStructureHandle_Arm_armToGs);
	}
	return dataReturn;
}

globalData_typeDef_drill_GS_DRILL 			    globalDataStructures_getDrill_GS_DRILL			()
{
	globalData_typeDef_drill_GS_DRILL dataReturn = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	if(osMutexWait(m_globalDataStructureHandle_Drill_GsToDrill, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_drill_gsToDrill;
		osMutexRelease(m_globalDataStructureHandle_Drill_GsToDrill);
	}
	return dataReturn;
}

globalData_typeDef_drill_DRILL_GS 			    globalDataStructures_getDrill_DRILL_GS			()
{
	globalData_typeDef_drill_DRILL_GS dataReturn = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	if(osMutexWait(m_globalDataStructureHandle_Drill_DrillToGs, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_drill_drillToGs;
		osMutexRelease(m_globalDataStructureHandle_Drill_DrillToGs);
	}
	return dataReturn;
}


globalData_typeDef_boschIMU 			globalDataStructures_getBoshImu()
{
	globalData_typeDef_boschIMU dataReturn = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	if(osMutexWait(m_globalDataStructureHandle_boschImu, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
	{
		dataReturn = globalData_boschImu;
		osMutexRelease(m_globalDataStructureHandle_boschImu);
	}
	return dataReturn;
}


/*CHANGE MTR 18.06.2019
 *All data can't be set anymore, because of change in system design. Each processing-mcu
 *is directly connected to Rovers Ethernet network.
 * *//*

			globalData_typeDef_allData 						globalDataStructures_getAllData					()
			{
				globalData_typeDef_allData dataReturn;
				if(osMutexWait(m_globalDataStructureHandle, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
				{
					dataReturn = globalDataStructures_allData;
					osMutexRelease(m_globalDataStructureHandle);
					return dataReturn;
				}
				return dataReturn;
			}

			globalData_typeDef_DataToGS				globalDataStructures_getDataToGS				()
			{
				globalData_typeDef_DataToGS dataReturn;
				if(osMutexWait(m_globalDataStructureHandle, GLOBAL_DATA_STRUCTURES_MS_WAIT)==osOK)
				{
					dataReturn = globalDataStructures_allData.DataToGS;
					osMutexRelease(m_globalDataStructureHandle);
					return dataReturn;
				}
				return dataReturn;
			}
*/


