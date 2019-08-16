/*
  *****************************************************************************************************************************************
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
  * @edit	 Max Triller 		20.06.2019	MTR
  * 			- format code & text /
  * 			- edit get() & set() functions /
  * 			- edit Mutex and Semaphores
  * 			- add function globalDataStrucutres_initData();
  **********************************************************************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GLOBALDATASTRUCTURES_CM_MTR_FROST_H_
#define GLOBALDATASTRUCTURES_CM_MTR_FROST_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
//#include "FreeRTOS.h" /* removed MTR 18.06.2019*/

/* External variables----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define GLOBAL_DATA_STRUCTURES_MS_WAIT  20 //Time to wait for getting mutex !?!?! not tested

#define GLOBAL_DATA_STRUCTURES_INIT_ZERO 0

/*Define Data IDs to differentiate UDP messages under same IP address */ /*MTR 18.06.2019*/
/*
#define GLOBALDATA_ID_DRIVETRAIN 		 0
#define GLOBALDATA_ID_BMS				 1
#define GLOBALDATA_ID_WEIGHTCELL		 2
#define GLOBALDATA_ID_SECURITY			 3
#define GLOBALDATA_ID_SAVETY			 4
#define GLOBALDATA_ID_ENVIROMENTSENSOR	 5
#define GLOBALDATA_ID_DRILL				 6
#define GLOBALDATA_ID_ARM				 7
#define GLOBALDATA_ID_POWERTRAIN 		 8
#define GLOBALDATA_ID_SYSTEMSTATE		 9
*/
/*Define Data IDs to differentiate UDP messages under same IP address */ /*MTR 20.06.2019*/
typedef enum
{
	GLOBALDATA_ID_ERROR 			=	0,
	//GLOBALDATA_ID_DRIVETRAIN		=	1, old but gold the drivetrain !
	GLOBALDATA_ID_BMS				=	2,
	GLOBALDATA_ID_WEIGHTCELL		=	3,
	GLOBALDATA_ID_SECURITY			=	4,
	GLOBALDATA_ID_SAFETY			=	5,
	GLOBALDATA_ID_ENVIROMENTSENSOR	=	6,
	GLOBALDATA_ID_DRILL				=	7,
	GLOBALDATA_ID_ARM				=	8,
	GLOBALDATA_ID_POWERTRAIN		=	9,
	GLOBALDATA_ID_SYSTEMSTATE		=	10,
	GLOBALDATA_ID_IMU 				=  	11
}globalData_enumTypeDef_dataID;

typedef enum
{
	GLOBALDATA_FLAG_RX_ERROR 			=	0,
	//GLOBALDATA_ID_DRIVETRAIN		=	1, old but gold the drivetrain !
	GLOBALDATA_FLAG_RX_BMS				=	2,
	GLOBALDATA_FLAG_RX_WEIGHTCELL		=	3,
	GLOBALDATA_FLAG_RX_SECURITY			=	4,
	GLOBALDATA_FLAG_RX_SAFETY			=	5,
	GLOBALDATA_FLAG_RX_ENVIROMENTSENSOR	=	6,
	GLOBALDATA_FLAG_RX_DRILL			=	7,
	GLOBALDATA_FLAG_RX_ARM				=	8,
	GLOBALDATA_FLAG_RX_POWERTRAIN		=	9,
	GLOBALDATA_FLAG_RX_SYSTEMSTATE		=	10
}globalData_enumTypeDef_signalFlagRx;

typedef enum
{
	GLOBALDATA_FLAG_TX_ERROR 			=	0,
	//GLOBALDATA_ID_DRIVETRAIN		=	1, old but gold the drivetrain !
	GLOBALDATA_FLAG_TX_BMS				=	2,
	GLOBALDATA_FLAG_TX_WEIGHTCELL		=	3,
	GLOBALDATA_FLAG_TX_SECURITY			=	4,
	GLOBALDATA_FLAG_TX_SAFETY			=	5,
	GLOBALDATA_FLAG_TX_ENVIROMENTSENSOR	=	6,
	GLOBALDATA_FLAG_TX_DRILL				=	7,
	GLOBALDATA_FLAG_TX_ARM				=	8,
	GLOBALDATA_FLAG_TX_POWERTRAIN		=	9,
	GLOBALDATA_FLAG_TX_SYSTEMSTATE		=	10
}globalData_enumTypeDef_signalFlagTx;

/* Private enumerations-------------------------------------------------------*/
typedef enum /*Created MTR 18.06.2019*/
{
	GLOBAL_DATA_STRUCT_ERROR 	= 0,
	GLOBAL_DATA_STRUCT_DISABLE 	= 1,
	GLOBAL_DATA_STRUCT_ENABLE 	= 0xFF
}globalData_enumTypeDef_activationStates;

typedef enum /* Created MTR 18.06.2019*/
{
	GLOBAL_DATA_STRUCT_GET_FAIL 	= 0,
	GLOBAL_DATA_STRUCT_SET_OK 		= 1,
	GLOBAL_DATA_STRUCT_SET_FAIL 	= 2,
	GLOBAL_DATA_STRUCT_GET_OK 	    = 0xAA
}globalData_enumTypeDef_functionReturn;

typedef enum
{
	 SYSTEMSTATE_READ_ERROR = 0 /*CHANGE MTR 18.06.2019 */
	,SYSTEMSTATE_ON				/*mode while rover boot-up and idle state */
	,SYSTEMSTATE_CONNECTED		/*mode after receiving heart-beat from GS */
	,SYSTEMSTATE_REMOTE			/*receiving remote control data*/
	,SYSTEMSTATE_AUTO		   /*Autonomous mode -> loads the skynet AI  */
	,SYSTEMSTATE_ERROR	= 0xFF /*activates self-ignition */
}globalData_enumTypeDef_systemState;

typedef enum
{
	 /*CHANGE SE 08.07.2019 */
	ROBOTARMMODE_DISABLE =1
	,ROBOTARMMODE_AXES
	,ROBOTARMMODE_IK
	,ROBOTARMMODE_TEACHED_POS
}globalData_enumTypeDef_robotArmMode;

typedef enum
{
	 /*CHANGE SE 08.07.2019 */
	ROBOTARMTP_HOME =1
	,ROBOTARMTP_FRONT
	,ROBOTARMTP_LEFT
	,ROBOTARMTP_RIGHT
	,ROBOTARMTP_CONTAINERX
	,ROBOTARMTP_CONTAINERY
	,ROBOTARMTP_CONTAINERZ
}globalData_enumTypeDef_robotArmTeachedPos;

typedef enum
{
	 /*CHANGE SE 08.07.2019 */
	ROBOTARMAXIS_1	=1
	,ROBOTARMAXIS_2
	,ROBOTARMAXIS_3
	,ROBOTARMAXIS_4
	,ROBOTARMAXIS_5
	,ROBOTARMAXIS_6
	,ROBOTARMAXIS_7
}globalData_enumTypeDef_robotArmAxis;

typedef enum
{
	 /*CHANGE SE 08.07.2019 */
	ROBOTARM2BUTTONS_DISABLE	=1
   ,ROBOTARM2BUTTONS_ON_OPEN_LEFT
   ,ROBOTARM2BUTTONS_OFF_CLOSE_RIGHT
}globalData_enumTypeDef_robotArm2Buttons;


/* Private typedef -----------------------------------------------------------*/
/* Note:
 * Programmers of processing cores can define their data structure here:
 * Check data direction ->  MODULE_typeDef_SUBMODULE_producer_consumer
 */
typedef struct
{
	uint8_t dataID;
	float  xEuler;
	float  yEuler;
	float  zEuler;
	float  xAcc;
	float  yAcc;
	float  zAcc;
}globalData_typeDef_boschIMU;

typedef struct
{
	uint8_t dataID;

	uint8_t actualState;				// enable / disable
	int16_t targetVelocityLeft;			// m/s
	int16_t actualVelocityLeft;			// m/s
	int16_t targetVelocityRight;		// m/s
	int16_t actualVelocityRight;		// m/s
}globalData_typeDef_powerTrain_IO_GS;

typedef struct
{
	uint8_t dataID;

	uint8_t maxVelocity;				// percent [0:100]
	int8_t 	xValue;						// -100 ... 100
	int8_t 	yValue;						// -100 ... 100
	uint8_t targetState;				// enable / disable
}globalData_typeDef_powerTrain_GS_IO;

typedef struct
{
	uint8_t dataID;

	uint16_t temperature;				// 0.1 °C -40 °C
	uint16_t pressure;					// hPa
	uint8_t  humidity;					// % RH
}globalData_typeDef_environmentSensor_IO_GS;

typedef struct
{
	uint8_t dataID;

	int16_t weight;						// grams

}globalData_typeDef_weightCell_IO_GS;

typedef struct
{
	uint8_t dataID;
	globalData_enumTypeDef_systemState systemState;
	uint8_t chipTemp;

}globalData_typeDef_systemState_IO_GS;

typedef struct
{
	uint8_t dataID;
	//uint8_t jetsonRestart;				// enable / disable
	//uint8_t powerElectronics;				// enable / disable
	uint8_t systemEnable;					// globals System State
	uint8_t systemPowerElectronics;
	uint8_t systemFuseByte;
	uint8_t systemStatusRGB;
	uint8_t dataValidation;
	uint8_t timeoutCounter; 				/*Added MTR 25.06.19*/


}globalData_typeDef_security_GS_IO;
typedef struct
{
	uint8_t dataID;
	float voltageCell1;				// 0.01 V
	float voltageCell2;				// 0.01 V
	float voltageCell3;				// 0.01 V
	float voltageCell4;				// 0.01 V
	float voltageCell5;				// 0.01 V
	float voltageCell6;				// 0.01 V
	float  current;					// 0.1 A
	uint8_t dataValidation;
}globalData_typeDef_BMS_BMS_GS;

typedef struct /*CHANGE SE 08.07.2019 */
{

	/* new data set '23'-bytes 31/07/2019 */
	uint8_t dataID;
	globalData_enumTypeDef_robotArmMode mode;				// see enum typedef
	globalData_enumTypeDef_robotArmTeachedPos TeachedPos;  // see enum typedef
	globalData_enumTypeDef_robotArmAxis ArmAxis;			// see enum typedef

	int8_t 	yValue;
	globalData_enumTypeDef_robotArm2Buttons Axis5State;
	int8_t  targetJointVelocity5;		//
	globalData_enumTypeDef_robotArm2Buttons endEffectorState;

	// -100 ... 100 Joystick
	int16_t  targetCoordinates_r;		// joystick - y; cylinder coordinates
	int16_t  targetCoordinates_h;		// joystick - ?

	int16_t  targetCoordinates_phi;		// joystick - x; 11
	int16_t  targetJointVelocity;		// [RPM]

	uint16_t  maxJointVelocity;		// [RPM]
	uint16_t velocityEndEffector;	//

	globalData_enumTypeDef_robotArm2Buttons microLinearActorState;
	uint8_t microLinearActorPercent;
	uint8_t dummyByte;
	uint8_t testVar;

}globalData_typeDef_robotArm_GS_ARM;

typedef struct /*CHANGE SE 08.07.2019 */
{
	uint8_t dataID;
	uint8_t  enable;					// enable / disable
	uint16_t actualCoordinates_r;		//
	uint16_t actualCoordinates_h;		//
	uint16_t actualCoordinates_phi;		//

	uint16_t actualJointAngle1;			// 0,00 ... 360,00° = 0 ... 36000
	uint16_t actualJointAngle2;			// 0 ... 360°
	uint16_t actualJointAngle3;			// 0 ... 360°
	uint16_t actualJointAngle4;			// 0 ... 360°
	uint16_t actualJointAngle5;			// 0 ... 360°

	int32_t actualPositionEndEffector;	//

}globalData_typeDef_robotArm_ARM_GS;

typedef struct
{
	uint8_t dataID;
	uint8_t  enable;					// enable / disable
	uint8_t  automaticMode;				// enable / disable
	uint8_t  lowerDrillUnit;			// lower / raise
	uint8_t	 targetDrillVelocity;		// 0 ... 100 rpm
	uint8_t  targetFeedRate;			// mm/min	*Vorschub*
	uint16_t targetDrillDepth;			// 0 ... 400 mm

}globalData_typeDef_drill_GS_DRILL;

typedef struct
{
	uint8_t dataID;
	uint8_t  enable;					// enable / disable
	uint16_t actualDrillPosition;		// 0 ... 300 mm (from lower/raise)
	uint8_t	 actualDrillVelocity;		// 0 ... 100 rpm
	uint8_t	 actualFeedRate;			// mm/min  	*Vorschub*
	uint16_t actualDrillDepth;			// 0 ... 400 mm

}globalData_typeDef_drill_DRILL_GS;

typedef struct
{
	uint8_t dataID;
	globalData_typeDef_powerTrain_IO_GS 		powerTrain_IO_GS;
	globalData_typeDef_environmentSensor_IO_GS 	environmentSensor_IO_GS;
	globalData_typeDef_weightCell_IO_GS 		weightCell_IO_GS;
	globalData_typeDef_systemState_IO_GS 		systemState_IO_GS;
	globalData_typeDef_BMS_BMS_GS 				BMS_BMS_GS;
	globalData_typeDef_robotArm_ARM_GS 			robotArm_ARM_GS;
	globalData_typeDef_drill_DRILL_GS 			drill_DRILL_GS;

}globalData_typeDef_DataToGS;

typedef struct
{
	uint8_t dataID;
	globalData_typeDef_DataToGS					DataToGS;
	globalData_typeDef_powerTrain_GS_IO 		powerTrain_GS_IO;
	globalData_typeDef_security_GS_IO 			security_GS_IO;
	globalData_typeDef_robotArm_GS_ARM 			robotArm_GS_ARM;
	globalData_typeDef_drill_GS_DRILL 			drill_GS_DRILL;

}globalData_typeDef_allData;



/* Function prototypes ------------------------------------------------------------*/
void globalDataStructures_initOsImplementation_MutexSemaphore (void);/*MTR 18.06.2019*/ // create all Mutex and Semaphores
void globalDataStrucutres_initData(void);			  /*MTR 20.06.2019*/ // set data validation byte and system status correctly

/*Set data functions*/ /*CHANGE MTR 18.06.2019 */
/*Functions give DATA_SET_OK or DATA_SET_FAIL as return value to check success of function call */
globalData_enumTypeDef_functionReturn 	globalDataStructures_setAllData					(globalData_typeDef_allData data);
globalData_enumTypeDef_functionReturn	globalDataStructures_setDataToGS				(globalData_typeDef_DataToGS data);
globalData_enumTypeDef_functionReturn	globalDataStructures_setPowerTrain_GS_IO		(globalData_typeDef_powerTrain_GS_IO data);			//RX
globalData_enumTypeDef_functionReturn	globalDataStructures_setPowerTrain_IO_GS		(globalData_typeDef_powerTrain_IO_GS data);			//TX
globalData_enumTypeDef_functionReturn	globalDataStructures_setEnvironmentSensor_IO_GS (globalData_typeDef_environmentSensor_IO_GS data);	//TX
globalData_enumTypeDef_functionReturn	globalDataStructures_setWeightCell_IO_GS		(globalData_typeDef_weightCell_IO_GS data);			//TX
globalData_enumTypeDef_functionReturn	globalDataStructures_setSystemState_IO_GS		(globalData_typeDef_systemState_IO_GS data);		//TX
globalData_enumTypeDef_functionReturn	globalDataStructures_setSecurity_GS_IO			(globalData_typeDef_security_GS_IO data);			//RX
globalData_enumTypeDef_functionReturn	globalDataStructures_setBMS_BMS_GS				(globalData_typeDef_BMS_BMS_GS data);				//TX
globalData_enumTypeDef_functionReturn	globalDataStructures_setRobotArm_GS_ARM			(globalData_typeDef_robotArm_GS_ARM data);			//RX
globalData_enumTypeDef_functionReturn	globalDataStructures_setRobotArm_ARM_GS			(globalData_typeDef_robotArm_ARM_GS data);			//TX
globalData_enumTypeDef_functionReturn	globalDataStructures_setDrill_GS_DRILL			(globalData_typeDef_drill_GS_DRILL data);			//RX
globalData_enumTypeDef_functionReturn 	globalDataStructures_setDrill_DRILL_GS			(globalData_typeDef_drill_DRILL_GS data);			//TX
globalData_enumTypeDef_functionReturn	globalDataStructures_setImuData_Gs				(globalData_typeDef_boschIMU data);					//Tx

/*Get data functions*/ /*CHANGE MTR 18.06.2019 */
/*Functions give a empty structure as return value, if data reading fails */
globalData_typeDef_allData 						globalDataStructures_getAllData					();
globalData_typeDef_DataToGS						globalDataStructures_getDataToGS				();
globalData_typeDef_powerTrain_GS_IO 			globalDataStructures_getPowerTrain_GS_IO		();		//RX
globalData_typeDef_powerTrain_IO_GS 		    globalDataStructures_getPowerTrain_IO_GS		();		//TX - okay
globalData_typeDef_environmentSensor_IO_GS 	    globalDataStructures_getEnvironmentSensor_IO_GS ();		//TX - okay
globalData_typeDef_weightCell_IO_GS 		    globalDataStructures_getWeightCell_IO_GS		();		//TX - okay
globalData_typeDef_systemState_IO_GS		    globalDataStructures_getSystemState_IO_GS		();		//TX - okay
globalData_typeDef_security_GS_IO 			    globalDataStructures_getSecurity_GS_IO			();		//RX
globalData_typeDef_BMS_BMS_GS 				    globalDataStructures_getBMS_BMS_GS				();		//TX
globalData_typeDef_robotArm_GS_ARM 			    globalDataStructures_getRobotArm_GS_ARM			();		//RX
globalData_typeDef_robotArm_ARM_GS 			    globalDataStructures_getRobotArm_ARM_GS			();		//TX - okay
globalData_typeDef_drill_GS_DRILL 			    globalDataStructures_getDrill_GS_DRILL			(); 	//RX
globalData_typeDef_drill_DRILL_GS 			    globalDataStructures_getDrill_DRILL_GS			();		//TX - okay
globalData_typeDef_boschIMU						globalDataStructures_getBoshImu					();

#endif /* GLOBALDATASTRUCTURES_CM_MTR_FROST_H_ */
