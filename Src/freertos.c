/* USER CODE BEGIN Header */

/*
*****************************************************************************
* @title  freertos.c
*
* @author Max Triller with CubeMX
*
* @date   Created on: ?!"?!.2018
*
* @brief  This is where magic happens! :D All application logic should be written in this file.
*
* 		  Necessary typedefs, defines and functions has to be in one extra pair of .h/,c file
*
* 			The entire program logic is processed in the corresponding task. Each
* 			task waits for new messages, when a new message is received the corresponding rx task is started.
* 			The tx task sends a message to the ground station in fixed time intervals. Read data Task are
* 			triggers by a semaphore that will be set
*
* 			Note: Safety/Timeout Task -> There will be a Task check Timeout (yea, you're right it's not ready yet)
* 					- when a Timeout occurs This task will call each active main Module e.g. drill, robotarm, powetrain
* 					- each module has to have an go to save state task only for this purpose ! please considere this fact ;)
*
* 			Note: Currently under heavy development!!
*
*
*
* @edit	- Last Change:
* 			- Max Triller 		24.06.2019	MTR - Change to Sensor Core
*
*******************************************************************************
*/


/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "globalDataStructures_CM_MTR_FRoST.h"
#include "udp_lwip_MTR_FRoST.h"
#include "can.h"
#include "PowerTrain.h"
#include "RobotArm.h"
#include "bno055_stm32.h"
#include <stdlib.h>
#include "adc.h"
#include "safety.h"
#include "bms_MTR_FRoST.h"
#include "rgb_indicator_MTR_FRoST.h"
#include "IkSolver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	FREERTOS_DISABLE 	= 0,
	FREERTOS_ENABLE 	= 1
}freertos_typedef_ONOFF;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*Added for conditional inclusion - Note: not all modules has to be compatible at the same time - MTR 20.06.2019*/
#define FREERTOS_DEMO_MODE		 	 0 /*Demo mode Rx messages are copied and send back to GS*/
#define FREERTOS_POWERTRAIN		 	 0 /*Activate all tasks for driving the rover*/
#define FREERTOS_ARM		 		 1 /*Activate all tasks for robot arm*/
#define FREERTOS_BMS				 0 /*Activate all tasks for measure cell voltage and current*/
#define FREERTOS_ENVIROMENT_SENSOR	 0 /*Activate all task for temperature, humidity and pressure */
#define FREERTOS_WEIGHT_CELL 		 0 /*Activate all tasks to measure probe, some scientific measurements can be also added here*/
#define FREERTOS_SAFETY				 1 /*Adds GS timeout on lost signal*/
#define FREERTOS_SYSTEM_STATE		 0 /*Adds Functionality for the RGB status indicator and sets gpio soft and hard reset pin*/
#define FREERTOS_DRILL				 0 /*Activate all tasks for the drilling unit*/
#define FREERTOS_BOSCH_IMU 			 0 /*Activate readout of BNO055 IMU sensor*/

/*inclusion of CANopen library - CM 10.07.2019*/
#if FREERTOS_POWERTRAIN || FREERTOS_ARM /* if needed by other hardware, add here */
#define FREERTOS_CANOPENNODE         1
#else
#define FREERTOS_CANOPENNODE         0
#endif // FREERTOS_POWERTRAIN || FREERTOS_ARM

/*Define priority levels for different types of tasks*/
/*Note: Not implemented yet! Must be checked MTR 20.06.2019*/
#define FREERTOS_PRIO_RX_DATA_GS 	osPriorityRealtime
#define FREERTOS_PRIO_TX_DATA_GS	osPriorityHigh
#define FREERTOS_PRIO_READ_DATA		osPriorityNormal

/*define parameters for signal flags not in use yet MTR 1.07.2019 */
#define FREERTOS_WHAIT_FOREVER 	0
#define FREERTOS_GO_FLAG 		1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart3;



extern globalData_typeDef_drill_DRILL_GS						globalData_drill_drillToGs			;
extern globalData_typeDef_drill_GS_DRILL						globalData_drill_gsToDrill			;
extern globalData_typeDef_robotArm_ARM_GS						globalData_arm_armToGs				;
extern globalData_typeDef_robotArm_GS_ARM						globalData_arm_gsToArm				;
extern globalData_typeDef_BMS_BMS_GS							globalData_bms_bmsToGs				;
extern globalData_typeDef_security_GS_IO						globalData_security_gsToIo			;
extern globalData_typeDef_systemState_IO_GS						globalData_systemState_ioToGs		;
extern globalData_typeDef_weightCell_IO_GS						globalData_weightCell_ioToGs		;
extern globalData_typeDef_environmentSensor_IO_GS				globalData_environmentSensor_ioToGs	;
extern globalData_typeDef_powerTrain_GS_IO						globalData_powerTrain_gsToIO		;
extern globalData_typeDef_powerTrain_IO_GS						globalData_powerTrain_ioToGs		;

/*Counter for run time measurement, will be incremented in tim6 ISR see also stm32f7xx_it.h/c
 *http://blog.atollic.com/visualizing-run-time-statistics-using-freertos */
volatile unsigned long ulHighFrequencyTimerTicks = 0;

/*Semaphore -----------------------------------------------------------------*/
/*Semaphore for signaling new Tx data to Gs - extern from globalDataStructure.h/c*/ /*MTR 18.06.2019*/
extern osSemaphoreId s_globalDataStructureHandle_newPowerTrain_ioToGs;
extern osSemaphoreId s_globalDataStructureHandle_newEnvironmentSensor_ioToGs;
extern osSemaphoreId s_globalDataStructureHandle_newWeightCell_ioToGs;
extern osSemaphoreId s_globalDataStructureHandle_newSystemState_ioToGs;
extern osSemaphoreId s_globalDataStructureHandle_newSecurity_ioToGs;
extern osSemaphoreId s_globalDataStructureHandle_newBMS_bmsToGs;
extern osSemaphoreId s_globalDataStructureHandle_newArm_armToGs;
extern osSemaphoreId s_globalDataStructureHandle_newDrill_DrillToGs;


/*Semaphore for signaling new Rx data from Gs*/ /*MTR 18.06.2019*/
extern osSemaphoreId s_globalDataStructureHandle_newPowerTrain_GsToIo;
extern osSemaphoreId s_globalDataStructureHandle_newSystemState_GsToIo;
extern osSemaphoreId s_globalDataStructureHandle_newSecurity_GsToIO;
extern osSemaphoreId s_globalDataStructureHandle_newArm_GsToAm;
extern osSemaphoreId s_globalDataStructureHandle_newDrill_GsToDrill;

/*Semaphore for trigger new Sensor readout*/
extern osSemaphoreId s_globalDataStructureHandle_readNewSensorData_BNO055;

/*Internal Semaphores for triggering new Sensor Data*/
osSemaphoreId s_freertosHandle_readTempSensor; //Added MTR 11.07.2019
osSemaphoreId s_freertosHandle_TempDataready;  //Added MTR 11.07.2019
osSemaphoreId s_freertosHandle_readBmsData;    //Added MTR 18.07.2019
osSemaphoreId s_freertosHandle_bmsDataReady;   //Added MTR 18.07.2019

osSemaphoreId s_freertosHandle_armIk;   //Added SE 12.08.2019
osSemaphoreId s_freertosHandle_armTp;
osSemaphoreId s_freertosHandle_armAxis;
osSemaphoreId s_freertosHandle_armCheckEncoder;

/*Add task handle for processing rover Data or do something else important */ /*MTR 18.06.2019*/
osThreadId t_freertosHandle_DemoMode_tx;					/* Added MTR 20.06.2019*/
osThreadId t_freertosHanlde_DemoMode_rx; 					/* Added MTR 24.06.2019*/
osThreadId t_freertosHandle_PowerTrain_GS_IO;				//RX - GS
osThreadId t_freertosHandle_PowerTrain_IO_GS;				//TX - GS
osThreadId t_freertosHandle_PowerTrain_cyclic;              //cyclic powertrain task
osThreadId t_freertosHandle_EnvironmentSensor_IO_GS;		//TX - GS
osThreadId t_freertosHandle_EnvironmentSensor_readData; 	/* Added MTR 20.06.2019*/
osThreadId t_freertosHandle_WeightCell_IO_GS;				//TX - GS
osThreadId t_freertosHandle_WeightCell_readData; 			/* Added MTR 20.06.2019*/
osThreadId t_freertosHandle_SystemState_IO_GS;				//TX - GS
osThreadId t_freertosHandle_SystemState_displayStateRGB;	/* Added MTR 20.06.2019*/
osThreadId t_freertosHandle_Security_GS_IO;					//RX - GS
osThreadId t_freertosHandle_Security_checkTimeout;			/* Added MTR 20.06.2019*/
osThreadId t_freertosHandle_BMS_BMS_GS;						//TX - GS
osThreadId t_freertosHandle_BMS_readData;					/* Added MTR 20.06.2019*/
osThreadId t_freertosHandle_RobotArm_GS_ARM;				//RX - GS
osThreadId t_freertosHandle_RobotArm_ARM_GS;				//TX - GS
osThreadId t_freertosHandle_RobotArm_readData;				/* Added MTR 20.06.2019*/
osThreadId t_freertosHandle_RobotArm_modeIk;				/* Added SE 12.08.2019*/
osThreadId t_freertosHandle_RobotArm_modeTp;				/* Added SE 12.08.2019*/
osThreadId t_freertosHandle_RobotArm_modeAxis;				/* Added SE 12.08.2019*/
osThreadId t_freertosHandle_RobotArm_checkEncoderLimits;	/* Added SE 12.08.2019*/
osThreadId t_freertosHandle_Drill_GS_DRILL;					//RX - GS
osThreadId t_freertosHandle_Drill_DRILL_GS;					//TX - GS
osThreadId t_freertosHandle_CanOpen_Cyclic;                 //cyclic CanOpen task
osThreadId t_freertosHandle_Can1RxFifo0;                    //CAN1 Fifo0 message pending task
osThreadId t_freertosHandle_IMU_BNO055_GS; 					//TX _ GS Added MTR 09.07.2019
osThreadId t_freertosHandle_IMU_BNO055_readNewData; 		//Read Data added MTR 09.07.2019
osThreadId t_freertosHanlde_readChipTemp; 					//Added MTR 11.07.2019

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osTimerId myTimer01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


/*Add task in their respective section! MTR 19.06.18*/

/*Demo mode tasks  ----------------------------*/
#if FREERTOS_DEMO_MODE
void freertosTask_demoMode_txData(void const * argument);
void freertosTask_demoMode_rxData(void const * argument);
#endif

/*Powertrain tasks  ----------------------------*/
#if FREERTOS_POWERTRAIN
void freertosTask_powerTrain_RxData_gs(void const * argument); //Receive data from GS
void freertosTask_powerTrain_TxData_gs(void const * argument); //Send data to GS
void freertosTask_powerTrain_CyclicTask(void const * argument);//Read actual velocity and state word
#endif

/*CanOpen tasks     ----------------------------*/
#if FREERTOS_CANOPENNODE
void freertosTask_Can1RxFifo0(void const * argument);
void freertosTask_CanOpen_CyclicTask(void const * argument);
#endif

/*Environment Sensor tasks  ----------------------------*/
#if FREERTOS_ENVIROMENT_SENSOR
void freertosTask_EnvironmentSensor_readData(void const * argument); //Read data from Sensor
void freertosTask_EnvironmentSensor_TxData_gs(void const * argument); //Send Sensor Data to GS
#endif

/*Weight Cell tasks  ----------------------------*/
#if FREERTOS_WEIGHT_CELL
void freertosTask_weightCell_readData(void const * argument);//Read data from Sensor
void freertosTask_weightCell_TxData_gs(void const * argument); //Send Sensor Data to GS
#endif

/*System State tasks  ----------------------------*/
#if FREERTOS_SYSTEM_STATE
void freertosTask_systemState_displayStateRGB(void const * argument); 	//Set state and send UART Data to RGB status Indicator
void freertosTask_systemState_TxSystemState_gs(void const * argument); 	//Check and send actual rover state to ground station
void freertosTask_systemState_readChipTemp(void const * argument); 		//Added MTR 11.07.2019
#endif

/*Security tasks  ----------------------------*/
#if FREERTOS_SAFETY
void freertosTask_safety_checkTimeout(void const * argument);	//Check message timeout from GS, if true go to predefined state
void freertosTask_safety_softEmergencyStop(void const * argument); //Software based emergency stop ->rover go to save state
#endif

/*Robot arm tasks  ----------------------------*/
#if FREERTOS_ARM
void freertosTask_arm_readData(void const* argument); //Read encoder data triggers from semaphore - set by arm tx Data Task
void freertosTask_arm_RxData_gs(void const* argument); // Receive new data from GS
void freertosTask_arm_TxData_gs(void const* argument); // Send data to GS
void freertosTask_arm_modeIk(void const* argument); // IK mode task
void freertosTask_arm_modeTp(void const* argument); // teached position mode task
void freertosTask_arm_modeAxis(void const* argument); // IK mode task
void freertosTask_arm_checkEncoderLimits(void const* argument); // IK mode task
#endif

/*Drill tasks  ----------------------------*/
#if FREERTOS_DRILL
void freertosTask_drill_TxData_gs(void const * argument); //Transmit drill unit state to Gs
void freertosTask_drill_RxData_gs(void const * argument); //Receive Data from Gs
#endif

/*BMS task-----------------------------------*/
#if FREERTOS_BMS
void freertosTask_BMS_TxData_gs(void const * argument); //Transmit bms data to gs
void freertosTask_BMS_readData(void const * argument); 	//Read ADC / BMS data
#endif
   
/*BOSCH IMU tasks---------------------------------*/
#if FREERTOS_BOSCH_IMU
void freertosTask_IMU_TxData_gs(void const * argument); //Transmit IMU data to gs
void freertosTask_IMU_readData(void const * argument); 	//Read IMU Data from I2C interface
#endif

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Callback01(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	/*Note: Not needed -> is done by Cube MX in Main MTR 24.06.2019
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 5400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
	Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
	Error_Handler();
  }
  */
  /*Start Timer MTR 24.06.2019*/
  HAL_TIM_Base_Start(&htim6);
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE );


}

__weak unsigned long getRunTimeCounterValue(void)
{

return ulHighFrequencyTimerTicks;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /*Initialize globalDataStructure correctly*/
  globalDataStructures_initOsImplementation_MutexSemaphore(); 	/*MTR 18.06.2019*/ // create all Mutex and Semaphores for globalDataStructures.h/c
  globalDataStrucutres_initData(); 								/*MTR 20.06.2019*/ // set data validation byte and system status correctly

  rgb_setState(SYSTEMSTATE_ON);
  rgb_displayState();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityRealtime, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */


  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN StartDefaultTask */

  /*Initialize MIDDELWARE, hardware and periphery section--------------------------------------------------------*/

  /*Initialize ETHERNET UDP interface correctly */
  udp_lwip_initClient();
  udp_lwip_initServer();
  udp_lwip_initInterface();			/*Note: Must be called while program initialization!*/
  udp_lwip_initTransmitMutex(); 	/* Call this on initialization when MUTEXES are used*/ /*Added MTR 22.06.2019*/
  CAN_FreeRTOS_Implementation();


  /*Initialize BOSCH IMU sensor over I2C Added by MTR 09.07.2019*/
  /*Note: Functions are not called in main.c because multiple definitions when including it freertos.c and main.c */
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeConfig();
	bno055_enableExternalCrystal();
	bno055_setOperationModeNDOF();

  /* Task creating section---------------------------------------------------------------------------------------------*/
  /*Create task in their respective section! MTR 20.06.18*/
   /*Note: While developing application, sometimes the stack size must be increased.*/
   /*See: -> #define osThreadDef(name, thread, priority, instances, STACK SIZE)*/

   /*Create demo mode tasks  ----------------------------------------*/
   #if FREERTOS_DEMO_MODE

 	  /*Task freertosTask_demoMode */ //Adds demo mode for GS development
 	  osThreadDef(t_DemoMode_tx, freertosTask_demoMode_txData, osPriorityRealtime, 0, 128*2);
 	  t_freertosHandle_DemoMode_tx = osThreadCreate(osThread(t_DemoMode_tx), NULL);

 	  osThreadDef(t_DemoMode_rx, freertosTask_demoMode_rxData, osPriorityRealtime, 0, 128);
 	  t_freertosHanlde_DemoMode_rx = osThreadCreate(osThread(t_DemoMode_rx), NULL);

   #endif

#if FREERTOS_CANOPENNODE
     canOpenNode_initialize();

       /* Task freertosTask_CanOpen_CyclicTask */ // cyclic check error and timeout
       osThreadDef(t_CanOpen_cyclic, freertosTask_CanOpen_CyclicTask, osPriorityAboveNormal, 0, 256);
       t_freertosHandle_CanOpen_Cyclic = osThreadCreate(osThread(t_CanOpen_cyclic),NULL);

       /* Task freertosTask_Can1RxFifo0 */ //Can1 Fifo0 Receive
       osThreadDef(t_Can1RxFifo0, freertosTask_Can1RxFifo0, osPriorityRealtime, 0, 128);
       t_freertosHandle_Can1RxFifo0 = osThreadCreate(osThread(t_Can1RxFifo0),NULL);
#endif

   /*Create powertrain tasks  ---------------------------------------------------*/
   #if FREERTOS_POWERTRAIN

       /*Task freertosTask_powerTrain_cyclic */ //cyclic read velocity and state word
       osThreadDef(t_PowerTrain_cyclic, freertosTask_powerTrain_CyclicTask, osPriorityNormal, 0, 1024);
       t_freertosHandle_PowerTrain_cyclic = osThreadCreate(osThread(t_PowerTrain_cyclic),NULL);

 	  /*Task freertosTask_powerTrain_RxData_gs */ //Receive data from GS
 	  osThreadDef(t_PowerTrain_GS_IO_Rx, freertosTask_powerTrain_RxData_gs, osPriorityRealtime, 0, 128);
 	  t_freertosHandle_PowerTrain_GS_IO = osThreadCreate(osThread(t_PowerTrain_GS_IO_Rx), NULL);

 	  /*Task freertosTask_powerTrain_TxData_gs */ //Send data to GS
 	  osThreadDef(t_PowerTrain_GS_IO_Tx, freertosTask_powerTrain_TxData_gs, osPriorityHigh, 0, 256);
 	  t_freertosHandle_PowerTrain_GS_IO = osThreadCreate(osThread(t_PowerTrain_GS_IO_Tx), NULL);

 	  /*enable power to electronics aka softreset when pin goes to low*/
   #endif

   /*Create environment sensor tasks  -------------------------------------------------*/
   #if FREERTOS_ENVIROMENT_SENSOR

 	  /*Task freertosTask_EnvironmentSensor_readData */ //Read data from Sensor
 	  osThreadDef(t_EnvironmentSensor_readData, freertosTask_EnvironmentSensor_readData, osPriorityNormal, 0, 128);
 	  t_freertosHandle_EnvironmentSensor_readData = osThreadCreate(osThread(t_EnvironmentSensor_readData), NULL);

 	  /*Task freertosTask_EnvironmentSensor_TxData_gs */
 	  osThreadDef(t_EnvironmentSensor_IO_GS, freertosTask_EnvironmentSensor_TxData_gs, osPriorityHigh, 0, 128);
 	  t_freertosHandle_EnvironmentSensor_readData = osThreadCreate(osThread(t_EnvironmentSensor_IO_GS), NULL);

   #endif

   /*Create weight cell tasks  -----------------------------------------------------*/
   #if FREERTOS_WEIGHT_CELL

 	  /*Task freertosTask_weightCell_readData */ //Read data from Sensor
 	  osThreadDef(t_WeightCell_readData, freertosTask_weightCell_readData, osPriorityNormal, 0, 128);
 	  t_freertosHandle_WeightCell_readData = osThreadCreate(osThread(t_WeightCell_readData), NULL);

 	  /*Task freertosTask_weightCell_TxData_gs */ //Send Sensor Data to GS
 	  osThreadDef(t_WeightCell_IO_GS, freertosTask_weightCell_TxData_gs, osPriorityHigh, 0, 128);
 	  t_freertosHandle_WeightCell_IO_GS = osThreadCreate(osThread(t_WeightCell_IO_GS), NULL);

   #endif

   /*Create system state tasks  ------------------------------------------------*/
   #if FREERTOS_SYSTEM_STATE


 	 osSemaphoreDef (s_freertos_readTempSensor);
 	 s_freertosHandle_readTempSensor = osSemaphoreCreate(osSemaphore(s_freertos_readTempSensor), 1);

	 osSemaphoreDef (s_freertosHandle_TempDataready);
	 s_freertosHandle_TempDataready = osSemaphoreCreate(osSemaphore(s_freertosHandle_TempDataready), 1);

	 //Reset Semaphore, this has to be done because semaphores are set by default
	 osSemaphoreWait(s_freertosHandle_readTempSensor,osWaitForever);
	 osSemaphoreWait(s_freertosHandle_TempDataready,osWaitForever);

 	  /*Task freertosTask_systemState_displayStateRGB */ //Set state and send UART Data to RGB status Indicator
 	  osThreadDef(t_SystemState_displayStateRGB, freertosTask_systemState_displayStateRGB, osPriorityHigh, 0, 128);
 	  t_freertosHandle_SystemState_displayStateRGB = osThreadCreate(osThread(t_SystemState_displayStateRGB), NULL);

 	  /*Task freertosTask_systemState_TxState_gs */ //Check and send actual rover state to ground station
 	  osThreadDef(t_SystemState_IO_GS, freertosTask_systemState_TxSystemState_gs, osPriorityRealtime, 0, 128*2); // in realtime ?!?! MTR
 	  t_freertosHandle_SystemState_IO_GS = osThreadCreate(osThread(t_SystemState_IO_GS), NULL);

 	  /* Task for reading chip Temperature for developing application - Added MTR 11.07.2019*/
 	  osThreadDef(t_freertos_readChipTemp, freertosTask_systemState_readChipTemp, osPriorityNormal, 0, 128*2);
 	  t_freertosHanlde_readChipTemp = osThreadCreate(osThread(t_freertos_readChipTemp), NULL);

   #endif

   /*Create security tasks  ------------------------------------------------------*/
   #if FREERTOS_SAFETY

 	  /*Task freertosTask_savety_checkTimeout */ //Check message timeout from GS, if true go to predefined state
 	  osThreadDef(t_Security_checkTimeout, freertosTask_safety_checkTimeout, osPriorityRealtime, 0, 128);
 	  t_freertosHandle_Security_checkTimeout = osThreadCreate(osThread(t_Security_checkTimeout), NULL);

 	  /*Task freertosTask_savety_softEmergencyStop */ //Software based emergency stop ->rover go to save state
// 	  osThreadDef(t_Security_GS_IO, freertosTask_safety_softEmergencyStop, osPriorityRealtime, 0, 128);
// 	  t_freertosHandle_Security_GS_IO = osThreadCreate(osThread(t_Security_GS_IO), NULL);

   #endif

   /*Create robot arm tasks  -------------------------------------------------------*/
   #if FREERTOS_ARM

  	  osSemaphoreDef (s_freertosHandle_armIk);
   	  s_freertosHandle_armIk = osSemaphoreCreate(osSemaphore(s_freertosHandle_armIk), 1);
 	  osSemaphoreRelease(s_freertosHandle_armIk);

  	  osSemaphoreDef (s_freertosHandle_armTp);
      s_freertosHandle_armTp = osSemaphoreCreate(osSemaphore(s_freertosHandle_armTp), 1);
 	  osSemaphoreRelease(s_freertosHandle_armTp);

  	  osSemaphoreDef (s_freertosHandle_armAxis);
   	  s_freertosHandle_armAxis = osSemaphoreCreate(osSemaphore(s_freertosHandle_armAxis), 1);
 	  osSemaphoreRelease(s_freertosHandle_armAxis);

  	  osSemaphoreDef (s_freertosHandle_armCheckEncoder);
  	  s_freertosHandle_armCheckEncoder = osSemaphoreCreate(osSemaphore(s_freertosHandle_armCheckEncoder), 1);
 	  osSemaphoreRelease(s_freertosHandle_armCheckEncoder);

 	  /* FIXME SE here INIT functions */
      RobotArm_init();

 	  /*Task freertosTask_arm_readData */ // E.g. read encoder data that triggers from semaphore - set by arm tx Data Task
 	  osThreadDef(t_RobotArm_readData, freertosTask_arm_readData, osPriorityNormal, 0, 128);
 	  t_freertosHandle_RobotArm_readData = osThreadCreate(osThread(t_RobotArm_readData), NULL);

 	  /*Task freertosTask_arm_RxData_gs */ // Receive new data from GS
 	  osThreadDef(t_RobotArm_GS_ARM, freertosTask_arm_RxData_gs, osPriorityRealtime, 0, 2048);
 	  t_freertosHandle_RobotArm_GS_ARM = osThreadCreate(osThread(t_RobotArm_GS_ARM), NULL);

 	  /*Task freertosTask_arm_TxData_gs */ // Send data to GS
 	  osThreadDef(t_RobotArm_ARM_GS, freertosTask_arm_TxData_gs, osPriorityNormal, 0, 128);
 	  t_freertosHandle_RobotArm_ARM_GS = osThreadCreate(osThread(t_RobotArm_ARM_GS), NULL);

 	  /*Task freertosTask_arm MODE IK */ //
 	  osThreadDef(t_RobotArm_modeIk, freertosTask_arm_modeIk, osPriorityRealtime, 0, 128*130);
 	  t_freertosHandle_RobotArm_modeIk = osThreadCreate(osThread(t_RobotArm_modeIk), NULL);

 	  /*Task freertosTask_arm */ //
 	  osThreadDef(t_RobotArm_modeTp, freertosTask_arm_modeTp, osPriorityNormal, 0, 128);
 	  t_freertosHandle_RobotArm_modeTp = osThreadCreate(osThread(t_RobotArm_modeTp), NULL);

 	  /*Task freertosTask_arm */ //
 	  osThreadDef(t_RobotArm_modeAxis, freertosTask_arm_modeAxis, osPriorityNormal, 0, 128);
 	  t_freertosHandle_RobotArm_modeAxis = osThreadCreate(osThread(t_RobotArm_modeAxis), NULL);

 	  /*Task freertosTask_arm */ //
 	  osThreadDef(t_RobotArm_checkEncoder, freertosTask_arm_checkEncoderLimits, osPriorityNormal, 0, 128);
 	  t_freertosHandle_RobotArm_checkEncoderLimits = osThreadCreate(osThread(t_RobotArm_checkEncoder), NULL);

   #endif

   /*Create drill tasks  ---------------------------------------------------------*/
   #if FREERTOS_DRILL

 	  /*Task freertosTask_drill_TxData_gs */ //Transmit drill unit state to Gs
 	  osThreadDef(t_Drill_DRILL_GS, freertosTask_drill_TxData_gs, osPriorityHigh, 0, 128);
 	  t_freertosHandle_Drill_DRILL_GS = osThreadCreate(osThread(t_Drill_DRILL_GS), NULL);

 	  /*Task freertosTask_drill_RxData_gs */ //Receive Data from Gs
 	  osThreadDef(t_Drill_GS_DRILL, freertosTask_drill_RxData_gs, osPriorityRealtime, 0, 128);
 	  t_freertosHandle_Drill_GS_DRILL = osThreadCreate(osThread(t_Drill_GS_DRILL), NULL);

   #endif

	#if FREERTOS_BOSCH_IMU

 	  /*Task for sending IMU Data to GS */
 	  osThreadDef(t_IMU_BNO055_GS, freertosTask_IMU_TxData_gs, osPriorityHigh, 0, 128*2);
 	 t_freertosHandle_IMU_BNO055_GS = osThreadCreate(osThread(t_IMU_BNO055_GS), NULL);

 	  /*Task for Sensor Readout*/
 	  osThreadDef(t_IMU_BNO055_readNewData, freertosTask_IMU_readData, osPriorityNormal, 0, 128*2);
 	 t_freertosHandle_IMU_BNO055_readNewData = osThreadCreate(osThread(t_IMU_BNO055_readNewData), NULL);

	#endif

	#if FREERTOS_BMS

 	 //create semaphores
	 osSemaphoreDef (s_freertosHandle_readBmsData);
	 s_freertosHandle_readBmsData = osSemaphoreCreate(osSemaphore(s_freertosHandle_readBmsData), 1);

	 osSemaphoreDef (s_freertosHandle_bmsDataReady);
	 s_freertosHandle_bmsDataReady = osSemaphoreCreate(osSemaphore(s_freertosHandle_bmsDataReady), 1);

	 //Reset Semaphore, this has to be done because semaphores are set by default
	 osSemaphoreWait(s_freertosHandle_readBmsData,osWaitForever);
	 osSemaphoreWait(s_freertosHandle_bmsDataReady,osWaitForever);

 	//Transmit bms data to gs
	  osThreadDef(t_BMS_BMS_GS, freertosTask_BMS_TxData_gs, osPriorityRealtime, 0, 128*2);
	  t_freertosHandle_BMS_BMS_GS = osThreadCreate(osThread(t_BMS_BMS_GS), NULL);

	 //Read ADC / BMS data
	  osThreadDef(t_BMS_readData, freertosTask_BMS_readData, osPriorityNormal, 0, 128*2);
	  t_freertosHandle_BMS_readData = osThreadCreate(osThread(t_BMS_readData), NULL);


	#endif


 	 //end creating tasks MTR 09.07.2019
  for(;;)
  {

    osDelay(10000);
    //Note: Task can be deleted?
  }
  /* USER CODE END StartDefaultTask */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
  
  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Write application in their respective section! MTR 20.06.18********************/
/*Demo mode application code  ----------------------------*/

#if FREERTOS_DEMO_MODE
/**
  * @brief  Implementing the freertosTask_demoMode RX and TX
  *
  * 		Note: This mode is for developing ground station and GUI functionality!
  * 		What the code does: Sending rover status data from globalDataStructures.h/c to ground station.
  * 		For details which data and default values see freertosTask_demoMode TX data initialization section.
  *
  * 		When control data is send to MCU the joystick values overwrite target velocity.
  * 		Actual velocity will be set to old target velocity.
  *
  * 		Note: Works actual only for power train data !! MTR 24.06.2019
  *
  * 		Tx task: Create Data, write to global structure and sends periodical data to GS.
  * 		Rx task: Triggered by incoming udp (Ethernet) data. Data can be accessed by globalDataStructures_set*******()
  * 					reference -> globalDataStructures.h
  *
  * 		Author: 	Max Triller
  * 		Last edit:	MTR 20.06.2019
  *
  * @param  argument: Not used
  * @retval None
  */

	/* Demo mode transmit task .... setup section -> generate demo data .....
	 * loop section -> wakes up in osDelay() intervals and send data from data structure MTR 25.06.2019*/
	void freertosTask_demoMode_txData(void const * argument)
	{
		/*Code that runs only once---------------------*/
		//codes does something e.g. create variables

		/*Create variables that will be send to GS by default*/
		static globalData_typeDef_powerTrain_IO_GS 		d_powerTrain_tx ;
		//globalData_typeDef_weightCell_IO_GS			d_weightCell_tx = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
		//globalData_typeDef_BMS_BMS_GS					d_batteryMS_tx  = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
		//globalData_typeDef_robotArm_ARM_GS			d_arm_tx 		= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
		//globalData_typeDef_environmentSensor_IO_GS	d_eSensor_tx	= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
		//globalData_typeDef_drill_DRILL_GS				d_drill_tx		= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

		/*Fill structure with demo data*/
		d_powerTrain_tx.actualState			= ENABLE  ;
		d_powerTrain_tx.actualVelocityLeft  = 2;
		d_powerTrain_tx.actualVelocityRight = 3;
		d_powerTrain_tx.targetVelocityLeft 	= 4;
		d_powerTrain_tx.targetVelocityRight = 6;

		d_powerTrain_tx.dataID = GLOBALDATA_ID_POWERTRAIN;

		/*Write Data to global structure and check success*/
		if (globalDataStructures_setPowerTrain_IO_GS(d_powerTrain_tx) != GLOBAL_DATA_STRUCT_SET_OK){
			Error_Handler();
		}

		/*End of code that runs only once ---------------------*/

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
			//do something before task gives runtime back to OS
			//...
			//do something when the OS gives runtime
			//...
			//Transmit demo powetrain data to GS and check success
			//d_powerTrain_tx = globalDataStructures_getPowerTrain_IO_GS();
			uint8_t returnVal = udp_lwip_txUdpMessage_gs(GLOBALDATA_ID_POWERTRAIN);
			if (returnVal!= UDP_LWIP_SEND_OK)
			{
				Error_Handler();
			}
			osDelay(100); //Give back the control to the scheduler in ms
		}
	}

	/* Demo mode rx task -> is triggered by receiving new powetrain data (semaphore s_globalDataStructureHandle_newPowerTrain_GsToIo)
	 * copies data from rx (target state) data to tx (actual state) */
	void freertosTask_demoMode_rxData(void const * arument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables
		static globalData_typeDef_powerTrain_GS_IO d_powerTrain_rx = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
		static globalData_typeDef_powerTrain_IO_GS d_powerTrain_tx = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

		d_powerTrain_tx.dataID = GLOBALDATA_ID_POWERTRAIN;
		d_powerTrain_rx.dataID = GLOBALDATA_ID_POWERTRAIN;

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//Wait for signal (semaphore) become set -> see also udpLwip_MTR_FRoST.h.c "dp_lwip_rxCallback_gs()"
			if (osSemaphoreWait(s_globalDataStructureHandle_newPowerTrain_GsToIo,osWaitForever)== osOK)
			//if (0)
			{
				//do something when semaphore is set

				//... e.g. read new data from structure
				d_powerTrain_rx = globalDataStructures_getPowerTrain_GS_IO(); //Note: When getData fails, function returns empty structure!
				//d_powerTrain_tx = globalDataStructures_getPowerTrain_IO_GS();

				/*Check returned data */
				if((d_powerTrain_rx.dataID) == GLOBALDATA_ID_POWERTRAIN )//Has to be correct, if incorrect or 0 reading data failed
				{
					/*Overwrite old data*/
					/*Old target velocity becomes actual velocity */
					d_powerTrain_tx.actualVelocityLeft 	= d_powerTrain_tx.targetVelocityLeft;
					d_powerTrain_tx.actualVelocityRight = d_powerTrain_tx.targetVelocityRight;

					/*Old target velocity becomes overwritten with XY joystick value*/
					d_powerTrain_tx.targetVelocityLeft 	= d_powerTrain_rx.xValue;
					d_powerTrain_tx.targetVelocityRight = d_powerTrain_rx.yValue;

					/*Write new data to data structure and check success*/
					if (globalDataStructures_setPowerTrain_IO_GS(d_powerTrain_tx)!=GLOBAL_DATA_STRUCT_SET_OK)
					{
						Error_Handler();
					}
				}
				else
				{
					Error_Handler();
				}
			}
			//osSemaphoreRelease(s_globalDataStructureHandle_newPowerTrain_GsToIo);
			osDelay(1);
		}
	}

#endif // FREERTOS_DEMO_MODE

/* PowerTrain application code --------------------------------------------------*/

#if FREERTOS_POWERTRAIN
    void freertosTask_powerTrain_RxData_gs(void const * argument)
    {
        /*Code that runs only once---------------------*/

        //codes does something e.g. create variables

        /* Infinite loop -------------------------------*/
        for(;;)
        {
            //Wait for signal (semaphore) become set -> see also udpLwip_MTR_FRoST.h.c "dp_lwip_rxCallback_gs()"
            if (osSemaphoreWait(s_globalDataStructureHandle_newPowerTrain_GsToIo,osWaitForever)== osOK)
            {
                PowerTrain_updateState();
            }
        }
    }

    void freertosTask_powerTrain_TxData_gs(void const * argument)
    {
        /*Code that runs only once---------------------*/

        //codes does something e.g. create variables

        /* Infinite loop -------------------------------*/
        for(;;)
        {
            uint8_t returnVal = udp_lwip_txUdpMessage_gs(GLOBALDATA_ID_POWERTRAIN);
            if (returnVal!= UDP_LWIP_SEND_OK)
            {
                Error_Handler();
            }
            osDelay(100);
        }
    }

    void freertosTask_powerTrain_CyclicTask(void const * argument)//Read actual velocity and state word
    {
//        static uint8_t counter=0;
        int32_t velocity_left;
        int32_t velocity_right;
        /* initialize motors at startup */
        PowerTrain_initMotors();
        /* Infinite loop */
        for(;;)
        {
            /* read state word */
        #if TEST_MODE
            PowerTrain_readStateWord(NODE_ID_MOTOR_TEST);
            PowerTrain_readControlWord(NODE_ID_MOTOR_TEST);
        #else
            PowerTrain_readStateWord(NODE_ID_MOTOR_LEFT);
            PowerTrain_readStateWord(NODE_ID_MOTOR_RIGHT);
            PowerTrain_readControlWord(NODE_ID_MOTOR_LEFT);
            PowerTrain_readControlWord(NODE_ID_MOTOR_RIGHT);
        #endif
            /* read velocity */
            PowerTrain_readVelocity(&velocity_left,&velocity_right);


//            PowerTrain_setVelocity(0,100,100);
//            if(counter==1)
//                PowerTrain_drive();
//            else if(counter==50)
//                PowerTrain_shutdown();
//            else if(counter==100)
//                counter=0;
//            counter++;
            osDelay(200);
        }
    }

#endif // FREERTOS_POWERTRAIN

#if FREERTOS_CANOPENNODE
	void freertosTask_Can1RxFifo0(void const * argument)
    {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t buffer[8];
        /* Infinite loop */
        for(;;)
        {
            /* wait for can receive message pending */
            if(osSemaphoreWait(semCan1Fifo0MessagePendingHandle,osWaitForever)==osOK)
            {
                /* get can mutex */
                osMutexWait(mhcan1Handle,osWaitForever);
                /* get message */
                HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxHeader, buffer);
                /* reactivate interrupt */
                HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
                /* release mutex */
                osMutexRelease(mhcan1Handle);
                /* call function to handle message */
                canOpenNode_packetReceived(RxHeader.StdId,buffer);
            }
        }
    }

    void freertosTask_CanOpen_CyclicTask(void const * argument)
    {
		/* initialize motors at startup */
		//PowerTrain_initMotors();
		/* Infinite loop */
		for(;;)
		{
			// if motor/encoder init FIXME
			if(true)
			{
				// read state
				Arm_readStateWord(0x21);
				Arm_readStateWord(0x22);
				Arm_readStateWord(0x23);
				Arm_readStateWord(0x24);
				Arm_readStateWord(0x25);

				// read mode
				Arm_readControlWord(0x21);
				Arm_readControlWord(0x22);
				Arm_readControlWord(0x23);
				Arm_readControlWord(0x24);
				Arm_readControlWord(0x25);

				// read velocity
				Arm_readVelocity(0x21);
				Arm_readVelocity(0x22);
				Arm_readVelocity(0x23);
				Arm_readVelocity(0x24);
				Arm_readVelocity(0x25);
			}
			osDelay(200);
		}
    }

#endif // FREERTOS_CANOPENNODE

/*Environment Sensor application code  ----------------------------*/
#if FREERTOS_ENVIROMENT_SENSOR
	//Read data from Sensor
	void freertosTask_EnvironmentSensor_readData(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//Wait for semaphore
			//osSemaphoreWait(s_globalDataStructureHandle_newEnvironmentSensor_ioToGs,osWaitForever);
			//do something when semaphore is set
			//...
			osDelay(100);
			//Give back the control to the scheduler
		}
	}
	//Send Sensor Data to GS
	void freertosTask_EnvironmentSensor_TxData_gs(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//do something before task gives runtime back to OS
			//...


			osDelay(1000); //Give back the control to the scheduler
			//do something when the OS gives runtime
			//..
		}
	}

#endif

/*Weight Cell application code  ----------------------------*/
#if FREERTOS_WEIGHT_CELL
	//Read data from Sensor
	void freertosTask_weightCell_readData(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//Wait for Signal
			//osSemaphoreWait(s_globalDataStructureHandle_newWeightCell_ioToGs,osWaitForever);
			//do something when semaphore is set
			//...
			osDelay(10);
			//Give back the control to the scheduler
		}
	}
	//Send Sensor Data to GS
	void freertosTask_weightCell_TxData_gs(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//do something before task gives runtime back to OS
			//...
			osDelay(10); //Give back the control to the scheduler
			//do something when the OS gives runtime
			//..
		}
	}
#endif

/*System State application code  ----------------------------*/
#if FREERTOS_SYSTEM_STATE
	//Set state and send UART Data to RGB status Indicator
	void freertosTask_systemState_displayStateRGB(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//do something before task gives runtime back to OS
			//...
			osDelay(9999999); //Give back the control to the scheduler
			//do something when the OS gives runtime
			//..
		}
	}
	//Check and send actual rover state to ground station ADDED MTR 11.07.2019
	void freertosTask_systemState_TxSystemState_gs(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//create data set for handling data
		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//do something before task gives runtime back to OS
			//...
			//read data from global data structure
				if (udp_lwip_txUdpMessage_gs(GLOBALDATA_ID_SYSTEMSTATE)== UDP_LWIP_SEND_OK)
				{
					osSemaphoreRelease(s_freertosHandle_readTempSensor);
				}else {
					Error_Handler();
				}
			//display system state
			rgb_displayState();
			osDelay(500); //Give back the control to the scheduler
			//do something when the OS gives runtime
			//..
		}
	}

	void freertosTask_systemState_readChipTemp(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		//create data container
		globalData_typeDef_systemState_IO_GS d_systemState_tx;
		d_systemState_tx.dataID = GLOBALDATA_ID_SYSTEMSTATE; //Set correct id - this is important to write data into global structure correctly

		//create variable to store raw adc data
		float  temperature;

		//Enable it


		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//wait for semaphore is set by system state tx task
			if(osSemaphoreWait(s_freertosHandle_readTempSensor,osWaitForever)== osOK)
			{
				HAL_ADC_Start_IT(&hadc1);
			 	 HAL_ADC_Start(&hadc1);
				//wait that adc completes conversion
				if (osSemaphoreWait(s_freertosHandle_TempDataready,osWaitForever)== osOK)
				{
					//read data from structure
					d_systemState_tx = globalDataStructures_getSystemState_IO_GS();

					//read ADC data and store new information into structure
					temperature = ((HAL_ADC_GetValue(&hadc1))/4095.0)*3300.0;
					temperature = ((temperature - 760.0)/2.5);
					temperature =  temperature + 25.0;
					d_systemState_tx.chipTemp =  (uint8_t )temperature;

					//try to store data into global data structure
					if(globalDataStructures_setSystemState_IO_GS(d_systemState_tx)!=GLOBAL_DATA_STRUCT_SET_OK)
					{
						Error_Handler();
					}
				}
			}
			//osDelay(100);
		}
	}

#endif

/*Security application code  ----------------------------*/
#if FREERTOS_SAFETY

	//When it comes to timeout this task will be called, but someone must code it !!!!
	void freertosTask_safety_checkTimeout(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
		    /* if there is no timestamp yet, don't compare */
		    if(safety_getTimestamp_last_rx()!=0)
		        /* if timestamp is too old -> timeout */
		        if(osKernelSysTick()-safety_getTimestamp_last_rx()>SAFETY_TIMEOUT_MS)
		        {
		        	rgb_setState(SYSTEMSTATE_ERROR);
		        	rgb_displayState();
		            /* reset timestamp_last_rx to trigger this only once */
		            safety_resetTimestamp_last_rx();
		            /* TIMEOUT! */
#if FREERTOS_POWERTRAIN
		            PowerTrain_quickstop();
#endif //FREERTOS_POWERTRAIN
#if FREERTOS_ARM
		            // TODO emergency stop
		            /* FIXME SE when timeout then go to a safe state! */
#endif //FREERTOS_ARM
#if FREERTOS_DRILL
		            // TODO emergency stop
#endif //FREERTOS_DRILL
#if FREERTOS_SYSTEM_STATE
		            // TODO show emergency
#endif //FREERTOS_SYSTEM_STATE
		            // TODO maybe add more functionalities
		        }
			osDelay(100); //Give back the control to the scheduler
		}
	}

	/* not necessary?! */
//	//Software based emergency stop ->rover go to safe state
//	void freertosTask_safety_softEmergencyStop(void const * argument)
//	{
//		/*Code that runs only once---------------------*/
//
//		//codes does something e.g. create variables
//
//		/* Infinite loop -------------------------------*/
//		for(;;)
//		{
//			//osSemaphoreWait(s_globalDataStructureHandle_newSecurity_GsToIO,osWaitForever);
//			//do something before task gives runtime back to OS
//			//...
//
//			osDelay(1000);
//			//do something when the OS gives runtime
//			//..
//		}
//	}

#endif

/*Robot arm application code  ----------------------------*/
#if FREERTOS_ARM

	//Read encoder data triggers from semaphore - set by arm tx Data Task
	void freertosTask_arm_readData(void const* argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//do something before task gives runtime back to OS
			//...
			//Note not the right semaphore yet TODO
			//osSemaphoreWait(s_globalDataStructureHandle_newArm_GsToAm,osWaitForever);

			osDelay(1000);

			//do something when the OS gives runtime
			//..
		}
	}

	// Receive new data from GS
	void freertosTask_arm_RxData_gs(void const* argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
//			Wait for incoming signal / data
//			if (osSemaphoreWait(s_globalDataStructureHandle_newArm_GsToAm,osWaitForever)== osOK)
//			{
//				RobotArm_updateState();
//			}
//			RobotArm_updateState();
			osDelay(3500); // FIXME SE set frm 350 to 3500
		}
	}

	// Send data to GS
	void freertosTask_arm_TxData_gs(void const* argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//do something before task gives runtime back to OS
			//...
			osDelay(1000); //Give back the control to the scheduler
			//do something when the OS gives runtime
			//..
		}
	}

	void freertosTask_arm_modeIk(void const* argument)
	{
		/*Code that runs only once---------------------*/
		double q_data[6]	= {0};
		double q1_data[1]	= {0};
		double q2_data[1]	= {0};
		double q3_data[1]	= {0};
		double q4_data[1]	= {0};
		double q_dataIk[5]	= {0};
		double q_dataOld[5]	= {0};
	    uint8_t zonk = 0;
		static globalData_typeDef_robotArm_ARM_GS encoder_actualAngleDegree = {0};
								//	r,   h,   phi
		double targetPosition[3][3]	=  {{0.8, 1.4, 0},    //
										{1, 1, 0},  //1 1.2 0
										{0.8, 1.4, 0}};  //
		static uint8_t status = 0;
		static uint8_t targetState[5] = {0};
		static double hysterese;


		/* Infinite loop -------------------------------*/
		for(;;)
		{

			encoder_actualAngleDegree = globalDataStructures_getRobotArm_ARM_GS();
			double q2actual = (double)encoder_actualAngleDegree.actualJointAngle2/100;
			double q3actual = (double)encoder_actualAngleDegree.actualJointAngle3/100;
			double q4actual = (double)encoder_actualAngleDegree.actualJointAngle4/100;
			targetState[0]=0; targetState[1]=0; targetState[2]=0; targetState[3]=0; targetState[4]=0;
			hysterese = 3.5;

			// check if actual angles are
			if(((uint16_t)q2actual == 0) || ((uint16_t)q3actual == 0) || ((uint16_t)q4actual == 0))
			{
				continue;
			}
//			safe position
//			RobotArm_setSingleAngleD(NODE_ID_MOTOR_2, q2actual, 180, hysterese, 40);
//			RobotArm_setSingleAngleD(NODE_ID_MOTOR_3, q3actual, 265, hysterese, 30);
//			RobotArm_setSingleAngleD(NODE_ID_MOTOR_4, q4actual, 135, hysterese, 24);
//			continue;


			switch(status)
			{
			case 0:
				IkSolver_getAngle(targetPosition[0][0], targetPosition[0][1], 180, q2actual, q3actual, q4actual, 180, q1_data, q2_data, q3_data, q4_data);
				q_dataIk[1]	= *q1_data;
				q_dataIk[2]	= *q2_data;
				q_dataIk[3]	= *q3_data;
				q_dataIk[4]	= *q4_data;
				RobotArm_setSingleAngleD(NODE_ID_MOTOR_2, q2actual, q_dataIk[2], hysterese, 40);
				RobotArm_setSingleAngleD(NODE_ID_MOTOR_3, q3actual, q_dataIk[3], hysterese, 30);
				RobotArm_setSingleAngleD(NODE_ID_MOTOR_4, q4actual, q_dataIk[4], hysterese, 24);
				targetState[0]=0; targetState[1]=0; targetState[2]=0; targetState[3]=0; targetState[4]=0;
				status = 1;
				break;
			case 1:
				targetState[2] = RobotArm_setSingleAngleD(NODE_ID_MOTOR_2, q2actual, q_dataIk[2], hysterese, 40);
				targetState[3] = RobotArm_setSingleAngleD(NODE_ID_MOTOR_3, q3actual, q_dataIk[3], hysterese, 30);
				targetState[4] = RobotArm_setSingleAngleD(NODE_ID_MOTOR_4, q4actual, q_dataIk[4], hysterese, 24);
				if((targetState[2]==1) && (targetState[3]==1) && (targetState[4]==1))
				{
					IkSolver_getAngle(targetPosition[1][0], targetPosition[1][1], 180, q2actual, q3actual, q4actual, 180, q1_data, q2_data, q3_data, q4_data);
					q_dataIk[1]	= *q1_data;
					q_dataIk[2]	= *q2_data;
					q_dataIk[3]	= *q3_data;
					q_dataIk[4]	= *q4_data;
					RobotArm_setSingleAngleD(NODE_ID_MOTOR_2, q2actual, q_dataIk[2], hysterese, 40);
					RobotArm_setSingleAngleD(NODE_ID_MOTOR_3, q3actual, q_dataIk[3], hysterese, 30);
					RobotArm_setSingleAngleD(NODE_ID_MOTOR_4, q4actual, q_dataIk[4], hysterese, 24);
					targetState[0]=0; targetState[1]=0; targetState[2]=0; targetState[3]=0; targetState[4]=0;
					status = 2;
					break;
				}
				else
				{
					break;
				}
			case 2:
				targetState[2] = RobotArm_setSingleAngleD(NODE_ID_MOTOR_2, q2actual, q_dataIk[2], hysterese, 40);
				targetState[3] = RobotArm_setSingleAngleD(NODE_ID_MOTOR_3, q3actual, q_dataIk[3], hysterese, 30);
				targetState[4] = RobotArm_setSingleAngleD(NODE_ID_MOTOR_4, q4actual, q_dataIk[4], hysterese, 24);
				if((targetState[2]==1) && (targetState[3]==1) && (targetState[4]==1))
				{
					IkSolver_getAngle(targetPosition[2][0], targetPosition[2][1], 180, q2actual, q3actual, q4actual, 180, q1_data, q2_data, q3_data, q4_data);
					q_dataIk[1]	= *q1_data;
					q_dataIk[2]	= *q2_data;
					q_dataIk[3]	= *q3_data;
					q_dataIk[4]	= *q4_data;
					RobotArm_setSingleAngleD(NODE_ID_MOTOR_2, q2actual, q_dataIk[2], hysterese, 40);
					RobotArm_setSingleAngleD(NODE_ID_MOTOR_3, q3actual, q_dataIk[3], hysterese, 30);
					RobotArm_setSingleAngleD(NODE_ID_MOTOR_4, q4actual, q_dataIk[4], hysterese, 24);
					targetState[0]=0; targetState[1]=0; targetState[2]=0; targetState[3]=0; targetState[4]=0;
					status = 3;
					break;
				}
				else
				{
					break;
				}
			case 3:
				targetState[2] = RobotArm_setSingleAngleD(NODE_ID_MOTOR_2, q2actual, q_dataIk[2], hysterese, 40);
				targetState[3] = RobotArm_setSingleAngleD(NODE_ID_MOTOR_3, q3actual, q_dataIk[3], hysterese, 30);
				targetState[4] = RobotArm_setSingleAngleD(NODE_ID_MOTOR_4, q4actual, q_dataIk[4], hysterese, 24);
				if((targetState[2]==1) && (targetState[3]==1) && (targetState[4]==1))
				{
					zonk++;
					targetState[0]=0; targetState[1]=0; targetState[2]=0; targetState[3]=0; targetState[4]=0;
					status = 0;
				}
				else
				{
					break;
				}
//				// done
			default:
				zonk++;
				break;
			}



			//do something before task gives runtime back to OS
			//...
			osDelay(350); //Give back the control to the scheduler
			//do something when the OS gives runtime
			//..
		}
	}

	void freertosTask_arm_modeTp(void const* argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
//			Wait for incoming signal / data
//			if (osSemaphoreWait(s_freertosHandle_armTp,osWaitForever)== osOK)
//			{
//
//			}
			osDelay(1000); //Give back the control to the scheduler
			//do something when the OS gives runtime
			//..
		}
	}

	void freertosTask_arm_modeAxis(void const* argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
//			Wait for incoming signal / data
//			if (osSemaphoreWait(s_freertosHandle_armAxis,osWaitForever)== osOK)
//			{
//
//			}
			osDelay(1000); //Give back the control to the scheduler
			//do something when the OS gives runtime
			//..
		}
	}

	void freertosTask_arm_checkEncoderLimits(void const* argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
//			Wait for incoming signal / data
//			if (osSemaphoreWait(s_freertosHandle_armCheckEncoder,osWaitForever)== osOK)
//			{
//
//			}
			osDelay(1000); //Give back the control to the scheduler
			//do something when the OS gives runtime
			//..
		}
	}

#endif

/*Drill application code  ----------------------------*/
#if FREERTOS_DRILL

	//Transmit drill unit state to Gs
	void freertosTask_drill_TxData_gs(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//do something before task gives runtime back to OS
			//...
			osDelay(1000); //Give back the control to the scheduler
			//do something when the OS gives runtime
			//..
		}
	}

	//Receive Data from Gs
	void freertosTask_drill_RxData_gs(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//Wait for semaphore
			//if (osSemaphoreWait(s_globalDataStructureHandle_newDrill_GsToDrill,0)== osOK){}

			osDelay(10000);
			//do something when semaphore is set
			//...

			//Give back the control to the scheduler
		}
	}
#endif


#if FREERTOS_BMS
	//Transmit bms data to gs
	void freertosTask_BMS_TxData_gs(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//Send BMS data to GS
			if (udp_lwip_txUdpMessage_gs(GLOBALDATA_ID_BMS) == UDP_LWIP_SEND_OK)
			{
			//Trigger new data readout
			osSemaphoreRelease(s_freertosHandle_readBmsData);
			}

			//Wait for next send interval
			osDelay(100);
		}
	}

	//Read ADC / BMS data
	void freertosTask_BMS_readData(void const * argument)
	{
		/*Code that runs only once---------------------*/

		//codes does something e.g. create variables

		globalData_typeDef_BMS_BMS_GS d_bmsData_tx = {0};
		d_bmsData_tx.dataID = GLOBALDATA_ID_BMS;
		uint16_t adcData;

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//Wait for semaphore is set by transmitting data to GS task
			if (osSemaphoreWait(s_freertosHandle_readBmsData,osWaitForever)== osOK){

				for (bms_typeDef_sourceSelect source = bmsCell1; source <= bmsCurrent; source++ )
				{
					//trigger new readout
					bmsSetSource(source);
					osDelay(10);
					HAL_ADC_Start_IT(&hadc2);
					HAL_ADC_Start(&hadc2);

					//wait for adc interrupt set semaphore that signals finished readout
					if (osSemaphoreWait(s_freertosHandle_bmsDataReady,osWaitForever)== osOK){

						//read data for cell 1
						adcData = HAL_ADC_GetValue(&hadc2);
						//store value
						if(source == bmsCell1 )	{ 	d_bmsData_tx.voltageCell1 	= bmsCalcVoltage(adcData);}
						if(source == bmsCell2 )	{ 	d_bmsData_tx.voltageCell2 	= bmsCalcVoltage(adcData);}
						if(source == bmsCell3 )	{ 	d_bmsData_tx.voltageCell3 	= bmsCalcVoltage(adcData);}
						if(source == bmsCell4 )	{ 	d_bmsData_tx.voltageCell4 	= bmsCalcVoltage(adcData);}
						if(source == bmsCell5 )	{ 	d_bmsData_tx.voltageCell5 	= bmsCalcVoltage(adcData);}
						if(source == bmsCell6 )	{ 	d_bmsData_tx.voltageCell6 	= bmsCalcVoltage(adcData);}
						if(source == bmsCurrent){ 	d_bmsData_tx.current 		= bmsCalcCurrent(adcData);}

					}
				}
			}
			if (globalDataStructures_setBMS_BMS_GS(d_bmsData_tx)!=GLOBAL_DATA_STRUCT_SET_OK)
			{
				Error_Handler();
			}
		}
	}
#endif

#if FREERTOS_BOSCH_IMU

//Transmit IMU data to gs - Added by MTR 09.07.2019
void freertosTask_IMU_TxData_gs(void const * argument)
{
	/*Code that runs only once---------------------*/

	/* Infinite loop -------------------------------*/
	for(;;)
	{
		if(udp_lwip_txUdpMessage_gs(GLOBALDATA_ID_IMU)!= UDP_LWIP_SEND_OK)
		{
			Error_Handler();
		}else{
			osSemaphoreRelease(s_globalDataStructureHandle_readNewSensorData_BNO055);
		}

		osDelay(100);

		//do something when semaphore is set
		//...

		//Give back the control to the scheduler

	}
}




//Read IMU Data from I2C interface - Added by MTR 09.07.2019
void freertosTask_IMU_readData(void const * argument)
{
	/*Code that runs only once---------------------*/
	//codes does something e.g. create variables
	bno055_vector_t dataEuler;
	bno055_vector_t dataAcc;

	globalData_typeDef_boschIMU d_imu;
	d_imu.dataID = GLOBALDATA_ID_IMU; //set correct data id to allow write and send methods

	//bno055_calibration_t data =  bno055_getCalibration();

		/* Infinite loop -------------------------------*/
		for(;;)
		{
			//Wait for semaphore
			if (osSemaphoreWait(s_globalDataStructureHandle_readNewSensorData_BNO055,osWaitForever)== osOK)
			{
				//HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin); //for debugging
				//Read sensor data
				dataEuler 	= bno055_getVectorEuler();
				//dataAcc		= bno055_getVectorAccelerometer();
				dataAcc		= bno055_getVectorLinearAccel();

				//Write in global data type
				//store euler angels
				d_imu.xEuler = (float) dataEuler.x;
				d_imu.yEuler = (float)  dataEuler.y;
				d_imu.zEuler =  (float) dataEuler.z;

				//store lin acceleration with gravity compensation
				d_imu.xAcc = (float)  dataAcc.x;
				d_imu.yAcc =  (float) dataAcc.y;
				d_imu.zAcc = (float)  dataAcc.z;

				// uint8_t temp = bno055_getTemp();

				//Try to store data in global data structure
				if (globalDataStructures_setImuData_Gs(d_imu) != GLOBAL_DATA_STRUCT_SET_OK)
				{
					Error_Handler();
				}
				//osDelay(200);
			}
		}
}
#endif



/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
