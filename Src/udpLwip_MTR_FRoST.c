/**
  *****************************************************************************
  * @title  udp_lwip_MTR.h
  * @author Max Triller
  * @date   26.03.2019
  * @brief  Initialization of the UDP client, based on the LwIP stack for FreeRTOS
  * 		Allows you to send UDP messages to server with known IP. Client IP is set in "lwip.c".
  * 		Network monitor Software:
  * 		 	- use https://github.com/PavelBansky/EchoTool
  * 		 	- ore https://www.wireshark.org/ (to start NPF driver in Windows use "sc start npf" in command line prompt)
  * @edit	Max Triller 25.06.2019
  *******************************************************************************
  */


/* Private Includes ----------------------------------------------------------*/
#include <udp_lwip_MTR_FRoST.h>
#include "main.h"
#include "globalDataStructures_CM_MTR_FRoST.h"
#include "cmsis_os.h"
#include "string.h"
#include "safety.h"
#include "rgb_indicator_MTR_FRoST.h"

/* Private variables ---------------------------------------------------------*/
struct udp_pcb *upcb;

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

osMutexId m_udp_lwipHandle_txEthernet;

/* Private callbacks-----------------------------------------------------------*/

/*
 * @ingroup callback functions
 *
 * @param user supplied argument (udp_pcb.recv_arg)
 * @param udp_pcb the UDP protocol control block
 * @param p the packet buffer that was received
 * @param addr the remote IP address from which the packet was received
 * @param port the remote port from which the packet was received
 *
 */
/* If UDP client is used, this callback trigger ?!? dose nothing :D */
__weak void udp_lwip_clientCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	if (p != NULL)
	{
    /* free the pbuf */
    pbuf_free(p);
	}
}



/* If UDP sever is used, this callback trigger ?!?! */
__weak void  udp_lwip_serverCallback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p != NULL) {
        /* send received packet back to sender */
        udp_sendto(pcb, p, addr, port);
        /* free the pbuf */
        pbuf_free(p);
    }
}

/* MTR 24.06.2019 UDP Callback that receives data via UDP and stores it in global data structure */
/* Note: Under heavy construction!*/
void  udp_lwip_rxCallback_gs(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
	/*Create data containers for storing incoming data*/
	/*Note: data type is set static to avoid constant re-initialization */
	static globalData_typeDef_powerTrain_GS_IO 	d_powerTrain_rx	= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
	static globalData_typeDef_robotArm_GS_ARM   d_arm_rx	  	= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
	static globalData_typeDef_drill_GS_DRILL	d_drill_rx		= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
	static globalData_typeDef_security_GS_IO	d_security_rx 	= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

	/*Create variable to store message data ID and data length*/
	uint8_t dataID;
	//uint16_t dataLength;

	if (p != NULL) //check pointer points to a valid address
    {
    	/*Read first field of payload that contains the data ID */
    	dataID 		=  	((uint8_t*)p->payload)[0] ;
    	/*Read data length from p pbuf structure */
    	//dataLength 	= 	(p->len);
    	/*Create buffer array */
    	safety_setTimestamp_last_rx(); //set timestamp for timeout function
    	rgb_setState(SYSTEMSTATE_CONNECTED);
        rgb_displayState();

    	/*Chose right data type and store it*/
    	switch (dataID){
    	case GLOBALDATA_ID_ERROR:
    		Error_Handler();
    		break;

    	case GLOBALDATA_ID_DRILL:
    		/*Copy drill data from Ethernet interface buffer to structure and try to store it !*/
    		memcpy(&d_drill_rx,(globalData_typeDef_drill_GS_DRILL*)p->payload,sizeof(globalData_typeDef_drill_GS_DRILL));
    		if( globalDataStructures_setDrill_GS_DRILL(d_drill_rx)== GLOBAL_DATA_STRUCT_SET_OK)
    		{
    			osSemaphoreRelease(s_globalDataStructureHandle_newDrill_GsToDrill);
    		}else
    		{
    			Error_Handler();
    		}

    		break;

    	case GLOBALDATA_ID_ARM:
    		/*Copy  robot arm data from Ethernet interface buffer to structure and try to store it !*/
    		memcpy(&d_arm_rx, (globalData_typeDef_robotArm_GS_ARM*)p->payload, sizeof(globalData_typeDef_robotArm_GS_ARM)); // original

    		if( globalDataStructures_setRobotArm_GS_ARM(d_arm_rx)== GLOBAL_DATA_STRUCT_SET_OK)
    		{
    			osSemaphoreRelease(s_globalDataStructureHandle_newArm_GsToAm);
    		}else
    		{
    			Error_Handler();
    		}
    		break;

    	case GLOBALDATA_ID_POWERTRAIN:
    		/*Copy powetrain data from Ethernet interface buffer to structure and try to store it !*/
    		memcpy(&d_powerTrain_rx,(globalData_typeDef_powerTrain_GS_IO*)p->payload,sizeof(globalData_typeDef_powerTrain_GS_IO));
    		if( globalDataStructures_setPowerTrain_GS_IO(d_powerTrain_rx)== GLOBAL_DATA_STRUCT_SET_OK)
    		{
    			osSemaphoreRelease(s_globalDataStructureHandle_newPowerTrain_GsToIo);
    		}
    		else
    		{
    			Error_Handler();
    		}
    		break;

    	case GLOBALDATA_ID_SECURITY:
    		/*Copy security data from Ethernet interface buffer to structure and try to store it !*/
    		memcpy(&d_security_rx,(globalData_typeDef_security_GS_IO*)p->payload,sizeof(globalData_typeDef_security_GS_IO));
    		if( globalDataStructures_setSecurity_GS_IO(d_security_rx)== GLOBAL_DATA_STRUCT_SET_OK)
    		{
    			osSemaphoreRelease(s_globalDataStructureHandle_newSecurity_GsToIO);
    		}
    		else
    		{
    			Error_Handler();
    		}
    		break;
    	default:
    		break;
    	}
    	/* free the pbuf */
    	pbuf_free(p);
    }
}


/* Private functions ---------------------------------------------------------*/

/*Send UDP Control packet -> recommenced for FRoST rover application */ /*Added MTR 20.06.2019*/
uint8_t udp_lwip_txUdpMessage_gs(uint8_t dataID)
{
	/*Create return variable for storing Message ID */
	uint8_t returnValue;

	/*Create data container for storing udp payload in it*/
	struct pbuf *p_txBuffer;

	/*Create structures as transmit containers */ /*Added MTR 22.06.2019*/
	/*Note may not be memory or speed efficient - UDP module has to have its own globals ?!?!*/
	/*Note: data type is set static to avoid constant re-initialization Added MTR 25.06.2019*/
	static globalData_typeDef_powerTrain_IO_GS 			d_powerTrain_tx = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
	static globalData_typeDef_robotArm_ARM_GS			d_arm_tx 		= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
	static globalData_typeDef_weightCell_IO_GS			d_weightCell_tx = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
	static globalData_typeDef_BMS_BMS_GS				d_batteryMS_tx  = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
	static globalData_typeDef_environmentSensor_IO_GS	d_eSensor_tx	= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
	static globalData_typeDef_systemState_IO_GS 		d_sState_tx 	= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
	static globalData_typeDef_drill_DRILL_GS			d_drill_tx		= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };
	static globalData_typeDef_boschIMU					d_imu_tx		= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };




	/*Check witch data has to be send----------------*/
	switch(dataID) //MAKE THIS SH**** WORK AGAIN !
	{

	case GLOBALDATA_ID_ERROR: //when ID == 0 -> is a non valid ID
		Error_Handler();
		returnValue = UDP_LWIP_SEND_ERROR;
		break;

	case GLOBALDATA_ID_POWERTRAIN:
		/*Create data container*/ //WHY NO NEW VAR IN SWITCH CASE STATEMENT ?!?!?!
		//globalData_typeDef_powerTrain_IO_GS 			d_powerTrain_tx;// = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

		/*Read data from global structure and store a copy */
		d_powerTrain_tx =  globalDataStructures_getPowerTrain_IO_GS();

		//Validate read data, if ID == 0 data could not be read because MUTEX is not free!
		if (d_powerTrain_tx.dataID == GLOBALDATA_ID_POWERTRAIN){
			/*Send Data via UDP to GS*/
			/*allocate data length for container structure*/
			p_txBuffer = pbuf_alloc(PBUF_TRANSPORT,sizeof(globalData_typeDef_powerTrain_IO_GS), PBUF_POOL);
			if (p_txBuffer != NULL)
			{
				if(osMutexWait(m_udp_lwipHandle_txEthernet, UDP_LWIP_TIMEOUT_ETHERNET_MUTEX)==osOK)
				{
					/* copy data to pbuf */
				    pbuf_take(p_txBuffer, &d_powerTrain_tx, sizeof(globalData_typeDef_powerTrain_IO_GS));
				    /* send udp data */
				    if(udp_send(upcb, p_txBuffer)==ERR_OK)
				    {
				    	returnValue = UDP_LWIP_SEND_OK;
				    }
				    else{
				    	returnValue = UDP_LWIP_SEND_ERROR;
				    }
				    /* free pbuf */
				    pbuf_free(p_txBuffer);
					returnValue =  UDP_LWIP_SEND_OK;

				osMutexRelease(m_udp_lwipHandle_txEthernet);
				}
				else
				{
					returnValue =  UDP_LWIP_SEND_ERROR;
				}
			}
		}
		else
		{
			/*If data could be read - call error handler !*/
			Error_Handler();
			returnValue =  UDP_LWIP_SEND_ERROR;
		}
		break; /*end case GLOBALDATA_ID_POWERTRAIN*/

	case GLOBALDATA_ID_ARM:
		/*Create data container*/ //WHY NO NEW VAR IN SWITCH CASE STATEMENT ?!?!?!
		//globalData_typeDef_robotArm_ARM_GS			d_arm_tx 		= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

		/*Read data from global structure and store a copy */
		d_arm_tx =  globalDataStructures_getRobotArm_ARM_GS();

		//Validate read data, if ID == 0 data could not be read because MUTEX is not free!
		if (d_arm_tx.dataID == GLOBALDATA_ID_ARM){
			/*Send Data via UDP to GS*/
			/*allocate data length for container structure*/
			p_txBuffer = pbuf_alloc(PBUF_TRANSPORT,sizeof(d_arm_tx), PBUF_POOL);
			if (p_txBuffer != NULL)
			{
				if(osMutexWait(m_udp_lwipHandle_txEthernet, UDP_LWIP_TIMEOUT_ETHERNET_MUTEX)==osOK)
				{
					/* copy data to pbuf */
				    pbuf_take(p_txBuffer, &d_arm_tx, sizeof(d_arm_tx));
				    /* send udp data */
				    if(udp_send(upcb, p_txBuffer)==ERR_OK)
				    {
				    	returnValue =  UDP_LWIP_SEND_OK;
				    }
				    else{
				    	returnValue =  UDP_LWIP_SEND_ERROR;
				    }
				    /* free pbuf */
				    pbuf_free(p_txBuffer);
					returnValue =  UDP_LWIP_SEND_OK;
					osMutexRelease(m_udp_lwipHandle_txEthernet);
				}
				else
				{
					returnValue =  UDP_LWIP_SEND_ERROR;
				}
			}
		}
		else
		{
			/*If data could be read - call error handler !*/
			Error_Handler();
			returnValue =  UDP_LWIP_SEND_ERROR;
		}
		break; /*end case GLOBALDATA_ID_ARM*/

	case GLOBALDATA_ID_WEIGHTCELL:
		/*Create data container*/ //WHY NO NEW VAR IN SWITCH CASE STATEMENT ?!?!?!
		//globalData_typeDef_weightCell_IO_GS			d_weightCell_tx = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

		/*Read data from global structure and store a copy */
		d_weightCell_tx =  globalDataStructures_getWeightCell_IO_GS();

		//Validate read data, if ID == 0 data could not be read because MUTEX is not free!
		if (d_weightCell_tx.dataID == GLOBALDATA_ID_WEIGHTCELL){
			/*Send Data via UDP to GS*/
			/*allocate data length for container structure*/
			p_txBuffer = pbuf_alloc(PBUF_TRANSPORT,sizeof(d_weightCell_tx), PBUF_POOL);
			if (p_txBuffer != NULL)
			{
				if(osMutexWait(m_udp_lwipHandle_txEthernet, UDP_LWIP_TIMEOUT_ETHERNET_MUTEX)==osOK)
				{
					/* copy data to pbuf */
				    pbuf_take(p_txBuffer, &d_weightCell_tx, sizeof(d_weightCell_tx));
				    /* send udp data */
				    if(udp_send(upcb, p_txBuffer)==ERR_OK)
				    {
				    	returnValue =  UDP_LWIP_SEND_OK;
				    }
				    else{
				    	returnValue =  UDP_LWIP_SEND_ERROR;
				    }
				    /* free pbuf */
				    pbuf_free(p_txBuffer);
					returnValue =  UDP_LWIP_SEND_OK;
					osMutexRelease(m_udp_lwipHandle_txEthernet);
				}
				else
				{
					returnValue =  UDP_LWIP_SEND_ERROR;
				}
			}
		}
		else
		{
			/*If data could be read - call error handler !*/
			Error_Handler();
			returnValue =  UDP_LWIP_SEND_ERROR;
		}
		break; /*end case GLOBALDATA_ID_WEIGHTCELL*/

	case GLOBALDATA_ID_BMS:
		/*Create data container*/ //WHY NO NEW VAR IN SWITCH CASE STATEMENT ?!?!?!
		//globalData_typeDef_BMS_BMS_GS				d_batteryMS_tx  = { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

		/*Read data from global structure and store a copy */
		d_batteryMS_tx =  globalDataStructures_getBMS_BMS_GS();

		//Validate read data, if ID == 0 data could not be read because MUTEX is not free!
		if (d_batteryMS_tx.dataID == GLOBALDATA_ID_BMS){
			/*Send Data via UDP to GS*/
			/*allocate data length for container structure*/
			p_txBuffer = pbuf_alloc(PBUF_TRANSPORT,sizeof(d_batteryMS_tx), PBUF_POOL);
			if (p_txBuffer != NULL)
			{
				if(osMutexWait(m_udp_lwipHandle_txEthernet, UDP_LWIP_TIMEOUT_ETHERNET_MUTEX)==osOK)
				{
					/* copy data to pbuf */
				    pbuf_take(p_txBuffer, &d_batteryMS_tx, sizeof(d_batteryMS_tx));
				    /* send udp data */
				    if(udp_send(upcb, p_txBuffer)==ERR_OK)
				    {
				    	returnValue =  UDP_LWIP_SEND_OK;
				    }
				    else{
				    	returnValue =  UDP_LWIP_SEND_ERROR;
				    }
				    /* free pbuf */
				    pbuf_free(p_txBuffer);
					returnValue =  UDP_LWIP_SEND_OK;
					osMutexRelease(m_udp_lwipHandle_txEthernet);
				}
				else
				{
					returnValue =  UDP_LWIP_SEND_ERROR;
				}
			}
		}
		else
		{
			/*If data could be read - call error handler !*/
			Error_Handler();
			returnValue =  UDP_LWIP_SEND_ERROR;
		}
		break; /*end case GLOBALDATA_ID_BMS*/

	case GLOBALDATA_ID_ENVIROMENTSENSOR:
		/*Create data container*/ //WHY NO NEW VAR IN SWITCH CASE STATEMENT ?!?!?!
		//globalData_typeDef_environmentSensor_IO_GS	d_eSensor_tx	= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

		/*Read data from global structure and store a copy */
		d_eSensor_tx =  globalDataStructures_getEnvironmentSensor_IO_GS();

		//Validate read data, if ID == 0 data could not be read because MUTEX is not free!
		if (d_eSensor_tx.dataID == GLOBALDATA_ID_ENVIROMENTSENSOR){
			/*Send Data via UDP to GS*/
			/*allocate data length for container structure*/
			p_txBuffer = pbuf_alloc(PBUF_TRANSPORT,sizeof(d_eSensor_tx), PBUF_POOL);
			if (p_txBuffer != NULL)
			{
				if(osMutexWait(m_udp_lwipHandle_txEthernet, UDP_LWIP_TIMEOUT_ETHERNET_MUTEX)==osOK)
				{
					/* copy data to pbuf */
				    pbuf_take(p_txBuffer, &d_eSensor_tx, sizeof(d_eSensor_tx));
				    /* send udp data */
				    if(udp_send(upcb, p_txBuffer)==ERR_OK)
				    {
				    	returnValue =  UDP_LWIP_SEND_OK;
				    }
				    else{
				    	returnValue =  UDP_LWIP_SEND_ERROR;
				    }
				    /* free pbuf */
				    pbuf_free(p_txBuffer);
					returnValue =  UDP_LWIP_SEND_OK;
					osMutexRelease(m_udp_lwipHandle_txEthernet);
				}
				else
				{
					returnValue =  UDP_LWIP_SEND_ERROR;
				}
			}
		}
		else
		{
			/*If data could be read - call error handler !*/
			Error_Handler();
			returnValue =  UDP_LWIP_SEND_ERROR;
		}
		break; /*end case GLOBALDATA_ID_ENVIROMENTSENSOR*/

	case GLOBALDATA_ID_SYSTEMSTATE:
		/*Create data container*/ //WHY NO NEW VAR IN SWITCH CASE STATEMENT ?!?!?!
		//globalData_typeDef_systemState_IO_GS 		d_sState_tx 	= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

		/*Read data from global structure and store a copy */
		d_sState_tx =  globalDataStructures_getSystemState_IO_GS();

		//Validate read data, if ID == 0 data could not be read because MUTEX is not free!
		if (d_sState_tx.dataID == GLOBALDATA_ID_SYSTEMSTATE){
			/*Send Data via UDP to GS*/
			/*allocate data length for container structure*/
			p_txBuffer = pbuf_alloc(PBUF_TRANSPORT,sizeof(d_sState_tx), PBUF_POOL);
			if (p_txBuffer != NULL)
			{
				if(osMutexWait(m_udp_lwipHandle_txEthernet, UDP_LWIP_TIMEOUT_ETHERNET_MUTEX)==osOK)
				{
					/* copy data to pbuf */
				    pbuf_take(p_txBuffer, &d_sState_tx, sizeof(d_sState_tx));
				    /* send udp data */
				    if(udp_send(upcb, p_txBuffer)==ERR_OK)
				    {
				    	returnValue =  UDP_LWIP_SEND_OK;
				    }
				    else{
				    	returnValue =  UDP_LWIP_SEND_ERROR;
				    }
				    /* free pbuf */
				    pbuf_free(p_txBuffer);
					returnValue =  UDP_LWIP_SEND_OK;
					osMutexRelease(m_udp_lwipHandle_txEthernet);
				}
				else
				{
					returnValue =  UDP_LWIP_SEND_ERROR;
				}
			}
		}
		else
		{
			/*If data could be read - call error handler !*/
			Error_Handler();
			returnValue =  UDP_LWIP_SEND_ERROR;
		}
		break; /*end case GLOBALDATA_ID_SYSTEMSTATE*/

	case GLOBALDATA_ID_DRILL:
		/*Create data container*/ //WHY NO NEW VAR IN SWITCH CASE STATEMENT ?!?!?!
		//globalData_typeDef_drill_DRILL_GS			d_drill_tx		= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

		/*Read data from global structure and store a copy */
		d_drill_tx =  globalDataStructures_getDrill_DRILL_GS();

		//Validate read data, if ID == 0 data could not be read because MUTEX is not free!
		if (d_drill_tx.dataID == GLOBALDATA_ID_DRILL){
			/*Send Data via UDP to GS*/
			/*allocate data length for container structure*/
			p_txBuffer = pbuf_alloc(PBUF_TRANSPORT,sizeof(d_drill_tx), PBUF_POOL);
			if (p_txBuffer != NULL)
			{
				if(osMutexWait(m_udp_lwipHandle_txEthernet, UDP_LWIP_TIMEOUT_ETHERNET_MUTEX)==osOK)
				{
					/* copy data to pbuf */
				    pbuf_take(p_txBuffer, &d_drill_tx, sizeof(d_drill_tx));
				    /* send udp data */
				    if(udp_send(upcb, p_txBuffer)==ERR_OK)
				    {
				    	returnValue =  UDP_LWIP_SEND_OK;
				    }
				    else{
				    	returnValue =  UDP_LWIP_SEND_ERROR;
				    }
				    /* free pbuf */
				    pbuf_free(p_txBuffer);
					returnValue =  UDP_LWIP_SEND_OK;
					osMutexRelease(m_udp_lwipHandle_txEthernet);
				}
				else
				{
					returnValue =  UDP_LWIP_SEND_ERROR;
				}
			}
		}
		else
		{
			/*If data could be read - call error handler !*/
			Error_Handler();
			returnValue =  UDP_LWIP_SEND_ERROR;
		}

		break; /*end case GLOBALDATA_ID_DRILL*/

	case GLOBALDATA_ID_IMU:
			/*Create data container*/ //WHY NO NEW VAR IN SWITCH CASE STATEMENT ?!?!?!
			//globalData_typeDef_drill_DRILL_GS			d_drill_tx		= { GLOBAL_DATA_STRUCTURES_INIT_ZERO };

			/*Read data from global structure and store a copy */
			d_imu_tx =  globalDataStructures_getBoshImu();

			//Validate read data, if ID == 0 data could not be read because MUTEX is not free!
			if (d_imu_tx.dataID == GLOBALDATA_ID_IMU){
				/*Send Data via UDP to GS*/
				/*allocate data length for container structure*/
				p_txBuffer = pbuf_alloc(PBUF_TRANSPORT,sizeof(globalData_typeDef_boschIMU), PBUF_POOL);
				if (p_txBuffer != NULL)
				{
					if(osMutexWait(m_udp_lwipHandle_txEthernet, UDP_LWIP_TIMEOUT_ETHERNET_MUTEX)==osOK)
					{
						/* copy data to pbuf */
					    pbuf_take(p_txBuffer, &d_imu_tx, sizeof(globalData_typeDef_boschIMU));
					    /* send udp data */
					    if(udp_send(upcb, p_txBuffer)==ERR_OK)
					    {
					    	returnValue =  UDP_LWIP_SEND_OK;
					    }
					    else{
					    	returnValue =  UDP_LWIP_SEND_ERROR;
					    }
					    /* free pbuf */
					    pbuf_free(p_txBuffer);
						returnValue =  UDP_LWIP_SEND_OK;
						osMutexRelease(m_udp_lwipHandle_txEthernet);
					}
					else
					{
						returnValue =  UDP_LWIP_SEND_ERROR;
					}
				}
			}
			else
			{
				/*If data could be read - call error handler !*/
				Error_Handler();
				returnValue =  UDP_LWIP_SEND_ERROR;
			}

			break; /*end case GLOBALDATA_ID_DRILL*/
	default:
		Error_Handler();
		returnValue = UDP_LWIP_SEND_ERROR;
	}
	return returnValue;
}


/* Send UDP data as client */
void udp_lwip_clientSendMessage(uint32_t myData)
{
	struct pbuf *p;
	/* allocate pbuf from pool*/
	p = pbuf_alloc(PBUF_TRANSPORT,sizeof(myData), PBUF_POOL);
	if (p != NULL){
		/* copy data to pbuf */
	    pbuf_take(p, &myData, sizeof(myData));
	    /* send udp data */
	    udp_send(upcb, p);
	    /* free pbuf */
	    pbuf_free(p);
	}
}

/* Setup an UDP Server */
void udp_lwip_initInterface(void)
{
    struct udp_pcb * pcb;
    /* get new pcb */
    pcb = udp_new();
    if (pcb == NULL) {
        LWIP_DEBUGF(UDP_DEBUG, ("udp_new failed!\n"));
        return;
    }
    /* bind to any IP address on port 7 */
    if (udp_bind(pcb, IP_ADDR_ANY, UDP_SERVER_PORT) != ERR_OK) {
        LWIP_DEBUGF(UDP_DEBUG, ("udp_bind failed!\n"));
        return;
    }
    /* set udp_echo_recv() as callback function
       for received packets */
    udp_recv(pcb, udp_lwip_serverCallback, NULL);
}

/*Setup an UDP client */
void udp_lwip_initClient(void)
{
	ip_addr_t DestIPaddr;
	err_t err;
	/* Create a new UDP control block  */
	upcb = udp_new();
	if (upcb!=NULL){
	    /*assign destination IP address */
	    IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
	    /* configure destination IP address and port */
	    err= udp_connect(upcb, &DestIPaddr, UDP_DEST_PORT);
	    if (err == ERR_OK){
	     	/* Set a receive callback for the upcb */
	    	udp_recv(upcb, udp_lwip_rxCallback_gs, NULL);
	    	}
	 	 }

}

void udp_lwip_initTransmitMutex(void){

	osMutexDef(m_udp_lwipHandle_txEthernet);
	m_udp_lwipHandle_txEthernet = osMutexCreate(osMutex(m_udp_lwipHandle_txEthernet));

}


/* Setup an UDP Server */
void udp_lwip_initServer(void)
{
    struct udp_pcb * pcb;
    /* get new pcb */
    pcb = udp_new();
    if (pcb == NULL) {
        LWIP_DEBUGF(UDP_DEBUG, ("udp_new failed!\n"));
        return;
    }
    /* bind to any IP address on port 7 */
    if (udp_bind(pcb, IP_ADDR_ANY, UDP_SERVER_PORT) != ERR_OK) {
        LWIP_DEBUGF(UDP_DEBUG, ("udp_bind failed!\n"));
        return;
    }
    /* set udp_echo_recv() as callback function
       for received packets */
    udp_recv(pcb, udp_lwip_rxCallback_gs, NULL);
}


