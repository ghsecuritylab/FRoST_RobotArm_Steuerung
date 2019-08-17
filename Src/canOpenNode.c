/*
 * canOpenNode.c
 *
 *  Created on: 13.05.2019
 *      Author: CM
 */

#include "canOpenNode.h"
#include "cmsis_os.h"
#include "can.h"
#include "RobotArm.h"

/* pointer array to instances
 * contains pointers to registered node instances
 * index equals node id
 * add instance by addInstance
 * size: 128 ([0:127]) */
canOpenNode_typeDef_Node* canOpenNodeInstances[CAN_ID_MASK+1];

osSemaphoreId semCanOpenSDORXHandle;
osMutexId mCanOpenSdoWrRdHandle;

/* build SDO by given parameters */
void canOpenNode_buildSDO(canOpen_typeDef_SDO* SDO, uint8_t NodeId, uint16_t index, uint8_t subIndex, int32_t data, uint8_t write)
{
    SDO->IDX[0]=index&0xFF;  // LSB first
    SDO->IDX[1]=index>>8;    // MSB second
    SDO->SUBIDX=subIndex;
    if(write)
    {
        SDO->CMD=CMD_WR_LENGTH_4_BYTES;         // data length 4 Bytes
        SDO->data[0]=data&0xFF;                 // LSB first
        SDO->data[1]=(data>>8)&0xFF;
        SDO->data[2]=(data>>16)&0xFF;
        SDO->data[3]=(data>>24)&0xFF;           // MSB last
    }
    else
    {
        SDO->CMD=CMD_RD_REQ;                    // read request
        for(size_t i=0;i<sizeof(SDO->data);i++)
        {
            SDO->data[i]=0;                     // no data needed
        }
    }
}

/* internal function for sending SDOs for initialization */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_sendInitSDOs(uint8_t NodeId)
{
    if(canOpenNodeInstances[NodeId]==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    for(size_t i=0;i<canOpenNodeInstances[NodeId]->initialization.initSDOsAmount;i++)
    {
//        if(canOpenNode_SdoWr(NodeId,canOpenNodeInstances[NodeId]->initialization.InitSDOs[i].index,canOpenNodeInstances[NodeId]->initialization.InitSDOs[i].subindex,canOpenNodeInstances[NodeId]->initialization.InitSDOs[i].data, canOpenNodeInstances[NodeId]->initialization.InitSDOs[i].length)!=APPLICATIONERROR_NONE)
//            return APPLICATIONERROR_DEFAULT_ERROR;
    	 if(canOpenNode_SdoWr(NodeId,canOpenNodeInstances[NodeId]->initialization.InitSDOs[i].index,canOpenNodeInstances[NodeId]->initialization.InitSDOs[i].subindex,canOpenNodeInstances[NodeId]->initialization.InitSDOs[i].data, canOpenNodeInstances[NodeId]->initialization.InitSDOs[i].length)!=APPLICATIONERROR_NONE)
    	 	 {return APPLICATIONERROR_DEFAULT_ERROR;}
    	 HAL_Delay(15);
    }
    return APPLICATIONERROR_NONE;
}


/* restart initialization of the node */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_resetInitialization(uint8_t NodeId)
{
    if(canOpenNodeInstances[NodeId]==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    canOpenNodeInstances[NodeId]->NodeState=NODESTATE_INITIALIZING;
    if(canOpenNode_sendInitSDOs(NodeId)==APPLICATIONERROR_NONE)
    {
        canOpenNodeInstances[NodeId]->NodeState=NODESTATE_READY;
        return APPLICATIONERROR_NONE;
    }
    else
        return APPLICATIONERROR_DEFAULT_ERROR;
}

/* add node to node array
 * starts node initialization */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_addInstance(canOpenNode_typeDef_Node* pointerToNode, canOpen_typeDef_SDOprimitive* initSDOs, size_t initSDOsAmount)
{
    if(canOpenNodeInstances[pointerToNode->NodeId]!=NULL)
        return APPLICATIONERROR_DEFAULT_ERROR;
    canOpenNodeInstances[pointerToNode->NodeId]=pointerToNode;
    pointerToNode->NodeState=NODESTATE_INITIALIZING;
//    canOpenNode_SdoWr(pointerToNode->NodeId,0x1017,0,CANOPEN_HEARTBEAT_TIMEOUT_MS/2,2);  // set heartbeat time to 250 ms
    canOpenNode_SdoWr(pointerToNode->NodeId, 0x1017, 0, 0, 2);  // disable heartbeat

    if(pointerToNode->NodeType != NODETYPE_EAM360)
    {
		for(size_t i=0;i<initSDOsAmount;i++)
		{
			pointerToNode->initialization.InitSDOs[i]=initSDOs[i];
		}
    }
    pointerToNode->initialization.initSDOsAmount=initSDOsAmount;

    if(canOpenNode_resetInitialization(pointerToNode->NodeId)!=APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;

    // TODO do more stuff here?
    return APPLICATIONERROR_NONE;
}

/* initializes this module by setting node array to nullptr
 * use only once at startup */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_initialize()
{
    for(size_t i=0;i<CAN_ID_MASK;i++)
    {
        canOpenNodeInstances[i]=NULL;
    }
    /* definition and creation of semCanOpenSDORX */
    osSemaphoreDef(semCanOpenSDORX);
    semCanOpenSDORXHandle = osSemaphoreCreate(osSemaphore(semCanOpenSDORX), 1);
    osSemaphoreWait(semCanOpenSDORXHandle,osWaitForever);
    /* definition and creation of mCanOpenSdoWrRd */
    osMutexDef(mCanOpenSdoWrRd);
    mCanOpenSdoWrRdHandle = osMutexCreate(osMutex(mCanOpenSdoWrRd));
    return APPLICATIONERROR_NONE;
}

/* called when packet has been received on CAN interface */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_packetReceived(uint16_t cob_id, uint8_t* data)
{
    uint8_t NodeId = cob_id & 0x7F;
    canOpen_enumTypeDef_COB_ID cob = cob_id & 0x780;
    canOpen_typeDef_SDO sdoRx;
    if(canOpenNodeInstances[NodeId]==NULL)
    	return APPLICATIONERROR_NOT_EXISTING;

    switch(cob)
    {
		case COB_ID_NMT:
	//    	RobotArm_allMotors_PSM_restart();
			break;
		case COB_ID_SYNC_EMCY:
			break;
		case COB_ID_TX_PDO_1:
			/* incoming encoder msg's */
			/* Encoder ID's 0x31 ... 0x35 */
			if(NodeId >= NODE_ID_ENCODER_1 && NodeId <= NODE_ID_ENCODER_5)
			{
				static uint16_t encoderRawData;
				encoderRawData = ((uint16_t)(data[1]) << 8) | (uint16_t)(data[0]);
				RobotArm_updateEncoder(NodeId, encoderRawData);
				// verstehe ich nicht TODO JJ Help
			}
			osSemaphoreRelease(semCanOpenSDORXHandle);
			break;
		case COB_ID_RX_PDO_1:
			break;
		case COB_ID_TX_PDO_2:
			break;
		case COB_ID_RX_PDO_2:
			break;
		case COB_ID_TX_PDO_3:
			break;
		case COB_ID_RX_PDO_3:
			break;
		case COB_ID_TX_PDO_4:
			break;
		case COB_ID_RX_PDO_4:
			break;
		case COB_ID_TX_SDO:
	//        break;
		case COB_ID_RX_SDO:
			for(uint32_t i=0;i<8;i++)
			{
				sdoRx.buffer[i]=data[i];
			}
			canOpenNodeInstances[NodeId]->SDOreceived = sdoRx;
			canOpenNodeInstances[NodeId]->SDOreceivedValid=1;
			osSemaphoreRelease(semCanOpenSDORXHandle);
			break;
		case COB_ID_HEARTBEAT:
			canOpenNodeInstances[NodeId]->timeLastHeartbeat=osKernelSysTick();
			break;
		default:
			break;
    }
    return APPLICATIONERROR_NONE;
}

/* check if a timeout has occurred at any node */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_checkTimeoutAllNodes()
{
    for(size_t i=0;i<=CAN_ID_MASK;i++)
    {
        if(canOpenNodeInstances[i]!=NULL)
        {
            canOpenNode_checkTimeout(i);
        }
    }
    return APPLICATIONERROR_NONE;
}

/* check if a timeout has occurred at this node */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_checkTimeout(uint8_t NodeId)
{
    if(canOpenNodeInstances[NodeId]==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    uint32_t actualTick=osKernelSysTick();
    if(canOpenNodeInstances[NodeId]->timeLastHeartbeat==0)  // first time -> no difference to calculate
        return APPLICATIONERROR_NONE;
    if((actualTick-canOpenNodeInstances[NodeId]->timeLastHeartbeat)>CANOPEN_HEARTBEAT_TIMEOUT_MS)
    {
        canOpenNodeInstances[NodeId]->NodeError=NODEERROR_HEARTBEAT_TIMEOUT;
        if(canOpenNodeInstances[NodeId]->errorCallback)
            canOpenNodeInstances[NodeId]->errorCallback(NodeId);
        return APPLICATIONERROR_DEFAULT_ERROR;
    }
    return APPLICATIONERROR_NONE;
}

/* read the error register of this node and save error in node struct */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_readErrorRegister(uint8_t NodeId)
{
    int32_t error;
    size_t length;
    if(canOpenNodeInstances[NodeId]==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    if(canOpenNode_SdoRd(NodeId,0x1001,0,&error,&length)==APPLICATIONERROR_NONE)
    {
        if(error!=0)
        {
            canOpenNodeInstances[NodeId]->NodeError=NODEERROR_DEFAULT;
            if(canOpenNodeInstances[NodeId]->errorCallback)
                canOpenNodeInstances[NodeId]->errorCallback(NodeId);
            return APPLICATIONERROR_DEFAULT_ERROR;
        }
        else
            return APPLICATIONERROR_NONE;
    }
    return APPLICATIONERROR_DEFAULT_ERROR;
}

/* read error registers of all nodes */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_checkErrorAllNodes()
{
    for(size_t i=0;i<CAN_ID_MASK;i++)
    {
        if(canOpenNodeInstances[i]!=NULL)
        {
            canOpenNode_readErrorRegister(i);
        }
    }
    return APPLICATIONERROR_NONE;
}

/* write a register with SDO over CAN
 * only one instance acting
 * -> protected by mCanOpenSdoWrRdHandle
 * -> waits forever to get mutex */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_SdoWr(uint8_t NodeId, uint16_t index, uint8_t subIndex, int32_t data, size_t length)
{
    /* if there is no registered node with this Node ID return error */
    if(canOpenNodeInstances[NodeId]==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    /* if this node has not started or finished initialization yet */
    if(canOpenNodeInstances[NodeId]->NodeState==NODESTATE_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    /* build cobID */
    uint16_t cob_id = COB_ID_RX_SDO | NodeId;
    CAN_TxHeaderTypeDef CanTxHeader;
    /* SDO that will be sent to node */
    canOpen_typeDef_SDO sdoTx;
    /* answer SDO from node */
    canOpen_typeDef_SDO sdoRx;

    /* configure message header */
    CanTxHeader.StdId = cob_id;
    CanTxHeader.DLC = 8;
    CanTxHeader.IDE = CAN_ID_STD;
    CanTxHeader.RTR = CAN_RTR_DATA;
    sdoTx.IDX[0]=index&0xFF;                            // LSB first
    sdoTx.IDX[1]=index>>8;                              // MSB second
    sdoTx.SUBIDX=subIndex;

    switch(length)
    {
    case 1:
        sdoTx.CMD=CMD_WR_LENGTH_1_BYTES;                    // data length 1 Bytes
        sdoTx.data[0]=data;
        sdoTx.data[1]=0;
        sdoTx.data[2]=0;
        sdoTx.data[3]=0;
        break;
    case 2:
        sdoTx.CMD=CMD_WR_LENGTH_2_BYTES;                    // data length 2 Bytes
        sdoTx.data[0]=data&0xFF;
        sdoTx.data[1]=(data>>8)&0xFF;
        sdoTx.data[2]=0;
        sdoTx.data[3]=0;
        break;
    case 3:
        sdoTx.CMD=CMD_WR_LENGTH_3_BYTES;                    // data length 3 Bytes
        sdoTx.data[0]=data&0xFF;
        sdoTx.data[1]=(data>>8)&0xFF;
        sdoTx.data[2]=(data>>16)&0xFF;
        sdoTx.data[3]=0;
        break;
    case 4:
        sdoTx.CMD=CMD_WR_LENGTH_4_BYTES;                    // data length 4 Bytes
        sdoTx.data[0]=data&0xFF;
        sdoTx.data[1]=(data>>8)&0xFF;
        sdoTx.data[2]=(data>>16)&0xFF;
        sdoTx.data[3]=(data>>24)&0xFF;
        break;
    }

    /* wait for mutex to prevent multiple instances of SdoWr or SdoRd..
     * ..running at the same time
     * necessary because of different task levels this can be called in */
    if(osMutexWait(mCanOpenSdoWrRdHandle,osWaitForever)==osOK)
    {
        /*  wait for CAN Mutex and send over CAN
         * necessary because CAN interface might be used by other applications
         * return error if the mutex is not defined*/
        if(canOpenNodeInstances[NodeId]->mhcanHandle==NULL)
        {
            /* release SdoWrRd Mutex */
            osMutexRelease(mCanOpenSdoWrRdHandle);
            return APPLICATIONERROR_DEFAULT_ERROR;
        }
        if(osMutexWait(*canOpenNodeInstances[NodeId]->mhcanHandle,osWaitForever)==osOK)
        {
            /* add message to can message queue */
            HAL_CAN_AddTxMessage(canOpenNodeInstances[NodeId]->hcan,&CanTxHeader,sdoTx.buffer,(uint32_t*)CAN_TX_MAILBOX0);
            /* release CAN Mutex after transmitting message */
            osMutexRelease(*canOpenNodeInstances[NodeId]->mhcanHandle);
        }
        /* return error if failed to get CAN Mutex */
        else
        {
            /* release SdoWrRd Mutex */
            osMutexRelease(mCanOpenSdoWrRdHandle);
            return APPLICATIONERROR_DEFAULT_ERROR;
        }
        /* wait for answer for some milliseconds
         * this semaphore will be set by SDOreceived
         * if timeout occurs -> timeout error */
        if(osSemaphoreWait(semCanOpenSDORXHandle,CANOPEN_SDO_TIMEOUT_MS)==osErrorOS)
        {
            /* release SdoWrRd Mutex */
            osMutexRelease(mCanOpenSdoWrRdHandle);
            /* write error code to node instance */
            canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO_TIMEOUT;
            /* if the error callback has been installed, call it */
            if(canOpenNodeInstances[NodeId]->errorCallback)
                canOpenNodeInstances[NodeId]->errorCallback(NodeId);
            return APPLICATIONERROR_DEFAULT_ERROR;
        }
        /* check if the SDO has been loaded into node instance */
        if(canOpenNodeInstances[NodeId]->SDOreceivedValid)  // if received CAN message was from the correct node
        {
            /* get message */
            sdoRx=canOpenNodeInstances[NodeId]->SDOreceived;
            /* set buffer to free */
            canOpenNodeInstances[NodeId]->SDOreceivedValid=0;
        }
        /* if message has not been loaded to node instance -> error */
        else
        {
            /* release SdoWrRd Mutex */
            osMutexRelease(mCanOpenSdoWrRdHandle);
            /* write error code to node instance */
            canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO;
            /* if the error callback has been installed, call it */
            if(canOpenNodeInstances[NodeId]->errorCallback)
                canOpenNodeInstances[NodeId]->errorCallback(NodeId);
            return APPLICATIONERROR_DEFAULT_ERROR;
        }
        /* release SdoWrRd Mutex */
        osMutexRelease(mCanOpenSdoWrRdHandle);
    }
    else
        /* if failed to get SdoWrRd Mutex -> application error */
        return APPLICATIONERROR_DEFAULT_ERROR;
    /* if the received message is an acknowledgment message -> no error */
    if(sdoRx.CMD==CMD_ACK)
    {
        /* check if written register is correct */
        if((sdoRx.IDX[0]==sdoTx.IDX[0]) &&
            (sdoRx.IDX[1]==sdoTx.IDX[1]) &&
            (sdoRx.SUBIDX==sdoTx.SUBIDX))
        {
            /* set no error in node instance and return no error */
            canOpenNodeInstances[NodeId]->NodeError=NODEERROR_NONE;
            return APPLICATIONERROR_NONE;
        }
    }
    /* if message was of different type
     * -> set error */
    canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO;
    /* if the error callback has been installed, call it */
    if(canOpenNodeInstances[NodeId]->errorCallback)
        canOpenNodeInstances[NodeId]->errorCallback(NodeId);
    return APPLICATIONERROR_DEFAULT_ERROR;
}

/* read a register with SDO over CAN
 * only one instance acting
 * -> protected by mCanOpenSdoWrRdHandle
 * -> waits forever to get mutex */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_SdoRd(uint8_t NodeId, uint16_t index, uint8_t subIndex, void* returndata, size_t* length)
{
    /* if there is no registered node with this Node ID return error */
    if(canOpenNodeInstances[NodeId]==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    /* if this node has not started or finished initialization yet */
    if(canOpenNodeInstances[NodeId]->NodeState==NODESTATE_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    /* build cobID */
    uint16_t cob_id = COB_ID_RX_SDO | NodeId;
    CAN_TxHeaderTypeDef CanTxHeader;
    /* SDO that will be sent to node */
    canOpen_typeDef_SDO sdoTx;
    /* answer SDO from node */
    canOpen_typeDef_SDO sdoRx;

    /* cast returndata for internal use */
    uint32_t* returndata_internal = (uint32_t*)returndata;
    /* maybe use later
    if(length>4)
        return APPLICATIONERROR_DEFAULT_ERROR;
    else if(length>2)
        uint32_t* returndata_internal = (uint32_t*)returndata;
    else if(length==2)
        uint16_t* returndata_internal = (uint16_t*)returndata;
    else if(length==1)
        uint8_t* returndata_internal = (uint8_t*)returndata;
    else return APPLICATIONERROR_DEFAULT_ERROR;
    */

    /* configure message header */
    CanTxHeader.StdId = cob_id;
    CanTxHeader.DLC = 8;
    CanTxHeader.IDE = CAN_ID_STD;
    CanTxHeader.RTR = CAN_RTR_DATA;
    sdoTx.IDX[0]=index&0xFF;                // LSB first
    sdoTx.IDX[1]=index>>8;                  // MSB second
    sdoTx.SUBIDX=subIndex;
    sdoTx.CMD=CMD_RD_REQ;                   // read request
    /* copy data to be sent into transmit SDO */
    for(size_t i=0;i<sizeof(sdoTx.data);i++)
    {
        sdoTx.data[i]=0;
    }
    /* wait for mutex to prevent multiple instances of SdoWr or SdoRd..
     * ..running at the same time
     * necessary because of different task levels this can be called in */
    if(osMutexWait(mCanOpenSdoWrRdHandle,osWaitForever)==osOK)
    {
        /*  wait for CAN Mutex and send over CAN
         * necessary because CAN interface might be used by other applications
         * return error if the mutex is not defined*/
        if(canOpenNodeInstances[NodeId]->mhcanHandle==NULL)
        {
            /* release SdoWrRd Mutex */
            osMutexRelease(mCanOpenSdoWrRdHandle);
            return APPLICATIONERROR_DEFAULT_ERROR;
        }
        if(osMutexWait(*canOpenNodeInstances[NodeId]->mhcanHandle,osWaitForever)==osOK)
        {
            /* add message to can message queue */
            HAL_CAN_AddTxMessage(canOpenNodeInstances[NodeId]->hcan,&CanTxHeader,sdoTx.buffer,(uint32_t*)CAN_TX_MAILBOX0);
            /* release CAN Mutex after transmitting message */
            osMutexRelease(*canOpenNodeInstances[NodeId]->mhcanHandle);
        }
        /* return error if failed to get CAN Mutex */
        else
        {
            /* release SdoWrRd Mutex */
            osMutexRelease(mCanOpenSdoWrRdHandle);
            return APPLICATIONERROR_DEFAULT_ERROR;
        }

        /* wait for answer for some milliseconds
         * this semaphore will be set by SDOreceived
         * if timeout occurs -> timeout error */
        if(osSemaphoreWait(semCanOpenSDORXHandle,CANOPEN_SDO_TIMEOUT_MS)==osErrorOS)
        {
            /* release SdoWrRd Mutex */
            osMutexRelease(mCanOpenSdoWrRdHandle);
            /* write error code to node instance */
            canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO_TIMEOUT;
            /* if the error callback has been installed, call it */
            if(canOpenNodeInstances[NodeId]->errorCallback)
                canOpenNodeInstances[NodeId]->errorCallback(NodeId);
            return APPLICATIONERROR_DEFAULT_ERROR;
        }

        /* check if the SDO has been loaded into node instance */
        if(canOpenNodeInstances[NodeId]->SDOreceivedValid)
        {
            /* get message */
            sdoRx=canOpenNodeInstances[NodeId]->SDOreceived;
            /* set buffer to free */
            canOpenNodeInstances[NodeId]->SDOreceivedValid=0;
        }
        /* if message has not been loaded to node instance -> error */
        else
        {
            /* release SdoWrRd Mutex */
            osMutexRelease(mCanOpenSdoWrRdHandle);
            /* write error code to node instance */
            canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO;
            /* if the error callback has been installed, call it */
            if(canOpenNodeInstances[NodeId]->errorCallback)
                canOpenNodeInstances[NodeId]->errorCallback(NodeId);
            return APPLICATIONERROR_DEFAULT_ERROR;
        }
        /* release SdoWrRd Mutex */
        osMutexRelease(mCanOpenSdoWrRdHandle);
    }
    else
        /* if failed to get SdoWrRd Mutex -> application error */
        return APPLICATIONERROR_DEFAULT_ERROR;
    /* check what type of SDO has been returned */
    switch(sdoRx.CMD)
    {

    /* if error message has been sent -> error */
    case CMD_ERROR:
        // TODO get exact error
        /* write error code to node instance */
        canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO;
        /* if the error callback has been installed, call it */
        if(canOpenNodeInstances[NodeId]->errorCallback)
            canOpenNodeInstances[NodeId]->errorCallback(NodeId);
        break;

    /* if acknowledgment message has been set -> error
     * did not expect acknowledgment! */
    case CMD_ACK:
        /* write error code to node instance */
        canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO;
        /* if the error callback has been installed, call it */
        if(canOpenNodeInstances[NodeId]->errorCallback)
            canOpenNodeInstances[NodeId]->errorCallback(NodeId);
        break;

    /* data read message with length from 1 until 4 Byte -> get data */
    case CMD_RD_LENGTH_1_BYTES:
        *returndata_internal = sdoRx.data[0];
        *length = 1;
        /* return no error */
        return APPLICATIONERROR_NONE;
    case CMD_RD_LENGTH_2_BYTES:
        *returndata_internal = (uint32_t)sdoRx.data[1] << 8 | sdoRx.data[0];
        *length = 2;
        /* return no error */
        return APPLICATIONERROR_NONE;
    case CMD_RD_LENGTH_3_BYTES:
        *returndata_internal = (uint32_t)sdoRx.data[2] << 16 | (uint32_t)sdoRx.data[1] << 8 | sdoRx.data[0];
        *length = 3;
        /* return no error */
        return APPLICATIONERROR_NONE;
    case CMD_RD_LENGTH_4_BYTES:
        *returndata_internal = (uint32_t)sdoRx.data[3] << 24 | (uint32_t)sdoRx.data[2] << 16 | (uint32_t)sdoRx.data[1] << 8 | sdoRx.data[0];
        *length = 4;
        /* return no error */
        return APPLICATIONERROR_NONE;

    /* data write message with length from 1 until 4 Byte -> error
     * nodes must never send write messages! */
    case CMD_WR_LENGTH_1_BYTES:
        /* write error code to node instance */
        canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO;
        /* if the error callback has been installed, call it */
        if(canOpenNodeInstances[NodeId]->errorCallback)
            canOpenNodeInstances[NodeId]->errorCallback(NodeId);
        break;
    case CMD_WR_LENGTH_2_BYTES:
        /* write error code to node instance */
        canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO;
        /* if the error callback has been installed, call it */
        if(canOpenNodeInstances[NodeId]->errorCallback)
            canOpenNodeInstances[NodeId]->errorCallback(NodeId);
        break;
    case CMD_WR_LENGTH_3_BYTES:
        /* write error code to node instance */
        canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO;
        /* if the error callback has been installed, call it */
        if(canOpenNodeInstances[NodeId]->errorCallback)
            canOpenNodeInstances[NodeId]->errorCallback(NodeId);
        break;
    case CMD_WR_LENGTH_4_BYTES:
        /* write error code to node instance */
        canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO;
        /* if the error callback has been installed, call it */
        if(canOpenNodeInstances[NodeId]->errorCallback)
            canOpenNodeInstances[NodeId]->errorCallback(NodeId);
        break;

    /* if read request message -> error
     * nodes must never send read request messages! */
    case CMD_RD_REQ:
        /* write error code to node instance */
        canOpenNodeInstances[NodeId]->NodeError=NODEERROR_SDO;
        /* if the error callback has been installed, call it */
        if(canOpenNodeInstances[NodeId]->errorCallback)
            canOpenNodeInstances[NodeId]->errorCallback(NodeId);
        break;
    }
    return APPLICATIONERROR_DEFAULT_ERROR;
}







// normale PDO's
// canOpen_PdoWr();
// pdo geschwindigkeitsdaten GLOBAL
// spezielle PDO's - eigenes mapping
canOpenNode_enumTypeDef_ApplicationError canOpenNode_PdoWrUserCob(uint16_t cobId, int32_t dataMotorOdd, int32_t dataMotorEven)
{
	/*
    CAN_TxHeaderTypeDef CanTxHeader;
	uint8_t data[8] = {0};

    // configure message header
    CanTxHeader.StdId = cobId;
    CanTxHeader.DLC = 8;
    CanTxHeader.IDE = CAN_ID_STD;
    CanTxHeader.RTR = CAN_RTR_DATA;

	// globale velocity values ziehen,
	// lokal speichern und entsprechend der CobID

    switch(cobId)
    {
    case 241: // motor 1&2
		break;
	case 242: // motor 3&4
		break;
	case 243: // motor 5&6
		break;
	case 244: // motor 7&8
		break;
	default:
		return APPLICATIONERROR_DEFAULT_ERROR; // no valid user cobId
		break;
	}

	data[0] = dataMotorOdd & 0xFF;
	data[1] = (dataMotorOdd >> 8) & 0xFF;
	data[2] = (dataMotorOdd >> 16) & 0xFF;
	data[3] = (dataMotorOdd >> 24) & 0xFF;
	data[4] = dataMotorEven & 0xFF;
	data[5] = (dataMotorEven >> 8) & 0xFF;
	data[6] = (dataMotorEven >> 16) & 0xFF;
	data[7] = (dataMotorEven >> 24) & 0xFF;
*/

//	/* FIXME TEST ME USER PDO MSG && new task's */
//
//// need own pdo mutex
//// mCanOpenPdoWrUserHandle
////	/* wait for mutex to prevent multiple instances of PdoWr..
////     * ..running at the same time
////     * necessary because of different task levels this can be called in */
//    if(osMutexWait(mCanOpenPdoWrUserHandle,osWaitForever)==osOK)
//	{
//        /*  wait for CAN Mutex and send over CAN
//         * necessary because CAN interface might be used by other applications
//         * return error if the mutex is not defined*/
//		if(osMutexWait(mhcan1Handle,osWaitForever)==NULL)
//		{
//            /* release pdo Mutex */
//            osMutexRelease(mCanOpenPdoWrUserHandle);
//            // SE return APPLICATIONERROR_DEFAULT_ERROR;
//		}
//
//		if(osMutexWait(mCanOpenPdoWrUserHandle,osWaitForever)==osOK)
//		{
//			/* add message to can message queue */
//			HAL_CAN_AddTxMessage(&hcan1,&CanTxHeader,data,(uint32_t*)CAN_TX_MAILBOX0);
//			/* release CAN Mutex after transmitting message */
//			osMutexRelease(mhcan1Handle);
//		}
//		/* return error if failed to get CAN Mutex */
//		else
//		{
//			/* release pdo Mutex */
//			osMutexRelease(mCanOpenPdoWrUserHandle);
//			// SE return APPLICATIONERROR_DEFAULT_ERROR;
//		}
//	    /* release pdo Mutex */
//        osMutexRelease(mCanOpenPdoWrUserHandle);
//	}
//	else
//	{
//        /* if failed to get SdoWrRd Mutex -> application error */
//        return APPLICATIONERROR_DEFAULT_ERROR;
//	}
	return APPLICATIONERROR_NONE;
}

void canOpenNode_MasterNmtWr(canOpen_enumTypedef_CMD_NMT CMD, uint8_t NodeId)
{
	if (NodeId != 0)
	{
		if(NodeId < NODE_ID_ROBOTARM_MIN || NodeId > NODE_ID_ROBOTARM_MAX)
		{
			return;
		}
	}
    /* build cobID */
    uint16_t cob_id = COB_ID_NMT | NodeId;
    CAN_TxHeaderTypeDef CanTxHeader;
    uint8_t txData[2];
    /* configure message header */
    CanTxHeader.StdId	= cob_id;
    CanTxHeader.DLC		= 2;
    CanTxHeader.IDE 	= CAN_ID_STD;
    CanTxHeader.RTR 	= CAN_RTR_DATA;
    txData[0]			= CMD;
    txData[1]			= NodeId;

	if(osMutexWait(mhcan1Handle, osWaitForever)==osOK)
	{
		/* add message to can message queue */
		HAL_CAN_AddTxMessage(&hcan1, &CanTxHeader, txData, (uint32_t*)CAN_TX_MAILBOX0);
		/* release CAN Mutex after transmitting message */
		osMutexRelease(mhcan1Handle);
	}
}
