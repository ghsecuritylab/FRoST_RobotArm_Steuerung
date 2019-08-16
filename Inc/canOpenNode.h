/*
 * canOpenNode.h
 *
 *  Created on: 13.05.2019
 *      Author: CM
 */

#ifndef CANOPENNODE_H_
#define CANOPENNODE_H_

#include "canOpen.h"
#include "cmsis_os.h"

#define CANOPENNODE_MAX_AMOUNT_INIT_SDOS (256)  // maximum count of init SDOs
#define CANOPEN_TIMELASTSDOTX_NONE (0xFFFFFFFF) // default value for no SDO sent
#define CANOPEN_SDO_TIMEOUT_MS (250)            // time for timeout waiting for SDO response in ms
#define CANOPEN_HEARTBEAT_TIMEOUT_MS (500)      // time for timeout since last heartbeat FIXME check suitable heartbeat time

extern osSemaphoreId semCanOpenSDORXHandle;     // semaphore handle for SDO receive indicating
extern osMutexId mCanOpenSdoWrRdHandle;         // mutex for protecting SdoWr and SdoRd

/* internal node states */
typedef enum
{
     NODESTATE_NONE             // default after start-up, can't do anything
    ,NODESTATE_INITIALIZING     // set during initialization
    ,NODESTATE_READY            // initialized and ready for operation
    ,NODESTATE_OPERATING        // operation started
}canOpenNode_enumTypeDef_NodeState;

typedef enum
{
	 NOTESTATE_NMT_NONE			= 1
    ,NOTESTATE_NMT_OPERATIONAL
    ,NOTESTATE_NMT_STOP
    ,NOTESTATE_NMT_PRE_OPERATIONAL
}canOpen_enumTypedef_NodeNMT;

/* available node types */
typedef enum
{
     NODETYPE_BGE6010           // external controller for electric motor (Dunkermotoren)
	,NODETYPE_CL4E				// external controller for electric motor (NANOTEC)
	,NODETYPE_EAM360			// external absolute single turn encoder (BAUMER)
     //TODO add more if necessary
}canOpenNode_enumTypeDef_NodeType;

/* node error types */
typedef enum
{
    NODEERROR_NONE = 0x00           // no error
   ,NODEERROR_SDO_TIMEOUT           // sdo response timeout after sending
   ,NODEERROR_HEARTBEAT_TIMEOUT     // heartbeat timeout
   ,NODEERROR_SDO                   // error in SDO response packet FIXME differentiate
   ,NODEERROR_DEFAULT = 0xFFFFFFFF  // other errors
}canOpenNode_enumTypeDef_NodeError;

/* internal errors for applications */
typedef enum
{
      APPLICATIONERROR_NONE  = 0x00         // no error
     ,APPLICATIONERROR_NOT_EXISTING         // could not find node
     // TODO
    ,APPLICATIONERROR_DEFAULT_ERROR = 0xFF  // other errors
}canOpenNode_enumTypeDef_ApplicationError;


typedef struct
{
    /* Node Identifier, 0:127 */
    uint8_t NodeId;
    /* Node Type from list */
    canOpenNode_enumTypeDef_NodeType NodeType;
    /* actual internal Node State */
    canOpenNode_enumTypeDef_NodeState NodeState;
    /* error code, 0x00 if no error occurred */
    canOpenNode_enumTypeDef_NodeError NodeError;
    /* NMT Status () */
    canOpen_enumTypedef_NodeNMT NodeNmtStatus;
    /* error callback function pointer
     * @param uint8_t NodeId: Node Identifier */
    void (*errorCallback)(uint8_t);
    /* pointer to can instance to use for communication */
    CAN_HandleTypeDef* hcan;
    /* pointer to mutex of this can instance */
    osMutexId* mhcanHandle;
    /* parameters for initialization
     * crucial part for setting up node!
     * must be configured by user!! */
    struct
    {
        /* total used amount of InitSDOs
         * needed because InitSDOs doesn't have dynamic size */
        uint8_t initSDOsAmount;
        /* SDOs that will be sent when addInstance is called
         * -> must be set before that!
         * user must configure that at all!
         * be careful with order!
         * when more than CANOPENNODE_MAX_AMOUNT_INIT_SDOS:
         * -> user must send after that manually using SdoWr */
        canOpen_typeDef_SDOprimitive InitSDOs[CANOPENNODE_MAX_AMOUNT_INIT_SDOS];
    }initialization;
    /* internal use
     * time stamp of the last received heartbeat
     * needed by checkTimeout
     * set by packetReceived */
    uint32_t timeLastHeartbeat;
    /* internal use only!
     * last received SDO from this node
     * needed by SdoWr and SdoRd
     * set by SDOreceived */
    canOpen_typeDef_SDO SDOreceived;
    /* internal use only!
     * use as boolean
     * set if SDOreceived is valid
     * set by SDOreceived
     * cleared by SdoWr or SdoRd */
    uint8_t SDOreceivedValid;
}canOpenNode_typeDef_Node;

/* make pointer array to instances public
 * contains pointers to registered node instances
 * index equals node id
 * add instance by addInstance */
extern canOpenNode_typeDef_Node* canOpenNodeInstances[CAN_ID_MASK+1];

/* build SDO by given parameters */
void canOpenNode_buildSDO(canOpen_typeDef_SDO* SDO, uint8_t NodeId, uint16_t index, uint8_t subIndex, int32_t data, uint8_t write);
/* restart initialization of the node */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_resetInitialization(uint8_t NodeId);
/* add node to node array
 * starts node initialization */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_addInstance(canOpenNode_typeDef_Node* pointerToNode, canOpen_typeDef_SDOprimitive* initSDOs, size_t initSDOsAmount);
/* initializes this module by setting node array to nullptr
 * use only once at startup */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_initialize();
/* called when packet has been received on CAN interface */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_packetReceived(uint16_t cob_id, uint8_t* data);
/* called when received packet is SDO */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_SDOreceived(uint8_t NodeId, canOpen_typeDef_SDO sdoRx);
/* check if a timeout has occurred at any node */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_checkTimeoutAllNodes();
/* check if a timeout has occurred at this node */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_checkTimeout(uint8_t NodeId);
/* read the error register of this node and save error in node struct */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_readErrorRegister(uint8_t NodeId);
/* read error registers of all nodes */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_checkErrorAllNodes();

/* write a register with SDO over CAN
 * only one instance acting
 * -> protected by mCanOpenSdoWrRdHandle
 * -> waits forever to get mutex */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_SdoWr(uint8_t NodeId, uint16_t index, uint8_t subIndex, int32_t data, size_t length);

/* read a register with SDO over CAN
 * only one instance acting
 * -> protected by mCanOpenSdoWrRdHandle
 * -> waits forever to get mutex */
canOpenNode_enumTypeDef_ApplicationError canOpenNode_SdoRd(uint8_t NodeId, uint16_t index, uint8_t subIndex, void* returndata, size_t* length);

canOpenNode_enumTypeDef_ApplicationError canOpenNode_PdoWrUserCob(uint16_t cobId, int32_t dataMotorOdd, int32_t dataMotorEven);
void canOpenNode_MasterNmtWr(uint8_t CMD, uint8_t NodeId);

#endif /* CANOPENNODE_H_ */
