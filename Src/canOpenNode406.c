/*
 * canOpenNode406.c
 *
 *  Created on: 25.05.2019
 *      Author: CM
 */

#include "canOpenNode406.h"

/* sends set of SDOs to initialize the node with parameters in velocity struct
 * call after initializing node via canOpenNode */
canOpenNode_enumTypeDef_ApplicationError canOpenNode406_sendInitValues(canOpenNode_typeDef_Node406* Node)
{
	// Einheiten? FIXME
    //if(canOpenNode_SdoWr(Node->NodeBasic.NodeId,CANOPEN_IDX_VELOCITY,CANOPEN_SUBIDX_VELOCITY_MIN,Node->ValuesVelocityMode.velocityMinAmount,CANOPEN_LENGTH_VELOCITY_MIN)            != APPLICATIONERROR_NONE)
	//    return APPLICATIONERROR_DEFAULT_ERROR;
	// Einheit Messschritt
	// Einheit Geschwindigkeit
	// Einheit Beschleunigung

    return APPLICATIONERROR_NONE;
}

/* read mode word by SDO */
canOpenNode_enumTypeDef_ApplicationError canOpenNode406_readDiagnose(uint8_t NodeId)
{
    return APPLICATIONERROR_NONE;
}

