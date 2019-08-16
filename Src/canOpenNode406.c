/*
 * canOpenNode406.c
 *
 *  Created on: 25.05.2019
 *      Author: CM
 */

#include "canOpenNode406.h"

/* sends set of SDOs to initialize the node with parameters in velocity struct
 * call after initializing node via canOpenNode */
canOpenNode_enumTypeDef_ApplicationError canOpenNode406_sendInitVelocityValues(canOpenNode_typeDef_Node406* Node)
{
    return APPLICATIONERROR_NONE;
}

/* read state word by SDO */
canOpenNode406_enumTypeDef_States canOpenNode406_readStateWord(uint8_t NodeId)
{
	return CANOPEN406_STATE_FAULT_REACTION_ACTIVE;
}

/* read mode word by SDO */
canOpenNode_enumTypeDef_ApplicationError canOpenNode406_readControlWord(uint8_t NodeId)
{
    return APPLICATIONERROR_NONE;
}

