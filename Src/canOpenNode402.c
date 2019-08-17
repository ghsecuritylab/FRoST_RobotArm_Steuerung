/*
 * canOpenNode402.c
 *
 *  Created on: 25.05.2019
 *      Author: CM
 */

#include "canOpenNode402.h"

/* sends set of SDOs to initialize the node with parameters in velocity struct
 * call after initializing node via canOpenNode */
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_sendInitVelocityValues(canOpenNode_typeDef_Node402* Node)
{
    if(canOpenNode_SdoWr(Node->NodeBasic.NodeId,CANOPEN_IDX_VELOCITY,CANOPEN_SUBIDX_VELOCITY_MIN,Node->ValuesVelocityMode.velocityMinAmount,CANOPEN_LENGTH_VELOCITY_MIN)            != APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    if(canOpenNode_SdoWr(Node->NodeBasic.NodeId,CANOPEN_IDX_VELOCITY,CANOPEN_SUBIDX_VELOCITY_MAX,Node->ValuesVelocityMode.velocityMaxAmount,CANOPEN_LENGTH_VELOCITY_MAX)            != APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    if(canOpenNode_SdoWr(Node->NodeBasic.NodeId,CANOPEN_IDX_ACCELERATION,CANOPEN_SUBIDX_ACC_DELTA_SPEED,Node->ValuesVelocityMode.accDeltaSpeed,CANOPEN_LENGTH_ACC_DELTA_SPEED)         != APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    if(canOpenNode_SdoWr(Node->NodeBasic.NodeId,CANOPEN_IDX_ACCELERATION,CANOPEN_SUBIDX_ACC_DELTA_TIME,Node->ValuesVelocityMode.accDeltaTime,CANOPEN_LENGTH_ACC_DELTA_TIME)           != APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    if(canOpenNode_SdoWr(Node->NodeBasic.NodeId,CANOPEN_IDX_DECELERATION,CANOPEN_SUBIDX_DEC_DELTA_SPEED,Node->ValuesVelocityMode.decDeltaSpeed,CANOPEN_LENGTH_DEC_DELTA_SPEED)         != APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    if(canOpenNode_SdoWr(Node->NodeBasic.NodeId,CANOPEN_IDX_DECELERATION,CANOPEN_SUBIDX_DEC_DELTA_TIME,Node->ValuesVelocityMode.decDeltaTime,CANOPEN_LENGTH_DEC_DELTA_TIME)           != APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    if(canOpenNode_SdoWr(Node->NodeBasic.NodeId,CANOPEN_IDX_QUICKSTOP_RAMP,CANOPEN_SUBIDX_QS_DELTA_SPEED,Node->ValuesVelocityMode.quickStopDeltaSpeed,CANOPEN_LENGTH_QS_DELTA_SPEED)  != APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    if(canOpenNode_SdoWr(Node->NodeBasic.NodeId,CANOPEN_IDX_QUICKSTOP_RAMP,CANOPEN_SUBIDX_QS_DELTA_TIME,Node->ValuesVelocityMode.quickStopDeltaTime,CANOPEN_LENGTH_QS_DELTA_TIME)    != APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    if(canOpenNode_SdoWr(Node->NodeBasic.NodeId,CANOPEN_IDX_SCALING,CANOPEN_SUBIDX_SCALING_NUMERATOR,Node->ValuesVelocityMode.scalingNumerator,CANOPEN_LENGTH_SCALING_NUMERATOR)         != APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    if(canOpenNode_SdoWr(Node->NodeBasic.NodeId,CANOPEN_IDX_SCALING,CANOPEN_SUBIDX_SCALING_DENUMERATOR,Node->ValuesVelocityMode.scalingDenumerator,CANOPEN_LENGTH_SCALING_DENUMERATOR)     != APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    return APPLICATIONERROR_NONE;
}

/* read state word by SDO */
canOpenNode402_enumTypeDef_States canOpenNode402_readStateWord(uint8_t NodeId)
{
    canOpenNode_typeDef_StateWord stateword;
    size_t length;
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return CANOPEN402_STATE_FAULT;
    canOpenNode_SdoRd(NodeId,CANOPEN_IDX_STATUS_WORD,0,&stateword,&length);
    if(stateword.fault)  // check for fault
    {
        if((stateword.data & CANOPENNODE402_STATE_WORD_FAULT_REACTION_ACTIVE_MASK) == CANOPENNODE402_STATE_WORD_FAULT_REACTION_ACTIVE)
            Node->Node402State = CANOPEN402_STATE_FAULT_REACTION_ACTIVE;
        else if((stateword.data & CANOPENNODE402_STATE_WORD_FAULT_REACTION_ACTIVE_MASK) == CANOPENNODE402_STATE_WORD_FAULT)
            Node->Node402State = CANOPEN402_STATE_FAULT;
    }
    else if(stateword.switchOnDisabled)
    {
        Node->Node402State = CANOPEN402_STATE_SWITCH_ON_DISABLED;
    }
    else if(stateword.quickStop==0)
    {
        if((stateword.data & CANOPENNODE402_STATE_WORD_QUICK_STOP_ACTIVE_MASK) == CANOPENNODE402_STATE_WORD_QUICK_STOP_ACTIVE)
            Node->Node402State = CANOPEN402_STATE_QUICK_STOP_ACTIVE;
        else if((stateword.data & CANOPENNODE402_STATE_WORD_NOT_READY_TO_SWITCH_ON_MASK) == CANOPENNODE402_STATE_WORD_NOT_READY_TO_SWITCH_ON)
            Node->Node402State = CANOPEN402_STATE_NOT_READY_TO_SWITCH_ON;
    }
    else if(stateword.operationEnabled)
    {
        Node->Node402State = CANOPEN402_STATE_OPERATION_ENABLED;
    }
    else if(stateword.switchedOn)
    {
        Node->Node402State = CANOPEN402_STATE_SWITCHED_ON;
    }
    else if(stateword.readyToSwitchOn)
    {
        Node->Node402State = CANOPEN402_STATE_READY_TO_SWITCH_ON;
    }
    return Node->Node402State;
}

canOpenNode_enumTypeDef_ApplicationError canOpenNode402_readControlWord(uint8_t NodeId)
{
    canOpenNode_typeDef_ControlWord controlword;
    canOpenNode_typeDef_Node402* Node;
    size_t length;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    if(canOpenNode_SdoRd(NodeId,CANOPEN_IDX_CONTROL_WORD,0,(void*)&controlword, &length)!=APPLICATIONERROR_NONE)
        return APPLICATIONERROR_DEFAULT_ERROR;
    switch(Node->Node402Mode)
    {
    case NODE402MODE_VELOCITY:
        Node->ValuesVelocityMode.stopActive=controlword.halt;
        break;
    // TODO add other modes
    default:
        break;
    }
    return APPLICATIONERROR_NONE;
}

canOpenNode_enumTypeDef_ApplicationError canOpenNode402_Stop(uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    if(Node->Node402State==CANOPEN402_STATE_OPERATION_ENABLED)
        if(canOpenNode_SdoWr(NodeId,CANOPEN_IDX_CONTROL_WORD,0,0x10F,2)==APPLICATIONERROR_NONE)
        {
            Node->ValuesVelocityMode.stopActive=1;
            Node->NodeBasic.NodeState=NODESTATE_READY;
            return APPLICATIONERROR_NONE;
        }
    return APPLICATIONERROR_DEFAULT_ERROR;
}

canOpenNode_enumTypeDef_ApplicationError canOpenNode402_quickStop(uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    if(canOpenNode_SdoWr(NodeId,CANOPEN_IDX_CONTROL_WORD,0,0x0B,2)==APPLICATIONERROR_NONE)
    {
        Node->ValuesVelocityMode.stopActive=0;
        Node->NodeBasic.NodeState=NODESTATE_READY;
        return APPLICATIONERROR_NONE;
    }
    return APPLICATIONERROR_DEFAULT_ERROR;
}

/* shutdown -> switch on disabled */
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_PSM_Transition_2_6_8(uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    if(canOpenNode_SdoWr(NodeId,CANOPEN_IDX_CONTROL_WORD,0,0x06,2)==APPLICATIONERROR_NONE)
    {
        Node->ValuesVelocityMode.stopActive=0;
        Node->NodeBasic.NodeState=NODESTATE_READY;
        Node->Node402State=CANOPEN402_STATE_READY_TO_SWITCH_ON;
        return APPLICATIONERROR_NONE;
    }
    return APPLICATIONERROR_DEFAULT_ERROR;
}

/* switch on -> switched on*/
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_PSM_Transition_3(uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    if(canOpenNode_SdoWr(NodeId,CANOPEN_IDX_CONTROL_WORD,0,0x07,2)==APPLICATIONERROR_NONE)
    {
        Node->ValuesVelocityMode.stopActive=0;
        Node->NodeBasic.NodeState=NODESTATE_READY;
        Node->Node402State=CANOPEN402_STATE_SWITCHED_ON;
        return APPLICATIONERROR_NONE;
    }
    return APPLICATIONERROR_DEFAULT_ERROR;
}

/* disable voltage -> switch on disabled */
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_PSM_Transition_7_9_10_12(uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    if(canOpenNode_SdoWr(NodeId,CANOPEN_IDX_CONTROL_WORD,0,0x00,2)==APPLICATIONERROR_NONE)
    {
        Node->ValuesVelocityMode.stopActive=0;
        Node->NodeBasic.NodeState=NODESTATE_READY;
        Node->Node402State=CANOPEN402_STATE_SWITCH_ON_DISABLED;
        return APPLICATIONERROR_NONE;
    }
    return APPLICATIONERROR_DEFAULT_ERROR;
}

/* quick stop -> quick stop active || switch on disabled */
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_PSM_Transition_7_10_11(uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    /* if operation enabled -> perform quick stop */
    if(Node->Node402State==CANOPEN402_STATE_OPERATION_ENABLED)
    {
        if(canOpenNode_SdoWr(NodeId,CANOPEN_IDX_CONTROL_WORD,0,0x02,2)==APPLICATIONERROR_NONE)
        {
            Node->ValuesVelocityMode.stopActive=0;
            Node->NodeBasic.NodeState=NODESTATE_READY;
            Node->Node402State=CANOPEN402_STATE_QUICK_STOP_ACTIVE;
            return APPLICATIONERROR_NONE;
        }
        return APPLICATIONERROR_DEFAULT_ERROR;
    }
    /* in other state -> switch in disabled */
    else
    {
        if(canOpenNode_SdoWr(NodeId,CANOPEN_IDX_CONTROL_WORD,0,0x02,2)==APPLICATIONERROR_NONE)
        {
            Node->ValuesVelocityMode.stopActive=0;
            Node->NodeBasic.NodeState=NODESTATE_READY;
            Node->Node402State=CANOPEN402_STATE_SWITCH_ON_DISABLED;
            return APPLICATIONERROR_NONE;
        }
    }
    return APPLICATIONERROR_DEFAULT_ERROR;
}

/* disable operation -> switch on */
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_PSM_Transition_5(uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    if(canOpenNode_SdoWr(NodeId,CANOPEN_IDX_CONTROL_WORD,0,0x07,2)==APPLICATIONERROR_NONE)
    {
        Node->ValuesVelocityMode.stopActive=0;
        Node->NodeBasic.NodeState=NODESTATE_READY;
        Node->Node402State=CANOPEN402_STATE_SWITCHED_ON;
        return APPLICATIONERROR_NONE;
    }
    return APPLICATIONERROR_DEFAULT_ERROR;
}

/* enable operation -> operation enabled*/
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_PSM_Transition_4_16(uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    if(canOpenNode_SdoWr(NodeId,CANOPEN_IDX_CONTROL_WORD,0,0x0F,2)==APPLICATIONERROR_NONE)
    {
        Node->ValuesVelocityMode.stopActive=0;
        Node->NodeBasic.NodeState=NODESTATE_OPERATING;
        Node->Node402State=CANOPEN402_STATE_OPERATION_ENABLED;
        return APPLICATIONERROR_NONE;
    }
    return APPLICATIONERROR_DEFAULT_ERROR;
}

/* fault reset -> switch on disabled*/
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_PSM_Transition_15(uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    if(canOpenNode_SdoWr(NodeId,CANOPEN_IDX_CONTROL_WORD,0,0x80,2)==APPLICATIONERROR_NONE)
    {
        Node->ValuesVelocityMode.stopActive=0;
        Node->NodeBasic.NodeState=NODESTATE_READY;
        Node->Node402State=CANOPEN402_STATE_SWITCH_ON_DISABLED;
        return APPLICATIONERROR_NONE;
    }
    return APPLICATIONERROR_DEFAULT_ERROR;
}

canOpenNode_enumTypeDef_ApplicationError canOpenNode402_PowerStateMachine(uint8_t NodeId, canOpenNode402_enumTypeDef_States targetState)
{
    canOpenNode_enumTypeDef_ApplicationError error;
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return APPLICATIONERROR_NOT_EXISTING;
    /* SE FIXME */
//    if(Node->NodeBasic.NodeState < NODESTATE_READY)
//        return APPLICATIONERROR_DEFAULT_ERROR;
    if(Node->Node402State==targetState)
        return APPLICATIONERROR_NONE;

    while(Node->Node402State!=targetState)
    {
        /* switch actual states */
        switch(Node->Node402State)
        {
        /* this state should never occur */
        case CANOPEN402_STATE_NOT_READY_TO_SWITCH_ON:
            return APPLICATIONERROR_DEFAULT_ERROR;
        /* this state should never occur */
        case CANOPEN402_STATE_FAULT:
            error = canOpenNode402_PSM_Transition_15(NodeId);
            if(error!=APPLICATIONERROR_NONE)
                return error;
            break;
            return APPLICATIONERROR_DEFAULT_ERROR;
        /* this state should never occur */
        case CANOPEN402_STATE_FAULT_REACTION_ACTIVE:
            return APPLICATIONERROR_DEFAULT_ERROR;
        /* action after quick stop */
        case CANOPEN402_STATE_QUICK_STOP_ACTIVE:
        {
            switch(targetState)
            {
            /* re-enable operation */
            case CANOPEN402_STATE_OPERATION_ENABLED:
            {
                error = canOpenNode402_PSM_Transition_4_16(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            /* go to switch on disabled */
            case CANOPEN402_STATE_SWITCH_ON_DISABLED:
            case CANOPEN402_STATE_READY_TO_SWITCH_ON:
            case CANOPEN402_STATE_SWITCHED_ON:
            {
                error = canOpenNode402_PSM_Transition_7_9_10_12(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            default:
                return APPLICATIONERROR_DEFAULT_ERROR;
            }
            break;
        }
        /* if operation enabled is requested, run through state machine */
        case CANOPEN402_STATE_OPERATION_ENABLED:
        {
            switch(targetState)
            {
            case CANOPEN402_STATE_SWITCHED_ON:
            {
                error = canOpenNode402_PSM_Transition_5(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            case CANOPEN402_STATE_READY_TO_SWITCH_ON:
            {
                error = canOpenNode402_PSM_Transition_2_6_8(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            case CANOPEN402_STATE_SWITCH_ON_DISABLED:
            {
                error = canOpenNode402_PSM_Transition_7_9_10_12(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            case CANOPEN402_STATE_QUICK_STOP_ACTIVE:
            {
                error = canOpenNode402_PSM_Transition_7_10_11(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            default:
                return APPLICATIONERROR_DEFAULT_ERROR;
            }
            break;
        }
        case CANOPEN402_STATE_SWITCH_ON_DISABLED:
        {
            switch(targetState)
            {
            case CANOPEN402_STATE_OPERATION_ENABLED:
            case CANOPEN402_STATE_READY_TO_SWITCH_ON:
            case CANOPEN402_STATE_SWITCHED_ON:
            {
                error = canOpenNode402_PSM_Transition_2_6_8(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            default:
                return APPLICATIONERROR_DEFAULT_ERROR;
            }
            break;
        }
        case CANOPEN402_STATE_SWITCHED_ON:
        {
            switch(targetState)
            {
            case CANOPEN402_STATE_OPERATION_ENABLED:
            {
                error = canOpenNode402_PSM_Transition_4_16(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            case CANOPEN402_STATE_READY_TO_SWITCH_ON:
            {
                error = canOpenNode402_PSM_Transition_2_6_8(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            case CANOPEN402_STATE_SWITCH_ON_DISABLED:
            {
                error = canOpenNode402_PSM_Transition_7_10_11(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            default:
                return APPLICATIONERROR_DEFAULT_ERROR;
            }
            break;
        }
        case CANOPEN402_STATE_READY_TO_SWITCH_ON:
        {
            switch(targetState)
            {
            case CANOPEN402_STATE_OPERATION_ENABLED:
            case CANOPEN402_STATE_SWITCHED_ON:
            {
                error = canOpenNode402_PSM_Transition_3(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            case CANOPEN402_STATE_SWITCH_ON_DISABLED:
            {
                error = canOpenNode402_PSM_Transition_7_10_11(NodeId);
                if(error!=APPLICATIONERROR_NONE)
                    return error;
                break;
            }
            default:
                return APPLICATIONERROR_DEFAULT_ERROR;
            }
            break;
        }
        }
    }
    return APPLICATIONERROR_NONE;
}


canOpenNode402_enumTypeDef_States Arm_readStateWord(uint8_t NodeId)
{
    return canOpenNode402_readStateWord(NodeId);
}

canOpenNode_enumTypeDef_ApplicationError Arm_readControlWord(uint8_t NodeId)
{
    return canOpenNode402_readControlWord(NodeId);
}

uint32_t Arm_readVelocity(uint8_t NodeId)
{
	uint32_t result;
	uint32_t* presult = &result;
    size_t length;

    if(canOpenNode_SdoRd(NodeId,0x3A04,0x01,presult,&length)!=APPLICATIONERROR_NONE)
        *presult = 0x7FFFFFFF;

    result=*presult;
    return result;
}
