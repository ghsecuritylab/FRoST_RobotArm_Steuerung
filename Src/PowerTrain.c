/*
 * PowerTrain.c
 *
 *  Created on: 01.04.2019
 *      Author: CM
 */

#include "PowerTrain.h"
#include "BGE6010_BG65S.h"
#include "can.h"
#include "globalDataStructures_CM_MTR_FRoST.h"

/* DEBUG only!*/
typedef struct
{
    int8_t x_direction;
    int8_t y_direction;
    uint8_t maxVelocity;
    int16_t velocity_right_perc;
    int16_t velocity_left_perc;
    int16_t velocity_right_rpm;
    int16_t velocity_left_rpm;
}tVelocityDebug;

#define VELOCITY_TABLE_COUNT (1000)
tVelocityDebug VelocityTable[VELOCITY_TABLE_COUNT];
uint32_t VelocityTableIndex;
void addVelocityTable(int8_t x_direction, int8_t y_direction, uint8_t maxVelocity, int16_t velocity_right_perc,
    int16_t velocity_left_perc, int16_t velocity_right_rpm, int16_t velocity_left_rpm)
{
    VelocityTable[VelocityTableIndex].x_direction=x_direction;
    VelocityTable[VelocityTableIndex].y_direction=y_direction;
    VelocityTable[VelocityTableIndex].maxVelocity=maxVelocity;
    VelocityTable[VelocityTableIndex].velocity_right_perc=velocity_right_perc;
    VelocityTable[VelocityTableIndex].velocity_left_perc=velocity_left_perc;
    VelocityTable[VelocityTableIndex].velocity_right_rpm=velocity_right_rpm;
    VelocityTable[VelocityTableIndex].velocity_left_rpm=velocity_left_rpm;
    VelocityTableIndex=(VelocityTableIndex+1)%VELOCITY_TABLE_COUNT;
}
/* DEBUG only end*/


#if TEST_MODE
canOpenNode_typeDef_Node402 powerTrain_MotorTest;   // use only in test mode
#else
canOpenNode_typeDef_Node402 powerTrain_MotorLeft;
canOpenNode_typeDef_Node402 powerTrain_MotorRight;
#endif

void PowerTrain_writeGlobalData()
{
    globalData_typeDef_powerTrain_IO_GS powerTrainData;
#if TEST_MODE
    powerTrainData.actualState=(powerTrain_MotorTest.Node402State==CANOPEN402_STATE_OPERATION_ENABLED);
    powerTrainData.actualVelocityLeft=powerTrain_MotorTest.ValuesVelocityMode.velocity_actual;
    powerTrainData.actualVelocityRight=powerTrain_MotorTest.ValuesVelocityMode.velocity_actual;
    powerTrainData.dataID=GLOBALDATA_ID_POWERTRAIN;
    powerTrainData.targetVelocityLeft=powerTrain_MotorTest.ValuesVelocityMode.velocity_demand;
    powerTrainData.targetVelocityRight=powerTrain_MotorTest.ValuesVelocityMode.velocity_demand;
#else
    powerTrainData.actualState=(powerTrain_MotorLeft.Node402State==CANOPEN402_STATE_OPERATION_ENABLED) && (powerTrain_MotorRight.Node402State==CANOPEN402_STATE_OPERATION_ENABLED);
    powerTrainData.actualVelocityLeft=powerTrain_MotorLeft.ValuesVelocityMode.velocity_actual;
    powerTrainData.actualVelocityRight=powerTrain_MotorRight.ValuesVelocityMode.velocity_actual;
    powerTrainData.dataID=GLOBALDATA_ID_POWERTRAIN;
    powerTrainData.targetVelocityLeft=powerTrain_MotorLeft.ValuesVelocityMode.velocity_demand;
    powerTrainData.targetVelocityRight=powerTrain_MotorRight.ValuesVelocityMode.velocity_demand;
#endif
    if(globalDataStructures_setPowerTrain_IO_GS(powerTrainData)!=GLOBAL_DATA_STRUCT_SET_OK)
        Error_Handler();
}

/* enables or disables engine depending of desired state */
void PowerTrain_updateState()
{
    globalData_typeDef_powerTrain_GS_IO messagePowerTrainRX = globalDataStructures_getPowerTrain_GS_IO();
    if(messagePowerTrainRX.dataID==GLOBALDATA_ID_POWERTRAIN)
    {
        PowerTrain_setVelocity(messagePowerTrainRX.xValue,messagePowerTrainRX.yValue,messagePowerTrainRX.maxVelocity);
        if(messagePowerTrainRX.targetState!=0)
        {
            PowerTrain_drive();
        }
        else
        {
            PowerTrain_shutdown();
        }
    }
    else
        Error_Handler();
}

void PowerTrain_writeVelocity(int16_t velocity_left_rpm, int16_t velocity_right_rpm)
{
#if TEST_MODE
    if(powerTrain_MotorTest.Node402State==CANOPEN402_STATE_OPERATION_ENABLED)
        if(canOpenNode_SdoWr(NODE_ID_MOTOR_TEST,0x6042,0,velocity_left_rpm,4)!=APPLICATIONERROR_NONE)
            return;
#else
    if(powerTrain_MotorLeft.Node402State==CANOPEN402_STATE_OPERATION_ENABLED)
        if(canOpenNode_SdoWr(NODE_ID_MOTOR_LEFT,0x6042,0,velocity_left_rpm,4)!=APPLICATIONERROR_NONE)
            return;
    if(powerTrain_MotorRight.Node402State==CANOPEN402_STATE_OPERATION_ENABLED)
        if(canOpenNode_SdoWr(NODE_ID_MOTOR_RIGHT,0x6042,0,velocity_right_rpm,4)!=APPLICATIONERROR_NONE)
            return;
#endif //TEST_MODE
}

/*
 * @param x_direction: [-100:100], value of direction transverse to direction of travel
 * @param y_direction: [-100:100], value of direction along to direction of travel
 * @param maxVelocity: [0:100], percent of max velocity
 *
 */
#define POWERTRAIN_DEATHCENTERPOINT_DEFLECTION (20)
void PowerTrain_setVelocity(int8_t x_direction, int8_t y_direction, uint8_t maxVelocity /*in percent*/)
{
    int16_t velocity_right_perc;
    int16_t velocity_left_perc;
    int16_t velocity_right_rpm;
    int16_t velocity_left_rpm;
    uint8_t maxVelocity_01m_s;
    maxVelocity_01m_s = maxVelocity*BGE6010_BG65S_VELOCITY_MAX_01_M_PER_S/100;
//    if(maxVelocity>BGE6010_BG65S_VELOCITY_MAX_01_M_PER_S)
//        maxVelocity=BGE6010_BG65S_VELOCITY_MAX_01_M_PER_S;

    /* if inside death center point
     * -> do not use value, set to zero */
    if(x_direction<=POWERTRAIN_DEATHCENTERPOINT_DEFLECTION && x_direction >= -POWERTRAIN_DEATHCENTERPOINT_DEFLECTION)
        x_direction=0;
    else
    {
        /* outside death center point:
         * -> map values from death center point to 100 to values between 0 and 100 */
        if(x_direction<0)
            x_direction = -POWERTRAIN_DEATHCENTERPOINT_DEFLECTION+(int32_t)x_direction*(100-POWERTRAIN_DEATHCENTERPOINT_DEFLECTION)/100;
        else
            x_direction = POWERTRAIN_DEATHCENTERPOINT_DEFLECTION+(int32_t)x_direction*(100-POWERTRAIN_DEATHCENTERPOINT_DEFLECTION)/100;
    }

    /* if inside death center point
     * -> do not use value, set to zero */
    if(y_direction<=POWERTRAIN_DEATHCENTERPOINT_DEFLECTION && y_direction >= -POWERTRAIN_DEATHCENTERPOINT_DEFLECTION)
        y_direction=0;
    else
    {
        /* outside death center point:
         * -> map values from death center point to 100 to values between 0 and 100 */
        if(y_direction<0)
            y_direction = -POWERTRAIN_DEATHCENTERPOINT_DEFLECTION+(int32_t)y_direction*(100-POWERTRAIN_DEATHCENTERPOINT_DEFLECTION)/100;
        else
            y_direction = POWERTRAIN_DEATHCENTERPOINT_DEFLECTION+(int32_t)y_direction*(100-POWERTRAIN_DEATHCENTERPOINT_DEFLECTION)/100;
    }

    /* calculate percentages of maximum velocity for both motors*/
    velocity_left_perc=y_direction+x_direction/3;
    velocity_right_perc=y_direction-x_direction/3;
    /* if any velocity percentage is above 100 -> set to 100 */
    if(velocity_left_perc>100)
        velocity_left_perc=100;
    if(velocity_right_perc>100)
        velocity_right_perc=100;

    /* invert left because motor is mounted contrary */
    velocity_left_rpm=0-(velocity_left_perc*maxVelocity_01m_s*SECONDS_PER_MIN*GEARING_NUMERATOR/GEARING_DENUMERATOR/GEAR_DIAMETER*MM_PER_M/100/10*CONST_PI_DENUM/CONST_PI_NUM);
    velocity_right_rpm=velocity_right_perc*maxVelocity_01m_s*SECONDS_PER_MIN*GEARING_NUMERATOR/GEARING_DENUMERATOR/GEAR_DIAMETER*MM_PER_M/100/10*CONST_PI_DENUM/CONST_PI_NUM;
    addVelocityTable(x_direction,y_direction,maxVelocity_01m_s,velocity_right_perc,velocity_left_perc,velocity_right_rpm,velocity_left_rpm);

    /* if we are outside the specification -> do not apply */
    if((velocity_left_rpm>RPM_MAX) || (velocity_left_rpm<(-RPM_MAX)))
        return;
    if((velocity_right_rpm>RPM_MAX) || (velocity_right_rpm<(-RPM_MAX)))
        return;

    /* in test mode use only the test motor */
#if TEST_MODE
    powerTrain_MotorTest.ValuesVelocityMode.velocity_demand = velocity_left_rpm;
    if(canOpenNode_SdoWr(NODE_ID_MOTOR_TEST,0x6042,0,velocity_left_rpm,4)!=APPLICATIONERROR_NONE)
        return;
#else
    powerTrain_MotorLeft.ValuesVelocityMode.velocity_demand = velocity_left_rpm;
    powerTrain_MotorRight.ValuesVelocityMode.velocity_demand = velocity_right_rpm;
    if(powerTrain_MotorLeft.Node402State==CANOPEN402_STATE_OPERATION_ENABLED)
        if(canOpenNode_SdoWr(NODE_ID_MOTOR_LEFT,0x6042,0,velocity_left_rpm,4)!=APPLICATIONERROR_NONE)
            return;
    if(powerTrain_MotorRight.Node402State==CANOPEN402_STATE_OPERATION_ENABLED)
        if(canOpenNode_SdoWr(NODE_ID_MOTOR_RIGHT,0x6042,0,velocity_right_rpm,4)!=APPLICATIONERROR_NONE)
            return;
#endif
    PowerTrain_writeGlobalData();
}


/* called when error occurs
 * stops motor and resets node */
void PowerTrain_errorHandler(uint8_t NodeId)
{
    static uint8_t active=0;
    canOpenNode_typeDef_Node402* Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return;
//    Error_Handler();    // FIXME do not do that!
    if(active)
        return;
    active=1;
    switch(Node->NodeBasic.NodeError)
    {
    case NODEERROR_HEARTBEAT_TIMEOUT:
    {
        // TODO disable power stage
        PowerTrain_quickstop();
        canOpenNode_resetInitialization(NodeId);
        break;
    }
    case NODEERROR_SDO_TIMEOUT:
    {
        // TODO disable power stage
        PowerTrain_quickstop();
        canOpenNode_resetInitialization(NodeId);
        break;
    }
    case NODEERROR_SDO:
    {
        // TODO disable power stage
        PowerTrain_quickstop();
        canOpenNode_resetInitialization(NodeId);
        break;
    }
    case NODEERROR_DEFAULT:
    {
        PowerTrain_quickstop();
        canOpenNode_resetInitialization(NodeId);
        break;
    }
    case NODEERROR_NONE:
        break;
    }
    active=0;
}

/* initialize the motors
 * registers the devices in canOpenNode
 * motor is ready for usage after that */
void PowerTrain_initMotors()
{
    /* set parameters to node structures */
#if TEST_MODE
    powerTrain_MotorTest.NodeBasic.NodeId =NODE_ID_MOTOR_TEST;
    powerTrain_MotorTest.NodeBasic.NodeType = NODETYPE_BGE6010;
    powerTrain_MotorTest.NodeBasic.NodeState = NODESTATE_NONE;
    powerTrain_MotorTest.NodeBasic.NodeError = NODEERROR_NONE;
    powerTrain_MotorTest.Node402Mode = NODE402MODE_VELOCITY;
    powerTrain_MotorTest.NodeBasic.hcan=&hcan1;
    powerTrain_MotorTest.NodeBasic.mhcanHandle=&mhcan1Handle;
    powerTrain_MotorTest.NodeBasic.errorCallback=&PowerTrain_errorHandler;
    powerTrain_MotorTest.ValuesVelocityMode.accDeltaSpeed=BGE6010_BG65S_ACC_DELTA_SPEED;
    powerTrain_MotorTest.ValuesVelocityMode.accDeltaTime=BGE6010_BG65S_ACC_DELTA_TIME;
    powerTrain_MotorTest.ValuesVelocityMode.decDeltaSpeed=BGE6010_BG65S_DEC_DELTA_SPEED;
    powerTrain_MotorTest.ValuesVelocityMode.decDeltaTime=BGE6010_BG65S_DEC_DELTA_TIME;
    powerTrain_MotorTest.ValuesVelocityMode.quickStopDeltaSpeed=BGE6010_BG65S_QS_DELTA_SPEED;
    powerTrain_MotorTest.ValuesVelocityMode.quickStopDeltaTime=BGE6010_BG65S_QS_DELTA_TIME;
    powerTrain_MotorTest.ValuesVelocityMode.scalingNumerator=BGE6010_BG65S_SCALING_NUMERATOR;
    powerTrain_MotorTest.ValuesVelocityMode.scalingDenumerator=BGE6010_BG65S_SCALING_DENUMERATOR;
    powerTrain_MotorTest.ValuesVelocityMode.velocityMaxAmount=BGE6010_BG65S_VELOCITY_MAX;
    powerTrain_MotorTest.ValuesVelocityMode.velocityMinAmount=BGE6010_BG65S_VELOCITY_MIN;
    powerTrain_MotorTest.NodeBasic.errorCallback=&PowerTrain_errorHandler;
    if(canOpenNode_addInstance(&powerTrain_MotorTest.NodeBasic, BGE6010_BG65S_initSDOs,sizeof(BGE6010_BG65S_initSDOs)/sizeof(canOpen_typeDef_SDOprimitive))!=APPLICATIONERROR_NONE)
    {
        // TODO something really bad
        Error_Handler();
    }
#else
    powerTrain_MotorLeft.NodeBasic.NodeId =NODE_ID_MOTOR_LEFT;
    powerTrain_MotorLeft.NodeBasic.NodeType = NODETYPE_BGE6010;
    powerTrain_MotorLeft.NodeBasic.NodeState = NODESTATE_NONE;
    powerTrain_MotorLeft.NodeBasic.NodeError = NODEERROR_NONE;
    powerTrain_MotorLeft.Node402Mode = NODE402MODE_VELOCITY;
    powerTrain_MotorLeft.NodeBasic.hcan=&hcan1;
    powerTrain_MotorLeft.NodeBasic.mhcanHandle=&mhcan1Handle;
    powerTrain_MotorLeft.NodeBasic.errorCallback=&PowerTrain_errorHandler;
    powerTrain_MotorLeft.ValuesVelocityMode.accDeltaSpeed=BGE6010_BG65S_ACC_DELTA_SPEED;
    powerTrain_MotorLeft.ValuesVelocityMode.accDeltaTime=BGE6010_BG65S_ACC_DELTA_TIME;
    powerTrain_MotorLeft.ValuesVelocityMode.decDeltaSpeed=BGE6010_BG65S_DEC_DELTA_SPEED;
    powerTrain_MotorLeft.ValuesVelocityMode.decDeltaTime=BGE6010_BG65S_DEC_DELTA_TIME;
    powerTrain_MotorLeft.ValuesVelocityMode.quickStopDeltaSpeed=BGE6010_BG65S_QS_DELTA_SPEED;
    powerTrain_MotorLeft.ValuesVelocityMode.quickStopDeltaTime=BGE6010_BG65S_QS_DELTA_TIME;
    powerTrain_MotorLeft.ValuesVelocityMode.scalingNumerator=BGE6010_BG65S_SCALING_NUMERATOR;
    powerTrain_MotorLeft.ValuesVelocityMode.scalingDenumerator=BGE6010_BG65S_SCALING_DENUMERATOR;
    powerTrain_MotorLeft.ValuesVelocityMode.velocityMaxAmount=BGE6010_BG65S_VELOCITY_MAX;
    powerTrain_MotorLeft.ValuesVelocityMode.velocityMinAmount=BGE6010_BG65S_VELOCITY_MIN;

    powerTrain_MotorRight.NodeBasic.NodeId =NODE_ID_MOTOR_RIGHT;
    powerTrain_MotorRight.NodeBasic.NodeType = NODETYPE_BGE6010;
    powerTrain_MotorRight.NodeBasic.NodeState = NODESTATE_NONE;
    powerTrain_MotorRight.NodeBasic.NodeError = NODEERROR_NONE;
    powerTrain_MotorRight.Node402Mode = NODE402MODE_VELOCITY;
    powerTrain_MotorRight.NodeBasic.hcan=&hcan1;
    powerTrain_MotorRight.NodeBasic.mhcanHandle=&mhcan1Handle;
    powerTrain_MotorRight.NodeBasic.errorCallback=&PowerTrain_errorHandler;
    powerTrain_MotorRight.ValuesVelocityMode.accDeltaSpeed=BGE6010_BG65S_ACC_DELTA_SPEED;
    powerTrain_MotorRight.ValuesVelocityMode.accDeltaTime=BGE6010_BG65S_ACC_DELTA_TIME;
    powerTrain_MotorRight.ValuesVelocityMode.decDeltaSpeed=BGE6010_BG65S_DEC_DELTA_SPEED;
    powerTrain_MotorRight.ValuesVelocityMode.decDeltaTime=BGE6010_BG65S_DEC_DELTA_TIME;
    powerTrain_MotorRight.ValuesVelocityMode.quickStopDeltaSpeed=BGE6010_BG65S_QS_DELTA_SPEED;
    powerTrain_MotorRight.ValuesVelocityMode.quickStopDeltaTime=BGE6010_BG65S_QS_DELTA_TIME;
    powerTrain_MotorRight.ValuesVelocityMode.scalingNumerator=BGE6010_BG65S_SCALING_NUMERATOR;
    powerTrain_MotorRight.ValuesVelocityMode.scalingDenumerator=BGE6010_BG65S_SCALING_DENUMERATOR;
    powerTrain_MotorRight.ValuesVelocityMode.velocityMaxAmount=BGE6010_BG65S_VELOCITY_MAX;
    powerTrain_MotorRight.ValuesVelocityMode.velocityMinAmount=BGE6010_BG65S_VELOCITY_MIN;

    if(canOpenNode_addInstance(&powerTrain_MotorLeft.NodeBasic,BGE6010_BG65S_initSDOs,sizeof(BGE6010_BG65S_initSDOs)/sizeof(canOpen_typeDef_SDOprimitive))!=APPLICATIONERROR_NONE)
    {
        // TODO something really bad
        Error_Handler();
    }
    if(canOpenNode402_sendInitVelocityValues(&powerTrain_MotorLeft)!=APPLICATIONERROR_NONE)
    {
        // TODO something really bad
        Error_Handler();
    }

    if(canOpenNode_addInstance(&powerTrain_MotorRight.NodeBasic, BGE6010_BG65S_initSDOs,sizeof(BGE6010_BG65S_initSDOs)/sizeof(canOpen_typeDef_SDOprimitive))!=APPLICATIONERROR_NONE)
    {
        // TODO something really bad
        Error_Handler();
    }
    if(canOpenNode402_sendInitVelocityValues(&powerTrain_MotorRight)!=APPLICATIONERROR_NONE)
    {
        // TODO something really bad
        Error_Handler();
    }
#endif
}

/* enable final stage
 * breaks will open
 * drive with set target velocity */
void PowerTrain_drive()
{
    /* first reach pre-operational state by going through state machine */
#if TEST_MODE
    globalData_typeDef_powerTrain_IO_GS powerTrainTXdata = globalDataStructures_getPowerTrain_IO_GS();
    PowerTrain_writeVelocity(powerTrainTXdata.targetVelocityLeft,powerTrainTXdata.targetVelocityRight);
    if(canOpenNode402_PowerStateMachine(NODE_ID_MOTOR_TEST,CANOPEN402_STATE_SWITCHED_ON)!=APPLICATIONERROR_NONE)
        Error_Handler();
    if(canOpenNode402_PowerStateMachine(NODE_ID_MOTOR_TEST,CANOPEN402_STATE_OPERATION_ENABLED)!=APPLICATIONERROR_NONE)
        Error_Handler();
#else
    globalData_typeDef_powerTrain_IO_GS powerTrainTXdata = globalDataStructures_getPowerTrain_IO_GS();
    PowerTrain_writeVelocity(powerTrainTXdata.targetVelocityLeft,powerTrainTXdata.targetVelocityRight);
    if(canOpenNode402_PowerStateMachine(NODE_ID_MOTOR_LEFT,CANOPEN402_STATE_SWITCHED_ON)!=APPLICATIONERROR_NONE)
        Error_Handler();
    if(canOpenNode402_PowerStateMachine(NODE_ID_MOTOR_RIGHT,CANOPEN402_STATE_SWITCHED_ON)!=APPLICATIONERROR_NONE)
        Error_Handler();
    /* finally enable operational mode */
    if(canOpenNode402_PowerStateMachine(NODE_ID_MOTOR_LEFT,CANOPEN402_STATE_OPERATION_ENABLED)!=APPLICATIONERROR_NONE)
        Error_Handler();
    if(canOpenNode402_PowerStateMachine(NODE_ID_MOTOR_RIGHT,CANOPEN402_STATE_OPERATION_ENABLED)!=APPLICATIONERROR_NONE)
        Error_Handler();

    PowerTrain_writeGlobalData();
#endif
}

/* stop the engine with stop bit
 * does not deactivate final stage
 * breaks will stay open */
void PowerTrain_stop()
{
#if TEST_MODE
    canOpenNode402_Stop(NODE_ID_MOTOR_TEST);
#else
    canOpenNode402_Stop(NODE_ID_MOTOR_LEFT);
    canOpenNode402_Stop(NODE_ID_MOTOR_RIGHT);
#endif
    PowerTrain_writeGlobalData();
}

/* stop motors and deactivate final stage
 * breaks will close when ready */
void PowerTrain_shutdown()
{
    int32_t velocity_left;
    int32_t velocity_right;
#if TEST_MODE
    if(powerTrain_MotorTest.NodeBasic.NodeState<NODESTATE_READY)
        return;
    PowerTrain_readVelocity(&velocity_left,&velocity_right);
    if(velocity_left!=0)
        PowerTrain_stop();
    do
    {
        PowerTrain_readVelocity(&velocity_left,&velocity_right);
    }while(velocity_left!=0);
#else

    if(powerTrain_MotorLeft.NodeBasic.NodeState<NODESTATE_READY)
        return;
    if(powerTrain_MotorRight.NodeBasic.NodeState<NODESTATE_READY)
        return;
    /* stop motors and wait for standstill */
    PowerTrain_readVelocity(&velocity_left,&velocity_right);
    if((velocity_left!=0) || (velocity_right!=0))
        PowerTrain_stop();
    do
    {
        PowerTrain_readVelocity(&velocity_left,&velocity_right);
    }while((velocity_left!=0) || (velocity_right!=0));
#endif
#if TEST_MODE
    canOpenNode402_PowerStateMachine(NODE_ID_MOTOR_TEST,CANOPEN402_STATE_SWITCH_ON_DISABLED);
#else
    /* disable final stage (almost) simultaneously */
    canOpenNode402_PowerStateMachine(NODE_ID_MOTOR_LEFT,CANOPEN402_STATE_SWITCH_ON_DISABLED);
    canOpenNode402_PowerStateMachine(NODE_ID_MOTOR_RIGHT,CANOPEN402_STATE_SWITCH_ON_DISABLED);
#endif
    PowerTrain_writeGlobalData();
}

/* stop the engine with quickstop ramp
 * disables final stage
 * called in case of error*/
void PowerTrain_quickstop()
{
#if TEST_MODE
    canOpenNode402_quickStop(NODE_ID_MOTOR_TEST);
#else
    canOpenNode402_quickStop(NODE_ID_MOTOR_LEFT);
    canOpenNode402_quickStop(NODE_ID_MOTOR_RIGHT);
#endif
    PowerTrain_writeGlobalData();
}

/* return actual state of node
 * call readStateWord before */
canOpenNode402_enumTypeDef_States PowerTrain_getNode402State(uint8_t NodeId)
{
#if TEST_MODE
    if(NodeId==NODE_ID_MOTOR_TEST)
    {
        return powerTrain_MotorTest.Node402State;
    }
#else
    if(NodeId==NODE_ID_MOTOR_LEFT)
    {
        return powerTrain_MotorLeft.Node402State;
    }
    if(NodeId==NODE_ID_MOTOR_RIGHT)
    {
        return powerTrain_MotorRight.Node402State;
    }
#endif
    return CANOPEN402_STATE_NOT_READY_TO_SWITCH_ON;
}

/* read velocities from register by sending SDO */
void PowerTrain_readVelocity(int32_t* velocity_left, int32_t* velocity_right)
{
//    size_t length;
//#if TEST_MODE
//    if(canOpenNode_SdoRd(NODE_ID_MOTOR_TEST,CANOPEN_IDX_VELOCITY_MEAS_RESULT,CANOPEN_SUBIDX_VELOCITY_MEAS_RESULT,velocity_left,&length)!=APPLICATIONERROR_NONE)
//        *velocity_left = 0x7FFFFFFF;
//    *velocity_right=0;
//    powerTrain_MotorTest.ValuesVelocityMode.velocity_actual=*velocity_left;
//#else
//    //if(canOpenNode_SdoRd(NODE_ID_MOTOR_LEFT,CANOPEN_IDX_VELOCITY_MEAS_RESULT,CANOPEN_SUBIDX_VELOCITY_MEAS_RESULT,velocity_left,&length)!=APPLICATIONERROR_NONE)
//    //   *velocity_left = 0x7FFFFFFF;
//        //if(canOpenNode_SdoRd(NODE_ID_MOTOR_RIGHT,CANOPEN_IDX_VELOCITY_MEAS_RESULT,CANOPEN_SUBIDX_VELOCITY_MEAS_RESULT,velocity_right,&length)!=APPLICATIONERROR_NONE)
//    //    *velocity_right = 0x7FFFFFFF;
//    powerTrain_MotorLeft.ValuesVelocityMode.velocity_actual=*velocity_left;
//    powerTrain_MotorRight.ValuesVelocityMode.velocity_actual=*velocity_right;
//#endif
}

/* read state word and save state to node struct */
canOpenNode402_enumTypeDef_States PowerTrain_readStateWord(uint8_t NodeId)
{
    return canOpenNode402_readStateWord(NodeId);
}

canOpenNode_enumTypeDef_ApplicationError PowerTrain_readControlWord(uint8_t NodeId)
{
    return canOpenNode402_readControlWord(NodeId);
}

/* returns state of stop bit
 * but does not read physical from node */
uint8_t PowerTrain_getStopActive(uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
    Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
    if(Node==NULL)
        return 0xFF;
    return Node->ValuesVelocityMode.stopActive;
}
