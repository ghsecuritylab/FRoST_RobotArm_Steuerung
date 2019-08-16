/*
 * canOpenNode402.h
 *
 *  Created on: 13.05.2019
 *      Author: CM
 */

#ifndef CANOPENNODE402_H_
#define CANOPENNODE402_H_

#include "canOpenNode.h"

/* canOpenNode402
 * node type for CiA 402
 * drives and motion control
 */


/* possible modes of operation
 * parameter 0x6060.00 */
typedef enum
{
     NODE402MODE_MODE_OFF                       = 0x00
    ,NODE402MODE_PROFILE_POSITION               = 0x01
    ,NODE402MODE_VELOCITY                       = 0x02
    ,NODE402MODE_PROFILE_VELOCITY               = 0x03
    ,NODE402MODE_PROFILE_TORQUE                 = 0x04

    ,NODE402MODE_HOMING                         = 0x06
    ,NODE402MODE_INTERPOLATED_POSITION          = 0x07
    ,NODE402MODE_CYCLIC_SYNCHRONOUS_POSITION    = 0x08
    ,NODE402MODE_OFF                            = 0xFF  // -1
}canOpenNode_enumTypeDef_Node402Mode;

/* parameters and values for op mode velocity */
typedef struct
{
    int32_t  velocity_demand;
    int32_t  velocity_actual;
    uint32_t velocityMinAmount;                     // default 0
    uint32_t velocityMaxAmount;                     // default 32767
    uint32_t accDeltaSpeed;                         // default 1000
    uint16_t accDeltaTime;                          // default 1
    uint32_t decDeltaSpeed;                         // default 1000
    uint16_t decDeltaTime;                          // default 1
    uint32_t quickStopDeltaSpeed;                   // default 1000
    uint32_t quickStopDeltaTime;                    // default 1
    int32_t  scalingNumerator;                      // default 1
    int32_t  scalingDenumerator;                    // default 1
    uint8_t  stopActive;                            // default 0
}canOpenNode_typeDef_ValuesVelocityMode;

/* node state, read from state word 0x6041.00 */
typedef enum
{
     CANOPEN402_STATE_NOT_READY_TO_SWITCH_ON
    ,CANOPEN402_STATE_SWITCH_ON_DISABLED
    ,CANOPEN402_STATE_READY_TO_SWITCH_ON
    ,CANOPEN402_STATE_SWITCHED_ON
    ,CANOPEN402_STATE_OPERATION_ENABLED
    ,CANOPEN402_STATE_QUICK_STOP_ACTIVE
    ,CANOPEN402_STATE_FAULT_REACTION_ACTIVE
    ,CANOPEN402_STATE_FAULT
}canOpenNode402_enumTypeDef_States;

typedef struct
{
    /* this must at the first position inside the structure!!!!
     * basic node, will be registered at canOpenNode */
    canOpenNode_typeDef_Node NodeBasic;
    /* mode of operation */
    canOpenNode_enumTypeDef_Node402Mode Node402Mode;
    /* actual state */
    canOpenNode402_enumTypeDef_States Node402State;
    /* information corresponding to the mode of operation */
    union
    {
        canOpenNode_typeDef_ValuesVelocityMode ValuesVelocityMode;
        // TODO add more mode parameters
    };
}canOpenNode_typeDef_Node402;


/* indices for communication objects */
#define CANOPEN_IDX_CONTROL_WORD        (0x6040)    // index of control word
#define CANOPEN_IDX_STATUS_WORD         (0x6041)    // index of status word
#define CANOPEN_IDX_OP_MODE_SET         (0x6060)    // index of op mode set
#define CANOPEN_IDX_OP_MODE_READ        (0x6061)    // index of op mode read
#define CANOPEN_IDX_VELOCITY            (0x6046)    // index of velocity register
#define CANOPEN_IDX_ACCELERATION        (0x6048)    // index of acceleration register
#define CANOPEN_IDX_DECELERATION        (0x6049)    // index of deceleration register
#define CANOPEN_IDX_QUICKSTOP_RAMP      (0x604A)    // index of quickstop ramp
#define CANOPEN_IDX_SCALING             (0x604C)    // index of scaling register

/* subindices for communication objects */
#define CANOPEN_SUBIDX_VELOCITY_MIN     (1)         // subindex of minimum velocity
#define CANOPEN_SUBIDX_VELOCITY_MAX     (2)         // subindex of maximum velocity
#define CANOPEN_SUBIDX_ACC_DELTA_SPEED  (1)         // ...
#define CANOPEN_SUBIDX_ACC_DELTA_TIME   (2)
#define CANOPEN_SUBIDX_DEC_DELTA_SPEED  (1)
#define CANOPEN_SUBIDX_DEC_DELTA_TIME   (2)
#define CANOPEN_SUBIDX_QS_DELTA_SPEED   (1)
#define CANOPEN_SUBIDX_QS_DELTA_TIME    (2)
#define CANOPEN_SUBIDX_SCALING_NUMERATOR    (1)
#define CANOPEN_SUBIDX_SCALING_DENUMERATOR  (2)

/* data length for communication objects */
#define CANOPEN_LENGTH_VELOCITY_MIN         (4)
#define CANOPEN_LENGTH_VELOCITY_MAX         (4)
#define CANOPEN_LENGTH_ACC_DELTA_SPEED      (4)
#define CANOPEN_LENGTH_ACC_DELTA_TIME       (2)
#define CANOPEN_LENGTH_DEC_DELTA_SPEED      (4)
#define CANOPEN_LENGTH_DEC_DELTA_TIME       (2)
#define CANOPEN_LENGTH_QS_DELTA_SPEED       (4)
#define CANOPEN_LENGTH_QS_DELTA_TIME        (2)
#define CANOPEN_LENGTH_SCALING_NUMERATOR    (4)
#define CANOPEN_LENGTH_SCALING_DENUMERATOR  (4)

/* parameters for interpreting state word content */
#define CANOPENNODE402_STATE_WORD_NOT_READY_TO_SWITCH_ON_MASK   (0x4F)
#define CANOPENNODE402_STATE_WORD_NOT_READY_TO_SWITCH_ON        (0x00)
#define CANOPENNODE402_STATE_WORD_SWITCH_ON_DISABLED_MASK       (0x4F)
#define CANOPENNODE402_STATE_WORD_SWITCH_ON_DISABLED            (0x40)
#define CANOPENNODE402_STATE_WORD_READY_TO_SWITCH_ON_MASK       (0x6F)
#define CANOPENNODE402_STATE_WORD_READY_TO_SWITCH_ON            (0x21)
#define CANOPENNODE402_STATE_WORD_SWITCHED_ON_MASK              (0x6F)
#define CANOPENNODE402_STATE_WORD_SWITCHED_ON                   (0x23)
#define CANOPENNODE402_STATE_WORD_OPERATION_ENABLED_MASK        (0x6F)
#define CANOPENNODE402_STATE_WORD_OPERATION_ENABLED             (0x27)
#define CANOPENNODE402_STATE_WORD_QUICK_STOP_ACTIVE_MASK        (0x6F)
#define CANOPENNODE402_STATE_WORD_QUICK_STOP_ACTIVE             (0x07)
#define CANOPENNODE402_STATE_WORD_FAULT_REACTION_ACTIVE_MASK    (0x4F)
#define CANOPENNODE402_STATE_WORD_FAULT_REACTION_ACTIVE         (0x0F)
#define CANOPENNODE402_STATE_WORD_FAULT_MASK                    (0x4F)
#define CANOPENNODE402_STATE_WORD_FAULT                         (0x08)
#define CANOPENNODE402_STATE_WORD_SWITCH_ON_DISABLED_BIT        (1<<6)
#define CANOPENNODE402_STATE_WORD_QUICK_STOP_BIT                (1<<5)
#define CANOPENNODE402_STATE_WORD_FAULT_BIT                     (1<<3)
#define CANOPENNODE402_STATE_WORD_OPERATION_ENABLED_BIT         (1<<2)
#define CANOPENNODE402_STATE_WORD_SWITCHED_ON_BIT               (1<<1)
#define CANOPENNODE402_STATE_WORD_READY_TO_SWITCH_ON_BIT        (1<<0)

typedef struct
{
    union
    {
        uint16_t data;
        struct
        {
            uint16_t readyToSwitchOn    :1;     // 0
            uint16_t switchedOn         :1;     // 1
            uint16_t operationEnabled   :1;     // 2
            uint16_t fault              :1;     // 3
            uint16_t voltageEnabled     :1;     // 4
            uint16_t quickStop          :1;     // 5
            uint16_t switchOnDisabled   :1;     // 6
            uint16_t warning            :1;     // 7
            uint16_t unused1            :1;     // 8
            uint16_t remote             :1;     // 9
            uint16_t targetReached      :1;     // 10
            uint16_t internalLimitActive:1;     // 11
            uint16_t OpModeSpecific1    :1;     // 12
            uint16_t OpModeSpecific2    :1;     // 13
            uint16_t unused2            :1;     // 14
            uint16_t unused3            :1;     // 15
        };
    };
}canOpenNode_typeDef_StateWord;

/* parameters for interpreting control word content */
typedef struct
{
    uint16_t switchOn       :1;     // 0
    uint16_t enableVoltage  :1;     // 1
    uint16_t quickstop      :1;     // 2
    uint16_t enableOperation:1;     // 3
    uint16_t OpModeSpecific1:1;     // 4
    uint16_t OpModeSpecific2:1;     // 5
    uint16_t OpModeSpecific3:1;     // 6
    uint16_t faultReset     :1;     // 7
    uint16_t halt           :1;     // 8
    uint16_t OpModeSpecific4:1;     // 9
    uint16_t unused1        :1;     // 10
    uint16_t unused2        :1;     // 11
    uint16_t unused3        :1;     // 12
    uint16_t unused4        :1;     // 13
    uint16_t unused5        :1;     // 14
    uint16_t unused6        :1;     // 15
}canOpenNode_typeDef_ControlWord;

/* sends set of SDOs to initialize the node with parameters in velocity struct
 * call after initializing node via canOpenNode */
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_sendInitVelocityValues(canOpenNode_typeDef_Node402* Node);
/* read state word by SDO */
canOpenNode402_enumTypeDef_States canOpenNode402_readStateWord(uint8_t NodeId);
/* read control word by SDO */
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_readControlWord(uint8_t NodeId);
/* performs standard stop */
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_Stop(uint8_t NodeId);
/* performs quickstop */
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_quickStop(uint8_t NodeId);
/* power state machine */
canOpenNode_enumTypeDef_ApplicationError canOpenNode402_PowerStateMachine(uint8_t NodeId, canOpenNode402_enumTypeDef_States targetState);

#endif /* CANOPENNODE402_H_ */
