/*
 * canOpenNode406.h
 *
 *  Created on: 13.05.2019
 *      Author: CM
 */

#ifndef CANOPENNODE406_H_
#define CANOPENNODE406_H_

#include "canOpenNode.h"

/* canOpenNode406
 * node type for CiA 406
 * drives and motion control
 */


/* possible modes of operation
 * parameter 0x6060.00 */
typedef enum
{
     NODE406MODE_MODE_OFF                       = 0x00
    ,NODE406MODE_PROFILE_POSITION               = 0x01
    ,NODE406MODE_VELOCITY                       = 0x02
    ,NODE406MODE_PROFILE_VELOCITY               = 0x03
    ,NODE406MODE_PROFILE_TORQUE                 = 0x04

    ,NODE406MODE_HOMING                         = 0x06
    ,NODE406MODE_INTERPOLATED_POSITION          = 0x07
    ,NODE406MODE_CYCLIC_SYNCHRONOUS_POSITION    = 0x08
    ,NODE406MODE_OFF                            = 0xFF  // -1
}canOpenNode_enumTypeDef_Node406Mode;

/* parameters and values for op mode velocity */
typedef struct
{
	int32_t	 position_actual;
    int32_t  velocity_actual;

}canOpenNode_typeDef_ValuesEncoder;

/* node state, read from state word 0x6041.00 */
typedef enum
{
     CANOPEN406_STATE_NOT_READY_TO_SWITCH_ON
    ,CANOPEN406_STATE_SWITCH_ON_DISABLED
    ,CANOPEN406_STATE_READY_TO_SWITCH_ON
    ,CANOPEN406_STATE_SWITCHED_ON
    ,CANOPEN406_STATE_OPERATION_ENABLED
    ,CANOPEN406_STATE_QUICK_STOP_ACTIVE
    ,CANOPEN406_STATE_FAULT_REACTION_ACTIVE
    ,CANOPEN406_STATE_FAULT
}canOpenNode406_enumTypeDef_States;

typedef struct
{
    /* this must at the first position inside the structure!!!!
     * basic node, will be registered at canOpenNode */
    canOpenNode_typeDef_Node NodeBasic;
    /* mode of operation */
    canOpenNode_enumTypeDef_Node406Mode Node406Mode;
    /* actual state */
    canOpenNode406_enumTypeDef_States Node406State;
    /* information corresponding to the mode of operation */
    union
    {
        canOpenNode_typeDef_ValuesEncoder ValuesEncoder;
        // TODO add more mode parameters
    };
}canOpenNode_typeDef_Node406;


/* indices for communication objects */
#define CANOPEN406_IDX_POSITION        		(0x6383)    	// index of diagnose
#define CANOPEN406_IDX_DIAGNOSE        		(0x2117)    	// index of diagnose
#define CANOPEN406_IDX_VELOCITY        		(0x2118)    	// index of velocity
#define CANOPEN406_IDX_SPEEDSAMPLING        	(0x4001)    	// index of speed sampling interval [ms]
#define CANOPEN406_IDX_OPERATINGSTATUS        	(0x63C0)    	// index of operating status
#define CANOPEN406_IDX_MESURINGSETTING        	(0x6384)    	// index of measuring step setting
#define CANOPEN406_IDX_OPERATINGPARAMETER      (0x6380)    	// index of operating parameter

/* subindices for communication objects */
#define CANOPEN406_SUBIDX_VELOCITY_MIN     (1)         // subindex of minimum velocity
#define CANOPEN406_SUBIDX_VELOCITY_MAX     (2)         // subindex of maximum velocity

/* data length for communication objects */
//#define CANOPEN_LENGTH_OPEN         (4)

/* parameters for interpreting error content */
#define CANOPENNODE406_STATE_WORD_NOT_READY_TO_SWITCH_ON_MASK   (0x4F)


/* parameters for interpreting warning content */
#define CANOPENNODE406_STATE_WORD_NOT_READY_TO_SWITCH_ON_MASK   (0x4F)

/*
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

// parameters for interpreting control word content
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
*/

/* sends set of SDOs to initialize the node with parameters in velocity struct
 * call after initializing node via canOpenNode */
canOpenNode_enumTypeDef_ApplicationError canOpenNode406_sendInitVelocityValues(canOpenNode_typeDef_Node406* Node);
/* read state word by SDO */
canOpenNode406_enumTypeDef_States canOpenNode406_readStateWord(uint8_t NodeId);
/* read control word by SDO */
canOpenNode_enumTypeDef_ApplicationError canOpenNode406_readControlWord(uint8_t NodeId);


#endif /* CANOPENNODE402_H_ */
