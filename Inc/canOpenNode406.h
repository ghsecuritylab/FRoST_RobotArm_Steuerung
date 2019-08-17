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


/* indices for communication objects */
#define CANOPEN406_IDX_POSITION        			(0x6383)    	// index of diagnose
#define CANOPEN406_IDX_DIAGNOSE        			(0x2117)    	// index of diagnose
#define CANOPEN406_IDX_VELOCITY        			(0x2118)    	// index of velocity
#define CANOPEN406_IDX_SPEEDSAMPLING        	(0x4001)    	// index of speed sampling interval [ms]
#define CANOPEN406_IDX_OPERATINGSTATUS        	(0x63C0)    	// index of operating status
#define CANOPEN406_IDX_MESURINGSETTING        	(0x6384)    	// index of measuring step setting
#define CANOPEN406_IDX_ALARM			      	(0x6503)    	// index of operating parameter
#define CANOPEN406_IDX_WARNING 			     	(0x6505)    	// index of operating parameter
#define CANOPEN406_IDX_ERROR      				(0x1001)    	// index of operating parameter
#define CANOPEN406_IDX_BAUDRATE      			(0x2100)    	// index of baudrate
#define CANOPEN406_IDX_NODEID      				(0x2101)    	// index of node ID
#define CANOPEN406_IDX_SAVE      				(0x1010)    	// index of save


/* subindices for communication objects */
#define CANOPEN406_SUBIDX_SAVE_ALL     			(1)         	// subindex of save all parameter
#define CANOPEN406_SUBIDX_SAVE_COMMUNICATION    (2)         	// subindex of save communication parameter
#define CANOPEN406_SUBIDX_SAVE_APPLICATION     	(3)         	// subindex of save application parameter
#define CANOPEN406_SUBIDX_SAVE_MANUFACTUR 	    (4)         	// subindex of save manucfactor parameter

/* data length for communication objects */
//#define CANOPEN_LENGTH_EMERGENCYMESSAGE         (8)



// supported NMT commands: 1014h
typedef enum
{
	CANOPEN406_NMT_START						= 0x01
	,CANOPEN406_NMT_PREOPERATIONAL				= 0x80
	,CANOPEN406_NMT_STOP						= 0x02
	,CANOPEN406_NMT_RESET						= 0x81
	,CANOPEN406_NMT_COMMUNICATION_RESET			= 0x82
}canOpenNode406_enumTypeDef_NMTCMD;


typedef enum
{
	CANOPEN406_ERROR_RESET						= 0x0000
	,CANOPEN406_ERROR_TEMPERATUR				= 0x4200
	,CANOPEN406_ERROR_EEPROM					= 0x5100
	,CANOPEN406_ERROR_SOFTWARE 					= 0x6100
	,CANOPEN406_ERROR_COMMUNICATION 			= 0x8100
	,CANOPEN406_ERROR_CAN_RX_OVERFLOW 			= 0x8110
	,CANOPEN406_ERROR_CAN_PASSIV_MODE 			= 0x8120
	,CANOPEN406_ERROR_LIFEGUARD_HEARTBEAT 		= 0x8130
	,CANOPEN406_ERROR_BATTERYLOW 				= 0xFF00
	,CANOPEN406_ERROR_BATTERYEMPTY 				= 0xFF01
	,CANOPEN406_ERROR_WEAKMAGNETICFIELD 		= 0xFF02
	,CANOPEN406_ERROR_INTERNALCOMMUNICATION		= 0xFF04
	,CANOPEN406_ERROR_SPEED 					= 0xFF05
	,CANOPEN406_ERROR_SINGLETURN_OUT_RANGE 		= 0xFF06
	,CANOPEN406_ERROR_OUT_OF_SYNC				= 0xFF07
	,CANOPEN406_ERROR_MANUFACTRUE  				= 0xFF08

}canOpenNode406_enumTypeDef_ERRORCODE;


typedef struct
{
    uint16_t FreqExc       						:1;     			// 0
    uint16_t WDT  								:1;     			// 1
    uint16_t OpTimeLim      					:1;     			// 2
    uint16_t BattLow							:1;     			// 3
    uint16_t MountErr							:1;     			// 4
    uint16_t MTSys								:1;     			// 5
    uint16_t STSys								:1;     			// 6
    uint16_t MTSuper     						:1;    				// 7
    uint16_t reserved1      					:1;     			// 8
    uint16_t reserved2							:1;     			// 9
    uint16_t reserved3      					:1;     			// 10
    uint16_t reserved4      					:1;     			// 11
    uint16_t CommErr        					:1;     			// 12
    uint16_t TempErr        					:1;     			// 13
    uint16_t PosErr      						:1;     			// 14
    uint16_t BattEmpt       					:1;     			// 15
}canOpenNode_typeDef_Diagnose;


// parameters and values for op mode velocity
typedef struct
{
	int32_t	 position_actual;
    int32_t  velocity_actual;
}canOpenNode_typeDef_ValuesEncoder;


typedef struct
{
    /* this must at the first position inside the structure!!!!
     * basic node, will be registered at canOpenNode */
    canOpenNode_typeDef_Node NodeBasic;
    /* mode of operation */
    //canOpenNode_enumTypeDef_Node406Mode Node406Mode;
    /* actual state */
    //canOpenNode406_enumTypeDef_States Node406State;
    /* information corresponding to the mode of operation */
    union
    {
        canOpenNode_typeDef_ValuesEncoder ValuesEncoder;
        // TODO add more mode parameters
    };
}canOpenNode_typeDef_Node406;




/* sends set of SDOs to initialize the node with parameters in velocity struct
 * call after initializing node via canOpenNode */
canOpenNode_enumTypeDef_ApplicationError canOpenNode406_sendInitValues(canOpenNode_typeDef_Node406* Node);
/* read control word by SDO */
canOpenNode_enumTypeDef_ApplicationError canOpenNode406_readDiagnose(uint8_t NodeId);


#endif /* CANOPENNODE406_H_ */
