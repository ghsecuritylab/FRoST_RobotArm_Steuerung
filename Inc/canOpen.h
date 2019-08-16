/*
 * canOpen.h
 *
 *  Created on: 13.05.2019
 *      Author: CM
 */

#ifndef CANOPEN_H_
#define CANOPEN_H_

#include "main.h"
#include "cmsis_os.h"

extern osSemaphoreId semCanOpenSDORXHandle;     // semaphore handle for SDO receive indicating
extern osMutexId mCanOpenSdoWrRdHandle;         // mutex for protecting SdoWr and SdoRd

/* mask for CAN ID inside cob-id
 * also max. node ID */
#define CAN_ID_MASK (0x7F)

/* parameters for cob-id */
typedef enum
{
     COB_ID_NMT         = 0x000
    ,COB_ID_SYNC_EMCY   = 0x080
    ,COB_ID_TX_PDO_1    = 0x180
    ,COB_ID_RX_PDO_1    = 0x200
    ,COB_ID_TX_PDO_2    = 0x280
    ,COB_ID_RX_PDO_2    = 0x300
    ,COB_ID_TX_PDO_3    = 0x380
    ,COB_ID_RX_PDO_3    = 0x400
    ,COB_ID_TX_PDO_4    = 0x480
    ,COB_ID_RX_PDO_4    = 0x500
    ,COB_ID_TX_SDO      = 0x580
    ,COB_ID_RX_SDO      = 0x600
    ,COB_ID_HEARTBEAT   = 0x700
	,COB_ID_RX_PDO_M1M2ARM	= 0x241
	,COB_ID_RX_PDO_M3M4ARM	= 0x242
	,COB_ID_RX_PDO_M5M6ARM	= 0x243
	,COB_ID_RX_PDO_M7M8ARM	= 0x244

}canOpen_enumTypeDef_COB_ID;

/* parameters for command byte */
typedef enum
{
     CMD_WR_LENGTH_1_BYTES  = 0x2F
    ,CMD_WR_LENGTH_2_BYTES  = 0x2B
    ,CMD_WR_LENGTH_3_BYTES  = 0x27
    ,CMD_WR_LENGTH_4_BYTES  = 0x23
    ,CMD_RD_LENGTH_1_BYTES  = 0x4F
    ,CMD_RD_LENGTH_2_BYTES  = 0x4B
    ,CMD_RD_LENGTH_3_BYTES  = 0x47
    ,CMD_RD_LENGTH_4_BYTES  = 0x43
    ,CMD_RD_REQ             = 0x40
    ,CMD_ACK                = 0x60
    ,CMD_ERROR              = 0x80
}canOpen_enumTypedef_CMD;

/* parameters for command byte NMT */
typedef enum
{
     CMD_OPERATIONAL			= 0x01
    ,CMD_STOP  					= 0x02
    ,CMD_PRE_OPERATIONAL  		= 0x80
    ,CMD_RESET_NODE  			= 0x81
    ,CMD_RESET_COMMUNICATION	= 0x82
}canOpen_enumTypedef_CMD_NMT;

/* prototype SDO */
typedef struct
{
    union   // union for sharing memory
    {
        uint8_t buffer[8];  // buffer for easy memory access
        struct
        {
            canOpen_enumTypedef_CMD CMD : 8;    // first command byte
            uint8_t                 IDX[2];     // index LSB first!
            uint8_t                 SUBIDX;     // subindex
            uint8_t                 data[4];    // data LSB first!
        };
    };
}canOpen_typeDef_SDO;

/* primitive SDO for easier user handling */
typedef struct
{
    uint16_t index;     // index MSB first
    uint8_t subindex;   // subindex
    uint32_t data;      // data MSB first
    size_t length;      // data length in bytes
}canOpen_typeDef_SDOprimitive;

///* prototype NMT */
//typedef struct
//{
//	 uint8_t data[2];  // buffer for easy memory access
//	        struct
//	        {
//		 	 	 canOpen_enumTypedef_CMD_NMT	CMD;	// first command byte
//		 	 	 uint8_t                     	NodeId; // last nodeId
//	        };
//}canOpen_typeDef_NMT;

typedef struct
{
//	canOpen_typeDef_NMT NMT_data;
    /* error callback function pointer
     * @param uint8_t NodeId: Node Identifier */
    void (*errorCallback)(uint8_t);
    /* pointer to can instance to use for communication */
    CAN_HandleTypeDef* hcan;
    /* pointer to mutex of this can instance */
    osMutexId* mhcanHandle;
}canOpenNode_typeDef_NMT;


/* function prototypes */


#endif /* CANOPEN_H_ */
