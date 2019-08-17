/*
 * BGE6010_BG65S.h
 *
 *  Created on: 13.05.2019
 *      Author: CM
 */

#ifndef BGE6010_BG65S_H_
#define BGE6010_BG65S_H_

#include "canOpen.h"


#define GEARING_NUMERATOR  (295)        // factor of the built-in gear
#define GEARING_DENUMERATOR (10)
#define GEAR_DIAMETER   (140)           // diameter of the powered wheel in mm
#define RPM_NOMINAL     (3240)          // nominal speed in rpm
#define RPM_MAX         (4580)          // maximal speed in rpm
#define CONST_PI_NUM    (314159)        // nominator of Pi
#define CONST_PI_DENUM  (100000)        // denominator of Pi
#define MM_PER_M        (1000)
#define SECONDS_PER_MIN (60)

#define BGE6010_BG65S_VELOCITY_MAX_01_M_PER_S ( (uint8_t) ( (uint64_t)RPM_MAX*GEAR_DIAMETER*CONST_PI_NUM*10/CONST_PI_DENUM*GEARING_DENUMERATOR/GEARING_NUMERATOR/MM_PER_M/SECONDS_PER_MIN) )
#define BGE6010_BG65S_ACC_DELTA_SPEED (1200)  // U/min acceleration ramp
#define BGE6010_BG65S_ACC_DELTA_TIME  (1)     // s acceleration ramp
#define BGE6010_BG65S_DEC_DELTA_SPEED (3000)  // U/min deceleration ramp
#define BGE6010_BG65S_DEC_DELTA_TIME  (1)     // s deceleration ramp
#define BGE6010_BG65S_QS_DELTA_SPEED  (10000) // U/min quickstop ramp
#define BGE6010_BG65S_QS_DELTA_TIME   (1)     // s quickstop ramp
#define BGE6010_BG65S_SCALING_NUMERATOR (1)   // scaling numerator
#define BGE6010_BG65S_SCALING_DENUMERATOR (1) // scaling denumerator
#define BGE6010_BG65S_VELOCITY_MIN    (0)     // U/min
#define BGE6010_BG65S_VELOCITY_MAX    (4580)  // U/min

/* init SDOs */
#define INIT_SDO_COUNT (35) // number of init SDOs
extern canOpen_typeDef_SDOprimitive BGE6010_BG65S_initSDOs[INIT_SDO_COUNT];

/* indices of DSA parameters*/

typedef enum
{
    CANOPEN_IDX_VELOCITY_MEAS_RESULT = 0x3A04   // velocity measurement result
}canOpenBGE6010_enumTypeDef_IDX_DSA;


typedef enum
{
     BGE6010_DOUT0
    ,BGE6010_DOUT1
}canOpenBGE6010_enumTypeDef_DOUT;

/* definition of subindices */
#define CANOPEN_SUBIDX_VELOCITY_MEAS_RESULT (0x01) // velocity measurement result

#endif /* BGE6010_BG65S_H_ */
