/*
 * CL4E.h
 *
 *  Created on: 23.07.2019
 *      Author: Sven
 */

#ifndef CL4E_H_
#define CL4E_H_

#include "canOpen.h"

/* FIXME check init SDOs */
#define INIT_SDO_COUNT (7) // number of init SDOs
extern canOpen_typeDef_SDOprimitive CL4E_initSDOs[INIT_SDO_COUNT];
extern canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor1[INIT_SDO_COUNT];
extern canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor2[INIT_SDO_COUNT];
extern canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor3[INIT_SDO_COUNT];
extern canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor4[INIT_SDO_COUNT];
extern canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor5[INIT_SDO_COUNT];
extern canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor6[INIT_SDO_COUNT];

#endif /* CL4E_H_ */
