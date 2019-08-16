/*
 * PowerTrain.h
 *
 *  Created on: 01.04.2019
 *      Author: CM
 */

#ifndef POWERTRAIN_H_
#define POWERTRAIN_H_

#include "main.h"
#include "canOpenNode402.h"

/* deactivates regular motors and activates test motor
 * only for stationary test purposes! */
#define TEST_MODE 0

#if TEST_MODE
#define NODE_ID_MOTOR_TEST 0x7F
#else
#define NODE_ID_MOTOR_LEFT 0x22
#define NODE_ID_MOTOR_RIGHT 0x33
#endif

/* update velocity and state according to actual global structure values
 * reads global structure
 * calls ErrorHandler if failed to get global structure
 */
void PowerTrain_updateState();

/* set velocity of both motors
 * @param x_direction: [-100:100], value of direction transverse to direction of travel
 * @param y_direction: [-100:100], value of direction along to direction of travel
 * @param maxVelocity: [0:100], percent of max velocity
 */
void PowerTrain_setVelocity(int8_t x_direction, int8_t y_direction, uint8_t maxVelocity);
/* initialize the motors
 * registers the devices in canOpenNode
 * motor is ready for usage after that */
void PowerTrain_initMotors();
/* stop motors and deactivate final stage
 * breaks will close when ready */
void PowerTrain_shutdown();
/* enable final stage
 * breaks will open
 * drive with set target velocity */
void PowerTrain_drive();
/* stop the engine with stop bit
 * does not deactivate final stage
 * breaks will stay open */
void PowerTrain_stop();
/* read velocities from register by sending SDO */
void PowerTrain_readVelocity(int32_t* velocity_left, int32_t* velocity_right);
/* stop the engine with quickstop ramp
 * disables final stage
 * called in case of error*/
void PowerTrain_quickstop();
/* return actual state of node
 * call readStateWord before */
canOpenNode402_enumTypeDef_States PowerTrain_getNode402State(uint8_t NodeId);
/* read control word and save values to node struct */
canOpenNode_enumTypeDef_ApplicationError PowerTrain_readControlWord(uint8_t NodeId);
/* read state word and save state to node struct */
canOpenNode402_enumTypeDef_States PowerTrain_readStateWord(uint8_t NodeId);
/* returns state of stop bit
 * but does not read physical from node */
uint8_t PowerTrain_getStopActive(uint8_t NodeId);

#endif /* POWERTRAIN_H_ */
