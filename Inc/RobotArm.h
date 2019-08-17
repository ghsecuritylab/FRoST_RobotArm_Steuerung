/*
 * RobotArm.h
 *
 *  Created on: 23.07.2019
 *      Author: FRoST
 *      last change SE 23.07.
 */

#ifndef ROBOTARM_H_
#define ROBOTARM_H_

#include "main.h"
#include "canOpenNode402.h"
#include "canOpenNode406.h"
#include "globalDataStructures_CM_MTR_FRoST.h"


//typedef enum
//{
//	ROBOTARM_MODE_DISABLE = 1
//   ,ROBOTARM_MODE_AXES
//   ,ROBOTARM_MODE_IK
//   ,ROBOTARM_MODE_TEACHED_POSITION
//}RobotArm_enumTypeDef_Mode;


/* deactivates regular motors and activates test motor
 * only for stationary test purposes! */
	/* FIXME: set mode - test on/off */
#define TEST_MODE_ARM 1


#define NODE_ID_MOTOR_TEST  0x25
#define NODE_ID_MOTOR_1		0x21
#define NODE_ID_MOTOR_2		0x22
#define NODE_ID_MOTOR_3		0x23
#define NODE_ID_MOTOR_4		0x24
#define NODE_ID_MOTOR_5		0x25
#define NODE_ID_MOTOR_6		0x26
#define NODE_ID_MOTOR_7		0x27
#define NODE_ID_ENCODER_1	0x31
#define NODE_ID_ENCODER_2	0x32
#define NODE_ID_ENCODER_3	0x33
#define NODE_ID_ENCODER_4	0x34
#define NODE_ID_ENCODER_5	0x35
#define NODE_ID_ROBOTARM_MIN  0x21
#define NODE_ID_ROBOTARM_MAX  0x35

#define ENDEFFECTOR_MAX_VELOCITY	20
#define AXIS_5_MAX_VELOCITY			5

/* update velocity and state according to actual global structure values
 * reads global structure
 * calls ErrorHandler if failed to get global structure
 */
void RobotArm_updateState(void);
void RobotArm_setPSM(uint8_t NodeID);
void RobotArm_initMotors(void);
void RobotArm_joystick2rpm(int16_t y_direction, uint16_t maxVelocity, uint8_t NodeId);
void RobotArm_setProfileVelocity(int32_t velocity_rpm, uint8_t NodeId);
void RobotArm_initLinearActuator(void);
void RobotArm_init(void);
void RobotArm_setLinearActuatorDutyCycle(uint8_t percent, globalData_enumTypeDef_robotArm2Buttons state);
void RobotArm_allMotorsHALT(void);
void RobotArm_setVelocityButtons(uint16_t velocity , globalData_enumTypeDef_robotArm2Buttons state, uint8_t NodeId);
void RobotArm_initEncoders(void);
void RobotArm_updateEncoder(uint8_t NodeId, uint16_t data);
void RobotArm_allMotors_PSM_restart(void);
void RobotArm_allMotorsQuickStop(void);
void RobotArm_setAngle(uint8_t NodeId, uint16_t actualAngle, uint16_t tagetAngle, uint16_t hysteresis, int32_t velocity);
void RobotArm_setSingleAngle(uint8_t NodeId, uint16_t actualAngle, uint16_t tagetAngle, uint16_t hysteresis, int32_t velocity);
uint8_t RobotArm_setSingleAngleD(uint8_t NodeId, double actualAngle, double tagetAngle, double hysteresis, double velocity);

//uint8_t isRobotArm_initEncodersSuccessfull(globalData_typeDef_robotArm* Check_Arm);
//uint8_t isRobotArm_initeEncoders_SetUpSucessfull(canOpenNode_typeDef_Node* Node,uint8_t Node_id);
//uint8_t isRobotArm_initMotorsSuccessful(globalData_typeDef_robotArm* Check_Arm);
//uint8_t isRobotArm_initMotors_SetUpSucessfull(canOpenNode_typeDef_Node402* Node,uint8_t Node_id);
canOpenNode_enumTypeDef_ApplicationError Arm_readControlWord(uint8_t NodeId);
canOpenNode402_enumTypeDef_States Arm_readStateWord(uint8_t NodeId);
uint32_t Arm_readVelocity(uint8_t NodeId);

void RobotArm_updateBSData(void);


typedef enum{
	Arm_State_Start
	,Arm_State_Init
	,Arm_State_SwitchOn
	,Arm_State_Operation_SetParameter
	,Arm_State_Operation_SetParameterVelocity
	,Arm_State_Operation_Motion
	,Arm_State_Operation_End
	,Arm_State_Init_Error
	,Arm_State_SwitchOn_Error
	,Arm_State_Operation_SetParameter_Error
	,Arm_State_Operation_Motion_Error
	,Arm_State_Operation_End_Error
	,Arm_State_Operation_SetParameterVelocity_Error
	,Arm_State_Error
	,Arm_State_UndefinedState_Error
}Arm_enumTypeDef_States;

typedef enum
{
	Arm_Mode_Disable
	,Arm_Mode_Axis
	,Arm_Mode_IK
	,Arm_Mode_TeachedPosition
	,Arm_Mode_Reture
}Arm_enumTypeDef_Modes;

typedef struct
{
	canOpenNode_typeDef_Node402 Motor;
	canOpenNode_typeDef_Node406 Encoder;
}globalData_typeDef_robotArm_Achse;

typedef struct
{
	// Modus
	Arm_enumTypeDef_States 						Arm_State_Soll;			// State
	Arm_enumTypeDef_States 						Arm_State_Ist;			// State
	globalData_enumTypeDef_robotArmMode 		Crrt_mode_Soll;
	globalData_enumTypeDef_robotArmMode 		Crrt_mode_Ist;

	// Test Var
	int8_t 										Do_Something;

	globalData_typeDef_robotArm_ARM_GS			robotArm_ARM_GS;
	globalData_typeDef_robotArm_GS_ARM			robotArm_GS_ARM;

	// Build
	canOpenNode_typeDef_Node402 				robotArm_Motor6;		// Tool
	globalData_typeDef_robotArm_Achse			Achse[5];

	// IK Ergebnise
	uint16_t 									Schritte[2][6];

}globalData_typeDef_robotArm;

///* set velocity of both motors
// * @param x_direction: [-100:100], value of direction transverse to direction of travel
// * @param y_direction: [-100:100], value of direction along to direction of travel
// * @param maxVelocity: [0:255], unit: m/s, factor: 1/10, maximum velocity (applied when x=0;y=100)
// */
//void RobotArm_setVelocity(int8_t x_direction, int8_t y_direction, uint8_t maxVelocity);
///* initialize the motors
// * registers the devices in canOpenNode
// * motor is ready for usage after that */
//void RobotArm_initMotors();
///* stop motors and deactivate final stage
// * breaks will close when ready */
//void RobotArm_shutdown();
///* enable final stage
// * breaks will open
// * drive with set target velocity */
//void RobotArm_drive();
///* stop the engine with stop bit
// * does not deactivate final stage
// * breaks will stay open */
//void RobotArm_stop();
///* read velocities from register by sending SDO */
//void RobotArm_readVelocity(int32_t* velocity_left, int32_t* velocity_right);
///* stop the engine with quickstop ramp
// * disables final stage
// * called in case of error*/
//void RobotArm_quickstop();
///* return actual state of node
// * call readStateWord before */
//canOpenNode402_enumTypeDef_States RobotArm_getNode402State(uint8_t NodeId);
///* read state word and save state to node struct */
//canOpenNode402_enumTypeDef_States RobotArm_readStateWord(uint8_t NodeId);
///* returns state of stop bit
// * but does not read physical from node */
//uint8_t RobotArm_getStopActive(uint8_t NodeId);

#endif /* ROBOTARM_H_ */
