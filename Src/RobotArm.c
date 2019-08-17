/*
 * RobotArm.c
 *
 *  Created on: 23.07.2019
 *      Author: Sven
 */

#include "RobotArm.h"
#include "CL4E.h"
#include "can.h"
#include "globalDataStructures_CM_MTR_FRoST.h"

//canOpenNode_typeDef_Node402 robotArm_MotorTest;   // use only in test mode
//canOpenNode_typeDef_Node402 robotArm_Motor1;
canOpenNode_typeDef_Node402 robotArm_Motor2;
canOpenNode_typeDef_Node402 robotArm_Motor3;
canOpenNode_typeDef_Node402 robotArm_Motor4;
canOpenNode_typeDef_Node402 robotArm_Motor5;
//canOpenNode_typeDef_Node402 robotArm_Motor6;

//canOpenNode_typeDef_Node robotArm_EncoderTest;
canOpenNode_typeDef_Node robotArm_Encoder1;
canOpenNode_typeDef_Node robotArm_Encoder2;
canOpenNode_typeDef_Node robotArm_Encoder3;
canOpenNode_typeDef_Node robotArm_Encoder4;
canOpenNode_typeDef_Node robotArm_Encoder5;

static canOpenNode_typeDef_Node402* Node;
int aktuellerSchritt = 0;
uint8_t boolschritt[5];

/* test values trajectoy */
static globalData_typeDef_robotArm_ARM_GS getAngles;
//static uint16_t setPointAngles[2][6] = {{0,0,0,0,18000,0},{0,0,0,0,27000,0}};
/* TEST VALUE'S */
//messageRobotArmRX.yValue 					=	50;
//messageRobotArmRX.maxJointVelocity 			=	10;
//messageRobotArmRX.mode 						=	ROBOTARMMODE_AXES;
//messageRobotArmRX.ArmAxis 					=	ROBOTARMAXIS_5;
//messageRobotArmRX.microLinearActorState 	= 	ROBOTARM2BUTTONS_DISABLE;
//messageRobotArmRX.microLinearActorPercent	=	50;
/* TEST VALUE'S  E N D */


/* NAME:		RobotArm_updateState
 * INPUT:		-none-
 * OUTPUT:		-none-
 * DESCRIPTION:
 * 		main function of the robot arm; called if new data from
 * 		ground station are available;
 * 		copy global data structure, first check mode
 * 		and then do some operations;
 */
globalData_typeDef_robotArm RobotArm_SetParameter(globalData_typeDef_robotArm_GS_ARM RX,globalData_typeDef_robotArm_ARM_GS TX)
{
	globalData_typeDef_robotArm result;


	return result;
}

void RobotArm_updateBSData(void)
{
	// copy global data
	static globalData_typeDef_robotArm_GS_ARM messageRobotArmRX_old;
	static globalData_typeDef_robotArm_GS_ARM messageRobotArmRX = {0};
	messageRobotArmRX = globalDataStructures_getRobotArm_GS_ARM();
	static globalData_typeDef_robotArm_ARM_GS messageRobotArmTX = {0};
	messageRobotArmTX = globalDataStructures_getRobotArm_ARM_GS();

	/* FIXME TESTMODE VAR's */
	messageRobotArmRX.mode = ROBOTARMMODE_IK;

	// Set Arm Struct
	globalData_typeDef_robotArm RobotArm;
	RobotArm = RobotArm_SetParameter(messageRobotArmRX,messageRobotArmTX);

	static uint8_t AxisNodeId;
	Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[AxisNodeId];

	if(messageRobotArmRX.dataID==GLOBALDATA_ID_ARM)
    {

		RobotArm_setLinearActuatorDutyCycle(messageRobotArmRX.microLinearActorPercent, messageRobotArmRX.microLinearActorState);
		RobotArm_setVelocityButtons(messageRobotArmRX.targetJointVelocity5,  messageRobotArmRX.Axis5State, NODE_ID_MOTOR_5);
		//      	    RobotArm_setVelocityButtons(messageRobotArmRX.velocityEndEffector,  messageRobotArmRX.endEffectorState, NODE_ID_MOTOR_6);


    	/* check if mode has changed */
    	if(messageRobotArmRX_old.mode != messageRobotArmRX.mode)
    	{
    		switch (messageRobotArmRX_old.mode)
    		{
    			case ROBOTARMMODE_DISABLE:
				case ROBOTARMMODE_AXES:
    				// set quickstop
    				Node->Node402State = CANOPEN402_STATE_QUICK_STOP_ACTIVE;
					if(canOpenNode_SdoWr(AxisNodeId,0x6040,0,0x0002,2)!=APPLICATIONERROR_NONE){};//{return;}
					//
					break;
				case ROBOTARMMODE_IK:
					break;
				case ROBOTARMMODE_TEACHED_POS:
					break;
				default:
					break;
    		}
    	}

    	/* check mode */
    	switch (messageRobotArmRX.mode)
    	{
    		case ROBOTARMMODE_DISABLE:
    			RobotArm_setLinearActuatorDutyCycle(0, ROBOTARM2BUTTONS_DISABLE);
    	    	//
				break;
    		case ROBOTARMMODE_AXES: /* control single axes */
				/*  */
    			if((messageRobotArmRX.ArmAxis+0x20 != AxisNodeId) || (AxisNodeId < NODE_ID_MOTOR_1 || AxisNodeId > NODE_ID_MOTOR_6))
    			{
    				// set quickstop
    				Node->Node402State = CANOPEN402_STATE_QUICK_STOP_ACTIVE;
    				if(canOpenNode_SdoWr(AxisNodeId,0x6040,0,0x0002,2)!=APPLICATIONERROR_NONE){};//{return;}
    				AxisNodeId = messageRobotArmRX.ArmAxis + 0x20;
    				return;
    			}
				AxisNodeId = messageRobotArmRX.ArmAxis + 0x20;
				RobotArm_joystick2rpm(messageRobotArmRX.yValue, messageRobotArmRX.maxJointVelocity, AxisNodeId);
				RobotArm_setPSM(AxisNodeId);
				break;

    		case ROBOTARMMODE_IK:
				// control hole robot arm
    			getAngles = globalDataStructures_getRobotArm_ARM_GS();

    			static uint32_t zonk3000 = 0;
				zonk3000++;
				if(zonk3000 <= 2)
				{
					canOpenNode_MasterNmtWr(0x01, 0);
				}
				if(zonk3000 < 50000){zonk3000 = 20;}

				if(getAngles.actualJointAngle2 == 0){return;}
				if(getAngles.actualJointAngle3 == 0){return;}
				if(getAngles.actualJointAngle4 == 0){return;}
//				if(getAngles.actualJointAngle5 == 0){return;}

				uint16_t setPointAxis2 = 18000;
				uint16_t setPointAxis3 = 26000;
				uint16_t setPointAxis4 = 9000;
				uint16_t setPointAxis5 = 18000;

				RobotArm_setSingleAngle(NODE_ID_MOTOR_2, getAngles.actualJointAngle2, setPointAxis2, 350, 40);
				RobotArm_setSingleAngle(NODE_ID_MOTOR_3, getAngles.actualJointAngle3, setPointAxis3, 450, 35);
				RobotArm_setSingleAngle(NODE_ID_MOTOR_4, getAngles.actualJointAngle4, setPointAxis4, 550, 20);
				RobotArm_setSingleAngle(NODE_ID_MOTOR_5, getAngles.actualJointAngle5, setPointAxis5, 1550, 1);

    	    	break;
    		case ROBOTARMMODE_TEACHED_POS:
    			break;
    		default:
    			// set error and switch off motor's
    			break;
    	}
    }
    else
    {
        Error_Handler();
    }
    messageRobotArmRX_old = messageRobotArmRX;
}

void RobotArm_updateState(void)
{
	// copy global data
	static globalData_typeDef_robotArm_GS_ARM messageRobotArmRX_old;
	static globalData_typeDef_robotArm_GS_ARM messageRobotArmRX = {0};
	messageRobotArmRX = globalDataStructures_getRobotArm_GS_ARM();
	static globalData_typeDef_robotArm_ARM_GS messageRobotArmTX = {0};
	messageRobotArmTX = globalDataStructures_getRobotArm_ARM_GS();

	/* FIXME TESTMODE VAR's */
	messageRobotArmRX.mode = ROBOTARMMODE_IK;

	// Set Arm Struct
	globalData_typeDef_robotArm RobotArm;
	RobotArm = RobotArm_SetParameter(messageRobotArmRX,messageRobotArmTX);

	static uint8_t AxisNodeId;
	Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[AxisNodeId];

	if(messageRobotArmRX.dataID==GLOBALDATA_ID_ARM)
    {

		RobotArm_setLinearActuatorDutyCycle(messageRobotArmRX.microLinearActorPercent, messageRobotArmRX.microLinearActorState);
		RobotArm_setVelocityButtons(messageRobotArmRX.targetJointVelocity5,  messageRobotArmRX.Axis5State, NODE_ID_MOTOR_5);
		//      	    RobotArm_setVelocityButtons(messageRobotArmRX.velocityEndEffector,  messageRobotArmRX.endEffectorState, NODE_ID_MOTOR_6);


    	/* check if mode has changed */
    	if(messageRobotArmRX_old.mode != messageRobotArmRX.mode)
    	{
    		switch (messageRobotArmRX_old.mode)
    		{
    			case ROBOTARMMODE_DISABLE:
				case ROBOTARMMODE_AXES:
    				// set quickstop
    				Node->Node402State = CANOPEN402_STATE_QUICK_STOP_ACTIVE;
					if(canOpenNode_SdoWr(AxisNodeId,0x6040,0,0x0002,2)!=APPLICATIONERROR_NONE){};//{return;}
					//
					break;
				case ROBOTARMMODE_IK:
					break;
				case ROBOTARMMODE_TEACHED_POS:
					break;
				default:
					break;
    		}
    	}

    	/* check mode */
    	switch (messageRobotArmRX.mode)
    	{
    		case ROBOTARMMODE_DISABLE:
    			RobotArm_setLinearActuatorDutyCycle(0, ROBOTARM2BUTTONS_DISABLE);
    	    	//
				break;
    		case ROBOTARMMODE_AXES: /* control single axes */
				/*  */
    			if((messageRobotArmRX.ArmAxis+0x20 != AxisNodeId) || (AxisNodeId < NODE_ID_MOTOR_1 || AxisNodeId > NODE_ID_MOTOR_6))
    			{
    				// set quickstop
    				Node->Node402State = CANOPEN402_STATE_QUICK_STOP_ACTIVE;
    				if(canOpenNode_SdoWr(AxisNodeId,0x6040,0,0x0002,2)!=APPLICATIONERROR_NONE){};//{return;}
    				AxisNodeId = messageRobotArmRX.ArmAxis + 0x20;
    				return;
    			}
				AxisNodeId = messageRobotArmRX.ArmAxis + 0x20;
				RobotArm_joystick2rpm(messageRobotArmRX.yValue, messageRobotArmRX.maxJointVelocity, AxisNodeId);
				RobotArm_setPSM(AxisNodeId);
				break;

    		case ROBOTARMMODE_IK:
				// control hole robot arm
    			getAngles = globalDataStructures_getRobotArm_ARM_GS();

    			static uint32_t zonk3000 = 0;
				zonk3000++;
				if(zonk3000 <= 2)
				{
					canOpenNode_MasterNmtWr(0x01, 0);
				}
				if(zonk3000 < 50000){zonk3000 = 20;}

				if(getAngles.actualJointAngle2 == 0){return;}
				if(getAngles.actualJointAngle3 == 0){return;}
				if(getAngles.actualJointAngle4 == 0){return;}
//				if(getAngles.actualJointAngle5 == 0){return;}

				uint16_t setPointAxis2 = 18000;
				uint16_t setPointAxis3 = 26000;
				uint16_t setPointAxis4 = 9000;
				uint16_t setPointAxis5 = 18000;

				RobotArm_setSingleAngle(NODE_ID_MOTOR_2, getAngles.actualJointAngle2, setPointAxis2, 350, 40);
				RobotArm_setSingleAngle(NODE_ID_MOTOR_3, getAngles.actualJointAngle3, setPointAxis3, 450, 35);
				RobotArm_setSingleAngle(NODE_ID_MOTOR_4, getAngles.actualJointAngle4, setPointAxis4, 550, 20);
				RobotArm_setSingleAngle(NODE_ID_MOTOR_5, getAngles.actualJointAngle5, setPointAxis5, 1550, 1);

    	    	break;
    		case ROBOTARMMODE_TEACHED_POS:
    			break;
    		default:
    			// set error and switch off motor's
    			break;
    	}
    }
    else
    {
        Error_Handler();
    }
    messageRobotArmRX_old = messageRobotArmRX;
}

void RobotArm_updateEncoder(uint8_t NodeId, uint16_t data)
{
	// Get
	static globalData_typeDef_robotArm_ARM_GS messageRobotArmTX = {0};
	messageRobotArmTX = globalDataStructures_getRobotArm_ARM_GS();

	if(messageRobotArmTX.dataID==GLOBALDATA_ID_ARM)
	{
		/* calculate from raw data (0 - 16384) the angle values (0 - 36000)  */
		uint16_t jointAngle = 0;
		float temp1 = 2.19727; // faktor - 0 ... 16384 rawData to 0 ... 36000
		temp1 = ((float)data * temp1) + 0.5; // round (+0.5)
		jointAngle = (uint16_t) temp1;

    	/* check id */
    	switch (NodeId)
    	{
    		case NODE_ID_ENCODER_1:
    			messageRobotArmTX.actualJointAngle1 = jointAngle;

				break;
    		case NODE_ID_ENCODER_2:
    			messageRobotArmTX.actualJointAngle2 = jointAngle;
    			break;
    		case NODE_ID_ENCODER_3:
    			messageRobotArmTX.actualJointAngle3 = jointAngle;
				break;
    		case NODE_ID_ENCODER_4:
    			messageRobotArmTX.actualJointAngle4 = jointAngle;
    			break;
    		case NODE_ID_ENCODER_5:
    			messageRobotArmTX.actualJointAngle5 = jointAngle;
    			break;
    		default:
    			break;
    	}
	}
	else
	{
		Error_Handler();
	}


	// Set
    if(globalDataStructures_setRobotArm_ARM_GS(messageRobotArmTX)!=GLOBAL_DATA_STRUCT_SET_OK)
    {
    	Error_Handler();
    }
}

void RobotArm_setSingleAngle(uint8_t NodeId, uint16_t actualAngle, uint16_t tagetAngle, uint16_t hysteresis, int32_t velocity)
{
	if(actualAngle <= tagetAngle+hysteresis && actualAngle >= tagetAngle-hysteresis)
	{
		// target position reached - set motor to quick stop;
		if(canOpenNode_SdoWr(NodeId, 0x6040, 0, 0x0003, 2)!=APPLICATIONERROR_NONE){};//{return;}
		if(canOpenNode_SdoWr(NodeId, 0x60FF, 0, 0, 4)!=APPLICATIONERROR_NONE){};//{return;}
	}
	else if(actualAngle >= tagetAngle+hysteresis)
	{
		// move in negative direction
		velocity = -velocity;
		if(canOpenNode_SdoWr(NodeId, 0x6040, 0, 0x000F, 2)!=APPLICATIONERROR_NONE){};//{return;}
		if(canOpenNode_SdoWr(NodeId, 0x60FF, 0, velocity, 4)!=APPLICATIONERROR_NONE){};//{return;}
	}
	else if(actualAngle <= tagetAngle-hysteresis)
	{
		// move in positive direction
		if(canOpenNode_SdoWr(NodeId, 0x6040, 0, 0x000F, 2)!=APPLICATIONERROR_NONE){};//{return;}
		if(canOpenNode_SdoWr(NodeId, 0x60FF, 0, velocity, 4)!=APPLICATIONERROR_NONE){};//{return;}
	}
	else
	{
		return;
	}
}

uint8_t RobotArm_setSingleAngleD(uint8_t NodeId, double actualAngle, double tagetAngle, double hysteresis, double velocity)
{
	velocity = velocity+0.5; // runden
	int32_t v = (int32_t)velocity;
	if(actualAngle <= tagetAngle+hysteresis && actualAngle >= tagetAngle-hysteresis)
	{
		// target position reached - set motor to quick stop;
		if(canOpenNode_SdoWr(NodeId, 0x6040, 0, 0x0003, 2)!=APPLICATIONERROR_NONE){};//{return;}
		if(canOpenNode_SdoWr(NodeId, 0x60FF, 0, 0, 4)!=APPLICATIONERROR_NONE){};//{return;}
		return 1;
	}
	else if(actualAngle >= tagetAngle+hysteresis)
	{
		// move in negative direction
		v = -v;
		if(canOpenNode_SdoWr(NodeId, 0x6040, 0, 0x000F, 2)!=APPLICATIONERROR_NONE){};//{return;}
		if(canOpenNode_SdoWr(NodeId, 0x60FF, 0, v, 4)!=APPLICATIONERROR_NONE){};//{return;}
		return 0;
	}
	else if(actualAngle <= tagetAngle-hysteresis)
	{
		// move in positive direction
		if(canOpenNode_SdoWr(NodeId, 0x6040, 0, 0x000F, 2)!=APPLICATIONERROR_NONE){};//{return;}
		if(canOpenNode_SdoWr(NodeId, 0x60FF, 0, v, 4)!=APPLICATIONERROR_NONE){};//{return;}
		return 0;
	}
	else
	{
		return -1;
	}
}

void RobotArm_setAngle(uint8_t NodeId, uint16_t actualAngle, uint16_t tagetAngle, uint16_t hysteresis, int32_t velocity)
{
	if(actualAngle <= tagetAngle+hysteresis && actualAngle >= tagetAngle-hysteresis)
	{
		// target position reached - set motor to quick stop;
		if(canOpenNode_SdoWr(NodeId, 0x6040, 0, 0x0003, 2)!=APPLICATIONERROR_NONE){};//{return;}
		if(canOpenNode_SdoWr(NodeId, 0x60FF, 0, 0, 4)!=APPLICATIONERROR_NONE){};//{return;}
		switch(NodeId)
		{
		case 0x25:
			boolschritt[4] = 1;
			break;
		case 0x24:
			boolschritt[3] = 1;
			break;
		case 0x23:
			boolschritt[2] = 1;
			break;
		case 0x22:
			boolschritt[1] = 1;
			break;
		}

		if(boolschritt[1] == 1 && boolschritt[2] == 1 && boolschritt[3] == 1 && boolschritt[4] == 1)
		{
			aktuellerSchritt = aktuellerSchritt +1;
			boolschritt[1] = 0;
			boolschritt[2] = 0;
			boolschritt[3] = 0;
			boolschritt[4] = 0;
		}

	}
	else if(actualAngle >= tagetAngle+hysteresis)
	{
		// move in negative direction
		velocity = -velocity;
		if(canOpenNode_SdoWr(NodeId, 0x6040, 0, 0x000F, 2)!=APPLICATIONERROR_NONE){};//{return;}
		if(canOpenNode_SdoWr(NodeId, 0x60FF, 0, velocity, 4)!=APPLICATIONERROR_NONE){};//{return;}
	}
	else if(actualAngle <= tagetAngle-hysteresis)
	{
		// move in positive direction
		if(canOpenNode_SdoWr(NodeId, 0x6040, 0, 0x000F, 2)!=APPLICATIONERROR_NONE){};//{return;}
		if(canOpenNode_SdoWr(NodeId, 0x60FF, 0, velocity, 4)!=APPLICATIONERROR_NONE){};//{return;}
	}
	else
	{
		return;
	}
}

void RobotArm_updateTX(void)
{

}

void RobotArm_setVelocityButtons(uint16_t velocity , globalData_enumTypeDef_robotArm2Buttons state, uint8_t NodeId)
{
	if(state == ROBOTARM2BUTTONS_ON_OPEN_LEFT || state == ROBOTARM2BUTTONS_OFF_CLOSE_RIGHT)
	{
		RobotArm_setPSM(NodeId);
	}
	else
	{
		return;
	}
	/* set limits to nodes */
	if(NodeId == NODE_ID_MOTOR_5) // axis 5
	{
		if(velocity >= AXIS_5_MAX_VELOCITY)
		{
			velocity = AXIS_5_MAX_VELOCITY;
		}
	}
	else if(NodeId == NODE_ID_MOTOR_6) // endeffector
	{
		if(velocity >= AXIS_5_MAX_VELOCITY)
		{
			velocity = ENDEFFECTOR_MAX_VELOCITY;
		}
	}
	else
	{
		return;
	}

	switch(state)
	{
		case ROBOTARM2BUTTONS_DISABLE:
			break;
		case ROBOTARM2BUTTONS_ON_OPEN_LEFT:
			RobotArm_setProfileVelocity(velocity, NodeId);
			break;
		case ROBOTARM2BUTTONS_OFF_CLOSE_RIGHT:
			RobotArm_setProfileVelocity(-velocity, NodeId);
			break;
		default:
			break;
	}
}

//void RobotArm_setProfileVelocity(int32_t velocity_rpm, uint8_t NodeId)


/* NAME:		RobotArm_joystick2rpm
 * INPUT:		int8_t y_direction -> joystick backwards/forwards (value range -100 to +100)
 * 				uint8_t maxVelocity -> 'GUI' value, end value for scaling
 * 				uint8_t NodeId -> addresses the corresponding motor via the node id
 * OUTPUT:		-none-
 * DESCRIPTION:
 *		calculate the RPM from the joystick and maxVelocity values - taking into account the joystick's
 *		"dead" zone; scale range from zero to maxVelocity;
 *		joystick value from ZERO to +-ROBOTARM_DEATHCENTERPOINT - rpm = 0;
 *		EXAMPLE: maxVelocity = 100; ROBOTARM_DEATHCENTERPOINT = 15;
 *		y_direction = 10,	18,		25, 	50, 	57.5,	 90, 	100;
 *		rpm 		= 0,	3.5,	11.8, 	41.2, 	50, 	 88.2, 	100;
 *		scaleValue = outMin + (y_direction - inMin) * ((outMax-outMin)/(inMax-inMin))
 */
#define ROBOTARM_DEATHCENTERPOINT	15
void RobotArm_joystick2rpm(int16_t y_direction, uint16_t maxVelocity, uint8_t NodeId)
{
	int32_t velocity_rpm = 0;
	float calcVelocity = 0;
	/* death center point - zone in which no operation is valid */
    if (y_direction <= ROBOTARM_DEATHCENTERPOINT && y_direction >= -ROBOTARM_DEATHCENTERPOINT)
    {
    	velocity_rpm = 0;
    }
    else
    {
    	/* scale from 0 - maxVelocity, consider death center point */
    	//scaleValue = outMin + (y_direction - inMin) * ((outMax-outMin)/(inMax-inMin))
        if(y_direction < 0)
        	{
        		calcVelocity = ((float)y_direction + ROBOTARM_DEATHCENTERPOINT) * ((float)maxVelocity/(100-ROBOTARM_DEATHCENTERPOINT));
        		calcVelocity = calcVelocity - 0.5; // round
        	}
        else
        	{
        		calcVelocity = ((float)y_direction - ROBOTARM_DEATHCENTERPOINT) * ((float)maxVelocity/(100-ROBOTARM_DEATHCENTERPOINT));
        		calcVelocity = calcVelocity + 0.5; // round
        	}
        velocity_rpm = (int32_t)calcVelocity;
    }

    /* if we are outside the specification -> do not apply */
    if((velocity_rpm > maxVelocity) || (velocity_rpm < (-maxVelocity))){return;}
    RobotArm_setProfileVelocity(velocity_rpm, NodeId);
}

/* NAME:		RobotArm_setProfileVelocity
 * INPUT:		int32_t velocity_rpm -> data value for motor controller
 * 				uint8_t NodeId -> addresses the corresponding motor via the node id
 * OUTPUT:		-none-
 * DESCRIPTION:
 *		write the velocity to motor controller XY is the state is OPERATION_ENABLED;
 */
void RobotArm_setProfileVelocity(int32_t velocity_rpm, uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
	Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
	if(Node->Node402State == CANOPEN402_STATE_OPERATION_ENABLED)
	{
		if(canOpenNode_SdoWr(NodeId, 0x60FF, 0, velocity_rpm, 4)!=APPLICATIONERROR_NONE){return;}
	}
}

/* NAME:		RobotArm_setPSM
 * INPUT:		uint8_t NodeId -> addresses the corresponding motor via the node id
 * OUTPUT:		-none-
 * DESCRIPTION:
 * 		PowerStateMachine of CANopen motor controller (CiA 402)
 *		if state is OPERATION_ENABLED to nothing; if state is different -
 *		set default status SWITCH_ON_DISABLED, after this set state to OPERATION_ENABLED
 */
void RobotArm_setPSM(uint8_t NodeId)
{
    canOpenNode_typeDef_Node402* Node;
	Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NodeId];
	if(Node->Node402State != CANOPEN402_STATE_OPERATION_ENABLED)
	{
		if(Node->Node402State >= CANOPEN402_STATE_SWITCHED_ON)
		{
			if(canOpenNode402_PowerStateMachine(NodeId,CANOPEN402_STATE_OPERATION_ENABLED)!=APPLICATIONERROR_NONE)
				Error_Handler();
		}
		else
		{
			if(canOpenNode402_PowerStateMachine(NodeId,CANOPEN402_STATE_SWITCHED_ON)!=APPLICATIONERROR_NONE)
				Error_Handler();
		}
	}
}

void RobotArm_quickstop()
{
#if TEST_MODE_ARM
    if(canOpenNode_SdoWr(NODE_ID_MOTOR_TEST,CANOPEN_IDX_CONTROL_WORD,0,0x0B, 2)==APPLICATIONERROR_NONE)
        {
//    		robotArm_MotorTest.ValuesProfileVelocityMode.stopActive = 0; /* FIXME SE comment */
        }
#else
#endif
}

void RobotArm_allMotorsQuickStop(void)
{
//	if(canOpenNode_SdoWr(NODE_ID_MOTOR_1,0x6040,0,0x0002,2)!=APPLICATIONERROR_NONE){};//{return;}
	if(canOpenNode_SdoWr(NODE_ID_MOTOR_2,0x6040,0,0x0002,2)!=APPLICATIONERROR_NONE){};//{return;}
	if(canOpenNode_SdoWr(NODE_ID_MOTOR_3,0x6040,0,0x0002,2)!=APPLICATIONERROR_NONE){};//{return;}
	if(canOpenNode_SdoWr(NODE_ID_MOTOR_4,0x6040,0,0x0002,2)!=APPLICATIONERROR_NONE){};//{return;}
	if(canOpenNode_SdoWr(NODE_ID_MOTOR_5,0x6040,0,0x0002,2)!=APPLICATIONERROR_NONE){};//{return;}
}


void RobotArm_allMotorsHALT(void)
{
//	if(canOpenNode_SdoWr(NODE_ID_MOTOR_TEST,0x6040,0,0x010F,2)!=APPLICATIONERROR_NONE){return;}
//	if(canOpenNode_SdoWr(NODE_ID_MOTOR_1,0x6040,0,0x010F,2)!=APPLICATIONERROR_NONE){return;}
//	if(canOpenNode_SdoWr(NODE_ID_MOTOR_2,0x6040,0,0x010F,2)!=APPLICATIONERROR_NONE){return;}
//	if(canOpenNode_SdoWr(NODE_ID_MOTOR_3,0x6040,0,0x010F,2)!=APPLICATIONERROR_NONE){return;}
//	if(canOpenNode_SdoWr(NODE_ID_MOTOR_4,0x6040,0,0x010F,2)!=APPLICATIONERROR_NONE){return;}
//	if(canOpenNode_SdoWr(NODE_ID_MOTOR_5,0x6040,0,0x010F,2)!=APPLICATIONERROR_NONE){return;}


	if(canOpenNode_SdoWr(NODE_ID_MOTOR_5,0x6040,0,0x010F,2)!=APPLICATIONERROR_NONE)
	{
		return;
	}

//	robotArm_Motor5.Node402State = CANOPEN402_STATE_SWITCH_ON_DISABLED;

	//	if(canOpenNode_SdoWr(NODE_ID_MOTOR_6,0x6040,0,0x010F,2)!=APPLICATIONERROR_NONE){return;}
}

void RobotArm_allMotors_PSM_restart(void)
{
	canOpenNode_typeDef_Node402* Node;
	Node = (canOpenNode_typeDef_Node402*) canOpenNodeInstances[NODE_ID_MOTOR_5];
	Node->Node402State = CANOPEN402_STATE_QUICK_STOP_ACTIVE;
//	Node->Node402State = CANOPEN402_STATE_SWITCHED_ON;
	if(canOpenNode_SdoWr(NODE_ID_MOTOR_5,0x6040,0,0x010F,2)!=APPLICATIONERROR_NONE)
	{
		return;
	}
//	robotArm_MotorTest.Node402State = CANOPEN402_STATE_READY_TO_SWITCH_ON;
//	robotArm_Motor1.Node402State = CANOPEN402_STATE_READY_TO_SWITCH_ON;
//	robotArm_Motor2.Node402State = CANOPEN402_STATE_READY_TO_SWITCH_ON;
//	robotArm_Motor3.Node402State = CANOPEN402_STATE_READY_TO_SWITCH_ON;
//	robotArm_Motor4.Node402State = CANOPEN402_STATE_READY_TO_SWITCH_ON;
//	robotArm_Motor5.Node402State = CANOPEN402_STATE_READY_TO_SWITCH_ON;
}

void RobotArm_setLinearActuatorDutyCycle(uint8_t percent, globalData_enumTypeDef_robotArm2Buttons state)
{
	uint32_t pulse = 0;
	uint32_t period = 0;
	switch (state)
	{
		case ROBOTARM2BUTTONS_DISABLE:
			htim4.Instance->CCR1 = 5399; // 10% duty cycle
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			return;
			break;
		case ROBOTARM2BUTTONS_ON_OPEN_LEFT:
			/* !!! CONFIG IS FOR CHANNEL 1 (CCR1) VALID !!! */
			if (percent <= 0 || percent > 100)
			{
				// set pulse to zero
				// if percent = 0 --->> init value (or last value) is taking
				htim4.Instance->CCR1 = 5399; // 10% duty cycle
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			}
			else
			{
				//  PERCENT duty cycle:     pulse_length = ((PERIOD + 1) * PERCENT) / 100 - 1 = VAL2REG
				//  25% duty cycle:     pulse_length = ((53999 + 1) * 25) / 100 - 1 = 2099
				period = htim4.Init.Period;
				pulse = (((period + 1) * percent) / 100) - 1;
				htim4.Instance->CCR1 = pulse;
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
			}
			break;
		case ROBOTARM2BUTTONS_OFF_CLOSE_RIGHT:
			htim4.Instance->CCR1 = 5399;
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			break;
		default:
			htim4.Instance->CCR1 = 5399;
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			break;
	}
}



/* called when error occurs
 * stops motor and resets node */
void RobotArm_errorHandler(uint8_t NodeId)
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
			RobotArm_quickstop();
			canOpenNode_resetInitialization(NodeId);
			break;
		}
		case NODEERROR_SDO_TIMEOUT:
		{
			// TODO disable power stage
//			RobotArm_quickstop();
//			canOpenNode_resetInitialization(NodeId);
			break;
		}
		case NODEERROR_SDO:
		{
			// TODO disable power stage
			// FIXME SE set PSM to default value
	//    	RobotArm_quickstop();
	//        canOpenNode_resetInitialization(NodeId);
			break;
		}
		case NODEERROR_DEFAULT:
		{
			RobotArm_quickstop();
			canOpenNode_resetInitialization(NodeId);
			break;
		}
		case NODEERROR_NONE:
			break;
    }
    active=0;
}

void RobotArm_initLinearActuator(void)
{
	TIM_OC_InitTypeDef usConfigOC;
	usConfigOC.OCMode = TIM_OCMODE_PWM1;
	usConfigOC.Pulse = 5399; // 10% duty cycle
	usConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	usConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim4, &usConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void RobotArm_init(void)
{
	canOpenNode_MasterNmtWr(0x80, 0); // pre operational
	RobotArm_initMotors();
	RobotArm_initEncoders();
	RobotArm_initLinearActuator();
	canOpenNode_MasterNmtWr(0x01, 0);
}


void RobotArm_initEncoders(void)
{
//    /* set parameters to node structures */
//	/* I N I T   ENCODER */
	/* ENCODER 1 */
	robotArm_Encoder1.NodeId 		= NODE_ID_ENCODER_1;
	robotArm_Encoder1.NodeType 		= NODETYPE_EAM360;
	robotArm_Encoder1.NodeState 	= NODESTATE_NONE;
	robotArm_Encoder1.NodeError 	= NODEERROR_NONE;
	robotArm_Encoder1.NodeNmtStatus	= NOTESTATE_NMT_NONE;
	robotArm_Encoder1.hcan			= &hcan1;
	robotArm_Encoder1.mhcanHandle	= &mhcan1Handle;
	robotArm_Encoder1.errorCallback	= &RobotArm_errorHandler;
	if(canOpenNode_addInstance(&robotArm_Encoder1, 0, 0)!=APPLICATIONERROR_NONE)
	{
		// TODO something really bad
		Error_Handler();
	}
	/* ENCODER 2 */
	robotArm_Encoder2.NodeId 		= NODE_ID_ENCODER_2;
	robotArm_Encoder2.NodeType 		= NODETYPE_EAM360;
	robotArm_Encoder2.NodeState 	= NODESTATE_NONE;
	robotArm_Encoder2.NodeError 	= NODEERROR_NONE;
	robotArm_Encoder2.NodeNmtStatus	= NOTESTATE_NMT_NONE;
	robotArm_Encoder2.hcan			= &hcan1;
	robotArm_Encoder2.mhcanHandle	= &mhcan1Handle;
	robotArm_Encoder2.errorCallback	= &RobotArm_errorHandler;
	if(canOpenNode_addInstance(&robotArm_Encoder2, 0, 0)!=APPLICATIONERROR_NONE)
	{
		// TODO something really bad
		Error_Handler();
	}

	/* ENCODER 3 */
	robotArm_Encoder3.NodeId 		= NODE_ID_ENCODER_3;
	robotArm_Encoder3.NodeType 		= NODETYPE_EAM360;
	robotArm_Encoder3.NodeState 	= NODESTATE_NONE;
	robotArm_Encoder3.NodeError 	= NODEERROR_NONE;
	robotArm_Encoder3.NodeNmtStatus	= NOTESTATE_NMT_NONE;
	robotArm_Encoder3.hcan			= &hcan1;
	robotArm_Encoder3.mhcanHandle	= &mhcan1Handle;
	robotArm_Encoder3.errorCallback	= &RobotArm_errorHandler;
	if(canOpenNode_addInstance(&robotArm_Encoder3, 0, 0)!=APPLICATIONERROR_NONE)
	{
		// TODO something really bad
		Error_Handler();
	}

	/* ENCODER 4 */
	robotArm_Encoder4.NodeId 		= NODE_ID_ENCODER_4;
	robotArm_Encoder4.NodeType 		= NODETYPE_EAM360;
	robotArm_Encoder4.NodeState 	= NODESTATE_NONE;
	robotArm_Encoder4.NodeError 	= NODEERROR_NONE;
	robotArm_Encoder4.NodeNmtStatus	= NOTESTATE_NMT_NONE;
	robotArm_Encoder4.hcan			= &hcan1;
	robotArm_Encoder4.mhcanHandle	= &mhcan1Handle;
	robotArm_Encoder4.errorCallback	= &RobotArm_errorHandler;
	if(canOpenNode_addInstance(&robotArm_Encoder4, 0, 0)!=APPLICATIONERROR_NONE)
	{
		// TODO something really bad
		Error_Handler();
	}

	/* ENCODER 5 */
	robotArm_Encoder5.NodeId 		= NODE_ID_ENCODER_5;
	robotArm_Encoder5.NodeType 		= NODETYPE_EAM360;
	robotArm_Encoder5.NodeState 	= NODESTATE_NONE;
	robotArm_Encoder5.NodeError 	= NODEERROR_NONE;
	robotArm_Encoder5.NodeNmtStatus	= NOTESTATE_NMT_NONE;
	robotArm_Encoder5.hcan			= &hcan1;
	robotArm_Encoder5.mhcanHandle	= &mhcan1Handle;
	robotArm_Encoder5.errorCallback	= &RobotArm_errorHandler;
	if(canOpenNode_addInstance(&robotArm_Encoder5, 0, 0)!=APPLICATIONERROR_NONE)
	{
		// TODO something really bad
		Error_Handler();
	}

}

/*

uint8_t isRobotArm_initEncodersSuccessfull(globalData_typeDef_robotArm* Check_Arm)
{
	if (!isRobotArm_initeEncoders_SetUpSucessfull(Check_Arm->robotArm_Encoder1,NODE_ID_ENCODER_1)) return 0;
	if (!isRobotArm_initeEncoders_SetUpSucessfull(Check_Arm->robotArm_Encoder2,NODE_ID_ENCODER_2)) return 0;
	if (!isRobotArm_initeEncoders_SetUpSucessfull(Check_Arm->robotArm_Encoder3,NODE_ID_ENCODER_3)) return 0;
	if (!isRobotArm_initeEncoders_SetUpSucessfull(Check_Arm->robotArm_Encoder4,NODE_ID_ENCODER_4)) return 0;
	if (!isRobotArm_initeEncoders_SetUpSucessfull(Check_Arm->robotArm_Encoder5,NODE_ID_ENCODER_5)) return 0;
	return 1;
}

uint8_t isRobotArm_initeEncoders_SetUpSucessfull(canOpenNode_typeDef_Node* Node,uint8_t Node_id)
{
	Node->NodeId 			= Node_id;
	Node->NodeType 			= NODETYPE_EAM360;
	Node->NodeState 		= NODESTATE_NONE;
	Node->NodeError 		= NODEERROR_NONE;
	Node->NodeNmtStatus		= NOTESTATE_NMT_NONE;
	Node->hcan				= &hcan1;
	Node->mhcanHandle		= &mhcan1Handle;
	Node->errorCallback	= &RobotArm_errorHandler;
	if(canOpenNode_addInstance(Node, 0, 0)!=APPLICATIONERROR_NONE) return 0;
	return 1;
}

uint8_t isRobotArm_initMotorsSuccessful(globalData_typeDef_robotArm* Check_Arm)
{
	if(!isRobotArm_initMotors_SetUpSucessfull(Check_Arm->robotArm_Motor2,NODE_ID_MOTOR_2)) return 0;
	if(!isRobotArm_initMotors_SetUpSucessfull(Check_Arm->robotArm_Motor3,NODE_ID_MOTOR_3)) return 0;
	if(!isRobotArm_initMotors_SetUpSucessfull(Check_Arm->robotArm_Motor4,NODE_ID_MOTOR_4)) return 0;
	if(!isRobotArm_initMotors_SetUpSucessfull(Check_Arm->robotArm_Motor5,NODE_ID_MOTOR_5)) return 0;
	return 1;
}

uint8_t isRobotArm_initMotors_SetUpSucessfull(canOpenNode_typeDef_Node402* Node,uint8_t Node_id)
{
	Node->NodeBasic.NodeId 				= Node_id;
	Node->NodeBasic.NodeType 			= NODETYPE_CL4E;
	Node->NodeBasic.NodeState 			= NODESTATE_NONE;
	Node->NodeBasic.NodeError 			= NODEERROR_NONE;
	Node->NodeBasic.NodeNmtStatus		= NOTESTATE_NMT_NONE;
	Node->Node402Mode 					= NODE402MODE_PROFILE_VELOCITY;
	Node->Node402State 					= CANOPEN402_STATE_SWITCH_ON_DISABLED;
	Node->NodeBasic.hcan				= &hcan1;
	Node->NodeBasic.mhcanHandle			= &mhcan1Handle;
	Node->NodeBasic.errorCallback		= &RobotArm_errorHandler;
	if(canOpenNode_addInstance(&Node->NodeBasic, CL4E_initSDOsMotor2,sizeof(CL4E_initSDOsMotor2)/sizeof(canOpen_typeDef_SDOprimitive))!=APPLICATIONERROR_NONE) return 0;
	return 1;
}

*/


/* initialize the motors
 * registers the devices in canOpenNode
 * motor is ready for usage after that */
void RobotArm_initMotors(void)
{
    /* set parameters to node structures */
	/* I N I T   M O T O R S */
//	/* INIT MOTOR 1 - AXIS */
//    robotArm_Motor1.NodeBasic.NodeId 			= NODE_ID_MOTOR_1;
//    robotArm_Motor1.NodeBasic.NodeType 		= NODETYPE_CL4E;
//    robotArm_Motor1.NodeBasic.NodeState 		= NODESTATE_NONE;
//    robotArm_Motor1.NodeBasic.NodeError 		= NODEERROR_NONE;
//    robotArm_Motor1.NodeBasic.NodeNmtStatus	= NOTESTATE_NMT_NONE;
//    robotArm_Motor1.Node402Mode 				= NODE402MODE_PROFILE_VELOCITY;
//    robotArm_Motor1.Node402State 				= CANOPEN402_STATE_SWITCH_ON_DISABLED;
//    robotArm_Motor1.NodeBasic.hcan			= &hcan1;
//    robotArm_Motor1.NodeBasic.mhcanHandle		= &mhcan1Handle;
//    robotArm_Motor1.NodeBasic.errorCallback	= &RobotArm_errorHandler;
//    if(canOpenNode_addInstance(&robotArm_Motor1.NodeBasic, CL4E_initSDOsMotor1,sizeof(CL4E_initSDOsMotor1)/sizeof(canOpen_typeDef_SDOprimitive))!=APPLICATIONERROR_NONE)
//    {
//        // TODO something really bad
//        Error_Handler();
//    }
//
	/* INIT MOTOR 2 - AXIS */
    robotArm_Motor2.NodeBasic.NodeId 			= NODE_ID_MOTOR_2;
    robotArm_Motor2.NodeBasic.NodeType 		= NODETYPE_CL4E;
    robotArm_Motor2.NodeBasic.NodeState 		= NODESTATE_NONE;
    robotArm_Motor2.NodeBasic.NodeError 		= NODEERROR_NONE;
    robotArm_Motor2.NodeBasic.NodeNmtStatus	= NOTESTATE_NMT_NONE;
    robotArm_Motor2.Node402Mode 				= NODE402MODE_PROFILE_VELOCITY;
    robotArm_Motor2.Node402State 				= CANOPEN402_STATE_SWITCH_ON_DISABLED;
    robotArm_Motor2.NodeBasic.hcan			= &hcan1;
    robotArm_Motor2.NodeBasic.mhcanHandle		= &mhcan1Handle;
    robotArm_Motor2.NodeBasic.errorCallback	= &RobotArm_errorHandler;
    if(canOpenNode_addInstance(&robotArm_Motor2.NodeBasic, CL4E_initSDOsMotor2,sizeof(CL4E_initSDOsMotor2)/sizeof(canOpen_typeDef_SDOprimitive))!=APPLICATIONERROR_NONE)
    {
        // TODO something really bad
        Error_Handler();
    }

	/* INIT MOTOR 3 - AXIS */
    robotArm_Motor3.NodeBasic.NodeId 			= NODE_ID_MOTOR_3;
    robotArm_Motor3.NodeBasic.NodeType 		= NODETYPE_CL4E;
    robotArm_Motor3.NodeBasic.NodeState 		= NODESTATE_NONE;
    robotArm_Motor3.NodeBasic.NodeError 		= NODEERROR_NONE;
    robotArm_Motor3.NodeBasic.NodeNmtStatus	= NOTESTATE_NMT_NONE;
    robotArm_Motor3.Node402Mode 				= NODE402MODE_PROFILE_VELOCITY;
    robotArm_Motor3.Node402State 				= CANOPEN402_STATE_SWITCH_ON_DISABLED;
    robotArm_Motor3.NodeBasic.hcan			= &hcan1;
    robotArm_Motor3.NodeBasic.mhcanHandle		= &mhcan1Handle;
    robotArm_Motor3.NodeBasic.errorCallback	= &RobotArm_errorHandler;
    if(canOpenNode_addInstance(&robotArm_Motor3.NodeBasic, CL4E_initSDOsMotor3,sizeof(CL4E_initSDOsMotor3)/sizeof(canOpen_typeDef_SDOprimitive))!=APPLICATIONERROR_NONE)
    {
        // TODO something really bad
        Error_Handler();
    }

	/* INIT MOTOR 4 - AXIS */
    robotArm_Motor4.NodeBasic.NodeId 			= NODE_ID_MOTOR_4;
    robotArm_Motor4.NodeBasic.NodeType 			= NODETYPE_CL4E;
    robotArm_Motor4.NodeBasic.NodeState 		= NODESTATE_NONE;
    robotArm_Motor4.NodeBasic.NodeError 		= NODEERROR_NONE;
    robotArm_Motor4.NodeBasic.NodeNmtStatus	= NOTESTATE_NMT_NONE;
    robotArm_Motor4.Node402Mode 				= NODE402MODE_PROFILE_VELOCITY;
    robotArm_Motor4.Node402State 				= CANOPEN402_STATE_SWITCH_ON_DISABLED;
    robotArm_Motor4.NodeBasic.hcan			= &hcan1;
    robotArm_Motor4.NodeBasic.mhcanHandle		= &mhcan1Handle;
    robotArm_Motor4.NodeBasic.errorCallback	= &RobotArm_errorHandler;
    if(canOpenNode_addInstance(&robotArm_Motor4.NodeBasic, CL4E_initSDOsMotor4,sizeof(CL4E_initSDOsMotor4)/sizeof(canOpen_typeDef_SDOprimitive))!=APPLICATIONERROR_NONE)
    {
        // TODO something really bad
        Error_Handler();
    }

	/* INIT MOTOR 5 - AXIS */
    robotArm_Motor5.NodeBasic.NodeId 			= NODE_ID_MOTOR_5;
    robotArm_Motor5.NodeBasic.NodeType 			= NODETYPE_CL4E;
    robotArm_Motor5.NodeBasic.NodeState 		= NODESTATE_NONE;
    robotArm_Motor5.NodeBasic.NodeError 		= NODEERROR_NONE;
    robotArm_Motor5.NodeBasic.NodeNmtStatus		= NOTESTATE_NMT_NONE;
    robotArm_Motor5.Node402Mode 				= NODE402MODE_PROFILE_VELOCITY;
    robotArm_Motor5.Node402State 				= CANOPEN402_STATE_SWITCH_ON_DISABLED;
    robotArm_Motor5.NodeBasic.hcan				= &hcan1;
    robotArm_Motor5.NodeBasic.mhcanHandle		= &mhcan1Handle;
    robotArm_Motor5.NodeBasic.errorCallback		= &RobotArm_errorHandler;
    if(canOpenNode_addInstance(&robotArm_Motor5.NodeBasic, CL4E_initSDOsMotor5,sizeof(CL4E_initSDOsMotor5)/sizeof(canOpen_typeDef_SDOprimitive))!=APPLICATIONERROR_NONE)
    {
        // TODO something really bad
        Error_Handler();
    }
//
//	/* INIT MOTOR 6 - AXIS */
//    robotArm_Motor6.NodeBasic.NodeId 			= NODE_ID_MOTOR_6;
//    robotArm_Motor6.NodeBasic.NodeType 		= NODETYPE_CL4E;
//    robotArm_Motor6.NodeBasic.NodeState 		= NODESTATE_NONE;
//    robotArm_Motor6.NodeBasic.NodeError 		= NODEERROR_NONE;
//    robotArm_Motor6.NodeBasic.NodeNmtStatus	= NOTESTATE_NMT_NONE;
//    robotArm_Motor6.Node402Mode 				= NODE402MODE_PROFILE_VELOCITY;
//    robotArm_Motor6.Node402State 				= CANOPEN402_STATE_SWITCH_ON_DISABLED;
//    robotArm_Motor6.NodeBasic.hcan			= &hcan1;
//    robotArm_Motor6.NodeBasic.mhcanHandle		= &mhcan1Handle;
//    robotArm_Motor6.NodeBasic.errorCallback	= &RobotArm_errorHandler;
//    if(canOpenNode_addInstance(&robotArm_Motor6.NodeBasic, CL4E_initSDOsMotor6,sizeof(CL4E_initSDOsMotor6)/sizeof(canOpen_typeDef_SDOprimitive))!=APPLICATIONERROR_NONE)
//    {
//        // TODO something really bad
//        Error_Handler();
//    }
}

/* read state word and save state to node struct */
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


