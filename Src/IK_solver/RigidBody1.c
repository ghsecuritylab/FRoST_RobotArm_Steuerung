/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RigidBody1.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/RigidBody1.h"
#include "IK_solver/RigidBodyTree1.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/strcmp.h"
#include "IK_solver/IK_solver_data.h"

/* Function Definitions */

/*
 * Arguments    : robotics_RigidBody *obj
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                f_robotics_manip_internal_Rigid *iobj_2
 *                e_robotics_manip_internal_Rigid *iobj_3
 * Return Type  : robotics_RigidBody *
 */
robotics_RigidBody *b_RigidBody_RigidBody(robotics_RigidBody *obj,
  c_robotics_manip_internal_Rigid *iobj_0, robotics_Joint *iobj_1,
  f_robotics_manip_internal_Rigid *iobj_2, e_robotics_manip_internal_Rigid
  *iobj_3)
{
  robotics_RigidBody *b_obj;
  int b_index;
  static const char bname[5] = { 'b', 'o', 'd', 'y', '1' };

  static const char jname[9] = { 'b', 'o', 'd', 'y', '1', '_', 'j', 'n', 't' };

  emxArray_char_T *switch_expression;
  static const char cv7[5] = { 'f', 'i', 'x', 'e', 'd' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv10[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv11[6] = { 0, 0, 0, 0, 0, 1 };

  signed char b_I[9];
  f_robotics_manip_internal_Rigid *b_default;
  b_obj = obj;
  b_index = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_0->NameInternal, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    iobj_0->NameInternal->data[b_index] = bname[b_index];
  }

  iobj_1->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 9;
  emxEnsureCapacity_char_T(iobj_1->NameInternal, b_index);
  for (b_index = 0; b_index < 9; b_index++) {
    iobj_1->NameInternal->data[b_index] = jname[b_index];
  }

  b_index = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_1->Type, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    iobj_1->Type->data[b_index] = cv7[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = iobj_1->Type->data[b_index];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    b_index = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      b_index = 1;
    } else {
      b_index = -1;
    }
  }

  emxFree_char_T(&switch_expression);
  switch (b_index) {
   case 0:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv10[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv11[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    iobj_1->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    iobj_1->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    iobj_1->HomePositionInternal->data[0] = 0.0;
  }

  iobj_0->JointInternal = iobj_1;
  iobj_0->Index = -1.0;
  iobj_0->ParentIndex = -1.0;
  iobj_0->MassInternal = 1.0;
  iobj_0->CenterOfMassInternal[0] = 0.0;
  iobj_0->CenterOfMassInternal[1] = 0.0;
  iobj_0->CenterOfMassInternal[2] = 0.0;
  for (b_index = 0; b_index < 9; b_index++) {
    b_I[b_index] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_index = 0; b_index < 9; b_index++) {
    iobj_0->InertiaInternal[b_index] = b_I[b_index];
  }

  for (b_index = 0; b_index < 36; b_index++) {
    msubspace_data[b_index] = 0;
  }

  for (b_index = 0; b_index < 6; b_index++) {
    msubspace_data[b_index + 6 * b_index] = 1;
  }

  for (b_index = 0; b_index < 36; b_index++) {
    iobj_0->SpatialInertia[b_index] = msubspace_data[b_index];
  }

  b_default = b_RigidBodyTree_RigidBodyTree(iobj_2, iobj_3);
  b_obj->BodyInternal = iobj_0;
  b_obj->TreeInternal = b_default;
  return b_obj;
}

/*
 * Arguments    : robotics_RigidBody *obj
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                f_robotics_manip_internal_Rigid *iobj_2
 *                e_robotics_manip_internal_Rigid *iobj_3
 * Return Type  : robotics_RigidBody *
 */
robotics_RigidBody *c_RigidBody_RigidBody(robotics_RigidBody *obj,
  c_robotics_manip_internal_Rigid *iobj_0, robotics_Joint *iobj_1,
  f_robotics_manip_internal_Rigid *iobj_2, e_robotics_manip_internal_Rigid
  *iobj_3)
{
  robotics_RigidBody *b_obj;
  int b_index;
  static const char bname[5] = { 'b', 'o', 'd', 'y', '2' };

  static const char jname[9] = { 'b', 'o', 'd', 'y', '2', '_', 'j', 'n', 't' };

  emxArray_char_T *switch_expression;
  static const char cv9[5] = { 'f', 'i', 'x', 'e', 'd' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv14[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv15[6] = { 0, 0, 0, 0, 0, 1 };

  signed char b_I[9];
  f_robotics_manip_internal_Rigid *b_default;
  b_obj = obj;
  b_index = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_0->NameInternal, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    iobj_0->NameInternal->data[b_index] = bname[b_index];
  }

  iobj_1->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 9;
  emxEnsureCapacity_char_T(iobj_1->NameInternal, b_index);
  for (b_index = 0; b_index < 9; b_index++) {
    iobj_1->NameInternal->data[b_index] = jname[b_index];
  }

  b_index = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_1->Type, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    iobj_1->Type->data[b_index] = cv9[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = iobj_1->Type->data[b_index];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    b_index = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      b_index = 1;
    } else {
      b_index = -1;
    }
  }

  emxFree_char_T(&switch_expression);
  switch (b_index) {
   case 0:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv14[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv15[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    iobj_1->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    iobj_1->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    iobj_1->HomePositionInternal->data[0] = 0.0;
  }

  iobj_0->JointInternal = iobj_1;
  iobj_0->Index = -1.0;
  iobj_0->ParentIndex = -1.0;
  iobj_0->MassInternal = 1.0;
  iobj_0->CenterOfMassInternal[0] = 0.0;
  iobj_0->CenterOfMassInternal[1] = 0.0;
  iobj_0->CenterOfMassInternal[2] = 0.0;
  for (b_index = 0; b_index < 9; b_index++) {
    b_I[b_index] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_index = 0; b_index < 9; b_index++) {
    iobj_0->InertiaInternal[b_index] = b_I[b_index];
  }

  for (b_index = 0; b_index < 36; b_index++) {
    msubspace_data[b_index] = 0;
  }

  for (b_index = 0; b_index < 6; b_index++) {
    msubspace_data[b_index + 6 * b_index] = 1;
  }

  for (b_index = 0; b_index < 36; b_index++) {
    iobj_0->SpatialInertia[b_index] = msubspace_data[b_index];
  }

  b_default = b_RigidBodyTree_RigidBodyTree(iobj_2, iobj_3);
  b_obj->BodyInternal = iobj_0;
  b_obj->TreeInternal = b_default;
  return b_obj;
}

/*
 * Arguments    : robotics_RigidBody *obj
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                f_robotics_manip_internal_Rigid *iobj_2
 *                e_robotics_manip_internal_Rigid *iobj_3
 * Return Type  : robotics_RigidBody *
 */
robotics_RigidBody *d_RigidBody_RigidBody(robotics_RigidBody *obj,
  c_robotics_manip_internal_Rigid *iobj_0, robotics_Joint *iobj_1,
  f_robotics_manip_internal_Rigid *iobj_2, e_robotics_manip_internal_Rigid
  *iobj_3)
{
  robotics_RigidBody *b_obj;
  int b_index;
  static const char bname[5] = { 'b', 'o', 'd', 'y', '3' };

  static const char jname[9] = { 'b', 'o', 'd', 'y', '3', '_', 'j', 'n', 't' };

  emxArray_char_T *switch_expression;
  static const char cv11[5] = { 'f', 'i', 'x', 'e', 'd' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv18[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv19[6] = { 0, 0, 0, 0, 0, 1 };

  signed char b_I[9];
  f_robotics_manip_internal_Rigid *b_default;
  b_obj = obj;
  b_index = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_0->NameInternal, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    iobj_0->NameInternal->data[b_index] = bname[b_index];
  }

  iobj_1->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 9;
  emxEnsureCapacity_char_T(iobj_1->NameInternal, b_index);
  for (b_index = 0; b_index < 9; b_index++) {
    iobj_1->NameInternal->data[b_index] = jname[b_index];
  }

  b_index = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_1->Type, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    iobj_1->Type->data[b_index] = cv11[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = iobj_1->Type->data[b_index];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    b_index = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      b_index = 1;
    } else {
      b_index = -1;
    }
  }

  emxFree_char_T(&switch_expression);
  switch (b_index) {
   case 0:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv18[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv19[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    iobj_1->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    iobj_1->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    iobj_1->HomePositionInternal->data[0] = 0.0;
  }

  iobj_0->JointInternal = iobj_1;
  iobj_0->Index = -1.0;
  iobj_0->ParentIndex = -1.0;
  iobj_0->MassInternal = 1.0;
  iobj_0->CenterOfMassInternal[0] = 0.0;
  iobj_0->CenterOfMassInternal[1] = 0.0;
  iobj_0->CenterOfMassInternal[2] = 0.0;
  for (b_index = 0; b_index < 9; b_index++) {
    b_I[b_index] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_index = 0; b_index < 9; b_index++) {
    iobj_0->InertiaInternal[b_index] = b_I[b_index];
  }

  for (b_index = 0; b_index < 36; b_index++) {
    msubspace_data[b_index] = 0;
  }

  for (b_index = 0; b_index < 6; b_index++) {
    msubspace_data[b_index + 6 * b_index] = 1;
  }

  for (b_index = 0; b_index < 36; b_index++) {
    iobj_0->SpatialInertia[b_index] = msubspace_data[b_index];
  }

  b_default = b_RigidBodyTree_RigidBodyTree(iobj_2, iobj_3);
  b_obj->BodyInternal = iobj_0;
  b_obj->TreeInternal = b_default;
  return b_obj;
}

/*
 * Arguments    : robotics_RigidBody *obj
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                f_robotics_manip_internal_Rigid *iobj_2
 *                e_robotics_manip_internal_Rigid *iobj_3
 * Return Type  : robotics_RigidBody *
 */
robotics_RigidBody *e_RigidBody_RigidBody(robotics_RigidBody *obj,
  c_robotics_manip_internal_Rigid *iobj_0, robotics_Joint *iobj_1,
  f_robotics_manip_internal_Rigid *iobj_2, e_robotics_manip_internal_Rigid
  *iobj_3)
{
  robotics_RigidBody *b_obj;
  int b_index;
  static const char bname[5] = { 'b', 'o', 'd', 'y', '4' };

  static const char jname[9] = { 'b', 'o', 'd', 'y', '4', '_', 'j', 'n', 't' };

  emxArray_char_T *switch_expression;
  static const char cv13[5] = { 'f', 'i', 'x', 'e', 'd' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv22[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv23[6] = { 0, 0, 0, 0, 0, 1 };

  signed char b_I[9];
  f_robotics_manip_internal_Rigid *b_default;
  b_obj = obj;
  b_index = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_0->NameInternal, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    iobj_0->NameInternal->data[b_index] = bname[b_index];
  }

  iobj_1->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 9;
  emxEnsureCapacity_char_T(iobj_1->NameInternal, b_index);
  for (b_index = 0; b_index < 9; b_index++) {
    iobj_1->NameInternal->data[b_index] = jname[b_index];
  }

  b_index = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_1->Type, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    iobj_1->Type->data[b_index] = cv13[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = iobj_1->Type->data[b_index];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    b_index = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      b_index = 1;
    } else {
      b_index = -1;
    }
  }

  emxFree_char_T(&switch_expression);
  switch (b_index) {
   case 0:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv22[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv23[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    iobj_1->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    iobj_1->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    iobj_1->HomePositionInternal->data[0] = 0.0;
  }

  iobj_0->JointInternal = iobj_1;
  iobj_0->Index = -1.0;
  iobj_0->ParentIndex = -1.0;
  iobj_0->MassInternal = 1.0;
  iobj_0->CenterOfMassInternal[0] = 0.0;
  iobj_0->CenterOfMassInternal[1] = 0.0;
  iobj_0->CenterOfMassInternal[2] = 0.0;
  for (b_index = 0; b_index < 9; b_index++) {
    b_I[b_index] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_index = 0; b_index < 9; b_index++) {
    iobj_0->InertiaInternal[b_index] = b_I[b_index];
  }

  for (b_index = 0; b_index < 36; b_index++) {
    msubspace_data[b_index] = 0;
  }

  for (b_index = 0; b_index < 6; b_index++) {
    msubspace_data[b_index + 6 * b_index] = 1;
  }

  for (b_index = 0; b_index < 36; b_index++) {
    iobj_0->SpatialInertia[b_index] = msubspace_data[b_index];
  }

  b_default = b_RigidBodyTree_RigidBodyTree(iobj_2, iobj_3);
  b_obj->BodyInternal = iobj_0;
  b_obj->TreeInternal = b_default;
  return b_obj;
}

/*
 * Arguments    : robotics_RigidBody *obj
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                f_robotics_manip_internal_Rigid *iobj_2
 *                e_robotics_manip_internal_Rigid *iobj_3
 * Return Type  : robotics_RigidBody *
 */
robotics_RigidBody *f_RigidBody_RigidBody(robotics_RigidBody *obj,
  c_robotics_manip_internal_Rigid *iobj_0, robotics_Joint *iobj_1,
  f_robotics_manip_internal_Rigid *iobj_2, e_robotics_manip_internal_Rigid
  *iobj_3)
{
  robotics_RigidBody *b_obj;
  int b_index;
  static const char jname[16] = { 'e', 'n', 'd', '_', 'e', 'f', 'f', 'e', 'c',
    't', 'o', 'r', '_', 'j', 'n', 't' };

  emxArray_char_T *switch_expression;
  static const char cv15[5] = { 'f', 'i', 'x', 'e', 'd' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv26[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv27[6] = { 0, 0, 0, 0, 0, 1 };

  signed char b_I[9];
  f_robotics_manip_internal_Rigid *b_default;
  b_obj = obj;
  b_index = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 12;
  emxEnsureCapacity_char_T(iobj_0->NameInternal, b_index);
  for (b_index = 0; b_index < 12; b_index++) {
    iobj_0->NameInternal->data[b_index] = cv1[b_index];
  }

  iobj_1->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = 16;
  emxEnsureCapacity_char_T(iobj_1->NameInternal, b_index);
  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->NameInternal->data[b_index] = jname[b_index];
  }

  b_index = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_1->Type, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    iobj_1->Type->data[b_index] = cv15[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = iobj_1->Type->data[b_index];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    b_index = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      b_index = 1;
    } else {
      b_index = -1;
    }
  }

  emxFree_char_T(&switch_expression);
  switch (b_index) {
   case 0:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv26[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv27[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_1->VelocityNumber = 1.0;
    iobj_1->PositionNumber = 1.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_1->VelocityNumber = 0.0;
    iobj_1->PositionNumber = 0.0;
    iobj_1->JointAxisInternal[0] = 0.0;
    iobj_1->JointAxisInternal[1] = 0.0;
    iobj_1->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = iobj_1->MotionSubspace->size[0] * iobj_1->MotionSubspace->size[1];
  iobj_1->MotionSubspace->size[0] = 6;
  iobj_1->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(iobj_1->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    iobj_1->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = iobj_1->PositionLimitsInternal->size[0] *
    iobj_1->PositionLimitsInternal->size[1];
  iobj_1->PositionLimitsInternal->size[0] = 1;
  iobj_1->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(iobj_1->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    iobj_1->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = iobj_1->HomePositionInternal->size[0];
  iobj_1->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(iobj_1->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    iobj_1->HomePositionInternal->data[0] = 0.0;
  }

  iobj_0->JointInternal = iobj_1;
  iobj_0->Index = -1.0;
  iobj_0->ParentIndex = -1.0;
  iobj_0->MassInternal = 1.0;
  iobj_0->CenterOfMassInternal[0] = 0.0;
  iobj_0->CenterOfMassInternal[1] = 0.0;
  iobj_0->CenterOfMassInternal[2] = 0.0;
  for (b_index = 0; b_index < 9; b_index++) {
    b_I[b_index] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_index = 0; b_index < 9; b_index++) {
    iobj_0->InertiaInternal[b_index] = b_I[b_index];
  }

  for (b_index = 0; b_index < 36; b_index++) {
    msubspace_data[b_index] = 0;
  }

  for (b_index = 0; b_index < 6; b_index++) {
    msubspace_data[b_index + 6 * b_index] = 1;
  }

  for (b_index = 0; b_index < 36; b_index++) {
    iobj_0->SpatialInertia[b_index] = msubspace_data[b_index];
  }

  b_default = b_RigidBodyTree_RigidBodyTree(iobj_2, iobj_3);
  b_obj->BodyInternal = iobj_0;
  b_obj->TreeInternal = b_default;
  return b_obj;
}

/*
 * File trailer for RigidBody1.c
 *
 * [EOF]
 */
