/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RigidBodyTree1.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include <string.h>
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/RigidBodyTree1.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/strcmp.h"
#include "IK_solver/Joint.h"
#include "IK_solver/all1.h"
#include "IK_solver/rand.h"
#include "IK_solver/RigidBody.h"
#include "IK_solver/IK_solver_data.h"

/* Function Declarations */
static void RigidBodyTree_ancestorIndices(const d_robotics_manip_internal_Rigid *
  obj, c_robotics_manip_internal_Rigid *body, emxArray_real_T *indices);
static void RigidBodyTree_clearAllBodies(d_robotics_manip_internal_Rigid *obj,
  c_robotics_manip_internal_Rigid *iobj_0, robotics_Joint *iobj_1,
  c_robotics_manip_internal_Rigid *iobj_2, robotics_Joint *iobj_3,
  c_robotics_manip_internal_Rigid *iobj_4, robotics_Joint *iobj_5,
  c_robotics_manip_internal_Rigid *iobj_6, robotics_Joint *iobj_7,
  c_robotics_manip_internal_Rigid *iobj_8, robotics_Joint *iobj_9);
static void c_RigidBodyTree_kinematicPathIn(const
  d_robotics_manip_internal_Rigid *obj, c_robotics_manip_internal_Rigid *body1,
  c_robotics_manip_internal_Rigid *body2, emxArray_real_T *indices);
static int div_s32_floor(int numerator, int denominator);

/* Function Definitions */

/*
 * Arguments    : const d_robotics_manip_internal_Rigid *obj
 *                c_robotics_manip_internal_Rigid *body
 *                emxArray_real_T *indices
 * Return Type  : void
 */
static void RigidBodyTree_ancestorIndices(const d_robotics_manip_internal_Rigid *
  obj, c_robotics_manip_internal_Rigid *body, emxArray_real_T *indices)
{
  int i32;
  int loop_ub;
  unsigned int i;
  i32 = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  indices->size[1] = (int)(obj->NumBodies + 1.0);
  emxEnsureCapacity_real_T(indices, i32);
  loop_ub = (int)(obj->NumBodies + 1.0);
  for (i32 = 0; i32 < loop_ub; i32++) {
    indices->data[i32] = 0.0;
  }

  i = 2U;
  indices->data[0] = body->Index;
  while (body->ParentIndex > 0.0) {
    body = obj->Bodies[(int)body->ParentIndex - 1];
    indices->data[(int)i - 1] = body->Index;
    i++;
  }

  if (body->Index > 0.0) {
    indices->data[(int)i - 1] = body->ParentIndex;
    i++;
  }

  i32 = indices->size[0] * indices->size[1];
  indices->size[1] = (int)(i - 1U);
  emxEnsureCapacity_real_T(indices, i32);
}

/*
 * Arguments    : d_robotics_manip_internal_Rigid *obj
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                c_robotics_manip_internal_Rigid *iobj_2
 *                robotics_Joint *iobj_3
 *                c_robotics_manip_internal_Rigid *iobj_4
 *                robotics_Joint *iobj_5
 *                c_robotics_manip_internal_Rigid *iobj_6
 *                robotics_Joint *iobj_7
 *                c_robotics_manip_internal_Rigid *iobj_8
 *                robotics_Joint *iobj_9
 * Return Type  : void
 */
static void RigidBodyTree_clearAllBodies(d_robotics_manip_internal_Rigid *obj,
  c_robotics_manip_internal_Rigid *iobj_0, robotics_Joint *iobj_1,
  c_robotics_manip_internal_Rigid *iobj_2, robotics_Joint *iobj_3,
  c_robotics_manip_internal_Rigid *iobj_4, robotics_Joint *iobj_5,
  c_robotics_manip_internal_Rigid *iobj_6, robotics_Joint *iobj_7,
  c_robotics_manip_internal_Rigid *iobj_8, robotics_Joint *iobj_9)
{
  int k;
  static const char bname[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x01' };

  signed char b_I[9];
  signed char c_I[36];
  static const char b_bname[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x03' };

  static const char jname[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x03', '_', 'j', 'n', 't' };

  emxArray_char_T *switch_expression;
  static const char cv4[5] = { 'f', 'i', 'x', 'e', 'd' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv3[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv4[6] = { 0, 0, 0, 0, 0, 1 };

  static const char c_bname[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x04' };

  static const char b_jname[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x04', '_', 'j', 'n', 't' };

  static const char d_bname[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x05' };

  static const char c_jname[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x05', '_', 'j', 'n', 't' };

  double unusedExpr[5];
  static const signed char iv5[10] = { 0, 0, 0, 0, 0, -1, -1, -1, -1, -1 };

  k = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 10;
  emxEnsureCapacity_char_T(iobj_0->NameInternal, k);
  for (k = 0; k < 10; k++) {
    iobj_0->NameInternal->data[k] = bname[k];
  }

  iobj_0->JointInternal = b_Joint_Joint(iobj_1);
  iobj_0->Index = -1.0;
  iobj_0->ParentIndex = -1.0;
  iobj_0->MassInternal = 1.0;
  iobj_0->CenterOfMassInternal[0] = 0.0;
  iobj_0->CenterOfMassInternal[1] = 0.0;
  iobj_0->CenterOfMassInternal[2] = 0.0;
  for (k = 0; k < 9; k++) {
    b_I[k] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (k = 0; k < 9; k++) {
    iobj_0->InertiaInternal[k] = b_I[k];
  }

  for (k = 0; k < 36; k++) {
    c_I[k] = 0;
  }

  for (k = 0; k < 6; k++) {
    c_I[k + 6 * k] = 1;
  }

  for (k = 0; k < 36; k++) {
    iobj_0->SpatialInertia[k] = c_I[k];
  }

  obj->Bodies[0] = iobj_0;
  obj->Bodies[1] = RigidBody_RigidBody(iobj_2, iobj_3);
  k = iobj_4->NameInternal->size[0] * iobj_4->NameInternal->size[1];
  iobj_4->NameInternal->size[0] = 1;
  iobj_4->NameInternal->size[1] = 10;
  emxEnsureCapacity_char_T(iobj_4->NameInternal, k);
  for (k = 0; k < 10; k++) {
    iobj_4->NameInternal->data[k] = b_bname[k];
  }

  iobj_5->InTree = false;
  for (k = 0; k < 16; k++) {
    iobj_5->JointToParentTransform[k] = iv0[k];
  }

  for (k = 0; k < 16; k++) {
    iobj_5->ChildToJointTransform[k] = iv0[k];
  }

  k = iobj_5->NameInternal->size[0] * iobj_5->NameInternal->size[1];
  iobj_5->NameInternal->size[0] = 1;
  iobj_5->NameInternal->size[1] = 14;
  emxEnsureCapacity_char_T(iobj_5->NameInternal, k);
  for (k = 0; k < 14; k++) {
    iobj_5->NameInternal->data[k] = jname[k];
  }

  k = iobj_5->Type->size[0] * iobj_5->Type->size[1];
  iobj_5->Type->size[0] = 1;
  iobj_5->Type->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_5->Type, k);
  for (k = 0; k < 5; k++) {
    iobj_5->Type->data[k] = cv4[k];
  }

  emxInit_char_T(&switch_expression, 2);
  k = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_5->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, k);
  loop_ub = iobj_5->Type->size[0] * iobj_5->Type->size[1];
  for (k = 0; k < loop_ub; k++) {
    switch_expression->data[k] = iobj_5->Type->data[k];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    k = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      k = 1;
    } else {
      k = -1;
    }
  }

  switch (k) {
   case 0:
    for (k = 0; k < 6; k++) {
      msubspace_data[k] = iv3[k];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_5->VelocityNumber = 1.0;
    iobj_5->PositionNumber = 1.0;
    iobj_5->JointAxisInternal[0] = 0.0;
    iobj_5->JointAxisInternal[1] = 0.0;
    iobj_5->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (k = 0; k < 6; k++) {
      msubspace_data[k] = iv4[k];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_5->VelocityNumber = 1.0;
    iobj_5->PositionNumber = 1.0;
    iobj_5->JointAxisInternal[0] = 0.0;
    iobj_5->JointAxisInternal[1] = 0.0;
    iobj_5->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (k = 0; k < 6; k++) {
      msubspace_data[k] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_5->VelocityNumber = 0.0;
    iobj_5->PositionNumber = 0.0;
    iobj_5->JointAxisInternal[0] = 0.0;
    iobj_5->JointAxisInternal[1] = 0.0;
    iobj_5->JointAxisInternal[2] = 0.0;
    break;
  }

  k = iobj_5->MotionSubspace->size[0] * iobj_5->MotionSubspace->size[1];
  iobj_5->MotionSubspace->size[0] = 6;
  iobj_5->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(iobj_5->MotionSubspace, k);
  for (k = 0; k < 6; k++) {
    iobj_5->MotionSubspace->data[k] = msubspace_data[k];
  }

  k = iobj_5->PositionLimitsInternal->size[0] * iobj_5->
    PositionLimitsInternal->size[1];
  iobj_5->PositionLimitsInternal->size[0] = 1;
  iobj_5->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(iobj_5->PositionLimitsInternal, k);
  for (k = 0; k < 2; k++) {
    iobj_5->PositionLimitsInternal->data[k] = poslim_data[k];
  }

  k = iobj_5->HomePositionInternal->size[0];
  iobj_5->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(iobj_5->HomePositionInternal, k);
  for (k = 0; k < 1; k++) {
    iobj_5->HomePositionInternal->data[0] = 0.0;
  }

  iobj_4->JointInternal = iobj_5;
  iobj_4->Index = -1.0;
  iobj_4->ParentIndex = -1.0;
  iobj_4->MassInternal = 1.0;
  iobj_4->CenterOfMassInternal[0] = 0.0;
  iobj_4->CenterOfMassInternal[1] = 0.0;
  iobj_4->CenterOfMassInternal[2] = 0.0;
  for (k = 0; k < 9; k++) {
    b_I[k] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (k = 0; k < 9; k++) {
    iobj_4->InertiaInternal[k] = b_I[k];
  }

  for (k = 0; k < 36; k++) {
    c_I[k] = 0;
  }

  for (k = 0; k < 6; k++) {
    c_I[k + 6 * k] = 1;
  }

  for (k = 0; k < 36; k++) {
    iobj_4->SpatialInertia[k] = c_I[k];
  }

  obj->Bodies[2] = iobj_4;
  k = iobj_6->NameInternal->size[0] * iobj_6->NameInternal->size[1];
  iobj_6->NameInternal->size[0] = 1;
  iobj_6->NameInternal->size[1] = 10;
  emxEnsureCapacity_char_T(iobj_6->NameInternal, k);
  for (k = 0; k < 10; k++) {
    iobj_6->NameInternal->data[k] = c_bname[k];
  }

  iobj_7->InTree = false;
  for (k = 0; k < 16; k++) {
    iobj_7->JointToParentTransform[k] = iv0[k];
  }

  for (k = 0; k < 16; k++) {
    iobj_7->ChildToJointTransform[k] = iv0[k];
  }

  k = iobj_7->NameInternal->size[0] * iobj_7->NameInternal->size[1];
  iobj_7->NameInternal->size[0] = 1;
  iobj_7->NameInternal->size[1] = 14;
  emxEnsureCapacity_char_T(iobj_7->NameInternal, k);
  for (k = 0; k < 14; k++) {
    iobj_7->NameInternal->data[k] = b_jname[k];
  }

  k = iobj_7->Type->size[0] * iobj_7->Type->size[1];
  iobj_7->Type->size[0] = 1;
  iobj_7->Type->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_7->Type, k);
  for (k = 0; k < 5; k++) {
    iobj_7->Type->data[k] = cv4[k];
  }

  k = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_7->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, k);
  loop_ub = iobj_7->Type->size[0] * iobj_7->Type->size[1];
  for (k = 0; k < loop_ub; k++) {
    switch_expression->data[k] = iobj_7->Type->data[k];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    k = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      k = 1;
    } else {
      k = -1;
    }
  }

  switch (k) {
   case 0:
    for (k = 0; k < 6; k++) {
      msubspace_data[k] = iv3[k];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_7->VelocityNumber = 1.0;
    iobj_7->PositionNumber = 1.0;
    iobj_7->JointAxisInternal[0] = 0.0;
    iobj_7->JointAxisInternal[1] = 0.0;
    iobj_7->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (k = 0; k < 6; k++) {
      msubspace_data[k] = iv4[k];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_7->VelocityNumber = 1.0;
    iobj_7->PositionNumber = 1.0;
    iobj_7->JointAxisInternal[0] = 0.0;
    iobj_7->JointAxisInternal[1] = 0.0;
    iobj_7->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (k = 0; k < 6; k++) {
      msubspace_data[k] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_7->VelocityNumber = 0.0;
    iobj_7->PositionNumber = 0.0;
    iobj_7->JointAxisInternal[0] = 0.0;
    iobj_7->JointAxisInternal[1] = 0.0;
    iobj_7->JointAxisInternal[2] = 0.0;
    break;
  }

  k = iobj_7->MotionSubspace->size[0] * iobj_7->MotionSubspace->size[1];
  iobj_7->MotionSubspace->size[0] = 6;
  iobj_7->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(iobj_7->MotionSubspace, k);
  for (k = 0; k < 6; k++) {
    iobj_7->MotionSubspace->data[k] = msubspace_data[k];
  }

  k = iobj_7->PositionLimitsInternal->size[0] * iobj_7->
    PositionLimitsInternal->size[1];
  iobj_7->PositionLimitsInternal->size[0] = 1;
  iobj_7->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(iobj_7->PositionLimitsInternal, k);
  for (k = 0; k < 2; k++) {
    iobj_7->PositionLimitsInternal->data[k] = poslim_data[k];
  }

  k = iobj_7->HomePositionInternal->size[0];
  iobj_7->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(iobj_7->HomePositionInternal, k);
  for (k = 0; k < 1; k++) {
    iobj_7->HomePositionInternal->data[0] = 0.0;
  }

  iobj_6->JointInternal = iobj_7;
  iobj_6->Index = -1.0;
  iobj_6->ParentIndex = -1.0;
  iobj_6->MassInternal = 1.0;
  iobj_6->CenterOfMassInternal[0] = 0.0;
  iobj_6->CenterOfMassInternal[1] = 0.0;
  iobj_6->CenterOfMassInternal[2] = 0.0;
  for (k = 0; k < 9; k++) {
    b_I[k] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (k = 0; k < 9; k++) {
    iobj_6->InertiaInternal[k] = b_I[k];
  }

  for (k = 0; k < 36; k++) {
    c_I[k] = 0;
  }

  for (k = 0; k < 6; k++) {
    c_I[k + 6 * k] = 1;
  }

  for (k = 0; k < 36; k++) {
    iobj_6->SpatialInertia[k] = c_I[k];
  }

  obj->Bodies[3] = iobj_6;
  k = iobj_8->NameInternal->size[0] * iobj_8->NameInternal->size[1];
  iobj_8->NameInternal->size[0] = 1;
  iobj_8->NameInternal->size[1] = 10;
  emxEnsureCapacity_char_T(iobj_8->NameInternal, k);
  for (k = 0; k < 10; k++) {
    iobj_8->NameInternal->data[k] = d_bname[k];
  }

  iobj_9->InTree = false;
  for (k = 0; k < 16; k++) {
    iobj_9->JointToParentTransform[k] = iv0[k];
  }

  for (k = 0; k < 16; k++) {
    iobj_9->ChildToJointTransform[k] = iv0[k];
  }

  k = iobj_9->NameInternal->size[0] * iobj_9->NameInternal->size[1];
  iobj_9->NameInternal->size[0] = 1;
  iobj_9->NameInternal->size[1] = 14;
  emxEnsureCapacity_char_T(iobj_9->NameInternal, k);
  for (k = 0; k < 14; k++) {
    iobj_9->NameInternal->data[k] = c_jname[k];
  }

  k = iobj_9->Type->size[0] * iobj_9->Type->size[1];
  iobj_9->Type->size[0] = 1;
  iobj_9->Type->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_9->Type, k);
  for (k = 0; k < 5; k++) {
    iobj_9->Type->data[k] = cv4[k];
  }

  k = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_9->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, k);
  loop_ub = iobj_9->Type->size[0] * iobj_9->Type->size[1];
  for (k = 0; k < loop_ub; k++) {
    switch_expression->data[k] = iobj_9->Type->data[k];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    k = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      k = 1;
    } else {
      k = -1;
    }
  }

  emxFree_char_T(&switch_expression);
  switch (k) {
   case 0:
    for (k = 0; k < 6; k++) {
      msubspace_data[k] = iv3[k];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_9->VelocityNumber = 1.0;
    iobj_9->PositionNumber = 1.0;
    iobj_9->JointAxisInternal[0] = 0.0;
    iobj_9->JointAxisInternal[1] = 0.0;
    iobj_9->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (k = 0; k < 6; k++) {
      msubspace_data[k] = iv4[k];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_9->VelocityNumber = 1.0;
    iobj_9->PositionNumber = 1.0;
    iobj_9->JointAxisInternal[0] = 0.0;
    iobj_9->JointAxisInternal[1] = 0.0;
    iobj_9->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (k = 0; k < 6; k++) {
      msubspace_data[k] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_9->VelocityNumber = 0.0;
    iobj_9->PositionNumber = 0.0;
    iobj_9->JointAxisInternal[0] = 0.0;
    iobj_9->JointAxisInternal[1] = 0.0;
    iobj_9->JointAxisInternal[2] = 0.0;
    break;
  }

  k = iobj_9->MotionSubspace->size[0] * iobj_9->MotionSubspace->size[1];
  iobj_9->MotionSubspace->size[0] = 6;
  iobj_9->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(iobj_9->MotionSubspace, k);
  for (k = 0; k < 6; k++) {
    iobj_9->MotionSubspace->data[k] = msubspace_data[k];
  }

  k = iobj_9->PositionLimitsInternal->size[0] * iobj_9->
    PositionLimitsInternal->size[1];
  iobj_9->PositionLimitsInternal->size[0] = 1;
  iobj_9->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(iobj_9->PositionLimitsInternal, k);
  for (k = 0; k < 2; k++) {
    iobj_9->PositionLimitsInternal->data[k] = poslim_data[k];
  }

  k = iobj_9->HomePositionInternal->size[0];
  iobj_9->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(iobj_9->HomePositionInternal, k);
  for (k = 0; k < 1; k++) {
    iobj_9->HomePositionInternal->data[0] = 0.0;
  }

  iobj_8->JointInternal = iobj_9;
  iobj_8->Index = -1.0;
  iobj_8->ParentIndex = -1.0;
  iobj_8->MassInternal = 1.0;
  iobj_8->CenterOfMassInternal[0] = 0.0;
  iobj_8->CenterOfMassInternal[1] = 0.0;
  iobj_8->CenterOfMassInternal[2] = 0.0;
  for (k = 0; k < 9; k++) {
    b_I[k] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (k = 0; k < 9; k++) {
    iobj_8->InertiaInternal[k] = b_I[k];
  }

  for (k = 0; k < 36; k++) {
    c_I[k] = 0;
  }

  for (k = 0; k < 6; k++) {
    c_I[k + 6 * k] = 1;
  }

  for (k = 0; k < 36; k++) {
    iobj_8->SpatialInertia[k] = c_I[k];
  }

  obj->Bodies[4] = iobj_8;
  obj->NumBodies = 0.0;
  obj->NumNonFixedBodies = 0.0;
  obj->PositionNumber = 0.0;
  obj->VelocityNumber = 0.0;
  b_rand(unusedExpr);
  for (k = 0; k < 10; k++) {
    obj->PositionDoFMap[k] = iv5[k];
  }

  for (k = 0; k < 10; k++) {
    obj->VelocityDoFMap[k] = iv5[k];
  }
}

/*
 * Arguments    : const d_robotics_manip_internal_Rigid *obj
 *                c_robotics_manip_internal_Rigid *body1
 *                c_robotics_manip_internal_Rigid *body2
 *                emxArray_real_T *indices
 * Return Type  : void
 */
static void c_RigidBodyTree_kinematicPathIn(const
  d_robotics_manip_internal_Rigid *obj, c_robotics_manip_internal_Rigid *body1,
  c_robotics_manip_internal_Rigid *body2, emxArray_real_T *indices)
{
  emxArray_real_T *ancestorIndices1;
  emxArray_real_T *ancestorIndices2;
  int u0;
  int minPathLength;
  boolean_T exitg1;
  double commonAncestorIndex;
  int i30;
  int loop_ub;
  int i31;
  int b_loop_ub;
  emxInit_real_T(&ancestorIndices1, 2);
  emxInit_real_T(&ancestorIndices2, 2);
  RigidBodyTree_ancestorIndices(obj, body1, ancestorIndices1);
  RigidBodyTree_ancestorIndices(obj, body2, ancestorIndices2);
  u0 = ancestorIndices1->size[1];
  minPathLength = ancestorIndices2->size[1];
  if (u0 < minPathLength) {
    minPathLength = u0;
  }

  u0 = 2;
  exitg1 = false;
  while ((!exitg1) && (u0 - 2 <= minPathLength - 2)) {
    if (ancestorIndices1->data[ancestorIndices1->size[1] - u0] !=
        ancestorIndices2->data[ancestorIndices2->size[1] - u0]) {
      minPathLength = u0 - 1;
      exitg1 = true;
    } else {
      u0++;
    }
  }

  commonAncestorIndex = ancestorIndices1->data[ancestorIndices1->size[1] -
    minPathLength];
  i30 = ancestorIndices1->size[1] - minPathLength;
  if (1 > i30) {
    loop_ub = 0;
  } else {
    loop_ub = i30;
  }

  i30 = ancestorIndices2->size[1] - minPathLength;
  if (1 > i30) {
    i30 = 1;
    u0 = 1;
    minPathLength = 0;
  } else {
    u0 = -1;
    minPathLength = 1;
  }

  i31 = indices->size[0] * indices->size[1];
  indices->size[0] = 1;
  b_loop_ub = div_s32_floor(minPathLength - i30, u0);
  indices->size[1] = (loop_ub + b_loop_ub) + 2;
  emxEnsureCapacity_real_T(indices, i31);
  for (minPathLength = 0; minPathLength < loop_ub; minPathLength++) {
    indices->data[minPathLength] = ancestorIndices1->data[minPathLength];
  }

  emxFree_real_T(&ancestorIndices1);
  indices->data[loop_ub] = commonAncestorIndex;
  for (minPathLength = 0; minPathLength <= b_loop_ub; minPathLength++) {
    indices->data[(minPathLength + loop_ub) + 1] = ancestorIndices2->data[(i30 +
      u0 * minPathLength) - 1];
  }

  emxFree_real_T(&ancestorIndices2);
}

/*
 * Arguments    : int numerator
 *                int denominator
 * Return Type  : int
 */
static int div_s32_floor(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  unsigned int tempAbsQuotient;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    if (numerator < 0) {
      absNumerator = ~(unsigned int)numerator + 1U;
    } else {
      absNumerator = (unsigned int)numerator;
    }

    if (denominator < 0) {
      absDenominator = ~(unsigned int)denominator + 1U;
    } else {
      absDenominator = (unsigned int)denominator;
    }

    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation) {
      absNumerator %= absDenominator;
      if (absNumerator > 0U) {
        tempAbsQuotient++;
      }

      quotient = -(int)tempAbsQuotient;
    } else {
      quotient = (int)tempAbsQuotient;
    }
  }

  return quotient;
}

/*
 * Arguments    : d_robotics_manip_internal_Rigid *obj
 *                robotics_Joint *iobj_0
 *                c_robotics_manip_internal_Rigid *iobj_1
 *                c_robotics_manip_internal_Rigid *iobj_2
 *                c_robotics_manip_internal_Rigid *iobj_3
 *                c_robotics_manip_internal_Rigid *iobj_4
 *                c_robotics_manip_internal_Rigid *iobj_5
 *                robotics_Joint *iobj_6
 *                robotics_Joint *iobj_7
 *                robotics_Joint *iobj_8
 *                robotics_Joint *iobj_9
 *                robotics_Joint *iobj_10
 * Return Type  : d_robotics_manip_internal_Rigid *
 */
d_robotics_manip_internal_Rigid *RigidBodyTree_RigidBodyTree
  (d_robotics_manip_internal_Rigid *obj, robotics_Joint *iobj_0,
   c_robotics_manip_internal_Rigid *iobj_1, c_robotics_manip_internal_Rigid
   *iobj_2, c_robotics_manip_internal_Rigid *iobj_3,
   c_robotics_manip_internal_Rigid *iobj_4, c_robotics_manip_internal_Rigid
   *iobj_5, robotics_Joint *iobj_6, robotics_Joint *iobj_7, robotics_Joint
   *iobj_8, robotics_Joint *iobj_9, robotics_Joint *iobj_10)
{
  d_robotics_manip_internal_Rigid *b_obj;
  int k;
  signed char b_I[9];
  signed char c_I[36];
  double unusedExpr[5];
  b_obj = obj;
  k = b_obj->Base.NameInternal->size[0] * b_obj->Base.NameInternal->size[1];
  b_obj->Base.NameInternal->size[0] = 1;
  b_obj->Base.NameInternal->size[1] = 4;
  emxEnsureCapacity_char_T(b_obj->Base.NameInternal, k);
  b_obj->Base.NameInternal->data[0] = 'b';
  b_obj->Base.NameInternal->data[1] = 'a';
  b_obj->Base.NameInternal->data[2] = 's';
  b_obj->Base.NameInternal->data[3] = 'e';
  b_obj->Base.JointInternal = Joint_Joint(iobj_0);
  b_obj->Base.Index = -1.0;
  b_obj->Base.ParentIndex = -1.0;
  b_obj->Base.MassInternal = 1.0;
  b_obj->Base.CenterOfMassInternal[0] = 0.0;
  b_obj->Base.CenterOfMassInternal[1] = 0.0;
  b_obj->Base.CenterOfMassInternal[2] = 0.0;
  for (k = 0; k < 9; k++) {
    b_I[k] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (k = 0; k < 9; k++) {
    b_obj->Base.InertiaInternal[k] = b_I[k];
  }

  for (k = 0; k < 36; k++) {
    c_I[k] = 0;
  }

  for (k = 0; k < 6; k++) {
    c_I[k + 6 * k] = 1;
  }

  for (k = 0; k < 36; k++) {
    b_obj->Base.SpatialInertia[k] = c_I[k];
  }

  b_obj->Base.Index = 0.0;
  b_rand(unusedExpr);
  b_obj->Gravity[0] = 0.0;
  b_obj->Gravity[1] = 0.0;
  b_obj->Gravity[2] = 0.0;
  RigidBodyTree_clearAllBodies(b_obj, iobj_1, iobj_6, iobj_2, iobj_7, iobj_3,
    iobj_8, iobj_4, iobj_9, iobj_5, iobj_10);
  return b_obj;
}

/*
 * Arguments    : f_robotics_manip_internal_Rigid *obj
 *                e_robotics_manip_internal_Rigid *iobj_0
 * Return Type  : f_robotics_manip_internal_Rigid *
 */
f_robotics_manip_internal_Rigid *b_RigidBodyTree_RigidBodyTree
  (f_robotics_manip_internal_Rigid *obj, e_robotics_manip_internal_Rigid *iobj_0)
{
  f_robotics_manip_internal_Rigid *b_obj;
  double unusedExpr[5];
  double b_unusedExpr[5];
  b_obj = obj;
  Joint_Joint(&b_obj->Base.JointInternal);
  b_rand(unusedExpr);
  b_Joint_Joint(&iobj_0->JointInternal);
  b_obj->Bodies[0] = iobj_0;
  b_rand(b_unusedExpr);
  return b_obj;
}

/*
 * Arguments    : d_robotics_manip_internal_Rigid *obj
 *                const emxArray_real_T *qv
 *                const emxArray_char_T *body1Name
 *                double T_data[]
 *                int T_size[2]
 *                emxArray_real_T *Jac
 * Return Type  : void
 */
void c_RigidBodyTree_efficientFKAndJ(d_robotics_manip_internal_Rigid *obj, const
  emxArray_real_T *qv, const emxArray_char_T *body1Name, double T_data[], int
  T_size[2], emxArray_real_T *Jac)
{
  emxArray_char_T *body2Name;
  int i28;
  int n;
  double bid1;
  double bid2;
  c_robotics_manip_internal_Rigid *body1;
  c_robotics_manip_internal_Rigid *body2;
  emxArray_real_T *kinematicPathIndices;
  double T1[16];
  emxArray_real_T *y;
  emxArray_real_T *b;
  emxArray_real_T *b_qv;
  int i;
  int i29;
  double X[36];
  boolean_T nextBodyIsParent;
  int jointSign;
  robotics_Joint *joint;
  double Tc2p[16];
  double qidx_idx_1;
  int coffset;
  double R[9];
  double Tj1[16];
  int Tj1_tmp;
  double b_R[9];
  int boffset;
  int k;
  int aoffset;
  double T1j[16];
  double c_R[3];
  int b_i;
  emxInit_char_T(&body2Name, 2);
  i28 = body2Name->size[0] * body2Name->size[1];
  body2Name->size[0] = 1;
  body2Name->size[1] = obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(body2Name, i28);
  n = obj->Base.NameInternal->size[0] * obj->Base.NameInternal->size[1];
  for (i28 = 0; i28 < n; i28++) {
    body2Name->data[i28] = obj->Base.NameInternal->data[i28];
  }

  bid1 = c_RigidBodyTree_validateInputBo(obj, body1Name);
  bid2 = c_RigidBodyTree_validateInputBo(obj, body2Name);
  emxFree_char_T(&body2Name);
  if (bid1 == 0.0) {
    body1 = &obj->Base;
  } else {
    body1 = obj->Bodies[(int)bid1 - 1];
  }

  if (bid2 == 0.0) {
    body2 = &obj->Base;
  } else {
    body2 = obj->Bodies[(int)bid2 - 1];
  }

  emxInit_real_T(&kinematicPathIndices, 2);
  c_RigidBodyTree_kinematicPathIn(obj, body1, body2, kinematicPathIndices);
  memset(&T1[0], 0, sizeof(double) << 4);
  T1[0] = 1.0;
  T1[5] = 1.0;
  T1[10] = 1.0;
  T1[15] = 1.0;
  i28 = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = (int)obj->PositionNumber;
  emxEnsureCapacity_real_T(Jac, i28);
  n = 6 * (int)obj->PositionNumber;
  for (i28 = 0; i28 < n; i28++) {
    Jac->data[i28] = 0.0;
  }

  i28 = kinematicPathIndices->size[1];
  emxInit_real_T(&y, 2);
  emxInit_real_T(&b, 2);
  emxInit_real_T(&b_qv, 1);
  for (i = 0; i <= i28 - 2; i++) {
    if (kinematicPathIndices->data[i] != 0.0) {
      body1 = obj->Bodies[(int)kinematicPathIndices->data[i] - 1];
    } else {
      body1 = &obj->Base;
    }

    if (kinematicPathIndices->data[i + 1] != 0.0) {
      body2 = obj->Bodies[(int)kinematicPathIndices->data[i + 1] - 1];
    } else {
      body2 = &obj->Base;
    }

    nextBodyIsParent = (body2->Index == body1->ParentIndex);
    if (nextBodyIsParent) {
      body2 = body1;
      jointSign = 1;
    } else {
      jointSign = -1;
    }

    joint = body2->JointInternal;
    if (d_strcmp(joint->Type)) {
      Joint_transformBodyToParent(joint, Tc2p);
    } else {
      n = (int)body2->Index;
      bid2 = obj->PositionDoFMap[n - 1];
      qidx_idx_1 = obj->PositionDoFMap[n + 4];
      if (bid2 > qidx_idx_1) {
        i29 = 0;
        coffset = 0;
      } else {
        i29 = (int)bid2 - 1;
        coffset = (int)qidx_idx_1;
      }

      Tj1_tmp = b_qv->size[0];
      n = coffset - i29;
      b_qv->size[0] = n;
      emxEnsureCapacity_real_T(b_qv, Tj1_tmp);
      for (coffset = 0; coffset < n; coffset++) {
        b_qv->data[coffset] = qv->data[i29 + coffset];
      }

      b_Joint_transformBodyToParent(joint, b_qv, Tc2p);
      n = (int)body2->Index;
      bid2 = obj->VelocityDoFMap[n - 1];
      qidx_idx_1 = obj->VelocityDoFMap[n + 4];
      if (nextBodyIsParent) {
        for (i29 = 0; i29 < 16; i29++) {
          Tj1[i29] = joint->ChildToJointTransform[i29];
        }
      } else {
        for (i29 = 0; i29 < 16; i29++) {
          Tj1[i29] = joint->JointToParentTransform[i29];
        }

        for (i29 = 0; i29 < 3; i29++) {
          R[3 * i29] = Tj1[i29];
          R[1 + 3 * i29] = Tj1[i29 + 4];
          R[2 + 3 * i29] = Tj1[i29 + 8];
        }

        for (i29 = 0; i29 < 9; i29++) {
          b_R[i29] = -R[i29];
        }

        for (i29 = 0; i29 < 3; i29++) {
          c_R[i29] = 0.0;
          bid1 = b_R[i29] * Tj1[12];
          Tj1_tmp = i29 << 2;
          Tj1[Tj1_tmp] = R[3 * i29];
          bid1 += b_R[i29 + 3] * Tj1[13];
          Tj1[1 + Tj1_tmp] = R[1 + 3 * i29];
          bid1 += b_R[i29 + 6] * Tj1[14];
          Tj1[2 + Tj1_tmp] = R[2 + 3 * i29];
          c_R[i29] = bid1;
        }

        Tj1[12] = c_R[0];
        Tj1[13] = c_R[1];
        Tj1[14] = c_R[2];
        Tj1[3] = 0.0;
        Tj1[7] = 0.0;
        Tj1[11] = 0.0;
        Tj1[15] = 1.0;
      }

      for (i29 = 0; i29 < 4; i29++) {
        for (coffset = 0; coffset < 4; coffset++) {
          n = coffset << 2;
          Tj1_tmp = i29 + n;
          T1j[Tj1_tmp] = 0.0;
          T1j[Tj1_tmp] = ((Tj1[i29] * T1[n] + Tj1[i29 + 4] * T1[1 + n]) +
                          Tj1[i29 + 8] * T1[2 + n]) + Tj1[i29 + 12] * T1[3 + n];
        }
      }

      for (i29 = 0; i29 < 3; i29++) {
        R[3 * i29] = T1j[i29];
        R[1 + 3 * i29] = T1j[i29 + 4];
        R[2 + 3 * i29] = T1j[i29 + 8];
      }

      for (i29 = 0; i29 < 9; i29++) {
        b_R[i29] = -R[i29];
      }

      for (i29 = 0; i29 < 3; i29++) {
        Tj1_tmp = i29 << 2;
        Tj1[Tj1_tmp] = R[3 * i29];
        Tj1[1 + Tj1_tmp] = R[1 + 3 * i29];
        Tj1[2 + Tj1_tmp] = R[2 + 3 * i29];
        Tj1[12 + i29] = (b_R[i29] * T1j[12] + b_R[i29 + 3] * T1j[13]) + b_R[i29
          + 6] * T1j[14];
      }

      Tj1[3] = 0.0;
      Tj1[7] = 0.0;
      Tj1[11] = 0.0;
      Tj1[15] = 1.0;
      R[0] = 0.0;
      R[3] = -Tj1[14];
      R[6] = Tj1[13];
      R[1] = Tj1[14];
      R[4] = 0.0;
      R[7] = -Tj1[12];
      R[2] = -Tj1[13];
      R[5] = Tj1[12];
      R[8] = 0.0;
      for (i29 = 0; i29 < 3; i29++) {
        for (coffset = 0; coffset < 3; coffset++) {
          n = i29 + 3 * coffset;
          b_R[n] = 0.0;
          Tj1_tmp = coffset << 2;
          b_R[n] = (R[i29] * Tj1[Tj1_tmp] + R[i29 + 3] * Tj1[1 + Tj1_tmp]) +
            R[i29 + 6] * Tj1[2 + Tj1_tmp];
          X[coffset + 6 * i29] = Tj1[coffset + (i29 << 2)];
          X[coffset + 6 * (i29 + 3)] = 0.0;
        }
      }

      for (i29 = 0; i29 < 3; i29++) {
        X[6 * i29 + 3] = b_R[3 * i29];
        n = i29 << 2;
        Tj1_tmp = 6 * (i29 + 3);
        X[Tj1_tmp + 3] = Tj1[n];
        X[6 * i29 + 4] = b_R[1 + 3 * i29];
        X[Tj1_tmp + 4] = Tj1[1 + n];
        X[6 * i29 + 5] = b_R[2 + 3 * i29];
        X[Tj1_tmp + 5] = Tj1[2 + n];
      }

      i29 = b->size[0] * b->size[1];
      b->size[0] = 6;
      b->size[1] = joint->MotionSubspace->size[1];
      emxEnsureCapacity_real_T(b, i29);
      n = joint->MotionSubspace->size[0] * joint->MotionSubspace->size[1];
      for (i29 = 0; i29 < n; i29++) {
        b->data[i29] = joint->MotionSubspace->data[i29];
      }

      n = b->size[1];
      i29 = y->size[0] * y->size[1];
      y->size[0] = 6;
      y->size[1] = b->size[1];
      emxEnsureCapacity_real_T(y, i29);
      for (Tj1_tmp = 0; Tj1_tmp < n; Tj1_tmp++) {
        coffset = Tj1_tmp * 6;
        boffset = Tj1_tmp * 6;
        for (b_i = 0; b_i < 6; b_i++) {
          y->data[coffset + b_i] = 0.0;
        }

        for (k = 0; k < 6; k++) {
          aoffset = k * 6;
          bid1 = b->data[boffset + k];
          for (b_i = 0; b_i < 6; b_i++) {
            i29 = coffset + b_i;
            y->data[i29] += bid1 * X[aoffset + b_i];
          }
        }
      }

      if (bid2 > qidx_idx_1) {
        i29 = 0;
      } else {
        i29 = (int)bid2 - 1;
      }

      n = y->size[1];
      for (coffset = 0; coffset < n; coffset++) {
        for (Tj1_tmp = 0; Tj1_tmp < 6; Tj1_tmp++) {
          Jac->data[Tj1_tmp + 6 * (i29 + coffset)] = y->data[Tj1_tmp + 6 *
            coffset] * (double)jointSign;
        }
      }
    }

    if (nextBodyIsParent) {
      for (i29 = 0; i29 < 4; i29++) {
        for (coffset = 0; coffset < 4; coffset++) {
          Tj1_tmp = coffset << 2;
          n = i29 + Tj1_tmp;
          Tj1[n] = 0.0;
          Tj1[n] = ((Tc2p[i29] * T1[Tj1_tmp] + Tc2p[i29 + 4] * T1[1 + Tj1_tmp])
                    + Tc2p[i29 + 8] * T1[2 + Tj1_tmp]) + Tc2p[i29 + 12] * T1[3 +
            Tj1_tmp];
        }
      }

      memcpy(&T1[0], &Tj1[0], sizeof(double) << 4);
    } else {
      for (i29 = 0; i29 < 3; i29++) {
        R[3 * i29] = Tc2p[i29];
        R[1 + 3 * i29] = Tc2p[i29 + 4];
        R[2 + 3 * i29] = Tc2p[i29 + 8];
      }

      for (i29 = 0; i29 < 9; i29++) {
        b_R[i29] = -R[i29];
      }

      for (i29 = 0; i29 < 3; i29++) {
        Tj1_tmp = i29 << 2;
        Tj1[Tj1_tmp] = R[3 * i29];
        Tj1[1 + Tj1_tmp] = R[1 + 3 * i29];
        Tj1[2 + Tj1_tmp] = R[2 + 3 * i29];
        Tj1[12 + i29] = (b_R[i29] * Tc2p[12] + b_R[i29 + 3] * Tc2p[13]) +
          b_R[i29 + 6] * Tc2p[14];
      }

      Tj1[3] = 0.0;
      Tj1[7] = 0.0;
      Tj1[11] = 0.0;
      Tj1[15] = 1.0;
      for (i29 = 0; i29 < 4; i29++) {
        for (coffset = 0; coffset < 4; coffset++) {
          n = coffset << 2;
          Tj1_tmp = i29 + n;
          T1j[Tj1_tmp] = 0.0;
          T1j[Tj1_tmp] = ((Tj1[i29] * T1[n] + Tj1[i29 + 4] * T1[1 + n]) +
                          Tj1[i29 + 8] * T1[2 + n]) + Tj1[i29 + 12] * T1[3 + n];
        }
      }

      memcpy(&T1[0], &T1j[0], sizeof(double) << 4);
    }
  }

  emxFree_real_T(&b_qv);
  emxFree_real_T(&y);
  emxFree_real_T(&kinematicPathIndices);
  for (i28 = 0; i28 < 3; i28++) {
    i29 = i28 << 2;
    X[6 * i28] = T1[i29];
    n = 6 * (i28 + 3);
    X[n] = 0.0;
    X[6 * i28 + 3] = 0.0;
    X[n + 3] = T1[i29];
    bid1 = T1[1 + i29];
    X[1 + 6 * i28] = bid1;
    X[1 + n] = 0.0;
    X[6 * i28 + 4] = 0.0;
    X[n + 4] = bid1;
    bid1 = T1[2 + i29];
    X[2 + 6 * i28] = bid1;
    X[2 + n] = 0.0;
    X[6 * i28 + 5] = 0.0;
    X[n + 5] = bid1;
  }

  i28 = b->size[0] * b->size[1];
  b->size[0] = 6;
  b->size[1] = Jac->size[1];
  emxEnsureCapacity_real_T(b, i28);
  n = Jac->size[0] * Jac->size[1];
  for (i28 = 0; i28 < n; i28++) {
    b->data[i28] = Jac->data[i28];
  }

  n = Jac->size[1];
  i28 = Jac->size[1];
  i29 = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  Jac->size[1] = i28;
  emxEnsureCapacity_real_T(Jac, i29);
  for (Tj1_tmp = 0; Tj1_tmp < n; Tj1_tmp++) {
    coffset = Tj1_tmp * 6;
    boffset = Tj1_tmp * 6;
    for (i = 0; i < 6; i++) {
      Jac->data[coffset + i] = 0.0;
    }

    for (k = 0; k < 6; k++) {
      aoffset = k * 6;
      bid1 = b->data[boffset + k];
      for (i = 0; i < 6; i++) {
        i28 = coffset + i;
        Jac->data[i28] += bid1 * X[aoffset + i];
      }
    }
  }

  emxFree_real_T(&b);
  T_size[0] = 4;
  T_size[1] = 4;
  memcpy(&T_data[0], &T1[0], sizeof(double) << 4);
}

/*
 * Arguments    : const d_robotics_manip_internal_Rigid *obj
 *                const emxArray_char_T *bodyname
 * Return Type  : double
 */
double c_RigidBodyTree_findBodyIndexBy(const d_robotics_manip_internal_Rigid
  *obj, const emxArray_char_T *bodyname)
{
  double bid;
  emxArray_char_T *bname;
  int i13;
  int loop_ub;
  double d1;
  int i;
  boolean_T exitg1;
  c_robotics_manip_internal_Rigid *b_obj;
  emxInit_char_T(&bname, 2);
  bid = -1.0;
  i13 = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, i13);
  loop_ub = obj->Base.NameInternal->size[0] * obj->Base.NameInternal->size[1];
  for (i13 = 0; i13 < loop_ub; i13++) {
    bname->data[i13] = obj->Base.NameInternal->data[i13];
  }

  if (e_strcmp(bname, bodyname)) {
    bid = 0.0;
  } else {
    d1 = obj->NumBodies;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= (int)d1 - 1)) {
      b_obj = obj->Bodies[i];
      i13 = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = b_obj->NameInternal->size[1];
      emxEnsureCapacity_char_T(bname, i13);
      loop_ub = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
      for (i13 = 0; i13 < loop_ub; i13++) {
        bname->data[i13] = b_obj->NameInternal->data[i13];
      }

      if (e_strcmp(bname, bodyname)) {
        bid = 1.0 + (double)i;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  emxFree_char_T(&bname);
  return bid;
}

/*
 * Arguments    : const d_robotics_manip_internal_Rigid *obj
 *                emxArray_real_T *limits
 * Return Type  : void
 */
void c_RigidBodyTree_get_JointPositi(const d_robotics_manip_internal_Rigid *obj,
  emxArray_real_T *limits)
{
  int i16;
  int loop_ub;
  double k;
  double pnum;
  int i;
  c_robotics_manip_internal_Rigid *body;
  int i17;
  robotics_Joint *b_obj;
  int i18;
  i16 = limits->size[0] * limits->size[1];
  limits->size[0] = (int)obj->PositionNumber;
  limits->size[1] = 2;
  emxEnsureCapacity_real_T(limits, i16);
  loop_ub = (int)obj->PositionNumber << 1;
  for (i16 = 0; i16 < loop_ub; i16++) {
    limits->data[i16] = 0.0;
  }

  k = 1.0;
  pnum = obj->NumBodies;
  i16 = (int)pnum;
  for (i = 0; i < i16; i++) {
    body = obj->Bodies[i];
    if (!d_strcmp(body->JointInternal->Type)) {
      pnum = body->JointInternal->PositionNumber;
      if (k > (k + pnum) - 1.0) {
        i17 = 0;
      } else {
        i17 = (int)k - 1;
      }

      b_obj = body->JointInternal;
      loop_ub = b_obj->PositionLimitsInternal->size[0];
      for (i18 = 0; i18 < loop_ub; i18++) {
        limits->data[i17 + i18] = b_obj->PositionLimitsInternal->data[i18];
      }

      loop_ub = b_obj->PositionLimitsInternal->size[0];
      for (i18 = 0; i18 < loop_ub; i18++) {
        limits->data[(i17 + i18) + limits->size[0]] =
          b_obj->PositionLimitsInternal->data[i18 +
          b_obj->PositionLimitsInternal->size[0]];
      }

      k += pnum;
    }
  }
}

/*
 * Arguments    : const d_robotics_manip_internal_Rigid *obj
 *                const c_struct_T Q[4]
 *                emxArray_real_T *qvec
 * Return Type  : void
 */
void c_RigidBodyTree_validateConfigu(const d_robotics_manip_internal_Rigid *obj,
  const c_struct_T Q[4], emxArray_real_T *qvec)
{
  int i21;
  int loop_ub;
  emxArray_int32_T *r6;
  emxArray_char_T *nm;
  int i;
  c_robotics_manip_internal_Rigid *body;
  emxArray_real_T *limits;
  robotics_Joint *jnt;
  emxArray_boolean_T *ubOK;
  int idx;
  int nx;
  boolean_T exitg1;
  double idx_idx_0;
  int i22;
  double idx_idx_1;
  emxArray_boolean_T *lbOK;
  int kstr;
  boolean_T b_bool;
  int exitg2;
  emxArray_int32_T *ii;
  emxArray_uint32_T *indicesUpperBoundViolation;
  i21 = qvec->size[0];
  loop_ub = (int)obj->PositionNumber;
  qvec->size[0] = loop_ub;
  emxEnsureCapacity_real_T(qvec, i21);
  for (i21 = 0; i21 < loop_ub; i21++) {
    qvec->data[i21] = 0.0;
  }

  i21 = (int)obj->NumBodies;
  emxInit_int32_T(&r6, 2);
  emxInit_char_T(&nm, 2);
  for (i = 0; i < i21; i++) {
    body = obj->Bodies[i];
    if (!d_strcmp(body->JointInternal->Type)) {
      jnt = body->JointInternal;
      idx = -2;
      nx = 0;
      exitg1 = false;
      while ((!exitg1) && (nx < 4)) {
        i22 = nm->size[0] * nm->size[1];
        nm->size[0] = 1;
        nm->size[1] = jnt->NameInternal->size[1];
        emxEnsureCapacity_char_T(nm, i22);
        loop_ub = jnt->NameInternal->size[0] * jnt->NameInternal->size[1];
        for (i22 = 0; i22 < loop_ub; i22++) {
          nm->data[i22] = jnt->NameInternal->data[i22];
        }

        b_bool = false;
        if (4 == nm->size[1]) {
          kstr = 0;
          do {
            exitg2 = 0;
            if (kstr < 4) {
              if (Q[nx].JointName[kstr] != nm->data[kstr]) {
                exitg2 = 1;
              } else {
                kstr++;
              }
            } else {
              b_bool = true;
              exitg2 = 1;
            }
          } while (exitg2 == 0);
        }

        if (b_bool) {
          idx = nx;
          exitg1 = true;
        } else {
          nx++;
        }
      }

      idx_idx_0 = obj->PositionDoFMap[i];
      idx_idx_1 = obj->PositionDoFMap[i + 5];
      if (idx_idx_0 > idx_idx_1) {
        i22 = 0;
        nx = 0;
      } else {
        i22 = (int)idx_idx_0 - 1;
        nx = (int)idx_idx_1;
      }

      kstr = r6->size[0] * r6->size[1];
      r6->size[0] = 1;
      loop_ub = nx - i22;
      r6->size[1] = loop_ub;
      emxEnsureCapacity_int32_T(r6, kstr);
      for (nx = 0; nx < loop_ub; nx++) {
        r6->data[nx] = i22 + nx;
      }

      loop_ub = r6->size[0] * r6->size[1];
      for (i22 = 0; i22 < loop_ub; i22++) {
        qvec->data[r6->data[i22]] = Q[idx].JointPosition;
      }
    }
  }

  emxFree_char_T(&nm);
  emxFree_int32_T(&r6);
  emxInit_real_T(&limits, 2);
  emxInit_boolean_T(&ubOK, 1);
  c_RigidBodyTree_get_JointPositi(obj, limits);
  i21 = ubOK->size[0];
  ubOK->size[0] = qvec->size[0];
  emxEnsureCapacity_boolean_T(ubOK, i21);
  loop_ub = qvec->size[0];
  for (i21 = 0; i21 < loop_ub; i21++) {
    ubOK->data[i21] = (qvec->data[i21] <= limits->data[i21 + limits->size[0]] +
                       4.4408920985006262E-16);
  }

  emxInit_boolean_T(&lbOK, 1);
  i21 = lbOK->size[0];
  lbOK->size[0] = qvec->size[0];
  emxEnsureCapacity_boolean_T(lbOK, i21);
  loop_ub = qvec->size[0];
  for (i21 = 0; i21 < loop_ub; i21++) {
    lbOK->data[i21] = (qvec->data[i21] >= limits->data[i21] -
                       4.4408920985006262E-16);
  }

  if (all(ubOK) && all(lbOK)) {
  } else {
    i21 = ubOK->size[0];
    emxEnsureCapacity_boolean_T(ubOK, i21);
    loop_ub = ubOK->size[0];
    for (i21 = 0; i21 < loop_ub; i21++) {
      ubOK->data[i21] = !ubOK->data[i21];
    }

    emxInit_int32_T(&ii, 1);
    nx = ubOK->size[0];
    idx = 0;
    i21 = ii->size[0];
    ii->size[0] = ubOK->size[0];
    emxEnsureCapacity_int32_T(ii, i21);
    kstr = 0;
    exitg1 = false;
    while ((!exitg1) && (kstr <= nx - 1)) {
      if (ubOK->data[kstr]) {
        idx++;
        ii->data[idx - 1] = kstr + 1;
        if (idx >= nx) {
          exitg1 = true;
        } else {
          kstr++;
        }
      } else {
        kstr++;
      }
    }

    if (ubOK->size[0] == 1) {
      if (idx == 0) {
        ii->size[0] = 0;
      }
    } else if (1 > idx) {
      ii->size[0] = 0;
    } else {
      i21 = ii->size[0];
      ii->size[0] = idx;
      emxEnsureCapacity_int32_T(ii, i21);
    }

    emxInit_uint32_T(&indicesUpperBoundViolation, 1);
    i21 = indicesUpperBoundViolation->size[0];
    indicesUpperBoundViolation->size[0] = ii->size[0];
    emxEnsureCapacity_uint32_T(indicesUpperBoundViolation, i21);
    loop_ub = ii->size[0];
    for (i21 = 0; i21 < loop_ub; i21++) {
      indicesUpperBoundViolation->data[i21] = (unsigned int)ii->data[i21];
    }

    loop_ub = indicesUpperBoundViolation->size[0];
    for (i21 = 0; i21 < loop_ub; i21++) {
      qvec->data[(int)indicesUpperBoundViolation->data[i21] - 1] = limits->data
        [((int)indicesUpperBoundViolation->data[i21] + limits->size[0]) - 1];
    }

    i21 = lbOK->size[0];
    emxEnsureCapacity_boolean_T(lbOK, i21);
    loop_ub = lbOK->size[0];
    for (i21 = 0; i21 < loop_ub; i21++) {
      lbOK->data[i21] = !lbOK->data[i21];
    }

    nx = lbOK->size[0];
    idx = 0;
    i21 = ii->size[0];
    ii->size[0] = lbOK->size[0];
    emxEnsureCapacity_int32_T(ii, i21);
    kstr = 0;
    exitg1 = false;
    while ((!exitg1) && (kstr <= nx - 1)) {
      if (lbOK->data[kstr]) {
        idx++;
        ii->data[idx - 1] = kstr + 1;
        if (idx >= nx) {
          exitg1 = true;
        } else {
          kstr++;
        }
      } else {
        kstr++;
      }
    }

    if (lbOK->size[0] == 1) {
      if (idx == 0) {
        ii->size[0] = 0;
      }
    } else if (1 > idx) {
      ii->size[0] = 0;
    } else {
      i21 = ii->size[0];
      ii->size[0] = idx;
      emxEnsureCapacity_int32_T(ii, i21);
    }

    i21 = indicesUpperBoundViolation->size[0];
    indicesUpperBoundViolation->size[0] = ii->size[0];
    emxEnsureCapacity_uint32_T(indicesUpperBoundViolation, i21);
    loop_ub = ii->size[0];
    for (i21 = 0; i21 < loop_ub; i21++) {
      indicesUpperBoundViolation->data[i21] = (unsigned int)ii->data[i21];
    }

    emxFree_int32_T(&ii);
    loop_ub = indicesUpperBoundViolation->size[0];
    for (i21 = 0; i21 < loop_ub; i21++) {
      qvec->data[(int)indicesUpperBoundViolation->data[i21] - 1] = limits->data
        [(int)indicesUpperBoundViolation->data[i21] - 1];
    }

    emxFree_uint32_T(&indicesUpperBoundViolation);
  }

  emxFree_boolean_T(&lbOK);
  emxFree_boolean_T(&ubOK);
  emxFree_real_T(&limits);
}

/*
 * Arguments    : const d_robotics_manip_internal_Rigid *obj
 *                const emxArray_char_T *bodyname
 * Return Type  : double
 */
double c_RigidBodyTree_validateInputBo(const d_robotics_manip_internal_Rigid
  *obj, const emxArray_char_T *bodyname)
{
  return c_RigidBodyTree_findBodyIndexBy(obj, bodyname);
}

/*
 * Arguments    : const d_robotics_manip_internal_Rigid *obj
 *                const emxArray_char_T *jointname
 * Return Type  : double
 */
double d_RigidBodyTree_findBodyIndexBy(const d_robotics_manip_internal_Rigid
  *obj, const emxArray_char_T *jointname)
{
  double bid;
  double d2;
  int i;
  emxArray_char_T *nm;
  boolean_T exitg1;
  robotics_Joint *jnt;
  int i14;
  int loop_ub;
  bid = -1.0;
  d2 = obj->NumBodies;
  i = 0;
  emxInit_char_T(&nm, 2);
  exitg1 = false;
  while ((!exitg1) && (i <= (int)d2 - 1)) {
    jnt = obj->Bodies[i]->JointInternal;
    i14 = nm->size[0] * nm->size[1];
    nm->size[0] = 1;
    nm->size[1] = jnt->NameInternal->size[1];
    emxEnsureCapacity_char_T(nm, i14);
    loop_ub = jnt->NameInternal->size[0] * jnt->NameInternal->size[1];
    for (i14 = 0; i14 < loop_ub; i14++) {
      nm->data[i14] = jnt->NameInternal->data[i14];
    }

    if (e_strcmp(nm, jointname)) {
      bid = 1.0 + (double)i;
      exitg1 = true;
    } else {
      i++;
    }
  }

  emxFree_char_T(&nm);
  return bid;
}

/*
 * Arguments    : d_robotics_manip_internal_Rigid *obj
 *                const c_robotics_manip_internal_Rigid *bodyin
 *                const emxArray_char_T *parentName
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                robotics_Joint *iobj_2
 * Return Type  : void
 */
void f_RigidBodyTree_addBody(d_robotics_manip_internal_Rigid *obj, const
  c_robotics_manip_internal_Rigid *bodyin, const emxArray_char_T *parentName,
  c_robotics_manip_internal_Rigid *iobj_0, robotics_Joint *iobj_1,
  robotics_Joint *iobj_2)
{
  emxArray_char_T *bname;
  int body;
  int loop_ub;
  double pid;
  robotics_Joint *jnt;
  double b_index;
  c_robotics_manip_internal_Rigid *b_body;
  emxInit_char_T(&bname, 2);
  body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = bodyin->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, body);
  loop_ub = bodyin->NameInternal->size[0] * bodyin->NameInternal->size[1];
  for (body = 0; body < loop_ub; body++) {
    bname->data[body] = bodyin->NameInternal->data[body];
  }

  c_RigidBodyTree_findBodyIndexBy(obj, bname);
  pid = c_RigidBodyTree_validateInputBo(obj, parentName);
  jnt = bodyin->JointInternal;
  body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, body);
  loop_ub = jnt->NameInternal->size[0] * jnt->NameInternal->size[1];
  for (body = 0; body < loop_ub; body++) {
    bname->data[body] = jnt->NameInternal->data[body];
  }

  d_RigidBodyTree_findBodyIndexBy(obj, bname);
  b_index = obj->NumBodies + 1.0;
  b_body = RigidBody_copy(bodyin, iobj_0, iobj_1, iobj_2);
  obj->Bodies[(int)b_index - 1] = b_body;
  b_body->Index = b_index;
  b_body->ParentIndex = pid;
  b_body->JointInternal->InTree = true;
  obj->NumBodies++;
  jnt = b_body->JointInternal;
  emxFree_char_T(&bname);
  if (!d_strcmp(jnt->Type)) {
    obj->NumNonFixedBodies++;
    jnt = b_body->JointInternal;
    body = (int)b_body->Index - 1;
    obj->PositionDoFMap[body] = obj->PositionNumber + 1.0;
    obj->PositionDoFMap[5 + body] = obj->PositionNumber + jnt->PositionNumber;
    jnt = b_body->JointInternal;
    body = (int)b_body->Index - 1;
    obj->VelocityDoFMap[body] = obj->VelocityNumber + 1.0;
    obj->VelocityDoFMap[5 + body] = obj->VelocityNumber + jnt->VelocityNumber;
  } else {
    body = (int)b_body->Index;
    obj->PositionDoFMap[body - 1] = 0.0;
    obj->PositionDoFMap[body + 4] = -1.0;
    body = (int)b_body->Index;
    obj->VelocityDoFMap[body - 1] = 0.0;
    obj->VelocityDoFMap[body + 4] = -1.0;
  }

  jnt = b_body->JointInternal;
  obj->PositionNumber += jnt->PositionNumber;
  jnt = b_body->JointInternal;
  obj->VelocityNumber += jnt->VelocityNumber;
}

/*
 * File trailer for RigidBodyTree1.c
 *
 * [EOF]
 */
