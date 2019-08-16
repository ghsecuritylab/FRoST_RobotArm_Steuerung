/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RigidBody.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/RigidBody.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/strcmp.h"
#include "IK_solver/Joint.h"
#include "IK_solver/IK_solver_data.h"

/* Function Definitions */

/*
 * Arguments    : c_robotics_manip_internal_Rigid *obj
 *                robotics_Joint *iobj_0
 * Return Type  : c_robotics_manip_internal_Rigid *
 */
c_robotics_manip_internal_Rigid *RigidBody_RigidBody
  (c_robotics_manip_internal_Rigid *obj, robotics_Joint *iobj_0)
{
  c_robotics_manip_internal_Rigid *b_obj;
  int b_index;
  static const char bname[10] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x02' };

  static const char jname[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x02', '_', 'j', 'n', 't' };

  emxArray_char_T *switch_expression;
  static const char cv6[5] = { 'f', 'i', 'x', 'e', 'd' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv8[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv9[6] = { 0, 0, 0, 0, 0, 1 };

  signed char b_I[9];
  b_obj = obj;
  b_index = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 10;
  emxEnsureCapacity_char_T(b_obj->NameInternal, b_index);
  for (b_index = 0; b_index < 10; b_index++) {
    b_obj->NameInternal->data[b_index] = bname[b_index];
  }

  iobj_0->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    iobj_0->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    iobj_0->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = 14;
  emxEnsureCapacity_char_T(iobj_0->NameInternal, b_index);
  for (b_index = 0; b_index < 14; b_index++) {
    iobj_0->NameInternal->data[b_index] = jname[b_index];
  }

  b_index = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_0->Type, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    iobj_0->Type->data[b_index] = cv6[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = iobj_0->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = iobj_0->Type->data[b_index];
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
      msubspace_data[b_index] = iv8[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv9[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    iobj_0->VelocityNumber = 1.0;
    iobj_0->PositionNumber = 1.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    iobj_0->VelocityNumber = 0.0;
    iobj_0->PositionNumber = 0.0;
    iobj_0->JointAxisInternal[0] = 0.0;
    iobj_0->JointAxisInternal[1] = 0.0;
    iobj_0->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = iobj_0->MotionSubspace->size[0] * iobj_0->MotionSubspace->size[1];
  iobj_0->MotionSubspace->size[0] = 6;
  iobj_0->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    iobj_0->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = 1;
  iobj_0->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    iobj_0->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    iobj_0->HomePositionInternal->data[0] = 0.0;
  }

  b_obj->JointInternal = iobj_0;
  b_obj->Index = -1.0;
  b_obj->ParentIndex = -1.0;
  b_obj->MassInternal = 1.0;
  b_obj->CenterOfMassInternal[0] = 0.0;
  b_obj->CenterOfMassInternal[1] = 0.0;
  b_obj->CenterOfMassInternal[2] = 0.0;
  for (b_index = 0; b_index < 9; b_index++) {
    b_I[b_index] = 0;
  }

  b_I[0] = 1;
  b_I[4] = 1;
  b_I[8] = 1;
  for (b_index = 0; b_index < 9; b_index++) {
    b_obj->InertiaInternal[b_index] = b_I[b_index];
  }

  for (b_index = 0; b_index < 36; b_index++) {
    msubspace_data[b_index] = 0;
  }

  for (b_index = 0; b_index < 6; b_index++) {
    msubspace_data[b_index + 6 * b_index] = 1;
  }

  for (b_index = 0; b_index < 36; b_index++) {
    b_obj->SpatialInertia[b_index] = msubspace_data[b_index];
  }

  return b_obj;
}

/*
 * Arguments    : const c_robotics_manip_internal_Rigid *obj
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                robotics_Joint *iobj_2
 * Return Type  : c_robotics_manip_internal_Rigid *
 */
c_robotics_manip_internal_Rigid *RigidBody_copy(const
  c_robotics_manip_internal_Rigid *obj, c_robotics_manip_internal_Rigid *iobj_0,
  robotics_Joint *iobj_1, robotics_Joint *iobj_2)
{
  c_robotics_manip_internal_Rigid *newbody;
  emxArray_char_T *input1;
  int b_index;
  int loop_ub;
  emxArray_char_T *jname;
  static const char cv22[5] = { 'f', 'i', 'x', 'e', 'd' };

  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv32[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv33[6] = { 0, 0, 0, 0, 0, 1 };

  signed char b_I[9];
  double obj_idx_0;
  double obj_idx_1;
  double obj_idx_2;
  double b_obj[9];
  double c_obj[36];
  emxInit_char_T(&input1, 2);
  b_index = input1->size[0] * input1->size[1];
  input1->size[0] = 1;
  input1->size[1] = obj->NameInternal->size[1];
  emxEnsureCapacity_char_T(input1, b_index);
  loop_ub = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    input1->data[b_index] = obj->NameInternal->data[b_index];
  }

  newbody = iobj_0;
  b_index = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = input1->size[1];
  emxEnsureCapacity_char_T(iobj_0->NameInternal, b_index);
  loop_ub = input1->size[0] * input1->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    iobj_0->NameInternal->data[b_index] = input1->data[b_index];
  }

  iobj_1->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    iobj_1->ChildToJointTransform[b_index] = iv0[b_index];
  }

  emxInit_char_T(&jname, 2);
  b_index = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = input1->size[1] + 4;
  emxEnsureCapacity_char_T(jname, b_index);
  loop_ub = input1->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    jname->data[b_index] = input1->data[b_index];
  }

  jname->data[input1->size[1]] = '_';
  jname->data[1 + input1->size[1]] = 'j';
  jname->data[2 + input1->size[1]] = 'n';
  jname->data[3 + input1->size[1]] = 't';
  b_index = iobj_1->NameInternal->size[0] * iobj_1->NameInternal->size[1];
  iobj_1->NameInternal->size[0] = 1;
  iobj_1->NameInternal->size[1] = jname->size[1];
  emxEnsureCapacity_char_T(iobj_1->NameInternal, b_index);
  loop_ub = jname->size[0] * jname->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    iobj_1->NameInternal->data[b_index] = jname->data[b_index];
  }

  emxFree_char_T(&jname);
  b_index = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  iobj_1->Type->size[0] = 1;
  iobj_1->Type->size[1] = 5;
  emxEnsureCapacity_char_T(iobj_1->Type, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    iobj_1->Type->data[b_index] = cv22[b_index];
  }

  b_index = input1->size[0] * input1->size[1];
  input1->size[0] = 1;
  input1->size[1] = iobj_1->Type->size[1];
  emxEnsureCapacity_char_T(input1, b_index);
  loop_ub = iobj_1->Type->size[0] * iobj_1->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    input1->data[b_index] = iobj_1->Type->data[b_index];
  }

  result = b_strcmp(input1);
  if (result) {
    b_index = 0;
  } else {
    result = c_strcmp(input1);
    if (result) {
      b_index = 1;
    } else {
      b_index = -1;
    }
  }

  emxFree_char_T(&input1);
  switch (b_index) {
   case 0:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv32[b_index];
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
      msubspace_data[b_index] = iv33[b_index];
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

  iobj_0->JointInternal = Joint_copy(obj->JointInternal, iobj_2);
  iobj_0->MassInternal = obj->MassInternal;
  obj_idx_0 = obj->CenterOfMassInternal[0];
  obj_idx_1 = obj->CenterOfMassInternal[1];
  obj_idx_2 = obj->CenterOfMassInternal[2];
  iobj_0->CenterOfMassInternal[0] = obj_idx_0;
  iobj_0->CenterOfMassInternal[1] = obj_idx_1;
  iobj_0->CenterOfMassInternal[2] = obj_idx_2;
  for (b_index = 0; b_index < 9; b_index++) {
    b_obj[b_index] = obj->InertiaInternal[b_index];
  }

  for (b_index = 0; b_index < 9; b_index++) {
    iobj_0->InertiaInternal[b_index] = b_obj[b_index];
  }

  for (b_index = 0; b_index < 36; b_index++) {
    c_obj[b_index] = obj->SpatialInertia[b_index];
  }

  for (b_index = 0; b_index < 36; b_index++) {
    iobj_0->SpatialInertia[b_index] = c_obj[b_index];
  }

  return newbody;
}

/*
 * File trailer for RigidBody.c
 *
 * [EOF]
 */
