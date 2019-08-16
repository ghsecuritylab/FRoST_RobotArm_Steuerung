/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RigidBodyTree.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/RigidBodyTree.h"
#include "IK_solver/strcmp.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/RigidBody.h"
#include "IK_solver/RigidBodyTree1.h"

/* Function Definitions */

/*
 * Arguments    : const robotics_RigidBodyTree *obj
 *                const robotics_RigidBody *bodyin
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                robotics_Joint *iobj_2
 * Return Type  : void
 */
void RigidBodyTree_addBody(const robotics_RigidBodyTree *obj, const
  robotics_RigidBody *bodyin, c_robotics_manip_internal_Rigid *iobj_0,
  robotics_Joint *iobj_1, robotics_Joint *iobj_2)
{
  emxArray_char_T *bname;
  d_robotics_manip_internal_Rigid *b_obj;
  c_robotics_manip_internal_Rigid *body;
  int b_body;
  int loop_ub;
  int pid;
  double b_index;
  int i;
  boolean_T exitg1;
  robotics_Joint *jnt;
  c_robotics_manip_internal_Rigid *c_obj;
  emxInit_char_T(&bname, 2);
  b_obj = obj->TreeInternal;
  body = bodyin->BodyInternal;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = body->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = body->NameInternal->size[0] * body->NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = body->NameInternal->data[b_body];
  }

  c_RigidBodyTree_findBodyIndexBy(b_obj, bname);
  pid = -1;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = b_obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = b_obj->Base.NameInternal->size[0] * b_obj->Base.NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = b_obj->Base.NameInternal->data[b_body];
  }

  if (f_strcmp(bname)) {
    pid = 0;
  } else {
    b_index = b_obj->NumBodies;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= (int)b_index - 1)) {
      c_obj = b_obj->Bodies[i];
      b_body = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = c_obj->NameInternal->size[1];
      emxEnsureCapacity_char_T(bname, b_body);
      loop_ub = c_obj->NameInternal->size[0] * c_obj->NameInternal->size[1];
      for (b_body = 0; b_body < loop_ub; b_body++) {
        bname->data[b_body] = c_obj->NameInternal->data[b_body];
      }

      if (f_strcmp(bname)) {
        pid = i + 1;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  jnt = body->JointInternal;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = jnt->NameInternal->size[0] * jnt->NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = jnt->NameInternal->data[b_body];
  }

  d_RigidBodyTree_findBodyIndexBy(b_obj, bname);
  b_index = b_obj->NumBodies + 1.0;
  body = RigidBody_copy(body, iobj_0, iobj_1, iobj_2);
  b_obj->Bodies[(int)b_index - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  b_obj->NumBodies++;
  jnt = body->JointInternal;
  emxFree_char_T(&bname);
  if (!d_strcmp(jnt->Type)) {
    b_obj->NumNonFixedBodies++;
    jnt = body->JointInternal;
    b_body = (int)body->Index - 1;
    b_obj->PositionDoFMap[b_body] = b_obj->PositionNumber + 1.0;
    b_obj->PositionDoFMap[5 + b_body] = b_obj->PositionNumber +
      jnt->PositionNumber;
    jnt = body->JointInternal;
    b_body = (int)body->Index - 1;
    b_obj->VelocityDoFMap[b_body] = b_obj->VelocityNumber + 1.0;
    b_obj->VelocityDoFMap[5 + b_body] = b_obj->VelocityNumber +
      jnt->VelocityNumber;
  } else {
    b_body = (int)body->Index;
    b_obj->PositionDoFMap[b_body - 1] = 0.0;
    b_obj->PositionDoFMap[b_body + 4] = -1.0;
    b_body = (int)body->Index;
    b_obj->VelocityDoFMap[b_body - 1] = 0.0;
    b_obj->VelocityDoFMap[b_body + 4] = -1.0;
  }

  jnt = body->JointInternal;
  b_obj->PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  b_obj->VelocityNumber += jnt->VelocityNumber;
}

/*
 * Arguments    : const robotics_RigidBodyTree *obj
 *                const robotics_RigidBody *bodyin
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                robotics_Joint *iobj_2
 * Return Type  : void
 */
void b_RigidBodyTree_addBody(const robotics_RigidBodyTree *obj, const
  robotics_RigidBody *bodyin, c_robotics_manip_internal_Rigid *iobj_0,
  robotics_Joint *iobj_1, robotics_Joint *iobj_2)
{
  emxArray_char_T *bname;
  d_robotics_manip_internal_Rigid *b_obj;
  c_robotics_manip_internal_Rigid *body;
  int b_body;
  int loop_ub;
  int pid;
  double b_index;
  int i;
  boolean_T exitg1;
  robotics_Joint *jnt;
  c_robotics_manip_internal_Rigid *c_obj;
  emxInit_char_T(&bname, 2);
  b_obj = obj->TreeInternal;
  body = bodyin->BodyInternal;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = body->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = body->NameInternal->size[0] * body->NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = body->NameInternal->data[b_body];
  }

  c_RigidBodyTree_findBodyIndexBy(b_obj, bname);
  pid = -1;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = b_obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = b_obj->Base.NameInternal->size[0] * b_obj->Base.NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = b_obj->Base.NameInternal->data[b_body];
  }

  if (g_strcmp(bname)) {
    pid = 0;
  } else {
    b_index = b_obj->NumBodies;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= (int)b_index - 1)) {
      c_obj = b_obj->Bodies[i];
      b_body = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = c_obj->NameInternal->size[1];
      emxEnsureCapacity_char_T(bname, b_body);
      loop_ub = c_obj->NameInternal->size[0] * c_obj->NameInternal->size[1];
      for (b_body = 0; b_body < loop_ub; b_body++) {
        bname->data[b_body] = c_obj->NameInternal->data[b_body];
      }

      if (g_strcmp(bname)) {
        pid = i + 1;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  jnt = body->JointInternal;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = jnt->NameInternal->size[0] * jnt->NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = jnt->NameInternal->data[b_body];
  }

  d_RigidBodyTree_findBodyIndexBy(b_obj, bname);
  b_index = b_obj->NumBodies + 1.0;
  body = RigidBody_copy(body, iobj_0, iobj_1, iobj_2);
  b_obj->Bodies[(int)b_index - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  b_obj->NumBodies++;
  jnt = body->JointInternal;
  emxFree_char_T(&bname);
  if (!d_strcmp(jnt->Type)) {
    b_obj->NumNonFixedBodies++;
    jnt = body->JointInternal;
    b_body = (int)body->Index - 1;
    b_obj->PositionDoFMap[b_body] = b_obj->PositionNumber + 1.0;
    b_obj->PositionDoFMap[5 + b_body] = b_obj->PositionNumber +
      jnt->PositionNumber;
    jnt = body->JointInternal;
    b_body = (int)body->Index - 1;
    b_obj->VelocityDoFMap[b_body] = b_obj->VelocityNumber + 1.0;
    b_obj->VelocityDoFMap[5 + b_body] = b_obj->VelocityNumber +
      jnt->VelocityNumber;
  } else {
    b_body = (int)body->Index;
    b_obj->PositionDoFMap[b_body - 1] = 0.0;
    b_obj->PositionDoFMap[b_body + 4] = -1.0;
    b_body = (int)body->Index;
    b_obj->VelocityDoFMap[b_body - 1] = 0.0;
    b_obj->VelocityDoFMap[b_body + 4] = -1.0;
  }

  jnt = body->JointInternal;
  b_obj->PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  b_obj->VelocityNumber += jnt->VelocityNumber;
}

/*
 * Arguments    : const robotics_RigidBodyTree *obj
 *                const robotics_RigidBody *bodyin
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                robotics_Joint *iobj_2
 * Return Type  : void
 */
void c_RigidBodyTree_addBody(const robotics_RigidBodyTree *obj, const
  robotics_RigidBody *bodyin, c_robotics_manip_internal_Rigid *iobj_0,
  robotics_Joint *iobj_1, robotics_Joint *iobj_2)
{
  emxArray_char_T *bname;
  d_robotics_manip_internal_Rigid *b_obj;
  c_robotics_manip_internal_Rigid *body;
  int b_body;
  int loop_ub;
  int pid;
  double b_index;
  int i;
  boolean_T exitg1;
  robotics_Joint *jnt;
  c_robotics_manip_internal_Rigid *c_obj;
  emxInit_char_T(&bname, 2);
  b_obj = obj->TreeInternal;
  body = bodyin->BodyInternal;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = body->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = body->NameInternal->size[0] * body->NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = body->NameInternal->data[b_body];
  }

  c_RigidBodyTree_findBodyIndexBy(b_obj, bname);
  pid = -1;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = b_obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = b_obj->Base.NameInternal->size[0] * b_obj->Base.NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = b_obj->Base.NameInternal->data[b_body];
  }

  if (h_strcmp(bname)) {
    pid = 0;
  } else {
    b_index = b_obj->NumBodies;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= (int)b_index - 1)) {
      c_obj = b_obj->Bodies[i];
      b_body = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = c_obj->NameInternal->size[1];
      emxEnsureCapacity_char_T(bname, b_body);
      loop_ub = c_obj->NameInternal->size[0] * c_obj->NameInternal->size[1];
      for (b_body = 0; b_body < loop_ub; b_body++) {
        bname->data[b_body] = c_obj->NameInternal->data[b_body];
      }

      if (h_strcmp(bname)) {
        pid = i + 1;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  jnt = body->JointInternal;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = jnt->NameInternal->size[0] * jnt->NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = jnt->NameInternal->data[b_body];
  }

  d_RigidBodyTree_findBodyIndexBy(b_obj, bname);
  b_index = b_obj->NumBodies + 1.0;
  body = RigidBody_copy(body, iobj_0, iobj_1, iobj_2);
  b_obj->Bodies[(int)b_index - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  b_obj->NumBodies++;
  jnt = body->JointInternal;
  emxFree_char_T(&bname);
  if (!d_strcmp(jnt->Type)) {
    b_obj->NumNonFixedBodies++;
    jnt = body->JointInternal;
    b_body = (int)body->Index - 1;
    b_obj->PositionDoFMap[b_body] = b_obj->PositionNumber + 1.0;
    b_obj->PositionDoFMap[5 + b_body] = b_obj->PositionNumber +
      jnt->PositionNumber;
    jnt = body->JointInternal;
    b_body = (int)body->Index - 1;
    b_obj->VelocityDoFMap[b_body] = b_obj->VelocityNumber + 1.0;
    b_obj->VelocityDoFMap[5 + b_body] = b_obj->VelocityNumber +
      jnt->VelocityNumber;
  } else {
    b_body = (int)body->Index;
    b_obj->PositionDoFMap[b_body - 1] = 0.0;
    b_obj->PositionDoFMap[b_body + 4] = -1.0;
    b_body = (int)body->Index;
    b_obj->VelocityDoFMap[b_body - 1] = 0.0;
    b_obj->VelocityDoFMap[b_body + 4] = -1.0;
  }

  jnt = body->JointInternal;
  b_obj->PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  b_obj->VelocityNumber += jnt->VelocityNumber;
}

/*
 * Arguments    : const robotics_RigidBodyTree *obj
 *                const robotics_RigidBody *bodyin
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                robotics_Joint *iobj_2
 * Return Type  : void
 */
void d_RigidBodyTree_addBody(const robotics_RigidBodyTree *obj, const
  robotics_RigidBody *bodyin, c_robotics_manip_internal_Rigid *iobj_0,
  robotics_Joint *iobj_1, robotics_Joint *iobj_2)
{
  emxArray_char_T *bname;
  d_robotics_manip_internal_Rigid *b_obj;
  c_robotics_manip_internal_Rigid *body;
  int b_body;
  int loop_ub;
  int pid;
  double b_index;
  int i;
  boolean_T exitg1;
  robotics_Joint *jnt;
  c_robotics_manip_internal_Rigid *c_obj;
  emxInit_char_T(&bname, 2);
  b_obj = obj->TreeInternal;
  body = bodyin->BodyInternal;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = body->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = body->NameInternal->size[0] * body->NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = body->NameInternal->data[b_body];
  }

  c_RigidBodyTree_findBodyIndexBy(b_obj, bname);
  pid = -1;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = b_obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = b_obj->Base.NameInternal->size[0] * b_obj->Base.NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = b_obj->Base.NameInternal->data[b_body];
  }

  if (i_strcmp(bname)) {
    pid = 0;
  } else {
    b_index = b_obj->NumBodies;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= (int)b_index - 1)) {
      c_obj = b_obj->Bodies[i];
      b_body = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = c_obj->NameInternal->size[1];
      emxEnsureCapacity_char_T(bname, b_body);
      loop_ub = c_obj->NameInternal->size[0] * c_obj->NameInternal->size[1];
      for (b_body = 0; b_body < loop_ub; b_body++) {
        bname->data[b_body] = c_obj->NameInternal->data[b_body];
      }

      if (i_strcmp(bname)) {
        pid = i + 1;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  jnt = body->JointInternal;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = jnt->NameInternal->size[0] * jnt->NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = jnt->NameInternal->data[b_body];
  }

  d_RigidBodyTree_findBodyIndexBy(b_obj, bname);
  b_index = b_obj->NumBodies + 1.0;
  body = RigidBody_copy(body, iobj_0, iobj_1, iobj_2);
  b_obj->Bodies[(int)b_index - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  b_obj->NumBodies++;
  jnt = body->JointInternal;
  emxFree_char_T(&bname);
  if (!d_strcmp(jnt->Type)) {
    b_obj->NumNonFixedBodies++;
    jnt = body->JointInternal;
    b_body = (int)body->Index - 1;
    b_obj->PositionDoFMap[b_body] = b_obj->PositionNumber + 1.0;
    b_obj->PositionDoFMap[5 + b_body] = b_obj->PositionNumber +
      jnt->PositionNumber;
    jnt = body->JointInternal;
    b_body = (int)body->Index - 1;
    b_obj->VelocityDoFMap[b_body] = b_obj->VelocityNumber + 1.0;
    b_obj->VelocityDoFMap[5 + b_body] = b_obj->VelocityNumber +
      jnt->VelocityNumber;
  } else {
    b_body = (int)body->Index;
    b_obj->PositionDoFMap[b_body - 1] = 0.0;
    b_obj->PositionDoFMap[b_body + 4] = -1.0;
    b_body = (int)body->Index;
    b_obj->VelocityDoFMap[b_body - 1] = 0.0;
    b_obj->VelocityDoFMap[b_body + 4] = -1.0;
  }

  jnt = body->JointInternal;
  b_obj->PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  b_obj->VelocityNumber += jnt->VelocityNumber;
}

/*
 * Arguments    : const robotics_RigidBodyTree *obj
 *                const robotics_RigidBody *bodyin
 *                c_robotics_manip_internal_Rigid *iobj_0
 *                robotics_Joint *iobj_1
 *                robotics_Joint *iobj_2
 * Return Type  : void
 */
void e_RigidBodyTree_addBody(const robotics_RigidBodyTree *obj, const
  robotics_RigidBody *bodyin, c_robotics_manip_internal_Rigid *iobj_0,
  robotics_Joint *iobj_1, robotics_Joint *iobj_2)
{
  emxArray_char_T *bname;
  d_robotics_manip_internal_Rigid *b_obj;
  c_robotics_manip_internal_Rigid *body;
  int b_body;
  int loop_ub;
  int pid;
  double b_index;
  int i;
  boolean_T exitg1;
  robotics_Joint *jnt;
  c_robotics_manip_internal_Rigid *c_obj;
  emxInit_char_T(&bname, 2);
  b_obj = obj->TreeInternal;
  body = bodyin->BodyInternal;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = body->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = body->NameInternal->size[0] * body->NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = body->NameInternal->data[b_body];
  }

  c_RigidBodyTree_findBodyIndexBy(b_obj, bname);
  pid = -1;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = b_obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = b_obj->Base.NameInternal->size[0] * b_obj->Base.NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = b_obj->Base.NameInternal->data[b_body];
  }

  if (j_strcmp(bname)) {
    pid = 0;
  } else {
    b_index = b_obj->NumBodies;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= (int)b_index - 1)) {
      c_obj = b_obj->Bodies[i];
      b_body = bname->size[0] * bname->size[1];
      bname->size[0] = 1;
      bname->size[1] = c_obj->NameInternal->size[1];
      emxEnsureCapacity_char_T(bname, b_body);
      loop_ub = c_obj->NameInternal->size[0] * c_obj->NameInternal->size[1];
      for (b_body = 0; b_body < loop_ub; b_body++) {
        bname->data[b_body] = c_obj->NameInternal->data[b_body];
      }

      if (j_strcmp(bname)) {
        pid = i + 1;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  jnt = body->JointInternal;
  b_body = bname->size[0] * bname->size[1];
  bname->size[0] = 1;
  bname->size[1] = jnt->NameInternal->size[1];
  emxEnsureCapacity_char_T(bname, b_body);
  loop_ub = jnt->NameInternal->size[0] * jnt->NameInternal->size[1];
  for (b_body = 0; b_body < loop_ub; b_body++) {
    bname->data[b_body] = jnt->NameInternal->data[b_body];
  }

  d_RigidBodyTree_findBodyIndexBy(b_obj, bname);
  b_index = b_obj->NumBodies + 1.0;
  body = RigidBody_copy(body, iobj_0, iobj_1, iobj_2);
  b_obj->Bodies[(int)b_index - 1] = body;
  body->Index = b_index;
  body->ParentIndex = pid;
  body->JointInternal->InTree = true;
  b_obj->NumBodies++;
  jnt = body->JointInternal;
  emxFree_char_T(&bname);
  if (!d_strcmp(jnt->Type)) {
    b_obj->NumNonFixedBodies++;
    jnt = body->JointInternal;
    b_body = (int)body->Index - 1;
    b_obj->PositionDoFMap[b_body] = b_obj->PositionNumber + 1.0;
    b_obj->PositionDoFMap[5 + b_body] = b_obj->PositionNumber +
      jnt->PositionNumber;
    jnt = body->JointInternal;
    b_body = (int)body->Index - 1;
    b_obj->VelocityDoFMap[b_body] = b_obj->VelocityNumber + 1.0;
    b_obj->VelocityDoFMap[5 + b_body] = b_obj->VelocityNumber +
      jnt->VelocityNumber;
  } else {
    b_body = (int)body->Index;
    b_obj->PositionDoFMap[b_body - 1] = 0.0;
    b_obj->PositionDoFMap[b_body + 4] = -1.0;
    b_body = (int)body->Index;
    b_obj->VelocityDoFMap[b_body - 1] = 0.0;
    b_obj->VelocityDoFMap[b_body + 4] = -1.0;
  }

  jnt = body->JointInternal;
  b_obj->PositionNumber += jnt->PositionNumber;
  jnt = body->JointInternal;
  b_obj->VelocityNumber += jnt->VelocityNumber;
}

/*
 * File trailer for RigidBodyTree.c
 *
 * [EOF]
 */
