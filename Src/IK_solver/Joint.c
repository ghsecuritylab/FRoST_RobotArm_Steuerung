/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Joint.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include <string.h>
#include <math.h>
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/Joint.h"
#include "IK_solver/cat.h"
#include "IK_solver/normalizeRows.h"
#include "IK_solver/strcmp.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/validatestring.h"
#include "IK_solver/all1.h"
#include "IK_solver/isfinite.h"
#include "IK_solver/any.h"
#include "IK_solver/IK_solver_data.h"

/* Function Declarations */
static void Joint_resetHomePosition(robotics_Joint *obj);
static void Joint_resetMotionSubspace(robotics_Joint *obj, const double ax[3]);

/* Function Definitions */

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
static void Joint_resetHomePosition(robotics_Joint *obj)
{
  emxArray_real_T *ub;
  int loop_ub;
  int i7;
  emxArray_real_T *lb;
  emxArray_boolean_T *r1;
  emxArray_boolean_T *r2;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  unsigned int unnamed_idx_0;
  emxInit_real_T(&ub, 1);
  loop_ub = obj->PositionLimitsInternal->size[0];
  i7 = ub->size[0];
  ub->size[0] = loop_ub;
  emxEnsureCapacity_real_T(ub, i7);
  for (i7 = 0; i7 < loop_ub; i7++) {
    ub->data[i7] = obj->PositionLimitsInternal->data[i7 +
      obj->PositionLimitsInternal->size[0]];
  }

  emxInit_real_T(&lb, 1);
  loop_ub = obj->PositionLimitsInternal->size[0];
  i7 = lb->size[0];
  lb->size[0] = loop_ub;
  emxEnsureCapacity_real_T(lb, i7);
  for (i7 = 0; i7 < loop_ub; i7++) {
    lb->data[i7] = obj->PositionLimitsInternal->data[i7];
  }

  if (obj->PositionNumber == 1.0) {
    emxInit_boolean_T(&r1, 1);
    b_isfinite(lb, r1);
    emxInit_boolean_T(&r2, 1);
    guard1 = false;
    guard2 = false;
    guard3 = false;
    if (all(r1)) {
      b_isfinite(ub, r1);
      if (all(r1)) {
        i7 = obj->HomePositionInternal->size[0];
        obj->HomePositionInternal->size[0] = lb->size[0];
        emxEnsureCapacity_real_T(obj->HomePositionInternal, i7);
        loop_ub = lb->size[0];
        for (i7 = 0; i7 < loop_ub; i7++) {
          obj->HomePositionInternal->data[i7] = 0.5 * (lb->data[i7] + ub->
            data[i7]);
        }
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }

    if (guard3) {
      b_isfinite(lb, r1);
      if (all(r1)) {
        b_isfinite(ub, r1);
        i7 = r2->size[0];
        r2->size[0] = r1->size[0];
        emxEnsureCapacity_boolean_T(r2, i7);
        loop_ub = r1->size[0];
        for (i7 = 0; i7 < loop_ub; i7++) {
          r2->data[i7] = !r1->data[i7];
        }

        if (any(r2)) {
          i7 = obj->HomePositionInternal->size[0];
          obj->HomePositionInternal->size[0] = lb->size[0];
          emxEnsureCapacity_real_T(obj->HomePositionInternal, i7);
          loop_ub = lb->size[0];
          for (i7 = 0; i7 < loop_ub; i7++) {
            obj->HomePositionInternal->data[i7] = lb->data[i7];
          }
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    }

    if (guard2) {
      b_isfinite(lb, r1);
      i7 = r2->size[0];
      r2->size[0] = r1->size[0];
      emxEnsureCapacity_boolean_T(r2, i7);
      loop_ub = r1->size[0];
      for (i7 = 0; i7 < loop_ub; i7++) {
        r2->data[i7] = !r1->data[i7];
      }

      if (any(r2)) {
        b_isfinite(ub, r1);
        if (all(r1)) {
          i7 = obj->HomePositionInternal->size[0];
          obj->HomePositionInternal->size[0] = ub->size[0];
          emxEnsureCapacity_real_T(obj->HomePositionInternal, i7);
          loop_ub = ub->size[0];
          for (i7 = 0; i7 < loop_ub; i7++) {
            obj->HomePositionInternal->data[i7] = ub->data[i7];
          }
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    }

    if (guard1) {
      unnamed_idx_0 = (unsigned int)lb->size[0];
      i7 = obj->HomePositionInternal->size[0];
      obj->HomePositionInternal->size[0] = (int)unnamed_idx_0;
      emxEnsureCapacity_real_T(obj->HomePositionInternal, i7);
      loop_ub = (int)unnamed_idx_0;
      for (i7 = 0; i7 < loop_ub; i7++) {
        obj->HomePositionInternal->data[i7] = 0.0;
      }
    }

    emxFree_boolean_T(&r2);
    emxFree_boolean_T(&r1);
  }

  emxFree_real_T(&lb);
  emxFree_real_T(&ub);
}

/*
 * Arguments    : robotics_Joint *obj
 *                const double ax[3]
 * Return Type  : void
 */
static void Joint_resetMotionSubspace(robotics_Joint *obj, const double ax[3])
{
  boolean_T result;
  int b_result;
  double dv1[6];
  int i4;
  result = b_strcmp(obj->Type);
  if (result) {
    b_result = 0;
  } else {
    b_result = -1;
  }

  switch (b_result) {
   case 0:
    dv1[0] = 0.0;
    dv1[1] = ax[1];
    dv1[2] = ax[2];
    dv1[3] = 0.0;
    dv1[4] = 0.0;
    dv1[5] = 0.0;
    i4 = obj->MotionSubspace->size[0] * obj->MotionSubspace->size[1];
    obj->MotionSubspace->size[0] = 6;
    obj->MotionSubspace->size[1] = 1;
    emxEnsureCapacity_real_T(obj->MotionSubspace, i4);
    for (i4 = 0; i4 < 6; i4++) {
      obj->MotionSubspace->data[i4] = dv1[i4];
    }
    break;

   default:
    dv1[0] = 0.0;
    dv1[1] = 0.0;
    dv1[2] = 0.0;
    dv1[3] = 0.0;
    dv1[4] = ax[1];
    dv1[5] = ax[2];
    i4 = obj->MotionSubspace->size[0] * obj->MotionSubspace->size[1];
    obj->MotionSubspace->size[0] = 6;
    obj->MotionSubspace->size[1] = 1;
    emxEnsureCapacity_real_T(obj->MotionSubspace, i4);
    for (i4 = 0; i4 < 6; i4++) {
      obj->MotionSubspace->data[i4] = dv1[i4];
    }
    break;
  }
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : robotics_Joint *
 */
robotics_Joint *Joint_Joint(robotics_Joint *obj)
{
  robotics_Joint *b_obj;
  int b_index;
  static const char jname[8] = { 'b', 'a', 's', 'e', '_', 'j', 'n', 't' };

  emxArray_char_T *switch_expression;
  static const char cv2[5] = { 'f', 'i', 'x', 'e', 'd' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv1[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv2[6] = { 0, 0, 0, 0, 0, 1 };

  b_obj = obj;
  b_obj->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 8;
  emxEnsureCapacity_char_T(b_obj->NameInternal, b_index);
  for (b_index = 0; b_index < 8; b_index++) {
    b_obj->NameInternal->data[b_index] = jname[b_index];
  }

  b_index = b_obj->Type->size[0] * b_obj->Type->size[1];
  b_obj->Type->size[0] = 1;
  b_obj->Type->size[1] = 5;
  emxEnsureCapacity_char_T(b_obj->Type, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    b_obj->Type->data[b_index] = cv2[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = b_obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = b_obj->Type->size[0] * b_obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = b_obj->Type->data[b_index];
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
      msubspace_data[b_index] = iv1[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv2[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    b_obj->VelocityNumber = 0.0;
    b_obj->PositionNumber = 0.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = b_obj->MotionSubspace->size[0] * b_obj->MotionSubspace->size[1];
  b_obj->MotionSubspace->size[0] = 6;
  b_obj->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(b_obj->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    b_obj->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = b_obj->PositionLimitsInternal->size[0] *
    b_obj->PositionLimitsInternal->size[1];
  b_obj->PositionLimitsInternal->size[0] = 1;
  b_obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(b_obj->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    b_obj->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = b_obj->HomePositionInternal->size[0];
  b_obj->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(b_obj->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    b_obj->HomePositionInternal->data[0] = 0.0;
  }

  return b_obj;
}

/*
 * Arguments    : const robotics_Joint *obj
 *                robotics_Joint *iobj_0
 * Return Type  : robotics_Joint *
 */
robotics_Joint *Joint_copy(const robotics_Joint *obj, robotics_Joint *iobj_0)
{
  robotics_Joint *newjoint;
  emxArray_char_T *jtype;
  int b_index;
  int loop_ub;
  emxArray_char_T *jname;
  char partial_match_data[9];
  int partial_match_size[2];
  double nmatched;
  int jointtype_size_idx_1;
  char jointtype_data[9];
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv30[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv31[6] = { 0, 0, 0, 0, 0, 1 };

  emxArray_real_T *b_obj;
  double obj_idx_1;
  double obj_idx_2;
  double c_obj[16];
  emxInit_char_T(&jtype, 2);
  b_index = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = obj->Type->size[1];
  emxEnsureCapacity_char_T(jtype, b_index);
  loop_ub = obj->Type->size[0] * obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    jtype->data[b_index] = obj->Type->data[b_index];
  }

  emxInit_char_T(&jname, 2);
  b_index = jname->size[0] * jname->size[1];
  jname->size[0] = 1;
  jname->size[1] = obj->NameInternal->size[1];
  emxEnsureCapacity_char_T(jname, b_index);
  loop_ub = obj->NameInternal->size[0] * obj->NameInternal->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    jname->data[b_index] = obj->NameInternal->data[b_index];
  }

  iobj_0->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    iobj_0->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    iobj_0->ChildToJointTransform[b_index] = iv0[b_index];
  }

  newjoint = iobj_0;
  b_index = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
  iobj_0->NameInternal->size[0] = 1;
  iobj_0->NameInternal->size[1] = jname->size[1];
  emxEnsureCapacity_char_T(iobj_0->NameInternal, b_index);
  loop_ub = jname->size[0] * jname->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    iobj_0->NameInternal->data[b_index] = jname->data[b_index];
  }

  emxFree_char_T(&jname);
  get_match(jtype, partial_match_data, partial_match_size, &nmatched);
  if ((nmatched == 0.0) || ((jtype->size[1] == 0) != (partial_match_size[1] == 0)))
  {
    jointtype_size_idx_1 = 0;
  } else {
    if (!(nmatched > 1.0)) {
      jointtype_size_idx_1 = partial_match_size[1];
      loop_ub = partial_match_size[0] * partial_match_size[1];
      if (0 <= loop_ub - 1) {
        memcpy(&jointtype_data[0], &partial_match_data[0], (unsigned int)
               (loop_ub * (int)sizeof(char)));
      }
    }
  }

  b_index = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  iobj_0->Type->size[0] = 1;
  iobj_0->Type->size[1] = jointtype_size_idx_1;
  emxEnsureCapacity_char_T(iobj_0->Type, b_index);
  for (b_index = 0; b_index < jointtype_size_idx_1; b_index++) {
    iobj_0->Type->data[b_index] = jointtype_data[b_index];
  }

  b_index = jtype->size[0] * jtype->size[1];
  jtype->size[0] = 1;
  jtype->size[1] = iobj_0->Type->size[1];
  emxEnsureCapacity_char_T(jtype, b_index);
  loop_ub = iobj_0->Type->size[0] * iobj_0->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    jtype->data[b_index] = iobj_0->Type->data[b_index];
  }

  result = b_strcmp(jtype);
  if (result) {
    b_index = 0;
  } else {
    result = c_strcmp(jtype);
    if (result) {
      b_index = 1;
    } else {
      b_index = -1;
    }
  }

  switch (b_index) {
   case 0:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv30[b_index];
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
      msubspace_data[b_index] = iv31[b_index];
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

  b_index = obj->NameInternal->size[1];
  if (b_index != 0) {
    b_index = jtype->size[0] * jtype->size[1];
    jtype->size[0] = 1;
    jtype->size[1] = obj->NameInternal->size[1];
    emxEnsureCapacity_char_T(jtype, b_index);
    loop_ub = obj->NameInternal->size[0] * obj->NameInternal->size[1];
    for (b_index = 0; b_index < loop_ub; b_index++) {
      jtype->data[b_index] = obj->NameInternal->data[b_index];
    }

    if (!iobj_0->InTree) {
      b_index = iobj_0->NameInternal->size[0] * iobj_0->NameInternal->size[1];
      iobj_0->NameInternal->size[0] = 1;
      iobj_0->NameInternal->size[1] = jtype->size[1];
      emxEnsureCapacity_char_T(iobj_0->NameInternal, b_index);
      loop_ub = jtype->size[0] * jtype->size[1];
      for (b_index = 0; b_index < loop_ub; b_index++) {
        iobj_0->NameInternal->data[b_index] = jtype->data[b_index];
      }
    }
  }

  emxFree_char_T(&jtype);
  emxInit_real_T(&b_obj, 1);
  loop_ub = obj->PositionLimitsInternal->size[0] * obj->
    PositionLimitsInternal->size[1];
  b_index = iobj_0->PositionLimitsInternal->size[0] *
    iobj_0->PositionLimitsInternal->size[1];
  iobj_0->PositionLimitsInternal->size[0] = obj->PositionLimitsInternal->size[0];
  iobj_0->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(iobj_0->PositionLimitsInternal, b_index);
  b_index = b_obj->size[0];
  b_obj->size[0] = loop_ub;
  emxEnsureCapacity_real_T(b_obj, b_index);
  for (b_index = 0; b_index < loop_ub; b_index++) {
    b_obj->data[b_index] = obj->PositionLimitsInternal->data[b_index];
  }

  loop_ub = b_obj->size[0];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    iobj_0->PositionLimitsInternal->data[b_index] = b_obj->data[b_index];
  }

  b_index = b_obj->size[0];
  b_obj->size[0] = obj->HomePositionInternal->size[0];
  emxEnsureCapacity_real_T(b_obj, b_index);
  loop_ub = obj->HomePositionInternal->size[0];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    b_obj->data[b_index] = obj->HomePositionInternal->data[b_index];
  }

  b_index = iobj_0->HomePositionInternal->size[0];
  iobj_0->HomePositionInternal->size[0] = b_obj->size[0];
  emxEnsureCapacity_real_T(iobj_0->HomePositionInternal, b_index);
  loop_ub = b_obj->size[0];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    iobj_0->HomePositionInternal->data[b_index] = b_obj->data[b_index];
  }

  nmatched = obj->JointAxisInternal[0];
  obj_idx_1 = obj->JointAxisInternal[1];
  obj_idx_2 = obj->JointAxisInternal[2];
  iobj_0->JointAxisInternal[0] = nmatched;
  iobj_0->JointAxisInternal[1] = obj_idx_1;
  iobj_0->JointAxisInternal[2] = obj_idx_2;
  loop_ub = obj->MotionSubspace->size[0] * obj->MotionSubspace->size[1];
  b_index = iobj_0->MotionSubspace->size[0] * iobj_0->MotionSubspace->size[1];
  iobj_0->MotionSubspace->size[0] = 6;
  iobj_0->MotionSubspace->size[1] = obj->MotionSubspace->size[1];
  emxEnsureCapacity_real_T(iobj_0->MotionSubspace, b_index);
  b_index = b_obj->size[0];
  b_obj->size[0] = loop_ub;
  emxEnsureCapacity_real_T(b_obj, b_index);
  for (b_index = 0; b_index < loop_ub; b_index++) {
    b_obj->data[b_index] = obj->MotionSubspace->data[b_index];
  }

  loop_ub = b_obj->size[0];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    iobj_0->MotionSubspace->data[b_index] = b_obj->data[b_index];
  }

  emxFree_real_T(&b_obj);
  for (b_index = 0; b_index < 16; b_index++) {
    c_obj[b_index] = obj->JointToParentTransform[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    iobj_0->JointToParentTransform[b_index] = c_obj[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    c_obj[b_index] = obj->ChildToJointTransform[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    iobj_0->ChildToJointTransform[b_index] = c_obj[b_index];
  }

  return newjoint;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void Joint_setFixedTransform(robotics_Joint *obj)
{
  emxArray_char_T *switch_expression;
  int b_index;
  int loop_ub;
  boolean_T result;
  signed char TL[16];
  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = obj->Type->size[0] * obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = obj->Type->data[b_index];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    b_index = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      b_index = 1;
    } else {
      result = d_strcmp(switch_expression);
      if (result) {
        b_index = 2;
      } else {
        b_index = -1;
      }
    }
  }

  emxFree_char_T(&switch_expression);
  switch (b_index) {
   case 0:
    for (b_index = 0; b_index < 16; b_index++) {
      TL[b_index] = iv0[b_index];
    }
    break;

   case 1:
    for (b_index = 0; b_index < 16; b_index++) {
      TL[b_index] = iv0[b_index];
    }
    break;

   case 2:
    for (b_index = 0; b_index < 16; b_index++) {
      TL[b_index] = iv0[b_index];
    }
    break;
  }

  for (b_index = 0; b_index < 16; b_index++) {
    obj->JointToParentTransform[b_index] = TL[b_index];
    obj->ChildToJointTransform[b_index] = 0.0;
  }

  obj->ChildToJointTransform[0] = 1.0;
  obj->ChildToJointTransform[5] = 1.0;
  obj->ChildToJointTransform[10] = 1.0;
  obj->ChildToJointTransform[15] = 1.0;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void Joint_set_HomePosition(robotics_Joint *obj)
{
  int i5;
  i5 = obj->HomePositionInternal->size[0];
  obj->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(obj->HomePositionInternal, i5);
  obj->HomePositionInternal->data[0] = 0.0;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void Joint_set_JointAxis(robotics_Joint *obj)
{
  static const double ax[3] = { 0.0, 0.0, 1.0 };

  obj->JointAxisInternal[0] = 0.0;
  obj->JointAxisInternal[1] = 0.0;
  obj->JointAxisInternal[2] = 1.0;
  Joint_resetMotionSubspace(obj, ax);
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void Joint_set_PositionLimits(robotics_Joint *obj)
{
  boolean_T resetHome;
  emxArray_boolean_T *b_obj;
  int i6;
  int loop_ub;
  resetHome = false;
  switch ((int)obj->PositionNumber) {
   case 0:
    break;

   default:
    emxInit_boolean_T(&b_obj, 1);
    i6 = b_obj->size[0];
    b_obj->size[0] = obj->HomePositionInternal->size[0];
    emxEnsureCapacity_boolean_T(b_obj, i6);
    loop_ub = obj->HomePositionInternal->size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      b_obj->data[i6] = (obj->HomePositionInternal->data[i6] >
                         3.1415926535897931);
    }

    if (any(b_obj)) {
      resetHome = true;
    } else {
      i6 = b_obj->size[0];
      b_obj->size[0] = obj->HomePositionInternal->size[0];
      emxEnsureCapacity_boolean_T(b_obj, i6);
      loop_ub = obj->HomePositionInternal->size[0];
      for (i6 = 0; i6 < loop_ub; i6++) {
        b_obj->data[i6] = (obj->HomePositionInternal->data[i6] <
                           -3.1415926535897931);
      }

      if (any(b_obj)) {
        resetHome = true;
      }
    }

    emxFree_boolean_T(&b_obj);
    break;
  }

  i6 = obj->PositionLimitsInternal->size[0] * obj->PositionLimitsInternal->size
    [1];
  obj->PositionLimitsInternal->size[0] = 1;
  obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(obj->PositionLimitsInternal, i6);
  obj->PositionLimitsInternal->data[0] = -3.1415926535897931;
  obj->PositionLimitsInternal->data[1] = 3.1415926535897931;
  if (resetHome) {
    Joint_resetHomePosition(obj);
  }
}

/*
 * Arguments    : const robotics_Joint *obj
 *                double T[16]
 * Return Type  : void
 */
void Joint_transformBodyToParent(const robotics_Joint *obj, double T[16])
{
  emxArray_char_T *switch_expression;
  int i33;
  int loop_ub;
  boolean_T result;
  double TJ[16];
  double v[3];
  double tempR[9];
  double b_v[3];
  int i34;
  double R[9];
  int obj_tmp;
  double b_obj[16];
  emxInit_char_T(&switch_expression, 2);
  i33 = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, i33);
  loop_ub = obj->Type->size[0] * obj->Type->size[1];
  for (i33 = 0; i33 < loop_ub; i33++) {
    switch_expression->data[i33] = obj->Type->data[i33];
  }

  result = d_strcmp(switch_expression);
  if (result) {
    loop_ub = 0;
  } else {
    result = b_strcmp(switch_expression);
    if (result) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }

  emxFree_char_T(&switch_expression);
  switch (loop_ub) {
   case 0:
    memset(&TJ[0], 0, sizeof(double) << 4);
    TJ[0] = 1.0;
    TJ[5] = 1.0;
    TJ[10] = 1.0;
    TJ[15] = 1.0;
    break;

   case 1:
    if (b_strcmp(obj->Type) || c_strcmp(obj->Type)) {
      v[0] = obj->JointAxisInternal[0];
      v[1] = obj->JointAxisInternal[1];
      v[2] = obj->JointAxisInternal[2];
    } else {
      v[0] = rtNaN;
      v[1] = rtNaN;
      v[2] = rtNaN;
    }

    b_v[0] = v[0];
    b_v[1] = v[1];
    b_v[2] = v[2];
    normalizeRows(b_v, v);
    cat(v[0] * v[0] * 0.0 + 1.0, v[1] * v[0] * 0.0 - v[2] * 0.0, v[2] * v[0] *
        0.0 + v[1] * 0.0, v[0] * v[1] * 0.0 + v[2] * 0.0, v[1] * v[1] * 0.0 +
        1.0, v[2] * v[1] * 0.0 - v[0] * 0.0, v[0] * v[2] * 0.0 - v[1] * 0.0, v[1]
        * v[2] * 0.0 + v[0] * 0.0, v[2] * v[2] * 0.0 + 1.0, tempR);
    cat(v[0] * v[0] * 0.0 + 1.0, v[1] * v[0] * 0.0 - v[2] * 0.0, v[2] * v[0] *
        0.0 + v[1] * 0.0, v[0] * v[1] * 0.0 + v[2] * 0.0, v[1] * v[1] * 0.0 +
        1.0, v[2] * v[1] * 0.0 - v[0] * 0.0, v[0] * v[2] * 0.0 - v[1] * 0.0, v[1]
        * v[2] * 0.0 + v[0] * 0.0, v[2] * v[2] * 0.0 + 1.0, R);
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      R[loop_ub] = tempR[3 * loop_ub];
      R[loop_ub + 3] = tempR[3 * loop_ub + 1];
      R[loop_ub + 6] = tempR[3 * loop_ub + 2];
    }

    memset(&TJ[0], 0, sizeof(double) << 4);
    for (i33 = 0; i33 < 3; i33++) {
      loop_ub = i33 << 2;
      TJ[loop_ub] = R[3 * i33];
      TJ[1 + loop_ub] = R[1 + 3 * i33];
      TJ[2 + loop_ub] = R[2 + 3 * i33];
    }

    TJ[15] = 1.0;
    break;

   default:
    if (b_strcmp(obj->Type) || c_strcmp(obj->Type)) {
      v[0] = obj->JointAxisInternal[0];
      v[1] = obj->JointAxisInternal[1];
      v[2] = obj->JointAxisInternal[2];
    } else {
      v[0] = rtNaN;
      v[1] = rtNaN;
      v[2] = rtNaN;
    }

    memset(&tempR[0], 0, 9U * sizeof(double));
    tempR[0] = 1.0;
    tempR[4] = 1.0;
    tempR[8] = 1.0;
    for (i33 = 0; i33 < 3; i33++) {
      loop_ub = i33 << 2;
      TJ[loop_ub] = tempR[3 * i33];
      TJ[1 + loop_ub] = tempR[1 + 3 * i33];
      TJ[2 + loop_ub] = tempR[2 + 3 * i33];
      TJ[12 + i33] = v[i33] * 0.0;
    }

    TJ[3] = 0.0;
    TJ[7] = 0.0;
    TJ[11] = 0.0;
    TJ[15] = 1.0;
    break;
  }

  for (i33 = 0; i33 < 4; i33++) {
    for (i34 = 0; i34 < 4; i34++) {
      loop_ub = i34 << 2;
      obj_tmp = i33 + loop_ub;
      b_obj[obj_tmp] = 0.0;
      b_obj[obj_tmp] = ((obj->JointToParentTransform[i33] * TJ[loop_ub] +
                         obj->JointToParentTransform[i33 + 4] * TJ[1 + loop_ub])
                        + obj->JointToParentTransform[i33 + 8] * TJ[2 + loop_ub])
        + obj->JointToParentTransform[i33 + 12] * TJ[3 + loop_ub];
    }

    for (i34 = 0; i34 < 4; i34++) {
      loop_ub = i34 << 2;
      obj_tmp = i33 + loop_ub;
      T[obj_tmp] = 0.0;
      T[obj_tmp] = ((b_obj[i33] * obj->ChildToJointTransform[loop_ub] +
                     b_obj[i33 + 4] * obj->ChildToJointTransform[1 + loop_ub]) +
                    b_obj[i33 + 8] * obj->ChildToJointTransform[2 + loop_ub]) +
        b_obj[i33 + 12] * obj->ChildToJointTransform[3 + loop_ub];
    }
  }
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : robotics_Joint *
 */
robotics_Joint *b_Joint_Joint(robotics_Joint *obj)
{
  robotics_Joint *b_obj;
  int b_index;
  static const char jname[14] = { 'd', 'u', 'm', 'm', 'y', 'b', 'o', 'd', 'y',
    '\x01', '_', 'j', 'n', 't' };

  emxArray_char_T *switch_expression;
  static const char cv5[5] = { 'f', 'i', 'x', 'e', 'd' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv6[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv7[6] = { 0, 0, 0, 0, 0, 1 };

  b_obj = obj;
  b_obj->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 14;
  emxEnsureCapacity_char_T(b_obj->NameInternal, b_index);
  for (b_index = 0; b_index < 14; b_index++) {
    b_obj->NameInternal->data[b_index] = jname[b_index];
  }

  b_index = b_obj->Type->size[0] * b_obj->Type->size[1];
  b_obj->Type->size[0] = 1;
  b_obj->Type->size[1] = 5;
  emxEnsureCapacity_char_T(b_obj->Type, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    b_obj->Type->data[b_index] = cv5[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = b_obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = b_obj->Type->size[0] * b_obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = b_obj->Type->data[b_index];
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
      msubspace_data[b_index] = iv6[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv7[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    b_obj->VelocityNumber = 0.0;
    b_obj->PositionNumber = 0.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = b_obj->MotionSubspace->size[0] * b_obj->MotionSubspace->size[1];
  b_obj->MotionSubspace->size[0] = 6;
  b_obj->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(b_obj->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    b_obj->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = b_obj->PositionLimitsInternal->size[0] *
    b_obj->PositionLimitsInternal->size[1];
  b_obj->PositionLimitsInternal->size[0] = 1;
  b_obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(b_obj->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    b_obj->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = b_obj->HomePositionInternal->size[0];
  b_obj->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(b_obj->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    b_obj->HomePositionInternal->data[0] = 0.0;
  }

  return b_obj;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void b_Joint_setFixedTransform(robotics_Joint *obj)
{
  emxArray_char_T *switch_expression;
  int b_index;
  int loop_ub;
  boolean_T result;
  double TL[16];
  static const double TL_tmp[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.12, 1.0 };

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = obj->Type->size[0] * obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = obj->Type->data[b_index];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    b_index = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      b_index = 1;
    } else {
      result = d_strcmp(switch_expression);
      if (result) {
        b_index = 2;
      } else {
        b_index = -1;
      }
    }
  }

  emxFree_char_T(&switch_expression);
  switch (b_index) {
   case 0:
    memcpy(&TL[0], &TL_tmp[0], sizeof(double) << 4);
    break;

   case 1:
    for (b_index = 0; b_index < 16; b_index++) {
      TL[b_index] = iv0[b_index];
    }
    break;

   case 2:
    memcpy(&TL[0], &TL_tmp[0], sizeof(double) << 4);
    break;
  }

  for (b_index = 0; b_index < 16; b_index++) {
    obj->JointToParentTransform[b_index] = TL[b_index];
    obj->ChildToJointTransform[b_index] = 0.0;
  }

  obj->ChildToJointTransform[0] = 1.0;
  obj->ChildToJointTransform[5] = 1.0;
  obj->ChildToJointTransform[10] = 1.0;
  obj->ChildToJointTransform[15] = 1.0;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void b_Joint_set_HomePosition(robotics_Joint *obj)
{
  int i9;
  i9 = obj->HomePositionInternal->size[0];
  obj->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(obj->HomePositionInternal, i9);
  obj->HomePositionInternal->data[0] = -1.5707963267948966;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void b_Joint_set_JointAxis(robotics_Joint *obj)
{
  static const double ax[3] = { 0.0, 1.0, 0.0 };

  obj->JointAxisInternal[0] = 0.0;
  obj->JointAxisInternal[1] = 1.0;
  obj->JointAxisInternal[2] = 0.0;
  Joint_resetMotionSubspace(obj, ax);
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void b_Joint_set_PositionLimits(robotics_Joint *obj)
{
  boolean_T resetHome;
  emxArray_boolean_T *b_obj;
  int i10;
  int loop_ub;
  resetHome = false;
  switch ((int)obj->PositionNumber) {
   case 0:
    break;

   default:
    emxInit_boolean_T(&b_obj, 1);
    i10 = b_obj->size[0];
    b_obj->size[0] = obj->HomePositionInternal->size[0];
    emxEnsureCapacity_boolean_T(b_obj, i10);
    loop_ub = obj->HomePositionInternal->size[0];
    for (i10 = 0; i10 < loop_ub; i10++) {
      b_obj->data[i10] = (obj->HomePositionInternal->data[i10] >
                          0.3490658503988659);
    }

    if (any(b_obj)) {
      resetHome = true;
    } else {
      i10 = b_obj->size[0];
      b_obj->size[0] = obj->HomePositionInternal->size[0];
      emxEnsureCapacity_boolean_T(b_obj, i10);
      loop_ub = obj->HomePositionInternal->size[0];
      for (i10 = 0; i10 < loop_ub; i10++) {
        b_obj->data[i10] = (obj->HomePositionInternal->data[i10] <
                            -1.5707963267948966);
      }

      if (any(b_obj)) {
        resetHome = true;
      }
    }

    emxFree_boolean_T(&b_obj);
    break;
  }

  i10 = obj->PositionLimitsInternal->size[0] * obj->PositionLimitsInternal->
    size[1];
  obj->PositionLimitsInternal->size[0] = 1;
  obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(obj->PositionLimitsInternal, i10);
  obj->PositionLimitsInternal->data[0] = -1.5707963267948966;
  obj->PositionLimitsInternal->data[1] = 0.3490658503988659;
  if (resetHome) {
    Joint_resetHomePosition(obj);
  }
}

/*
 * Arguments    : const robotics_Joint *obj
 *                const emxArray_real_T *q
 *                double T[16]
 * Return Type  : void
 */
void b_Joint_transformBodyToParent(const robotics_Joint *obj, const
  emxArray_real_T *q, double T[16])
{
  emxArray_char_T *switch_expression;
  int i35;
  int loop_ub;
  boolean_T result;
  double TJ[16];
  double v[3];
  double tempR[9];
  signed char input_sizes_idx_1;
  double result_data[4];
  int i36;
  double cth;
  int obj_tmp;
  double b_obj[16];
  double sth;
  double tempR_tmp;
  double b_tempR_tmp;
  double c_tempR_tmp;
  double R[9];
  emxInit_char_T(&switch_expression, 2);
  i35 = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, i35);
  loop_ub = obj->Type->size[0] * obj->Type->size[1];
  for (i35 = 0; i35 < loop_ub; i35++) {
    switch_expression->data[i35] = obj->Type->data[i35];
  }

  result = d_strcmp(switch_expression);
  if (result) {
    loop_ub = 0;
  } else {
    result = b_strcmp(switch_expression);
    if (result) {
      loop_ub = 1;
    } else {
      loop_ub = -1;
    }
  }

  emxFree_char_T(&switch_expression);
  switch (loop_ub) {
   case 0:
    memset(&TJ[0], 0, sizeof(double) << 4);
    TJ[0] = 1.0;
    TJ[5] = 1.0;
    TJ[10] = 1.0;
    TJ[15] = 1.0;
    break;

   case 1:
    if (b_strcmp(obj->Type) || c_strcmp(obj->Type)) {
      v[0] = obj->JointAxisInternal[0];
      v[1] = obj->JointAxisInternal[1];
      v[2] = obj->JointAxisInternal[2];
    } else {
      v[0] = rtNaN;
      v[1] = rtNaN;
      v[2] = rtNaN;
    }

    input_sizes_idx_1 = (signed char)(q->size[0] != 0);
    result_data[0] = v[0];
    result_data[1] = v[1];
    result_data[2] = v[2];
    loop_ub = input_sizes_idx_1;
    for (i35 = 0; i35 < loop_ub; i35++) {
      result_data[3] = q->data[0];
    }

    normalizeRows(*(double (*)[3])&result_data[0], v);
    cth = cos(result_data[3]);
    sth = sin(result_data[3]);
    tempR_tmp = v[2] * sth;
    b_tempR_tmp = v[1] * sth;
    c_tempR_tmp = v[0] * sth;
    cat(v[0] * v[0] * (1.0 - cth) + cth, v[1] * v[0] * (1.0 - cth) - tempR_tmp,
        v[2] * v[0] * (1.0 - cth) + b_tempR_tmp, v[0] * v[1] * (1.0 - cth) +
        tempR_tmp, v[1] * v[1] * (1.0 - cth) + cth, v[2] * v[1] * (1.0 - cth) -
        c_tempR_tmp, v[0] * v[2] * (1.0 - cth) - b_tempR_tmp, v[1] * v[2] * (1.0
         - cth) + c_tempR_tmp, v[2] * v[2] * (1.0 - cth) + cth, tempR);
    cat(v[0] * v[0] * (1.0 - cth) + cth, v[1] * v[0] * (1.0 - cth) - v[2] * sth,
        v[2] * v[0] * (1.0 - cth) + v[1] * sth, v[0] * v[1] * (1.0 - cth) + v[2]
        * sth, v[1] * v[1] * (1.0 - cth) + cth, v[2] * v[1] * (1.0 - cth) - v[0]
        * sth, v[0] * v[2] * (1.0 - cth) - v[1] * sth, v[1] * v[2] * (1.0 - cth)
        + v[0] * sth, v[2] * v[2] * (1.0 - cth) + cth, R);
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      R[loop_ub] = tempR[3 * loop_ub];
      R[loop_ub + 3] = tempR[3 * loop_ub + 1];
      R[loop_ub + 6] = tempR[3 * loop_ub + 2];
    }

    memset(&TJ[0], 0, sizeof(double) << 4);
    for (i35 = 0; i35 < 3; i35++) {
      loop_ub = i35 << 2;
      TJ[loop_ub] = R[3 * i35];
      TJ[1 + loop_ub] = R[1 + 3 * i35];
      TJ[2 + loop_ub] = R[2 + 3 * i35];
    }

    TJ[15] = 1.0;
    break;

   default:
    if (b_strcmp(obj->Type) || c_strcmp(obj->Type)) {
      v[0] = obj->JointAxisInternal[0];
      v[1] = obj->JointAxisInternal[1];
      v[2] = obj->JointAxisInternal[2];
    } else {
      v[0] = rtNaN;
      v[1] = rtNaN;
      v[2] = rtNaN;
    }

    memset(&tempR[0], 0, 9U * sizeof(double));
    tempR[0] = 1.0;
    tempR[4] = 1.0;
    tempR[8] = 1.0;
    cth = q->data[0];
    for (i35 = 0; i35 < 3; i35++) {
      loop_ub = i35 << 2;
      TJ[loop_ub] = tempR[3 * i35];
      TJ[1 + loop_ub] = tempR[1 + 3 * i35];
      TJ[2 + loop_ub] = tempR[2 + 3 * i35];
      TJ[12 + i35] = v[i35] * cth;
    }

    TJ[3] = 0.0;
    TJ[7] = 0.0;
    TJ[11] = 0.0;
    TJ[15] = 1.0;
    break;
  }

  for (i35 = 0; i35 < 4; i35++) {
    for (i36 = 0; i36 < 4; i36++) {
      loop_ub = i36 << 2;
      obj_tmp = i35 + loop_ub;
      b_obj[obj_tmp] = 0.0;
      b_obj[obj_tmp] = ((obj->JointToParentTransform[i35] * TJ[loop_ub] +
                         obj->JointToParentTransform[i35 + 4] * TJ[1 + loop_ub])
                        + obj->JointToParentTransform[i35 + 8] * TJ[2 + loop_ub])
        + obj->JointToParentTransform[i35 + 12] * TJ[3 + loop_ub];
    }

    for (i36 = 0; i36 < 4; i36++) {
      loop_ub = i36 << 2;
      obj_tmp = i35 + loop_ub;
      T[obj_tmp] = 0.0;
      T[obj_tmp] = ((b_obj[i35] * obj->ChildToJointTransform[loop_ub] +
                     b_obj[i35 + 4] * obj->ChildToJointTransform[1 + loop_ub]) +
                    b_obj[i35 + 8] * obj->ChildToJointTransform[2 + loop_ub]) +
        b_obj[i35 + 12] * obj->ChildToJointTransform[3 + loop_ub];
    }
  }
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : robotics_Joint *
 */
robotics_Joint *c_Joint_Joint(robotics_Joint *obj)
{
  robotics_Joint *b_obj;
  int b_index;
  emxArray_char_T *switch_expression;
  static const char cv8[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv12[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv13[6] = { 0, 0, 0, 0, 0, 1 };

  b_obj = obj;
  b_obj->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 4;
  emxEnsureCapacity_char_T(b_obj->NameInternal, b_index);
  b_obj->NameInternal->data[0] = 'j';
  b_obj->NameInternal->data[1] = 'n';
  b_obj->NameInternal->data[2] = 't';
  b_obj->NameInternal->data[3] = '1';
  b_index = b_obj->Type->size[0] * b_obj->Type->size[1];
  b_obj->Type->size[0] = 1;
  b_obj->Type->size[1] = 8;
  emxEnsureCapacity_char_T(b_obj->Type, b_index);
  for (b_index = 0; b_index < 8; b_index++) {
    b_obj->Type->data[b_index] = cv8[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = b_obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = b_obj->Type->size[0] * b_obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = b_obj->Type->data[b_index];
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
      msubspace_data[b_index] = iv12[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv13[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    b_obj->VelocityNumber = 0.0;
    b_obj->PositionNumber = 0.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = b_obj->MotionSubspace->size[0] * b_obj->MotionSubspace->size[1];
  b_obj->MotionSubspace->size[0] = 6;
  b_obj->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(b_obj->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    b_obj->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = b_obj->PositionLimitsInternal->size[0] *
    b_obj->PositionLimitsInternal->size[1];
  b_obj->PositionLimitsInternal->size[0] = 1;
  b_obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(b_obj->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    b_obj->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = b_obj->HomePositionInternal->size[0];
  b_obj->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(b_obj->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    b_obj->HomePositionInternal->data[0] = 0.0;
  }

  return b_obj;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void c_Joint_setFixedTransform(robotics_Joint *obj)
{
  emxArray_char_T *switch_expression;
  int b_index;
  int loop_ub;
  boolean_T result;
  double TL[16];
  static const double TL_tmp[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.6, 0.0, 0.0, 1.0 };

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = obj->Type->size[0] * obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = obj->Type->data[b_index];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    b_index = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      b_index = 1;
    } else {
      result = d_strcmp(switch_expression);
      if (result) {
        b_index = 2;
      } else {
        b_index = -1;
      }
    }
  }

  emxFree_char_T(&switch_expression);
  switch (b_index) {
   case 0:
    memcpy(&TL[0], &TL_tmp[0], sizeof(double) << 4);
    break;

   case 1:
    memcpy(&TL[0], &TL_tmp[0], sizeof(double) << 4);
    break;

   case 2:
    memcpy(&TL[0], &TL_tmp[0], sizeof(double) << 4);
    break;
  }

  for (b_index = 0; b_index < 16; b_index++) {
    obj->JointToParentTransform[b_index] = TL[b_index];
    obj->ChildToJointTransform[b_index] = 0.0;
  }

  obj->ChildToJointTransform[0] = 1.0;
  obj->ChildToJointTransform[5] = 1.0;
  obj->ChildToJointTransform[10] = 1.0;
  obj->ChildToJointTransform[15] = 1.0;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void c_Joint_set_PositionLimits(robotics_Joint *obj)
{
  boolean_T resetHome;
  emxArray_boolean_T *b_obj;
  int i11;
  int loop_ub;
  resetHome = false;
  switch ((int)obj->PositionNumber) {
   case 0:
    break;

   default:
    emxInit_boolean_T(&b_obj, 1);
    i11 = b_obj->size[0];
    b_obj->size[0] = obj->HomePositionInternal->size[0];
    emxEnsureCapacity_boolean_T(b_obj, i11);
    loop_ub = obj->HomePositionInternal->size[0];
    for (i11 = 0; i11 < loop_ub; i11++) {
      b_obj->data[i11] = (obj->HomePositionInternal->data[i11] >
                          2.6179938779914944);
    }

    if (any(b_obj)) {
      resetHome = true;
    } else {
      i11 = b_obj->size[0];
      b_obj->size[0] = obj->HomePositionInternal->size[0];
      emxEnsureCapacity_boolean_T(b_obj, i11);
      loop_ub = obj->HomePositionInternal->size[0];
      for (i11 = 0; i11 < loop_ub; i11++) {
        b_obj->data[i11] = (obj->HomePositionInternal->data[i11] <
                            -2.6179938779914944);
      }

      if (any(b_obj)) {
        resetHome = true;
      }
    }

    emxFree_boolean_T(&b_obj);
    break;
  }

  i11 = obj->PositionLimitsInternal->size[0] * obj->PositionLimitsInternal->
    size[1];
  obj->PositionLimitsInternal->size[0] = 1;
  obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(obj->PositionLimitsInternal, i11);
  obj->PositionLimitsInternal->data[0] = -2.6179938779914944;
  obj->PositionLimitsInternal->data[1] = 2.6179938779914944;
  if (resetHome) {
    Joint_resetHomePosition(obj);
  }
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : robotics_Joint *
 */
robotics_Joint *d_Joint_Joint(robotics_Joint *obj)
{
  robotics_Joint *b_obj;
  int b_index;
  emxArray_char_T *switch_expression;
  static const char cv10[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv16[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv17[6] = { 0, 0, 0, 0, 0, 1 };

  b_obj = obj;
  b_obj->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 4;
  emxEnsureCapacity_char_T(b_obj->NameInternal, b_index);
  b_obj->NameInternal->data[0] = 'j';
  b_obj->NameInternal->data[1] = 'n';
  b_obj->NameInternal->data[2] = 't';
  b_obj->NameInternal->data[3] = '2';
  b_index = b_obj->Type->size[0] * b_obj->Type->size[1];
  b_obj->Type->size[0] = 1;
  b_obj->Type->size[1] = 8;
  emxEnsureCapacity_char_T(b_obj->Type, b_index);
  for (b_index = 0; b_index < 8; b_index++) {
    b_obj->Type->data[b_index] = cv10[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = b_obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = b_obj->Type->size[0] * b_obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = b_obj->Type->data[b_index];
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
      msubspace_data[b_index] = iv16[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv17[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    b_obj->VelocityNumber = 0.0;
    b_obj->PositionNumber = 0.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = b_obj->MotionSubspace->size[0] * b_obj->MotionSubspace->size[1];
  b_obj->MotionSubspace->size[0] = 6;
  b_obj->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(b_obj->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    b_obj->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = b_obj->PositionLimitsInternal->size[0] *
    b_obj->PositionLimitsInternal->size[1];
  b_obj->PositionLimitsInternal->size[0] = 1;
  b_obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(b_obj->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    b_obj->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = b_obj->HomePositionInternal->size[0];
  b_obj->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(b_obj->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    b_obj->HomePositionInternal->data[0] = 0.0;
  }

  return b_obj;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void d_Joint_setFixedTransform(robotics_Joint *obj)
{
  emxArray_char_T *switch_expression;
  int b_index;
  int loop_ub;
  boolean_T result;
  double TL[16];
  static const double TL_tmp[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.52, 0.0, 0.0, 1.0 };

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = obj->Type->size[0] * obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = obj->Type->data[b_index];
  }

  result = b_strcmp(switch_expression);
  if (result) {
    b_index = 0;
  } else {
    result = c_strcmp(switch_expression);
    if (result) {
      b_index = 1;
    } else {
      result = d_strcmp(switch_expression);
      if (result) {
        b_index = 2;
      } else {
        b_index = -1;
      }
    }
  }

  emxFree_char_T(&switch_expression);
  switch (b_index) {
   case 0:
    memcpy(&TL[0], &TL_tmp[0], sizeof(double) << 4);
    break;

   case 1:
    memcpy(&TL[0], &TL_tmp[0], sizeof(double) << 4);
    break;

   case 2:
    memcpy(&TL[0], &TL_tmp[0], sizeof(double) << 4);
    break;
  }

  for (b_index = 0; b_index < 16; b_index++) {
    obj->JointToParentTransform[b_index] = TL[b_index];
    obj->ChildToJointTransform[b_index] = 0.0;
  }

  obj->ChildToJointTransform[0] = 1.0;
  obj->ChildToJointTransform[5] = 1.0;
  obj->ChildToJointTransform[10] = 1.0;
  obj->ChildToJointTransform[15] = 1.0;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : void
 */
void d_Joint_set_PositionLimits(robotics_Joint *obj)
{
  boolean_T resetHome;
  emxArray_boolean_T *b_obj;
  int i12;
  int loop_ub;
  resetHome = false;
  switch ((int)obj->PositionNumber) {
   case 0:
    break;

   default:
    emxInit_boolean_T(&b_obj, 1);
    i12 = b_obj->size[0];
    b_obj->size[0] = obj->HomePositionInternal->size[0];
    emxEnsureCapacity_boolean_T(b_obj, i12);
    loop_ub = obj->HomePositionInternal->size[0];
    for (i12 = 0; i12 < loop_ub; i12++) {
      b_obj->data[i12] = (obj->HomePositionInternal->data[i12] >
                          1.7453292519943295);
    }

    if (any(b_obj)) {
      resetHome = true;
    } else {
      i12 = b_obj->size[0];
      b_obj->size[0] = obj->HomePositionInternal->size[0];
      emxEnsureCapacity_boolean_T(b_obj, i12);
      loop_ub = obj->HomePositionInternal->size[0];
      for (i12 = 0; i12 < loop_ub; i12++) {
        b_obj->data[i12] = (obj->HomePositionInternal->data[i12] <
                            -1.7453292519943295);
      }

      if (any(b_obj)) {
        resetHome = true;
      }
    }

    emxFree_boolean_T(&b_obj);
    break;
  }

  i12 = obj->PositionLimitsInternal->size[0] * obj->PositionLimitsInternal->
    size[1];
  obj->PositionLimitsInternal->size[0] = 1;
  obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(obj->PositionLimitsInternal, i12);
  obj->PositionLimitsInternal->data[0] = -1.7453292519943295;
  obj->PositionLimitsInternal->data[1] = 1.7453292519943295;
  if (resetHome) {
    Joint_resetHomePosition(obj);
  }
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : robotics_Joint *
 */
robotics_Joint *e_Joint_Joint(robotics_Joint *obj)
{
  robotics_Joint *b_obj;
  int b_index;
  emxArray_char_T *switch_expression;
  static const char cv12[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv20[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv21[6] = { 0, 0, 0, 0, 0, 1 };

  b_obj = obj;
  b_obj->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 4;
  emxEnsureCapacity_char_T(b_obj->NameInternal, b_index);
  b_obj->NameInternal->data[0] = 'j';
  b_obj->NameInternal->data[1] = 'n';
  b_obj->NameInternal->data[2] = 't';
  b_obj->NameInternal->data[3] = '3';
  b_index = b_obj->Type->size[0] * b_obj->Type->size[1];
  b_obj->Type->size[0] = 1;
  b_obj->Type->size[1] = 8;
  emxEnsureCapacity_char_T(b_obj->Type, b_index);
  for (b_index = 0; b_index < 8; b_index++) {
    b_obj->Type->data[b_index] = cv12[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = b_obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = b_obj->Type->size[0] * b_obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = b_obj->Type->data[b_index];
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
      msubspace_data[b_index] = iv20[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv21[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    b_obj->VelocityNumber = 0.0;
    b_obj->PositionNumber = 0.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = b_obj->MotionSubspace->size[0] * b_obj->MotionSubspace->size[1];
  b_obj->MotionSubspace->size[0] = 6;
  b_obj->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(b_obj->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    b_obj->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = b_obj->PositionLimitsInternal->size[0] *
    b_obj->PositionLimitsInternal->size[1];
  b_obj->PositionLimitsInternal->size[0] = 1;
  b_obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(b_obj->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    b_obj->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = b_obj->HomePositionInternal->size[0];
  b_obj->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(b_obj->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    b_obj->HomePositionInternal->data[0] = 0.0;
  }

  return b_obj;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : robotics_Joint *
 */
robotics_Joint *f_Joint_Joint(robotics_Joint *obj)
{
  robotics_Joint *b_obj;
  int b_index;
  emxArray_char_T *switch_expression;
  static const char cv14[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv24[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv25[6] = { 0, 0, 0, 0, 0, 1 };

  b_obj = obj;
  b_obj->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 4;
  emxEnsureCapacity_char_T(b_obj->NameInternal, b_index);
  b_obj->NameInternal->data[0] = 'j';
  b_obj->NameInternal->data[1] = 'n';
  b_obj->NameInternal->data[2] = 't';
  b_obj->NameInternal->data[3] = '4';
  b_index = b_obj->Type->size[0] * b_obj->Type->size[1];
  b_obj->Type->size[0] = 1;
  b_obj->Type->size[1] = 8;
  emxEnsureCapacity_char_T(b_obj->Type, b_index);
  for (b_index = 0; b_index < 8; b_index++) {
    b_obj->Type->data[b_index] = cv14[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = b_obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = b_obj->Type->size[0] * b_obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = b_obj->Type->data[b_index];
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
      msubspace_data[b_index] = iv24[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv25[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    b_obj->VelocityNumber = 0.0;
    b_obj->PositionNumber = 0.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = b_obj->MotionSubspace->size[0] * b_obj->MotionSubspace->size[1];
  b_obj->MotionSubspace->size[0] = 6;
  b_obj->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(b_obj->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    b_obj->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = b_obj->PositionLimitsInternal->size[0] *
    b_obj->PositionLimitsInternal->size[1];
  b_obj->PositionLimitsInternal->size[0] = 1;
  b_obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(b_obj->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    b_obj->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = b_obj->HomePositionInternal->size[0];
  b_obj->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(b_obj->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    b_obj->HomePositionInternal->data[0] = 0.0;
  }

  return b_obj;
}

/*
 * Arguments    : robotics_Joint *obj
 * Return Type  : robotics_Joint *
 */
robotics_Joint *g_Joint_Joint(robotics_Joint *obj)
{
  robotics_Joint *b_obj;
  int b_index;
  emxArray_char_T *switch_expression;
  static const char cv16[5] = { 'f', 'i', 'x', 'e', 'd' };

  int loop_ub;
  boolean_T result;
  double poslim_data[12];
  signed char msubspace_data[36];
  static const signed char iv28[6] = { 0, 0, 1, 0, 0, 0 };

  static const signed char iv29[6] = { 0, 0, 0, 0, 0, 1 };

  b_obj = obj;
  b_obj->InTree = false;
  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->JointToParentTransform[b_index] = iv0[b_index];
  }

  for (b_index = 0; b_index < 16; b_index++) {
    b_obj->ChildToJointTransform[b_index] = iv0[b_index];
  }

  b_index = b_obj->NameInternal->size[0] * b_obj->NameInternal->size[1];
  b_obj->NameInternal->size[0] = 1;
  b_obj->NameInternal->size[1] = 4;
  emxEnsureCapacity_char_T(b_obj->NameInternal, b_index);
  b_obj->NameInternal->data[0] = 'j';
  b_obj->NameInternal->data[1] = 'n';
  b_obj->NameInternal->data[2] = 't';
  b_obj->NameInternal->data[3] = '5';
  b_index = b_obj->Type->size[0] * b_obj->Type->size[1];
  b_obj->Type->size[0] = 1;
  b_obj->Type->size[1] = 5;
  emxEnsureCapacity_char_T(b_obj->Type, b_index);
  for (b_index = 0; b_index < 5; b_index++) {
    b_obj->Type->data[b_index] = cv16[b_index];
  }

  emxInit_char_T(&switch_expression, 2);
  b_index = switch_expression->size[0] * switch_expression->size[1];
  switch_expression->size[0] = 1;
  switch_expression->size[1] = b_obj->Type->size[1];
  emxEnsureCapacity_char_T(switch_expression, b_index);
  loop_ub = b_obj->Type->size[0] * b_obj->Type->size[1];
  for (b_index = 0; b_index < loop_ub; b_index++) {
    switch_expression->data[b_index] = b_obj->Type->data[b_index];
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
      msubspace_data[b_index] = iv28[b_index];
    }

    poslim_data[0] = -3.1415926535897931;
    poslim_data[1] = 3.1415926535897931;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   case 1:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = iv29[b_index];
    }

    poslim_data[0] = -0.5;
    poslim_data[1] = 0.5;
    b_obj->VelocityNumber = 1.0;
    b_obj->PositionNumber = 1.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 1.0;
    break;

   default:
    for (b_index = 0; b_index < 6; b_index++) {
      msubspace_data[b_index] = 0;
    }

    poslim_data[0] = 0.0;
    poslim_data[1] = 0.0;
    b_obj->VelocityNumber = 0.0;
    b_obj->PositionNumber = 0.0;
    b_obj->JointAxisInternal[0] = 0.0;
    b_obj->JointAxisInternal[1] = 0.0;
    b_obj->JointAxisInternal[2] = 0.0;
    break;
  }

  b_index = b_obj->MotionSubspace->size[0] * b_obj->MotionSubspace->size[1];
  b_obj->MotionSubspace->size[0] = 6;
  b_obj->MotionSubspace->size[1] = 1;
  emxEnsureCapacity_real_T(b_obj->MotionSubspace, b_index);
  for (b_index = 0; b_index < 6; b_index++) {
    b_obj->MotionSubspace->data[b_index] = msubspace_data[b_index];
  }

  b_index = b_obj->PositionLimitsInternal->size[0] *
    b_obj->PositionLimitsInternal->size[1];
  b_obj->PositionLimitsInternal->size[0] = 1;
  b_obj->PositionLimitsInternal->size[1] = 2;
  emxEnsureCapacity_real_T(b_obj->PositionLimitsInternal, b_index);
  for (b_index = 0; b_index < 2; b_index++) {
    b_obj->PositionLimitsInternal->data[b_index] = poslim_data[b_index];
  }

  b_index = b_obj->HomePositionInternal->size[0];
  b_obj->HomePositionInternal->size[0] = 1;
  emxEnsureCapacity_real_T(b_obj->HomePositionInternal, b_index);
  for (b_index = 0; b_index < 1; b_index++) {
    b_obj->HomePositionInternal->data[0] = 0.0;
  }

  return b_obj;
}

/*
 * File trailer for Joint.c
 *
 * [EOF]
 */
