/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Joint.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef JOINT_H
#define JOINT_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern robotics_Joint *Joint_Joint(robotics_Joint *obj);
extern robotics_Joint *Joint_copy(const robotics_Joint *obj, robotics_Joint
  *iobj_0);
extern void Joint_setFixedTransform(robotics_Joint *obj);
extern void Joint_set_HomePosition(robotics_Joint *obj);
extern void Joint_set_JointAxis(robotics_Joint *obj);
extern void Joint_set_PositionLimits(robotics_Joint *obj);
extern void Joint_transformBodyToParent(const robotics_Joint *obj, double T[16]);
extern robotics_Joint *b_Joint_Joint(robotics_Joint *obj);
extern void b_Joint_setFixedTransform(robotics_Joint *obj);
extern void b_Joint_set_HomePosition(robotics_Joint *obj);
extern void b_Joint_set_JointAxis(robotics_Joint *obj);
extern void b_Joint_set_PositionLimits(robotics_Joint *obj);
extern void b_Joint_transformBodyToParent(const robotics_Joint *obj, const
  emxArray_real_T *q, double T[16]);
extern robotics_Joint *c_Joint_Joint(robotics_Joint *obj);
extern void c_Joint_setFixedTransform(robotics_Joint *obj);
extern void c_Joint_set_PositionLimits(robotics_Joint *obj);
extern robotics_Joint *d_Joint_Joint(robotics_Joint *obj);
extern void d_Joint_setFixedTransform(robotics_Joint *obj);
extern void d_Joint_set_PositionLimits(robotics_Joint *obj);
extern robotics_Joint *e_Joint_Joint(robotics_Joint *obj);
extern robotics_Joint *f_Joint_Joint(robotics_Joint *obj);
extern robotics_Joint *g_Joint_Joint(robotics_Joint *obj);

#endif

/*
 * File trailer for Joint.h
 *
 * [EOF]
 */
