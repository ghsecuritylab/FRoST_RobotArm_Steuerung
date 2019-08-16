/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RigidBody.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern c_robotics_manip_internal_Rigid *RigidBody_RigidBody
  (c_robotics_manip_internal_Rigid *obj, robotics_Joint *iobj_0);
extern c_robotics_manip_internal_Rigid *RigidBody_copy(const
  c_robotics_manip_internal_Rigid *obj, c_robotics_manip_internal_Rigid *iobj_0,
  robotics_Joint *iobj_1, robotics_Joint *iobj_2);

#endif

/*
 * File trailer for RigidBody.h
 *
 * [EOF]
 */
