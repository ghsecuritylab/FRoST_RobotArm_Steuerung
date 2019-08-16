/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: InverseKinematics.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern void InverseKinematics_stepImpl(const robotics_InverseKinematics *obj,
  const double tform[16], const c_struct_T initialGuess[4], emxArray_struct_T
  *QSol);
extern robotics_InverseKinematics *c_InverseKinematics_InverseKine
  (robotics_InverseKinematics *obj, const robotics_RigidBodyTree *varargin_2,
   c_robotics_core_internal_Damped *iobj_0, d_robotics_manip_internal_Rigid
   *iobj_1, d_robotics_manip_internal_Rigid *iobj_2,
   c_robotics_manip_internal_Rigid *iobj_3, c_robotics_manip_internal_Rigid
   *iobj_4, c_robotics_manip_internal_Rigid *iobj_5,
   c_robotics_manip_internal_Rigid *iobj_6, c_robotics_manip_internal_Rigid
   *iobj_7, c_robotics_manip_internal_Rigid *iobj_8,
   c_robotics_manip_internal_Rigid *iobj_9, c_robotics_manip_internal_Rigid
   *iobj_10, c_robotics_manip_internal_Rigid *iobj_11,
   c_robotics_manip_internal_Rigid *iobj_12, c_robotics_manip_internal_Rigid
   *iobj_13, c_robotics_manip_internal_Rigid *iobj_14,
   c_robotics_manip_internal_Rigid *iobj_15, c_robotics_manip_internal_Rigid
   *iobj_16, c_robotics_manip_internal_Rigid *iobj_17, robotics_Joint *iobj_18,
   robotics_Joint *iobj_19, robotics_Joint *iobj_20, robotics_Joint *iobj_21,
   robotics_Joint *iobj_22, robotics_Joint *iobj_23, robotics_Joint *iobj_24,
   robotics_Joint *iobj_25, robotics_Joint *iobj_26, robotics_Joint *iobj_27,
   robotics_Joint *iobj_28, robotics_Joint *iobj_29, robotics_Joint *iobj_30,
   robotics_Joint *iobj_31, robotics_Joint *iobj_32, robotics_Joint *iobj_33,
   robotics_Joint *iobj_34, robotics_Joint *iobj_35, robotics_Joint *iobj_36,
   robotics_Joint *iobj_37, robotics_Joint *iobj_38, robotics_Joint *iobj_39);

#endif

/*
 * File trailer for InverseKinematics.h
 *
 * [EOF]
 */
