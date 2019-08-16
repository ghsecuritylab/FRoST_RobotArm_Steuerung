/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RigidBodyTree1.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef RIGIDBODYTREE1_H
#define RIGIDBODYTREE1_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern d_robotics_manip_internal_Rigid *RigidBodyTree_RigidBodyTree
  (d_robotics_manip_internal_Rigid *obj, robotics_Joint *iobj_0,
   c_robotics_manip_internal_Rigid *iobj_1, c_robotics_manip_internal_Rigid
   *iobj_2, c_robotics_manip_internal_Rigid *iobj_3,
   c_robotics_manip_internal_Rigid *iobj_4, c_robotics_manip_internal_Rigid
   *iobj_5, robotics_Joint *iobj_6, robotics_Joint *iobj_7, robotics_Joint
   *iobj_8, robotics_Joint *iobj_9, robotics_Joint *iobj_10);
extern f_robotics_manip_internal_Rigid *b_RigidBodyTree_RigidBodyTree
  (f_robotics_manip_internal_Rigid *obj, e_robotics_manip_internal_Rigid *iobj_0);
extern void c_RigidBodyTree_efficientFKAndJ(d_robotics_manip_internal_Rigid *obj,
  const emxArray_real_T *qv, const emxArray_char_T *body1Name, double T_data[],
  int T_size[2], emxArray_real_T *Jac);
extern double c_RigidBodyTree_findBodyIndexBy(const
  d_robotics_manip_internal_Rigid *obj, const emxArray_char_T *bodyname);
extern void c_RigidBodyTree_get_JointPositi(const
  d_robotics_manip_internal_Rigid *obj, emxArray_real_T *limits);
extern void c_RigidBodyTree_validateConfigu(const
  d_robotics_manip_internal_Rigid *obj, const c_struct_T Q[4], emxArray_real_T
  *qvec);
extern double c_RigidBodyTree_validateInputBo(const
  d_robotics_manip_internal_Rigid *obj, const emxArray_char_T *bodyname);
extern double d_RigidBodyTree_findBodyIndexBy(const
  d_robotics_manip_internal_Rigid *obj, const emxArray_char_T *jointname);
extern void f_RigidBodyTree_addBody(d_robotics_manip_internal_Rigid *obj, const
  c_robotics_manip_internal_Rigid *bodyin, const emxArray_char_T *parentName,
  c_robotics_manip_internal_Rigid *iobj_0, robotics_Joint *iobj_1,
  robotics_Joint *iobj_2);

#endif

/*
 * File trailer for RigidBodyTree1.h
 *
 * [EOF]
 */
