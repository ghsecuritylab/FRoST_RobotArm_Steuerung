/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IK_solver_emxutil.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef IK_SOLVER_EMXUTIL_H
#define IK_SOLVER_EMXUTIL_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern void c_emxFreeStruct_robotics_Invers(robotics_InverseKinematics *pStruct);
extern void c_emxFreeStruct_robotics_core_i(c_robotics_core_internal_Damped
  *pStruct);
extern void c_emxFreeStruct_robotics_manip_(c_robotics_manip_internal_IKExt
  *pStruct);
extern void c_emxInitStruct_robotics_Invers(robotics_InverseKinematics *pStruct);
extern void c_emxInitStruct_robotics_core_i(c_robotics_core_internal_Damped
  *pStruct);
extern void c_emxInitStruct_robotics_manip_(c_robotics_manip_internal_IKExt
  *pStruct);
extern void d_emxFreeStruct_robotics_manip_(c_robotics_manip_internal_Rigid
  *pStruct);
extern void d_emxInitStruct_robotics_manip_(c_robotics_manip_internal_Rigid
  *pStruct);
extern void e_emxFreeStruct_robotics_manip_(d_robotics_manip_internal_Rigid
  *pStruct);
extern void e_emxInitStruct_robotics_manip_(d_robotics_manip_internal_Rigid
  *pStruct);
extern void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int
  oldNumel);
extern void emxEnsureCapacity_char_T(emxArray_char_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_struct_T(b_emxArray_struct_T *emxArray, int
  oldNumel);
extern void emxEnsureCapacity_struct_T1(emxArray_struct_T *emxArray, int
  oldNumel);
extern void emxEnsureCapacity_uint32_T(emxArray_uint32_T *emxArray, int oldNumel);
extern void emxFreeStruct_robotics_Joint(robotics_Joint *pStruct);
extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
extern void emxFree_char_T(emxArray_char_T **pEmxArray);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_int8_T(emxArray_int8_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxFree_struct_T(emxArray_struct_T **pEmxArray);
extern void emxFree_struct_T1(b_emxArray_struct_T **pEmxArray);
extern void emxFree_uint32_T(emxArray_uint32_T **pEmxArray);
extern void emxInitStruct_robotics_Joint(robotics_Joint *pStruct);
extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions);
extern void emxInit_char_T(emxArray_char_T **pEmxArray, int numDimensions);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
extern void emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxInit_struct_T(emxArray_struct_T **pEmxArray, int numDimensions);
extern void emxInit_struct_T1(b_emxArray_struct_T **pEmxArray, int numDimensions);
extern void emxInit_uint32_T(emxArray_uint32_T **pEmxArray, int numDimensions);
extern void f_emxFreeStruct_robotics_manip_(e_robotics_manip_internal_Rigid
  *pStruct);
extern void f_emxInitStruct_robotics_manip_(e_robotics_manip_internal_Rigid
  *pStruct);
extern void g_emxFreeStruct_robotics_manip_(f_robotics_manip_internal_Rigid
  *pStruct);
extern void g_emxInitStruct_robotics_manip_(f_robotics_manip_internal_Rigid
  *pStruct);

#endif

/*
 * File trailer for IK_solver_emxutil.h
 *
 * [EOF]
 */
