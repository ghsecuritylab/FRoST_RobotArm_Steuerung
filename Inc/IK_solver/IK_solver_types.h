/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IK_solver_types.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef IK_SOLVER_TYPES_H
#define IK_SOLVER_TYPES_H

/* Include Files */
#include "IK_solver/rtwtypes.h"

/* Type Definitions */
#ifndef struct_emxArray_char_T_1x200
#define struct_emxArray_char_T_1x200

struct emxArray_char_T_1x200
{
  char data[200];
  int size[2];
};

#endif                                 /*struct_emxArray_char_T_1x200*/

#ifndef typedef_emxArray_char_T_1x200
#define typedef_emxArray_char_T_1x200

typedef struct emxArray_char_T_1x200 emxArray_char_T_1x200;

#endif                                 /*typedef_emxArray_char_T_1x200*/

#ifndef struct_emxArray_real_T_1x7
#define struct_emxArray_real_T_1x7

struct emxArray_real_T_1x7
{
  double data[7];
  int size[2];
};

#endif                                 /*struct_emxArray_real_T_1x7*/

#ifndef typedef_emxArray_real_T_1x7
#define typedef_emxArray_real_T_1x7

typedef struct emxArray_real_T_1x7 emxArray_real_T_1x7;

#endif                                 /*typedef_emxArray_real_T_1x7*/

#ifndef typedef_b_struct_T
#define typedef_b_struct_T

typedef struct {
  emxArray_char_T_1x200 JointName;
  emxArray_real_T_1x7 JointPosition;
} b_struct_T;

#endif                                 /*typedef_b_struct_T*/

#ifndef typedef_b_emxArray_struct_T
#define typedef_b_emxArray_struct_T

typedef struct {
  b_struct_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
} b_emxArray_struct_T;

#endif                                 /*typedef_b_emxArray_struct_T*/

#ifndef typedef_c_robotics_core_internal_System
#define typedef_c_robotics_core_internal_System

typedef struct {
  double StartTime;
} c_robotics_core_internal_System;

#endif                                 /*typedef_c_robotics_core_internal_System*/

#ifndef struct_emxArray_char_T
#define struct_emxArray_char_T

struct emxArray_char_T
{
  char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_char_T*/

#ifndef typedef_emxArray_char_T
#define typedef_emxArray_char_T

typedef struct emxArray_char_T emxArray_char_T;

#endif                                 /*typedef_emxArray_char_T*/

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

#ifndef typedef_robotics_Joint
#define typedef_robotics_Joint

typedef struct {
  emxArray_char_T *Type;
  double VelocityNumber;
  double PositionNumber;
  emxArray_real_T *MotionSubspace;
  boolean_T InTree;
  double JointToParentTransform[16];
  double ChildToJointTransform[16];
  emxArray_char_T *NameInternal;
  emxArray_real_T *PositionLimitsInternal;
  emxArray_real_T *HomePositionInternal;
  double JointAxisInternal[3];
} robotics_Joint;

#endif                                 /*typedef_robotics_Joint*/

#ifndef typedef_c_robotics_manip_internal_Rigid
#define typedef_c_robotics_manip_internal_Rigid

typedef struct {
  double Index;
  emxArray_char_T *NameInternal;
  robotics_Joint *JointInternal;
  double ParentIndex;
  double MassInternal;
  double CenterOfMassInternal[3];
  double InertiaInternal[9];
  double SpatialInertia[36];
} c_robotics_manip_internal_Rigid;

#endif                                 /*typedef_c_robotics_manip_internal_Rigid*/

#ifndef typedef_d_robotics_manip_internal_Rigid
#define typedef_d_robotics_manip_internal_Rigid

typedef struct {
  double NumBodies;
  c_robotics_manip_internal_Rigid Base;
  double Gravity[3];
  c_robotics_manip_internal_Rigid *Bodies[5];
  double NumNonFixedBodies;
  double PositionNumber;
  double VelocityNumber;
  double PositionDoFMap[10];
  double VelocityDoFMap[10];
} d_robotics_manip_internal_Rigid;

#endif                                 /*typedef_d_robotics_manip_internal_Rigid*/

#ifndef typedef_c_robotics_manip_internal_IKExt
#define typedef_c_robotics_manip_internal_IKExt

typedef struct {
  d_robotics_manip_internal_Rigid *Robot;
  double WeightMatrix[36];
  emxArray_char_T *BodyName;
  double Tform[16];
  emxArray_real_T *ErrTemp;
  double CostTemp;
  emxArray_real_T *GradTemp;
} c_robotics_manip_internal_IKExt;

#endif                                 /*typedef_c_robotics_manip_internal_IKExt*/

#ifndef typedef_c_robotics_core_internal_Damped
#define typedef_c_robotics_core_internal_Damped

typedef struct {
  emxArray_real_T *ConstraintMatrix;
  emxArray_real_T *ConstraintBound;
  boolean_T ConstraintsOn;
  double SolutionTolerance;
  boolean_T RandomRestart;
  c_robotics_manip_internal_IKExt *ExtraArgs;
  double MaxNumIteration;
  double MaxTime;
  emxArray_real_T *SeedInternal;
  double MaxTimeInternal;
  double MaxNumIterationInternal;
  double StepTolerance;
  c_robotics_core_internal_System TimeObj;
  double GradientTolerance;
  double ArmijoRuleBeta;
  double ArmijoRuleSigma;
  c_robotics_core_internal_System TimeObjInternal;
} c_robotics_core_internal_Damped;

#endif                                 /*typedef_c_robotics_core_internal_Damped*/

#ifndef enum_c_robotics_core_internal_NLPSol
#define enum_c_robotics_core_internal_NLPSol

enum c_robotics_core_internal_NLPSol
{
  LocalMinimumFound = 1,               /* Default value */
  IterationLimitExceeded,
  TimeLimitExceeded,
  StepSizeBelowMinimum,
  ChangeInErrorBelowMinimum,
  SearchDirectionInvalid,
  HessianNotPositiveSemidefinite,
  TrustRegionRadiusBelowMinimum
};

#endif                                 /*enum_c_robotics_core_internal_NLPSol*/

#ifndef typedef_c_robotics_core_internal_NLPSol
#define typedef_c_robotics_core_internal_NLPSol

typedef enum c_robotics_core_internal_NLPSol c_robotics_core_internal_NLPSol;

#endif                                 /*typedef_c_robotics_core_internal_NLPSol*/

#ifndef typedef_c_struct_T
#define typedef_c_struct_T

typedef struct {
  char JointName[4];
  double JointPosition;
} c_struct_T;

#endif                                 /*typedef_c_struct_T*/

#ifndef typedef_e_robotics_manip_internal_Rigid
#define typedef_e_robotics_manip_internal_Rigid

typedef struct {
  robotics_Joint JointInternal;
} e_robotics_manip_internal_Rigid;

#endif                                 /*typedef_e_robotics_manip_internal_Rigid*/

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T

struct emxArray_boolean_T
{
  boolean_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_boolean_T*/

#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T

typedef struct emxArray_boolean_T emxArray_boolean_T;

#endif                                 /*typedef_emxArray_boolean_T*/

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_int32_T*/

#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T

typedef struct emxArray_int32_T emxArray_int32_T;

#endif                                 /*typedef_emxArray_int32_T*/

#ifndef struct_emxArray_int8_T
#define struct_emxArray_int8_T

struct emxArray_int8_T
{
  signed char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_int8_T*/

#ifndef typedef_emxArray_int8_T
#define typedef_emxArray_int8_T

typedef struct emxArray_int8_T emxArray_int8_T;

#endif                                 /*typedef_emxArray_int8_T*/

#ifndef struct_emxArray_real_T_1x1
#define struct_emxArray_real_T_1x1

struct emxArray_real_T_1x1
{
  double data[1];
  int size[2];
};

#endif                                 /*struct_emxArray_real_T_1x1*/

#ifndef typedef_emxArray_real_T_1x1
#define typedef_emxArray_real_T_1x1

typedef struct emxArray_real_T_1x1 emxArray_real_T_1x1;

#endif                                 /*typedef_emxArray_real_T_1x1*/

#ifndef struct_sys7SBABFfL9diW6etWLGGF_tag
#define struct_sys7SBABFfL9diW6etWLGGF_tag

struct sys7SBABFfL9diW6etWLGGF_tag
{
  emxArray_char_T_1x200 JointName;
  emxArray_real_T_1x1 JointPosition;
};

#endif                                 /*struct_sys7SBABFfL9diW6etWLGGF_tag*/

#ifndef typedef_struct_T
#define typedef_struct_T

typedef struct sys7SBABFfL9diW6etWLGGF_tag struct_T;

#endif                                 /*typedef_struct_T*/

#ifndef struct_c_emxArray_sys7SBABFfL9diW6etWL
#define struct_c_emxArray_sys7SBABFfL9diW6etWL

struct c_emxArray_sys7SBABFfL9diW6etWL
{
  struct_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_c_emxArray_sys7SBABFfL9diW6etWL*/

#ifndef typedef_emxArray_struct_T
#define typedef_emxArray_struct_T

typedef struct c_emxArray_sys7SBABFfL9diW6etWL emxArray_struct_T;

#endif                                 /*typedef_emxArray_struct_T*/

#ifndef struct_emxArray_uint32_T
#define struct_emxArray_uint32_T

struct emxArray_uint32_T
{
  unsigned int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_uint32_T*/

#ifndef typedef_emxArray_uint32_T
#define typedef_emxArray_uint32_T

typedef struct emxArray_uint32_T emxArray_uint32_T;

#endif                                 /*typedef_emxArray_uint32_T*/

#ifndef typedef_f_robotics_manip_internal_Rigid
#define typedef_f_robotics_manip_internal_Rigid

typedef struct {
  e_robotics_manip_internal_Rigid Base;
  e_robotics_manip_internal_Rigid *Bodies[1];
} f_robotics_manip_internal_Rigid;

#endif                                 /*typedef_f_robotics_manip_internal_Rigid*/

#ifndef typedef_robotics_InverseKinematics
#define typedef_robotics_InverseKinematics

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int isInitialized;
  boolean_T isSetupComplete;
  c_robotics_core_internal_Damped *Solver;
  emxArray_real_T *Limits;
  d_robotics_manip_internal_Rigid *RigidBodyTreeInternal;
} robotics_InverseKinematics;

#endif                                 /*typedef_robotics_InverseKinematics*/

#ifndef typedef_robotics_RigidBody
#define typedef_robotics_RigidBody

typedef struct {
  c_robotics_manip_internal_Rigid *BodyInternal;
  f_robotics_manip_internal_Rigid *TreeInternal;
} robotics_RigidBody;

#endif                                 /*typedef_robotics_RigidBody*/

#ifndef typedef_robotics_RigidBodyTree
#define typedef_robotics_RigidBodyTree

typedef struct {
  d_robotics_manip_internal_Rigid *TreeInternal;
} robotics_RigidBodyTree;

#endif                                 /*typedef_robotics_RigidBodyTree*/
#endif

/*
 * File trailer for IK_solver_types.h
 *
 * [EOF]
 */
