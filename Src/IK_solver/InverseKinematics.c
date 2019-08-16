/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: InverseKinematics.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include <string.h>
#include <math.h>
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/InverseKinematics.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/strcmp.h"
#include "IK_solver/RigidBodyTree1.h"
#include "IK_solver/NLPSolverInterface.h"
#include "IK_solver/IK_solver_data.h"

/* Function Declarations */
static void InverseKinematics_setPoseGoal(const robotics_InverseKinematics *obj,
  const double tform[16]);
static void c_InverseKinematics_set_SolverA(robotics_InverseKinematics *obj,
  c_robotics_core_internal_Damped *iobj_0);

/* Function Definitions */

/*
 * Arguments    : const robotics_InverseKinematics *obj
 *                const double tform[16]
 * Return Type  : void
 */
static void InverseKinematics_setPoseGoal(const robotics_InverseKinematics *obj,
  const double tform[16])
{
  double weightMatrix[36];
  int j;
  c_robotics_manip_internal_IKExt *args;
  static const double dv2[6] = { 0.1, 0.1, 0.0, 1.0, 1.0, 1.0 };

  memset(&weightMatrix[0], 0, 36U * sizeof(double));
  for (j = 0; j < 6; j++) {
    weightMatrix[j + 6 * j] = dv2[j];
  }

  args = obj->Solver->ExtraArgs;
  for (j = 0; j < 36; j++) {
    args->WeightMatrix[j] = weightMatrix[j];
  }

  j = args->BodyName->size[0] * args->BodyName->size[1];
  args->BodyName->size[0] = 1;
  args->BodyName->size[1] = 12;
  emxEnsureCapacity_char_T(args->BodyName, j);
  for (j = 0; j < 12; j++) {
    args->BodyName->data[j] = cv1[j];
  }

  for (j = 0; j < 16; j++) {
    args->Tform[j] = tform[j];
  }
}

/*
 * Arguments    : robotics_InverseKinematics *obj
 *                c_robotics_core_internal_Damped *iobj_0
 * Return Type  : void
 */
static void c_InverseKinematics_set_SolverA(robotics_InverseKinematics *obj,
  c_robotics_core_internal_Damped *iobj_0)
{
  iobj_0->MaxNumIteration = 1500.0;
  iobj_0->MaxTime = 10.0;
  iobj_0->GradientTolerance = 1.0E-7;
  iobj_0->SolutionTolerance = 1.0E-6;
  iobj_0->ArmijoRuleBeta = 0.4;
  iobj_0->ArmijoRuleSigma = 1.0E-5;
  iobj_0->ConstraintsOn = true;
  iobj_0->RandomRestart = true;
  iobj_0->StepTolerance = 1.0E-14;
  iobj_0->ConstraintMatrix->size[0] = 0;
  iobj_0->ConstraintMatrix->size[1] = 0;
  iobj_0->ConstraintBound->size[0] = 0;
  iobj_0->TimeObj.StartTime = -1.0;
  iobj_0->TimeObjInternal.StartTime = -1.0;
  obj->Solver = iobj_0;
}

/*
 * Arguments    : const robotics_InverseKinematics *obj
 *                const double tform[16]
 *                const c_struct_T initialGuess[4]
 *                emxArray_struct_T *QSol
 * Return Type  : void
 */
void InverseKinematics_stepImpl(const robotics_InverseKinematics *obj, const
  double tform[16], const c_struct_T initialGuess[4], emxArray_struct_T *QSol)
{
  emxArray_real_T *iniGuessVec;
  emxArray_real_T *qvSolRaw;
  emxArray_char_T *endEffectorName;
  double bid;
  double numPositions;
  double ndbl;
  double apnd;
  char expl_temp_data[14];
  int expl_temp_size[2];
  d_robotics_manip_internal_Rigid *b_obj;
  int i19;
  int loop_ub;
  emxArray_real_T *bodyIndices;
  c_robotics_manip_internal_Rigid *body;
  unsigned int i;
  int end;
  emxArray_real_T *b_bodyIndices;
  int nm1d2;
  int b_i;
  emxArray_int32_T *r4;
  emxArray_real_T *positionMap;
  emxArray_real_T *positionIndices;
  emxArray_int32_T *r5;
  emxArray_real_T *y;
  int i20;
  b_emxArray_struct_T *Q;
  double cdiff;
  double absa;
  double absb;
  robotics_Joint *c_obj;
  emxInit_real_T(&iniGuessVec, 1);
  emxInit_real_T(&qvSolRaw, 1);
  emxInit_char_T(&endEffectorName, 2);
  InverseKinematics_setPoseGoal(obj, tform);
  c_RigidBodyTree_validateConfigu(obj->RigidBodyTreeInternal, initialGuess,
    iniGuessVec);
  NLPSolverInterface_solve(obj->Solver, iniGuessVec, qvSolRaw, &bid,
    &numPositions, &ndbl, &apnd, expl_temp_data, expl_temp_size);
  b_obj = obj->RigidBodyTreeInternal;
  i19 = endEffectorName->size[0] * endEffectorName->size[1];
  endEffectorName->size[0] = 1;
  endEffectorName->size[1] = obj->Solver->ExtraArgs->BodyName->size[1];
  emxEnsureCapacity_char_T(endEffectorName, i19);
  loop_ub = obj->Solver->ExtraArgs->BodyName->size[0] * obj->Solver->
    ExtraArgs->BodyName->size[1];
  for (i19 = 0; i19 < loop_ub; i19++) {
    endEffectorName->data[i19] = obj->Solver->ExtraArgs->BodyName->data[i19];
  }

  emxInit_real_T(&bodyIndices, 1);
  i19 = bodyIndices->size[0];
  bodyIndices->size[0] = (int)b_obj->NumBodies;
  emxEnsureCapacity_real_T(bodyIndices, i19);
  loop_ub = (int)b_obj->NumBodies;
  for (i19 = 0; i19 < loop_ub; i19++) {
    bodyIndices->data[i19] = 0.0;
  }

  bid = c_RigidBodyTree_validateInputBo(b_obj, endEffectorName);
  if (bid == 0.0) {
    i19 = bodyIndices->size[0];
    bodyIndices->size[0] = 1;
    emxEnsureCapacity_real_T(bodyIndices, i19);
    bodyIndices->data[0] = 0.0;
  } else {
    body = b_obj->Bodies[(int)bid - 1];
    i = 1U;
    while (body->ParentIndex != 0.0) {
      bodyIndices->data[(int)i - 1] = body->Index;
      body = b_obj->Bodies[(int)body->ParentIndex - 1];
      i++;
    }

    if (1 > (int)(i - 1U)) {
      loop_ub = 0;
    } else {
      loop_ub = (int)(i - 1U);
    }

    emxInit_real_T(&b_bodyIndices, 1);
    i19 = b_bodyIndices->size[0];
    b_bodyIndices->size[0] = loop_ub + 2;
    emxEnsureCapacity_real_T(b_bodyIndices, i19);
    for (i19 = 0; i19 < loop_ub; i19++) {
      b_bodyIndices->data[i19] = bodyIndices->data[i19];
    }

    b_bodyIndices->data[loop_ub] = body->Index;
    b_bodyIndices->data[loop_ub + 1] = 0.0;
    i19 = bodyIndices->size[0];
    bodyIndices->size[0] = b_bodyIndices->size[0];
    emxEnsureCapacity_real_T(bodyIndices, i19);
    loop_ub = b_bodyIndices->size[0];
    for (i19 = 0; i19 < loop_ub; i19++) {
      bodyIndices->data[i19] = b_bodyIndices->data[i19];
    }

    emxFree_real_T(&b_bodyIndices);
  }

  b_obj = obj->RigidBodyTreeInternal;
  end = bodyIndices->size[0] - 1;
  nm1d2 = 0;
  for (b_i = 0; b_i <= end; b_i++) {
    if (bodyIndices->data[b_i] != 0.0) {
      nm1d2++;
    }
  }

  emxInit_int32_T(&r4, 1);
  i19 = r4->size[0];
  r4->size[0] = nm1d2;
  emxEnsureCapacity_int32_T(r4, i19);
  nm1d2 = 0;
  for (b_i = 0; b_i <= end; b_i++) {
    if (bodyIndices->data[b_i] != 0.0) {
      r4->data[nm1d2] = b_i + 1;
      nm1d2++;
    }
  }

  emxInit_real_T(&positionMap, 2);
  i19 = positionMap->size[0] * positionMap->size[1];
  positionMap->size[0] = r4->size[0];
  positionMap->size[1] = 2;
  emxEnsureCapacity_real_T(positionMap, i19);
  loop_ub = r4->size[0];
  for (i19 = 0; i19 < loop_ub; i19++) {
    positionMap->data[i19] = b_obj->PositionDoFMap[(int)bodyIndices->data
      [r4->data[i19] - 1] - 1];
  }

  loop_ub = r4->size[0];
  for (i19 = 0; i19 < loop_ub; i19++) {
    positionMap->data[i19 + positionMap->size[0]] = b_obj->PositionDoFMap[(int)
      bodyIndices->data[r4->data[i19] - 1] + 4];
  }

  emxFree_real_T(&bodyIndices);
  emxInit_real_T(&positionIndices, 2);
  i19 = positionIndices->size[0] * positionIndices->size[1];
  positionIndices->size[0] = 1;
  positionIndices->size[1] = (int)b_obj->PositionNumber;
  emxEnsureCapacity_real_T(positionIndices, i19);
  loop_ub = (int)b_obj->PositionNumber;
  for (i19 = 0; i19 < loop_ub; i19++) {
    positionIndices->data[i19] = 0.0;
  }

  bid = 0.0;
  i19 = positionMap->size[0];
  emxInit_int32_T(&r5, 2);
  emxInit_real_T(&y, 2);
  for (b_i = 0; b_i < i19; b_i++) {
    numPositions = (positionMap->data[b_i + positionMap->size[0]] -
                    positionMap->data[b_i]) + 1.0;
    if (numPositions > 0.0) {
      if (numPositions < 1.0) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else if (rtIsInf(numPositions) && (1.0 == numPositions)) {
        i20 = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = 1;
        emxEnsureCapacity_real_T(y, i20);
        y->data[0] = rtNaN;
      } else {
        i20 = y->size[0] * y->size[1];
        y->size[0] = 1;
        loop_ub = (int)floor(numPositions - 1.0);
        y->size[1] = loop_ub + 1;
        emxEnsureCapacity_real_T(y, i20);
        for (i20 = 0; i20 <= loop_ub; i20++) {
          y->data[i20] = 1.0 + (double)i20;
        }
      }

      i20 = r5->size[0] * r5->size[1];
      r5->size[0] = 1;
      r5->size[1] = y->size[1];
      emxEnsureCapacity_int32_T(r5, i20);
      loop_ub = y->size[0] * y->size[1];
      for (i20 = 0; i20 < loop_ub; i20++) {
        r5->data[i20] = (int)(bid + y->data[i20]);
      }

      if (rtIsNaN(positionMap->data[b_i]) || rtIsNaN(positionMap->data[b_i +
           positionMap->size[0]])) {
        i20 = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = 1;
        emxEnsureCapacity_real_T(y, i20);
        y->data[0] = rtNaN;
      } else if (positionMap->data[b_i + positionMap->size[0]] <
                 positionMap->data[b_i]) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else if ((rtIsInf(positionMap->data[b_i]) || rtIsInf(positionMap->
                   data[b_i + positionMap->size[0]])) && (positionMap->data[b_i]
                  == positionMap->data[b_i + positionMap->size[0]])) {
        i20 = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = 1;
        emxEnsureCapacity_real_T(y, i20);
        y->data[0] = rtNaN;
      } else if (floor(positionMap->data[b_i]) == positionMap->data[b_i]) {
        i20 = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = (int)floor(positionMap->data[b_i + positionMap->size[0]] -
          positionMap->data[b_i]) + 1;
        emxEnsureCapacity_real_T(y, i20);
        loop_ub = (int)floor(positionMap->data[b_i + positionMap->size[0]] -
                             positionMap->data[b_i]);
        for (i20 = 0; i20 <= loop_ub; i20++) {
          y->data[i20] = positionMap->data[b_i] + (double)i20;
        }
      } else {
        ndbl = floor((positionMap->data[b_i + positionMap->size[0]] -
                      positionMap->data[b_i]) + 0.5);
        apnd = positionMap->data[b_i] + ndbl;
        cdiff = apnd - positionMap->data[b_i + positionMap->size[0]];
        absa = fabs(positionMap->data[b_i]);
        absb = fabs(positionMap->data[b_i + positionMap->size[0]]);
        if ((absa > absb) || rtIsNaN(absb)) {
          absb = absa;
        }

        if (fabs(cdiff) < 4.4408920985006262E-16 * absb) {
          ndbl++;
          apnd = positionMap->data[b_i + positionMap->size[0]];
        } else if (cdiff > 0.0) {
          apnd = positionMap->data[b_i] + (ndbl - 1.0);
        } else {
          ndbl++;
        }

        if (ndbl >= 0.0) {
          loop_ub = (int)ndbl;
        } else {
          loop_ub = 0;
        }

        i20 = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = loop_ub;
        emxEnsureCapacity_real_T(y, i20);
        if (loop_ub > 0) {
          y->data[0] = positionMap->data[b_i];
          if (loop_ub > 1) {
            y->data[loop_ub - 1] = apnd;
            nm1d2 = (loop_ub - 1) / 2;
            for (end = 0; end <= nm1d2 - 2; end++) {
              y->data[1 + end] = positionMap->data[b_i] + (1.0 + (double)end);
              y->data[(loop_ub - end) - 2] = apnd - (1.0 + (double)end);
            }

            if (nm1d2 << 1 == loop_ub - 1) {
              y->data[nm1d2] = (positionMap->data[b_i] + apnd) / 2.0;
            } else {
              y->data[nm1d2] = positionMap->data[b_i] + (double)nm1d2;
              y->data[nm1d2 + 1] = apnd - (double)nm1d2;
            }
          }
        }
      }

      loop_ub = y->size[0] * y->size[1] - 1;
      for (i20 = 0; i20 <= loop_ub; i20++) {
        positionIndices->data[r5->data[i20] - 1] = y->data[i20];
      }

      bid += numPositions;
    }
  }

  emxFree_real_T(&y);
  emxFree_int32_T(&r5);
  emxFree_real_T(&positionMap);
  i19 = positionIndices->size[0] * positionIndices->size[1];
  if (1.0 > bid) {
    positionIndices->size[1] = 0;
  } else {
    positionIndices->size[1] = (int)bid;
  }

  emxEnsureCapacity_real_T(positionIndices, i19);
  loop_ub = positionIndices->size[0] * positionIndices->size[1];
  for (i19 = 0; i19 < loop_ub; i19++) {
    iniGuessVec->data[(int)positionIndices->data[i19] - 1] = qvSolRaw->data[(int)
      positionIndices->data[i19] - 1];
  }

  emxFree_real_T(&qvSolRaw);
  emxFree_real_T(&positionIndices);
  emxInit_struct_T1(&Q, 2);
  b_obj = obj->RigidBodyTreeInternal;
  bid = b_obj->NumNonFixedBodies;
  i19 = Q->size[0] * Q->size[1];
  Q->size[0] = 1;
  i20 = (int)bid;
  Q->size[1] = i20;
  emxEnsureCapacity_struct_T(Q, i19);
  for (nm1d2 = 0; nm1d2 < i20; nm1d2++) {
    Q->data[nm1d2].JointName.size[0] = 1;
    Q->data[nm1d2].JointName.size[1] = 0;
    Q->data[nm1d2].JointPosition.size[0] = 1;
    Q->data[nm1d2].JointPosition.size[1] = 1;
    Q->data[nm1d2].JointPosition.data[0] = 0.0;
  }

  i = 1U;
  bid = b_obj->NumBodies;
  i19 = (int)bid;
  for (b_i = 0; b_i < i19; b_i++) {
    body = b_obj->Bodies[b_i];
    if (!d_strcmp(body->JointInternal->Type)) {
      nm1d2 = (int)body->Index;
      bid = b_obj->PositionDoFMap[nm1d2 - 1];
      numPositions = b_obj->PositionDoFMap[nm1d2 + 4];
      c_obj = body->JointInternal;
      i20 = endEffectorName->size[0] * endEffectorName->size[1];
      endEffectorName->size[0] = 1;
      endEffectorName->size[1] = c_obj->NameInternal->size[1];
      emxEnsureCapacity_char_T(endEffectorName, i20);
      loop_ub = c_obj->NameInternal->size[0] * c_obj->NameInternal->size[1];
      for (i20 = 0; i20 < loop_ub; i20++) {
        endEffectorName->data[i20] = c_obj->NameInternal->data[i20];
      }

      loop_ub = endEffectorName->size[0] * endEffectorName->size[1] - 1;
      Q->data[(int)i - 1].JointName.size[0] = 1;
      Q->data[(int)i - 1].JointName.size[1] = endEffectorName->size[1];
      for (i20 = 0; i20 <= loop_ub; i20++) {
        Q->data[(int)i - 1].JointName.data[i20] = endEffectorName->data[i20];
      }

      if (bid > numPositions) {
        i20 = 1;
        end = 1;
      } else {
        i20 = (int)bid;
        end = (int)numPositions + 1;
      }

      nm1d2 = end - i20;
      loop_ub = nm1d2 - 1;
      Q->data[(int)i - 1].JointPosition.size[0] = 1;
      Q->data[(int)i - 1].JointPosition.size[1] = 1;
      end = r4->size[0];
      r4->size[0] = nm1d2;
      emxEnsureCapacity_int32_T(r4, end);
      for (end = 0; end < nm1d2; end++) {
        r4->data[end] = i20 + end;
      }

      for (i20 = 0; i20 <= loop_ub; i20++) {
        Q->data[(int)i - 1].JointPosition.data[i20] = iniGuessVec->data[r4->
          data[i20] - 1];
      }

      i++;
    }
  }

  emxFree_int32_T(&r4);
  emxFree_char_T(&endEffectorName);
  emxFree_real_T(&iniGuessVec);
  i19 = QSol->size[0] * QSol->size[1];
  QSol->size[0] = 1;
  QSol->size[1] = Q->size[1];
  emxEnsureCapacity_struct_T1(QSol, i19);
  for (nm1d2 = 0; nm1d2 < Q->size[1]; nm1d2++) {
    loop_ub = Q->data[nm1d2].JointName.size[0] * Q->data[nm1d2].JointName.size[1]
      - 1;
    QSol->data[nm1d2].JointName.size[0] = 1;
    QSol->data[nm1d2].JointName.size[1] = Q->data[nm1d2].JointName.size[1];
    for (i19 = 0; i19 <= loop_ub; i19++) {
      QSol->data[nm1d2].JointName.data[i19] = Q->data[nm1d2].JointName.data[i19];
    }

    QSol->data[nm1d2].JointPosition.size[0] = 1;
    QSol->data[nm1d2].JointPosition.size[1] = 1;
    QSol->data[nm1d2].JointPosition.data[0] = Q->data[nm1d2].JointPosition.data
      [0];
  }

  emxFree_struct_T1(&Q);
}

/*
 * Arguments    : robotics_InverseKinematics *obj
 *                const robotics_RigidBodyTree *varargin_2
 *                c_robotics_core_internal_Damped *iobj_0
 *                d_robotics_manip_internal_Rigid *iobj_1
 *                d_robotics_manip_internal_Rigid *iobj_2
 *                c_robotics_manip_internal_Rigid *iobj_3
 *                c_robotics_manip_internal_Rigid *iobj_4
 *                c_robotics_manip_internal_Rigid *iobj_5
 *                c_robotics_manip_internal_Rigid *iobj_6
 *                c_robotics_manip_internal_Rigid *iobj_7
 *                c_robotics_manip_internal_Rigid *iobj_8
 *                c_robotics_manip_internal_Rigid *iobj_9
 *                c_robotics_manip_internal_Rigid *iobj_10
 *                c_robotics_manip_internal_Rigid *iobj_11
 *                c_robotics_manip_internal_Rigid *iobj_12
 *                c_robotics_manip_internal_Rigid *iobj_13
 *                c_robotics_manip_internal_Rigid *iobj_14
 *                c_robotics_manip_internal_Rigid *iobj_15
 *                c_robotics_manip_internal_Rigid *iobj_16
 *                c_robotics_manip_internal_Rigid *iobj_17
 *                robotics_Joint *iobj_18
 *                robotics_Joint *iobj_19
 *                robotics_Joint *iobj_20
 *                robotics_Joint *iobj_21
 *                robotics_Joint *iobj_22
 *                robotics_Joint *iobj_23
 *                robotics_Joint *iobj_24
 *                robotics_Joint *iobj_25
 *                robotics_Joint *iobj_26
 *                robotics_Joint *iobj_27
 *                robotics_Joint *iobj_28
 *                robotics_Joint *iobj_29
 *                robotics_Joint *iobj_30
 *                robotics_Joint *iobj_31
 *                robotics_Joint *iobj_32
 *                robotics_Joint *iobj_33
 *                robotics_Joint *iobj_34
 *                robotics_Joint *iobj_35
 *                robotics_Joint *iobj_36
 *                robotics_Joint *iobj_37
 *                robotics_Joint *iobj_38
 *                robotics_Joint *iobj_39
 * Return Type  : robotics_InverseKinematics *
 */
robotics_InverseKinematics *c_InverseKinematics_InverseKine
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
   robotics_Joint *iobj_37, robotics_Joint *iobj_38, robotics_Joint *iobj_39)
{
  robotics_InverseKinematics *b_obj;
  emxArray_char_T *b_basename;
  d_robotics_manip_internal_Rigid *c_obj;
  d_robotics_manip_internal_Rigid *newRobotInternal;
  int i15;
  int loop_ub;
  double bid;
  double g_idx_1;
  double g_idx_2;
  c_robotics_manip_internal_Rigid *body;
  c_robotics_manip_internal_Rigid *parent;
  emxInit_char_T(&b_basename, 2);
  b_obj = obj;
  b_obj->isInitialized = 0;
  c_InverseKinematics_set_SolverA(b_obj, iobj_0);
  c_obj = varargin_2->TreeInternal;
  newRobotInternal = RigidBodyTree_RigidBodyTree(iobj_1, iobj_18, iobj_3, iobj_4,
    iobj_5, iobj_6, iobj_7, iobj_19, iobj_20, iobj_21, iobj_22, iobj_23);
  i15 = b_basename->size[0] * b_basename->size[1];
  b_basename->size[0] = 1;
  b_basename->size[1] = c_obj->Base.NameInternal->size[1];
  emxEnsureCapacity_char_T(b_basename, i15);
  loop_ub = c_obj->Base.NameInternal->size[0] * c_obj->Base.NameInternal->size[1];
  for (i15 = 0; i15 < loop_ub; i15++) {
    b_basename->data[i15] = c_obj->Base.NameInternal->data[i15];
  }

  bid = c_RigidBodyTree_findBodyIndexBy(newRobotInternal, b_basename);
  if ((!(bid == 0.0)) && (bid < 0.0)) {
    i15 = newRobotInternal->Base.NameInternal->size[0] *
      newRobotInternal->Base.NameInternal->size[1];
    newRobotInternal->Base.NameInternal->size[0] = 1;
    newRobotInternal->Base.NameInternal->size[1] = b_basename->size[1];
    emxEnsureCapacity_char_T(newRobotInternal->Base.NameInternal, i15);
    loop_ub = b_basename->size[0] * b_basename->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      newRobotInternal->Base.NameInternal->data[i15] = b_basename->data[i15];
    }
  }

  bid = c_obj->Gravity[0];
  g_idx_1 = c_obj->Gravity[1];
  g_idx_2 = c_obj->Gravity[2];
  newRobotInternal->Gravity[0] = bid;
  newRobotInternal->Gravity[1] = g_idx_1;
  newRobotInternal->Gravity[2] = g_idx_2;
  if (1.0 <= c_obj->NumBodies) {
    body = c_obj->Bodies[0];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = c_obj->Bodies[(int)bid - 1];
    } else {
      parent = &c_obj->Base;
    }

    i15 = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = parent->NameInternal->size[1];
    emxEnsureCapacity_char_T(b_basename, i15);
    loop_ub = parent->NameInternal->size[0] * parent->NameInternal->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      b_basename->data[i15] = parent->NameInternal->data[i15];
    }

    f_RigidBodyTree_addBody(newRobotInternal, body, b_basename, iobj_8, iobj_24,
      iobj_25);
  }

  if (2.0 <= c_obj->NumBodies) {
    body = c_obj->Bodies[1];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = c_obj->Bodies[(int)bid - 1];
    } else {
      parent = &c_obj->Base;
    }

    i15 = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = parent->NameInternal->size[1];
    emxEnsureCapacity_char_T(b_basename, i15);
    loop_ub = parent->NameInternal->size[0] * parent->NameInternal->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      b_basename->data[i15] = parent->NameInternal->data[i15];
    }

    f_RigidBodyTree_addBody(newRobotInternal, body, b_basename, iobj_9, iobj_26,
      iobj_27);
  }

  if (3.0 <= c_obj->NumBodies) {
    body = c_obj->Bodies[2];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = c_obj->Bodies[(int)bid - 1];
    } else {
      parent = &c_obj->Base;
    }

    i15 = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = parent->NameInternal->size[1];
    emxEnsureCapacity_char_T(b_basename, i15);
    loop_ub = parent->NameInternal->size[0] * parent->NameInternal->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      b_basename->data[i15] = parent->NameInternal->data[i15];
    }

    f_RigidBodyTree_addBody(newRobotInternal, body, b_basename, iobj_10, iobj_28,
      iobj_29);
  }

  if (4.0 <= c_obj->NumBodies) {
    body = c_obj->Bodies[3];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = c_obj->Bodies[(int)bid - 1];
    } else {
      parent = &c_obj->Base;
    }

    i15 = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = parent->NameInternal->size[1];
    emxEnsureCapacity_char_T(b_basename, i15);
    loop_ub = parent->NameInternal->size[0] * parent->NameInternal->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      b_basename->data[i15] = parent->NameInternal->data[i15];
    }

    f_RigidBodyTree_addBody(newRobotInternal, body, b_basename, iobj_11, iobj_30,
      iobj_31);
  }

  if (5.0 <= c_obj->NumBodies) {
    body = c_obj->Bodies[4];
    bid = body->ParentIndex;
    if (bid > 0.0) {
      parent = c_obj->Bodies[(int)bid - 1];
    } else {
      parent = &c_obj->Base;
    }

    i15 = b_basename->size[0] * b_basename->size[1];
    b_basename->size[0] = 1;
    b_basename->size[1] = parent->NameInternal->size[1];
    emxEnsureCapacity_char_T(b_basename, i15);
    loop_ub = parent->NameInternal->size[0] * parent->NameInternal->size[1];
    for (i15 = 0; i15 < loop_ub; i15++) {
      b_basename->data[i15] = parent->NameInternal->data[i15];
    }

    f_RigidBodyTree_addBody(newRobotInternal, body, b_basename, iobj_12, iobj_32,
      iobj_33);
  }

  emxFree_char_T(&b_basename);
  RigidBodyTree_RigidBodyTree(iobj_2, iobj_34, iobj_13, iobj_14, iobj_15,
    iobj_16, iobj_17, iobj_35, iobj_36, iobj_37, iobj_38, iobj_39);
  b_obj->RigidBodyTreeInternal = newRobotInternal;
  b_obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

/*
 * File trailer for InverseKinematics.c
 *
 * [EOF]
 */
