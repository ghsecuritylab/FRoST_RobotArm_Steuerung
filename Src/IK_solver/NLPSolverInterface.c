/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: NLPSolverInterface.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/NLPSolverInterface.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/DampedBFGSwGradientProjection.h"
#include "IK_solver/rand.h"
#include "IK_solver/abs1.h"
#include "IK_solver/randn.h"
#include "IK_solver/all1.h"
#include "IK_solver/isfinite.h"
#include "IK_solver/any.h"
#include "IK_solver/ctimefun.h"

/* Function Definitions */

/*
 * Arguments    : c_robotics_core_internal_Damped *obj
 *                const emxArray_real_T *seed
 *                emxArray_real_T *xSol
 *                double *solutionInfo_Iterations
 *                double *solutionInfo_RRAttempts
 *                double *solutionInfo_Error
 *                double *solutionInfo_ExitFlag
 *                char solutionInfo_Status_data[]
 *                int solutionInfo_Status_size[2]
 * Return Type  : void
 */
void NLPSolverInterface_solve(c_robotics_core_internal_Damped *obj, const
  emxArray_real_T *seed, emxArray_real_T *xSol, double *solutionInfo_Iterations,
  double *solutionInfo_RRAttempts, double *solutionInfo_Error, double
  *solutionInfo_ExitFlag, char solutionInfo_Status_data[], int
  solutionInfo_Status_size[2])
{
  int i23;
  int loop_ub;
  double tol;
  c_robotics_core_internal_NLPSol exitFlag;
  double err;
  double iter;
  double rrAttempts;
  double iterations;
  double errPrev;
  c_robotics_core_internal_NLPSol exitFlagPrev;
  emxArray_real_T *newseed;
  emxArray_real_T *qi;
  emxArray_int32_T *r7;
  emxArray_real_T *ub;
  emxArray_real_T *lb;
  emxArray_real_T *rn;
  emxArray_boolean_T *r8;
  emxArray_real_T *r9;
  emxArray_boolean_T *r10;
  boolean_T exitg1;
  d_robotics_manip_internal_Rigid *b_obj;
  static const char cv27[14] = { 'b', 'e', 's', 't', ' ', 'a', 'v', 'a', 'i',
    'l', 'a', 'b', 'l', 'e' };

  static const char cv28[7] = { 's', 'u', 'c', 'c', 'e', 's', 's' };

  int i;
  robotics_Joint *c_obj;
  int i24;
  int i25;
  int i26;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  double b_ub[2];
  obj->MaxNumIterationInternal = obj->MaxNumIteration;
  obj->MaxTimeInternal = obj->MaxTime;
  i23 = obj->SeedInternal->size[0];
  obj->SeedInternal->size[0] = seed->size[0];
  emxEnsureCapacity_real_T(obj->SeedInternal, i23);
  loop_ub = seed->size[0];
  for (i23 = 0; i23 < loop_ub; i23++) {
    obj->SeedInternal->data[i23] = seed->data[i23];
  }

  tol = obj->SolutionTolerance;
  obj->TimeObj.StartTime = ctimefun();
  c_DampedBFGSwGradientProjection(obj, xSol, &exitFlag, &err, &iter);
  rrAttempts = 0.0;
  iterations = iter;
  errPrev = err;
  exitFlagPrev = exitFlag;
  emxInit_real_T(&newseed, 1);
  emxInit_real_T(&qi, 2);
  emxInit_int32_T(&r7, 2);
  emxInit_real_T(&ub, 1);
  emxInit_real_T(&lb, 1);
  emxInit_real_T(&rn, 1);
  emxInit_boolean_T(&r8, 1);
  emxInit_real_T(&r9, 2);
  emxInit_boolean_T(&r10, 1);
  exitg1 = false;
  while ((!exitg1) && (obj->RandomRestart && (err > tol))) {
    obj->MaxNumIterationInternal -= iter;
    err = ctimefun();
    err -= obj->TimeObj.StartTime;
    obj->MaxTimeInternal = obj->MaxTime - err;
    if (obj->MaxNumIterationInternal <= 0.0) {
      exitFlag = IterationLimitExceeded;
    }

    if ((exitFlag == IterationLimitExceeded) || (exitFlag == TimeLimitExceeded))
    {
      exitFlagPrev = exitFlag;
      exitg1 = true;
    } else {
      b_obj = obj->ExtraArgs->Robot;
      i23 = newseed->size[0];
      newseed->size[0] = (int)b_obj->PositionNumber;
      emxEnsureCapacity_real_T(newseed, i23);
      loop_ub = (int)b_obj->PositionNumber;
      for (i23 = 0; i23 < loop_ub; i23++) {
        newseed->data[i23] = 0.0;
      }

      err = b_obj->NumBodies;
      i23 = (int)err;
      for (i = 0; i < i23; i++) {
        err = b_obj->PositionDoFMap[i];
        iter = b_obj->PositionDoFMap[i + 5];
        if (err <= iter) {
          c_obj = b_obj->Bodies[i]->JointInternal;
          switch ((int)c_obj->PositionNumber) {
           case 0:
            i24 = qi->size[0] * qi->size[1];
            qi->size[0] = 1;
            qi->size[1] = 1;
            emxEnsureCapacity_real_T(qi, i24);
            qi->data[0] = rtNaN;
            break;

           default:
            loop_ub = c_obj->PositionLimitsInternal->size[0];
            i24 = ub->size[0];
            ub->size[0] = loop_ub;
            emxEnsureCapacity_real_T(ub, i24);
            for (i24 = 0; i24 < loop_ub; i24++) {
              ub->data[i24] = c_obj->PositionLimitsInternal->data[i24 +
                c_obj->PositionLimitsInternal->size[0]];
            }

            loop_ub = c_obj->PositionLimitsInternal->size[0];
            i24 = lb->size[0];
            lb->size[0] = loop_ub;
            emxEnsureCapacity_real_T(lb, i24);
            for (i24 = 0; i24 < loop_ub; i24++) {
              lb->data[i24] = c_obj->PositionLimitsInternal->data[i24];
            }

            b_isfinite(lb, r8);
            guard1 = false;
            guard2 = false;
            guard3 = false;
            if (all(r8)) {
              b_isfinite(ub, r8);
              if (all(r8)) {
                c_rand(c_obj->PositionNumber, rn);
                i24 = qi->size[0] * qi->size[1];
                qi->size[0] = lb->size[0];
                qi->size[1] = 1;
                emxEnsureCapacity_real_T(qi, i24);
                loop_ub = lb->size[0];
                for (i24 = 0; i24 < loop_ub; i24++) {
                  qi->data[i24] = lb->data[i24] + rn->data[i24] * (ub->data[i24]
                    - lb->data[i24]);
                }
              } else {
                guard3 = true;
              }
            } else {
              guard3 = true;
            }

            if (guard3) {
              b_isfinite(lb, r8);
              if (all(r8)) {
                b_isfinite(ub, r8);
                i24 = r10->size[0];
                r10->size[0] = r8->size[0];
                emxEnsureCapacity_boolean_T(r10, i24);
                loop_ub = r8->size[0];
                for (i24 = 0; i24 < loop_ub; i24++) {
                  r10->data[i24] = !r8->data[i24];
                }

                if (any(r10)) {
                  b_ub[0] = lb->size[0];
                  b_ub[1] = 1.0;
                  randn(b_ub, r9);
                  b_abs(r9, qi);
                  loop_ub = lb->size[0];
                  i24 = qi->size[0] * qi->size[1];
                  qi->size[0] = loop_ub;
                  qi->size[1] = 1;
                  emxEnsureCapacity_real_T(qi, i24);
                  for (i24 = 0; i24 < loop_ub; i24++) {
                    qi->data[i24] += lb->data[i24];
                  }
                } else {
                  guard2 = true;
                }
              } else {
                guard2 = true;
              }
            }

            if (guard2) {
              b_isfinite(lb, r8);
              i24 = r10->size[0];
              r10->size[0] = r8->size[0];
              emxEnsureCapacity_boolean_T(r10, i24);
              loop_ub = r8->size[0];
              for (i24 = 0; i24 < loop_ub; i24++) {
                r10->data[i24] = !r8->data[i24];
              }

              if (any(r10)) {
                b_isfinite(ub, r8);
                if (all(r8)) {
                  b_ub[0] = ub->size[0];
                  b_ub[1] = 1.0;
                  randn(b_ub, r9);
                  b_abs(r9, qi);
                  loop_ub = ub->size[0];
                  i24 = qi->size[0] * qi->size[1];
                  qi->size[0] = loop_ub;
                  qi->size[1] = 1;
                  emxEnsureCapacity_real_T(qi, i24);
                  for (i24 = 0; i24 < loop_ub; i24++) {
                    qi->data[i24] = ub->data[i24] - qi->data[i24];
                  }
                } else {
                  guard1 = true;
                }
              } else {
                guard1 = true;
              }
            }

            if (guard1) {
              b_ub[0] = ub->size[0];
              b_ub[1] = 1.0;
              randn(b_ub, qi);
            }
            break;
          }

          if (err > iter) {
            i24 = 0;
            i25 = 0;
          } else {
            i24 = (int)err - 1;
            i25 = (int)iter;
          }

          i26 = r7->size[0] * r7->size[1];
          r7->size[0] = 1;
          loop_ub = i25 - i24;
          r7->size[1] = loop_ub;
          emxEnsureCapacity_int32_T(r7, i26);
          for (i25 = 0; i25 < loop_ub; i25++) {
            r7->data[i25] = i24 + i25;
          }

          loop_ub = r7->size[0] * r7->size[1];
          for (i24 = 0; i24 < loop_ub; i24++) {
            newseed->data[r7->data[i24]] = qi->data[i24];
          }
        }
      }

      i23 = obj->SeedInternal->size[0];
      obj->SeedInternal->size[0] = newseed->size[0];
      emxEnsureCapacity_real_T(obj->SeedInternal, i23);
      loop_ub = newseed->size[0];
      for (i23 = 0; i23 < loop_ub; i23++) {
        obj->SeedInternal->data[i23] = newseed->data[i23];
      }

      c_DampedBFGSwGradientProjection(obj, newseed, &exitFlag, &err, &iter);
      if (err < errPrev) {
        i23 = xSol->size[0];
        xSol->size[0] = newseed->size[0];
        emxEnsureCapacity_real_T(xSol, i23);
        loop_ub = newseed->size[0];
        for (i23 = 0; i23 < loop_ub; i23++) {
          xSol->data[i23] = newseed->data[i23];
        }

        errPrev = err;
        exitFlagPrev = exitFlag;
      }

      rrAttempts++;
      iterations += iter;
    }
  }

  emxFree_boolean_T(&r10);
  emxFree_real_T(&r9);
  emxFree_boolean_T(&r8);
  emxFree_real_T(&rn);
  emxFree_real_T(&lb);
  emxFree_real_T(&ub);
  emxFree_int32_T(&r7);
  emxFree_real_T(&qi);
  emxFree_real_T(&newseed);
  if (errPrev < tol) {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 7;
    for (i23 = 0; i23 < 7; i23++) {
      solutionInfo_Status_data[i23] = cv28[i23];
    }
  } else {
    solutionInfo_Status_size[0] = 1;
    solutionInfo_Status_size[1] = 14;
    for (i23 = 0; i23 < 14; i23++) {
      solutionInfo_Status_data[i23] = cv27[i23];
    }
  }

  *solutionInfo_Iterations = iterations;
  *solutionInfo_RRAttempts = rrAttempts;
  *solutionInfo_Error = errPrev;
  *solutionInfo_ExitFlag = (double)exitFlagPrev;
}

/*
 * File trailer for NLPSolverInterface.c
 *
 * [EOF]
 */
