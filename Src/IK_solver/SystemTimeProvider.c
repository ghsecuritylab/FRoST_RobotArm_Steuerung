/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: SystemTimeProvider.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/SystemTimeProvider.h"
#include "IK_solver/ctimefun.h"

/* Function Definitions */

/*
 * Arguments    : c_robotics_core_internal_System *obj
 * Return Type  : void
 */
void SystemTimeProvider_reset(c_robotics_core_internal_System *obj)
{
  obj->StartTime = ctimefun();
}

/*
 * Arguments    : const c_robotics_core_internal_System *obj
 * Return Type  : double
 */
double c_SystemTimeProvider_getElapsed(const c_robotics_core_internal_System
  *obj)
{
  double systemTime;
  systemTime = ctimefun();
  return systemTime - obj->StartTime;
}

/*
 * File trailer for SystemTimeProvider.c
 *
 * [EOF]
 */
