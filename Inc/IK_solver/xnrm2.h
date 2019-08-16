/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xnrm2.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef XNRM2_H
#define XNRM2_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern double b_xnrm2(const double x[3], int ix0);
extern double c_xnrm2(int n, const emxArray_real_T *x, int ix0);
extern double xnrm2(int n, const double x[9], int ix0);

#endif

/*
 * File trailer for xnrm2.h
 *
 * [EOF]
 */
