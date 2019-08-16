/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xaxpy.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef XAXPY_H
#define XAXPY_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern void b_xaxpy(int n, double a, const double x[9], int ix0, double y[3],
                    int iy0);
extern void c_xaxpy(int n, double a, const double x[3], int ix0, double y[9],
                    int iy0);
extern void xaxpy(int n, double a, int ix0, double y[9], int iy0);

#endif

/*
 * File trailer for xaxpy.h
 *
 * [EOF]
 */
