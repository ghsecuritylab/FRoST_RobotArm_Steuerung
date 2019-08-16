/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IK_solver.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef IK_SOLVER_H
#define IK_SOLVER_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern void IK_solver(double r, double h, double phi, double q1_init, double
                      q2_init, double q3_init, double q4_init, double q1_data[1],
                      int q1_size[2], double q2_data[1], int q2_size[2], double
                      q3_data[1], int q3_size[2], double q4_data[1], int q4_size[2]);

#endif

/*
 * File trailer for IK_solver.h
 *
 * [EOF]
 */
