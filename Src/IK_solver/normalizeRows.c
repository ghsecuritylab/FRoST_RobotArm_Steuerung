/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: normalizeRows.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include <math.h>
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/normalizeRows.h"

/* Function Definitions */

/*
 * Arguments    : const double matrix[3]
 *                double normRowMatrix[3]
 * Return Type  : void
 */
void normalizeRows(const double matrix[3], double normRowMatrix[3])
{
  double b;
  b = 1.0 / sqrt((matrix[0] * matrix[0] + matrix[1] * matrix[1]) + matrix[2] *
                 matrix[2]);
  normRowMatrix[0] = matrix[0] * b;
  normRowMatrix[1] = matrix[1] * b;
  normRowMatrix[2] = matrix[2] * b;
}

/*
 * File trailer for normalizeRows.c
 *
 * [EOF]
 */
