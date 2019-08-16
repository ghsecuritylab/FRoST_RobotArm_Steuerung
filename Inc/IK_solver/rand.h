/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rand.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef RAND_H
#define RAND_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern void b_rand(double r[5]);
extern void c_rand(double varargin_1, emxArray_real_T *r);
extern void genrand_uint32_vector(unsigned int mt[625], unsigned int u[2]);
extern double genrandu(unsigned int mt[625]);

#endif

/*
 * File trailer for rand.h
 *
 * [EOF]
 */
