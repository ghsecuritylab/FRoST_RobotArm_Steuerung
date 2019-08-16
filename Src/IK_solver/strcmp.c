/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: strcmp.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/strcmp.h"
#include "IK_solver/IK_solver_data.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_char_T *a
 * Return Type  : boolean_T
 */
boolean_T b_strcmp(const emxArray_char_T *a)
{
  boolean_T b_bool;
  int kstr;
  int exitg1;
  static const char cv3[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  b_bool = false;
  if (a->size[1] == 8) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 8) {
        if (a->data[kstr] != cv3[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

/*
 * Arguments    : const emxArray_char_T *a
 * Return Type  : boolean_T
 */
boolean_T c_strcmp(const emxArray_char_T *a)
{
  boolean_T b_bool;
  int kstr;
  int exitg1;
  b_bool = false;
  if (a->size[1] == 9) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 9) {
        if (a->data[kstr] != cv0[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

/*
 * Arguments    : const emxArray_char_T *a
 * Return Type  : boolean_T
 */
boolean_T d_strcmp(const emxArray_char_T *a)
{
  boolean_T b_bool;
  int kstr;
  int exitg1;
  static const char cv17[5] = { 'f', 'i', 'x', 'e', 'd' };

  b_bool = false;
  if (a->size[1] == 5) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (a->data[kstr] != cv17[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

/*
 * Arguments    : const emxArray_char_T *a
 *                const emxArray_char_T *b
 * Return Type  : boolean_T
 */
boolean_T e_strcmp(const emxArray_char_T *a, const emxArray_char_T *b)
{
  boolean_T b_bool;
  boolean_T b1;
  int kstr;
  int exitg1;
  b_bool = false;
  b1 = (a->size[1] == 0);
  if (b1 && (b->size[1] == 0)) {
    b_bool = true;
  } else if (a->size[1] != b->size[1]) {
  } else {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr <= b->size[1] - 1) {
        if (a->data[kstr] != b->data[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

/*
 * Arguments    : const emxArray_char_T *a
 * Return Type  : boolean_T
 */
boolean_T f_strcmp(const emxArray_char_T *a)
{
  boolean_T b_bool;
  int kstr;
  int exitg1;
  static const char cv21[4] = { 'b', 'a', 's', 'e' };

  b_bool = false;
  if (a->size[1] == 4) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 4) {
        if (a->data[kstr] != cv21[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

/*
 * Arguments    : const emxArray_char_T *a
 * Return Type  : boolean_T
 */
boolean_T g_strcmp(const emxArray_char_T *a)
{
  boolean_T b_bool;
  int kstr;
  int exitg1;
  static const char cv23[5] = { 'b', 'o', 'd', 'y', '1' };

  b_bool = false;
  if (a->size[1] == 5) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (a->data[kstr] != cv23[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

/*
 * Arguments    : const emxArray_char_T *a
 * Return Type  : boolean_T
 */
boolean_T h_strcmp(const emxArray_char_T *a)
{
  boolean_T b_bool;
  int kstr;
  int exitg1;
  static const char cv24[5] = { 'b', 'o', 'd', 'y', '2' };

  b_bool = false;
  if (a->size[1] == 5) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (a->data[kstr] != cv24[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

/*
 * Arguments    : const emxArray_char_T *a
 * Return Type  : boolean_T
 */
boolean_T i_strcmp(const emxArray_char_T *a)
{
  boolean_T b_bool;
  int kstr;
  int exitg1;
  static const char cv25[5] = { 'b', 'o', 'd', 'y', '3' };

  b_bool = false;
  if (a->size[1] == 5) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (a->data[kstr] != cv25[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

/*
 * Arguments    : const emxArray_char_T *a
 * Return Type  : boolean_T
 */
boolean_T j_strcmp(const emxArray_char_T *a)
{
  boolean_T b_bool;
  int kstr;
  int exitg1;
  static const char cv26[5] = { 'b', 'o', 'd', 'y', '4' };

  b_bool = false;
  if (a->size[1] == 5) {
    kstr = 0;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (a->data[kstr] != cv26[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return b_bool;
}

/*
 * File trailer for strcmp.c
 *
 * [EOF]
 */
