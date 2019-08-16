/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: validatestring.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/validatestring.h"
#include "IK_solver/IK_solver_data.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_char_T *str
 *                char match_data[]
 *                int match_size[2]
 *                double *nmatched
 * Return Type  : void
 */
void get_match(const emxArray_char_T *str, char match_data[], int match_size[2],
               double *nmatched)
{
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  int ns;
  boolean_T b_bool;
  int minnanb;
  boolean_T matched;
  boolean_T b_guard1 = false;
  int exitg1;
  static const char cv18[128] = { '\x00', '\x01', '\x02', '\x03', '\x04', '\x05',
    '\x06', '\x07', '\x08', '	', '\x0a', '\x0b', '\x0c', '\x0d', '\x0e', '\x0f',
    '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18',
    '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#',
    '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2',
    '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a',
    'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
    'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']', '^', '_',
    '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}',
    '~', '\x7f' };

  static const char cv19[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char vstr[8] = { 'r', 'e', 'v', 'o', 'l', 'u', 't', 'e' };

  static const char cv20[5] = { 'f', 'i', 'x', 'e', 'd' };

  static const char b_vstr[5] = { 'f', 'i', 'x', 'e', 'd' };

  match_size[0] = 1;
  match_size[1] = 0;
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (str->size[1] <= 8) {
    ns = str->size[1];
    b_bool = false;
    minnanb = str->size[1];
    if (minnanb >= 8) {
      minnanb = 8;
    }

    b_guard1 = false;
    if (ns <= minnanb) {
      if (minnanb < ns) {
        ns = minnanb;
      }

      b_guard1 = true;
    } else {
      if (str->size[1] == 8) {
        ns = 8;
        b_guard1 = true;
      }
    }

    if (b_guard1) {
      minnanb = 0;
      do {
        exitg1 = 0;
        if (minnanb <= ns - 1) {
          if (cv18[(unsigned char)str->data[minnanb] & 127] != cv18[(int)
              cv19[minnanb]]) {
            exitg1 = 1;
          } else {
            minnanb++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }

    if (b_bool) {
      if (str->size[1] == 8) {
        *nmatched = 1.0;
        match_size[0] = 1;
        match_size[1] = 8;
        for (minnanb = 0; minnanb < 8; minnanb++) {
          match_data[minnanb] = vstr[minnanb];
        }
      } else {
        match_size[0] = 1;
        match_size[1] = 8;
        for (minnanb = 0; minnanb < 8; minnanb++) {
          match_data[minnanb] = vstr[minnanb];
        }

        matched = true;
        *nmatched = 1.0;
        guard3 = true;
      }
    } else {
      guard4 = true;
    }
  } else {
    guard4 = true;
  }

  if (guard4) {
    matched = false;
    *nmatched = 0.0;
    guard3 = true;
  }

  if (guard3) {
    if (str->size[1] <= 9) {
      ns = str->size[1];
      b_bool = false;
      minnanb = str->size[1];
      if (minnanb >= 9) {
        minnanb = 9;
      }

      b_guard1 = false;
      if (ns <= minnanb) {
        if (minnanb < ns) {
          ns = minnanb;
        }

        b_guard1 = true;
      } else {
        if (str->size[1] == 9) {
          ns = 9;
          b_guard1 = true;
        }
      }

      if (b_guard1) {
        minnanb = 0;
        do {
          exitg1 = 0;
          if (minnanb <= ns - 1) {
            if (cv18[(unsigned char)str->data[minnanb] & 127] != cv18[(int)
                cv0[minnanb]]) {
              exitg1 = 1;
            } else {
              minnanb++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_bool) {
        if (str->size[1] == 9) {
          *nmatched = 1.0;
          match_size[0] = 1;
          match_size[1] = 9;
          for (minnanb = 0; minnanb < 9; minnanb++) {
            match_data[minnanb] = cv0[minnanb];
          }
        } else {
          if (!matched) {
            match_size[0] = 1;
            match_size[1] = 9;
            for (minnanb = 0; minnanb < 9; minnanb++) {
              match_data[minnanb] = cv0[minnanb];
            }
          }

          matched = true;
          (*nmatched)++;
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }

  if (guard2) {
    if (str->size[1] <= 5) {
      ns = str->size[1];
      b_bool = false;
      minnanb = str->size[1];
      if (minnanb >= 5) {
        minnanb = 5;
      }

      b_guard1 = false;
      if (ns <= minnanb) {
        if (minnanb < ns) {
          ns = minnanb;
        }

        b_guard1 = true;
      } else {
        if (str->size[1] == 5) {
          ns = 5;
          b_guard1 = true;
        }
      }

      if (b_guard1) {
        minnanb = 0;
        do {
          exitg1 = 0;
          if (minnanb <= ns - 1) {
            if (cv18[(unsigned char)str->data[minnanb] & 127] != cv18[(int)
                cv20[minnanb]]) {
              exitg1 = 1;
            } else {
              minnanb++;
            }
          } else {
            b_bool = true;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      }

      if (b_bool) {
        if (str->size[1] == 5) {
          *nmatched = 1.0;
          match_size[0] = 1;
          match_size[1] = 5;
          for (minnanb = 0; minnanb < 5; minnanb++) {
            match_data[minnanb] = b_vstr[minnanb];
          }
        } else {
          if (!matched) {
            match_size[0] = 1;
            match_size[1] = 5;
            for (minnanb = 0; minnanb < 5; minnanb++) {
              match_data[minnanb] = b_vstr[minnanb];
            }
          }

          (*nmatched)++;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1 && (*nmatched == 0.0)) {
    match_size[0] = 1;
    match_size[1] = 0;
  }
}

/*
 * File trailer for validatestring.c
 *
 * [EOF]
 */
