/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IK_solver_emxutil.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include <stdlib.h>
#include <string.h>
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/IK_solver_emxutil.h"

/* Function Declarations */
static void emxExpand_struct_T(b_emxArray_struct_T *emxArray, int fromIndex, int
  toIndex);
static void emxExpand_struct_T1(emxArray_struct_T *emxArray, int fromIndex, int
  toIndex);
static void emxInitStruct_struct_T(b_struct_T *pStruct);
static void emxInitStruct_struct_T1(struct_T *pStruct);

/* Function Definitions */

/*
 * Arguments    : b_emxArray_struct_T *emxArray
 *                int fromIndex
 *                int toIndex
 * Return Type  : void
 */
static void emxExpand_struct_T(b_emxArray_struct_T *emxArray, int fromIndex, int
  toIndex)
{
  int i;
  for (i = fromIndex; i < toIndex; i++) {
    emxInitStruct_struct_T(&emxArray->data[i]);
  }
}

/*
 * Arguments    : emxArray_struct_T *emxArray
 *                int fromIndex
 *                int toIndex
 * Return Type  : void
 */
static void emxExpand_struct_T1(emxArray_struct_T *emxArray, int fromIndex, int
  toIndex)
{
  int i;
  for (i = fromIndex; i < toIndex; i++) {
    emxInitStruct_struct_T1(&emxArray->data[i]);
  }
}

/*
 * Arguments    : b_struct_T *pStruct
 * Return Type  : void
 */
static void emxInitStruct_struct_T(b_struct_T *pStruct)
{
  pStruct->JointName.size[0] = 0;
  pStruct->JointName.size[1] = 0;
  pStruct->JointPosition.size[0] = 0;
  pStruct->JointPosition.size[1] = 0;
}

/*
 * Arguments    : struct_T *pStruct
 * Return Type  : void
 */
static void emxInitStruct_struct_T1(struct_T *pStruct)
{
  pStruct->JointName.size[0] = 0;
  pStruct->JointName.size[1] = 0;
  pStruct->JointPosition.size[0] = 0;
  pStruct->JointPosition.size[1] = 0;
}

/*
 * Arguments    : robotics_InverseKinematics *pStruct
 * Return Type  : void
 */
void c_emxFreeStruct_robotics_Invers(robotics_InverseKinematics *pStruct)
{
  emxFree_real_T(&pStruct->Limits);
}

/*
 * Arguments    : c_robotics_core_internal_Damped *pStruct
 * Return Type  : void
 */
void c_emxFreeStruct_robotics_core_i(c_robotics_core_internal_Damped *pStruct)
{
  emxFree_real_T(&pStruct->ConstraintMatrix);
  emxFree_real_T(&pStruct->ConstraintBound);
  emxFree_real_T(&pStruct->SeedInternal);
}

/*
 * Arguments    : c_robotics_manip_internal_IKExt *pStruct
 * Return Type  : void
 */
void c_emxFreeStruct_robotics_manip_(c_robotics_manip_internal_IKExt *pStruct)
{
  emxFree_char_T(&pStruct->BodyName);
  emxFree_real_T(&pStruct->ErrTemp);
  emxFree_real_T(&pStruct->GradTemp);
}

/*
 * Arguments    : robotics_InverseKinematics *pStruct
 * Return Type  : void
 */
void c_emxInitStruct_robotics_Invers(robotics_InverseKinematics *pStruct)
{
  emxInit_real_T(&pStruct->Limits, 2);
}

/*
 * Arguments    : c_robotics_core_internal_Damped *pStruct
 * Return Type  : void
 */
void c_emxInitStruct_robotics_core_i(c_robotics_core_internal_Damped *pStruct)
{
  emxInit_real_T(&pStruct->ConstraintMatrix, 2);
  emxInit_real_T(&pStruct->ConstraintBound, 1);
  emxInit_real_T(&pStruct->SeedInternal, 1);
}

/*
 * Arguments    : c_robotics_manip_internal_IKExt *pStruct
 * Return Type  : void
 */
void c_emxInitStruct_robotics_manip_(c_robotics_manip_internal_IKExt *pStruct)
{
  emxInit_char_T(&pStruct->BodyName, 2);
  emxInit_real_T(&pStruct->ErrTemp, 1);
  emxInit_real_T(&pStruct->GradTemp, 1);
}

/*
 * Arguments    : c_robotics_manip_internal_Rigid *pStruct
 * Return Type  : void
 */
void d_emxFreeStruct_robotics_manip_(c_robotics_manip_internal_Rigid *pStruct)
{
  emxFree_char_T(&pStruct->NameInternal);
}

/*
 * Arguments    : c_robotics_manip_internal_Rigid *pStruct
 * Return Type  : void
 */
void d_emxInitStruct_robotics_manip_(c_robotics_manip_internal_Rigid *pStruct)
{
  emxInit_char_T(&pStruct->NameInternal, 2);
}

/*
 * Arguments    : d_robotics_manip_internal_Rigid *pStruct
 * Return Type  : void
 */
void e_emxFreeStruct_robotics_manip_(d_robotics_manip_internal_Rigid *pStruct)
{
  d_emxFreeStruct_robotics_manip_(&pStruct->Base);
}

/*
 * Arguments    : d_robotics_manip_internal_Rigid *pStruct
 * Return Type  : void
 */
void e_emxInitStruct_robotics_manip_(d_robotics_manip_internal_Rigid *pStruct)
{
  d_emxInitStruct_robotics_manip_(&pStruct->Base);
}

/*
 * Arguments    : emxArray_boolean_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((unsigned int)i, sizeof(boolean_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(boolean_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (boolean_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_char_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_char_T(emxArray_char_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((unsigned int)i, sizeof(char));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(char) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (char *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_int32_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((unsigned int)i, sizeof(int));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (int *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_int8_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((unsigned int)i, sizeof(signed char));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(signed char) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (signed char *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_real_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((unsigned int)i, sizeof(double));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(double) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (double *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : b_emxArray_struct_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_struct_T(b_emxArray_struct_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((unsigned int)i, sizeof(b_struct_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(b_struct_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (b_struct_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }

  if (oldNumel > newNumel) {
    emxExpand_struct_T(emxArray, oldNumel, newNumel);
  }
}

/*
 * Arguments    : emxArray_struct_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_struct_T1(emxArray_struct_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((unsigned int)i, sizeof(struct_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(struct_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (struct_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }

  if (oldNumel > newNumel) {
    emxExpand_struct_T1(emxArray, oldNumel, newNumel);
  }
}

/*
 * Arguments    : emxArray_uint32_T *emxArray
 *                int oldNumel
 * Return Type  : void
 */
void emxEnsureCapacity_uint32_T(emxArray_uint32_T *emxArray, int oldNumel)
{
  int newNumel;
  int i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((unsigned int)i, sizeof(unsigned int));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(unsigned int) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (unsigned int *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : robotics_Joint *pStruct
 * Return Type  : void
 */
void emxFreeStruct_robotics_Joint(robotics_Joint *pStruct)
{
  emxFree_char_T(&pStruct->Type);
  emxFree_real_T(&pStruct->MotionSubspace);
  emxFree_char_T(&pStruct->NameInternal);
  emxFree_real_T(&pStruct->PositionLimitsInternal);
  emxFree_real_T(&pStruct->HomePositionInternal);
}

/*
 * Arguments    : emxArray_boolean_T **pEmxArray
 * Return Type  : void
 */
void emxFree_boolean_T(emxArray_boolean_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T *)NULL) {
    if (((*pEmxArray)->data != (boolean_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_char_T **pEmxArray
 * Return Type  : void
 */
void emxFree_char_T(emxArray_char_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_char_T *)NULL) {
    if (((*pEmxArray)->data != (char *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_char_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_int32_T **pEmxArray
 * Return Type  : void
 */
void emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_int8_T **pEmxArray
 * Return Type  : void
 */
void emxFree_int8_T(emxArray_int8_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int8_T *)NULL) {
    if (((*pEmxArray)->data != (signed char *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int8_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 * Return Type  : void
 */
void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (double *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_struct_T **pEmxArray
 * Return Type  : void
 */
void emxFree_struct_T(emxArray_struct_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_struct_T *)NULL) {
    if (((*pEmxArray)->data != (struct_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_struct_T *)NULL;
  }
}

/*
 * Arguments    : b_emxArray_struct_T **pEmxArray
 * Return Type  : void
 */
void emxFree_struct_T1(b_emxArray_struct_T **pEmxArray)
{
  if (*pEmxArray != (b_emxArray_struct_T *)NULL) {
    if (((*pEmxArray)->data != (b_struct_T *)NULL) && (*pEmxArray)->canFreeData)
    {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (b_emxArray_struct_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_uint32_T **pEmxArray
 * Return Type  : void
 */
void emxFree_uint32_T(emxArray_uint32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_uint32_T *)NULL) {
    if (((*pEmxArray)->data != (unsigned int *)NULL) && (*pEmxArray)
        ->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_uint32_T *)NULL;
  }
}

/*
 * Arguments    : robotics_Joint *pStruct
 * Return Type  : void
 */
void emxInitStruct_robotics_Joint(robotics_Joint *pStruct)
{
  emxInit_char_T(&pStruct->Type, 2);
  emxInit_real_T(&pStruct->MotionSubspace, 2);
  emxInit_char_T(&pStruct->NameInternal, 2);
  emxInit_real_T(&pStruct->PositionLimitsInternal, 2);
  emxInit_real_T(&pStruct->HomePositionInternal, 1);
}

/*
 * Arguments    : emxArray_boolean_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions)
{
  emxArray_boolean_T *emxArray;
  int i;
  *pEmxArray = (emxArray_boolean_T *)malloc(sizeof(emxArray_boolean_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_char_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_char_T(emxArray_char_T **pEmxArray, int numDimensions)
{
  emxArray_char_T *emxArray;
  int i;
  *pEmxArray = (emxArray_char_T *)malloc(sizeof(emxArray_char_T));
  emxArray = *pEmxArray;
  emxArray->data = (char *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_int32_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_int8_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions)
{
  emxArray_int8_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int8_T *)malloc(sizeof(emxArray_int8_T));
  emxArray = *pEmxArray;
  emxArray->data = (signed char *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_struct_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_struct_T(emxArray_struct_T **pEmxArray, int numDimensions)
{
  emxArray_struct_T *emxArray;
  int i;
  *pEmxArray = (emxArray_struct_T *)malloc(sizeof(emxArray_struct_T));
  emxArray = *pEmxArray;
  emxArray->data = (struct_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : b_emxArray_struct_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_struct_T1(b_emxArray_struct_T **pEmxArray, int numDimensions)
{
  b_emxArray_struct_T *emxArray;
  int i;
  *pEmxArray = (b_emxArray_struct_T *)malloc(sizeof(b_emxArray_struct_T));
  emxArray = *pEmxArray;
  emxArray->data = (b_struct_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_uint32_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInit_uint32_T(emxArray_uint32_T **pEmxArray, int numDimensions)
{
  emxArray_uint32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_uint32_T *)malloc(sizeof(emxArray_uint32_T));
  emxArray = *pEmxArray;
  emxArray->data = (unsigned int *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : e_robotics_manip_internal_Rigid *pStruct
 * Return Type  : void
 */
void f_emxFreeStruct_robotics_manip_(e_robotics_manip_internal_Rigid *pStruct)
{
  emxFreeStruct_robotics_Joint(&pStruct->JointInternal);
}

/*
 * Arguments    : e_robotics_manip_internal_Rigid *pStruct
 * Return Type  : void
 */
void f_emxInitStruct_robotics_manip_(e_robotics_manip_internal_Rigid *pStruct)
{
  emxInitStruct_robotics_Joint(&pStruct->JointInternal);
}

/*
 * Arguments    : f_robotics_manip_internal_Rigid *pStruct
 * Return Type  : void
 */
void g_emxFreeStruct_robotics_manip_(f_robotics_manip_internal_Rigid *pStruct)
{
  f_emxFreeStruct_robotics_manip_(&pStruct->Base);
}

/*
 * Arguments    : f_robotics_manip_internal_Rigid *pStruct
 * Return Type  : void
 */
void g_emxInitStruct_robotics_manip_(f_robotics_manip_internal_Rigid *pStruct)
{
  f_emxInitStruct_robotics_manip_(&pStruct->Base);
}

/*
 * File trailer for IK_solver_emxutil.c
 *
 * [EOF]
 */
