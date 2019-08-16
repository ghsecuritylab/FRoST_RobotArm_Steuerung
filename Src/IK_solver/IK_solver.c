/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IK_solver.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include <string.h>
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/all1.h"
#include "IK_solver/InverseKinematics.h"
#include "IK_solver/trvec2tform.h"
#include "IK_solver/sind.h"
#include "IK_solver/cosd.h"
#include "IK_solver/RigidBodyTree1.h"
#include "IK_solver/strcmp.h"
#include "IK_solver/RigidBodyTree.h"
#include "IK_solver/Joint.h"
#include "IK_solver/RigidBody1.h"

/* Function Definitions */

/*
 * IK_SOLVER Summary of this function goes here
 *    Detailed explanation goes here
 * Arguments    : double r
 *                double h
 *                double phi
 *                double q1_init
 *                double q2_init
 *                double q3_init
 *                double q4_init
 *                double q1_data[]
 *                int q1_size[2]
 *                double q2_data[]
 *                int q2_size[2]
 *                double q3_data[]
 *                int q3_size[2]
 *                double q4_data[]
 *                int q4_size[2]
 * Return Type  : void
 */
void IK_solver(double r, double h, double phi, double q1_init, double q2_init,
               double q3_init, double q4_init, double q1_data[1], int q1_size[2],
               double q2_data[1], int q2_size[2], double q3_data[1], int q3_size[2],
               double q4_data[1], int q4_size[2])
{
  c_robotics_manip_internal_IKExt lobj_104;
  robotics_Joint lobj_103;
  robotics_Joint lobj_102;
  robotics_Joint lobj_101;
  robotics_Joint lobj_100;
  robotics_Joint lobj_99;
  robotics_Joint lobj_98;
  robotics_Joint lobj_97;
  robotics_Joint lobj_96;
  robotics_Joint lobj_95;
  robotics_Joint lobj_94;
  robotics_Joint lobj_93;
  robotics_Joint lobj_92;
  robotics_Joint lobj_91;
  robotics_Joint lobj_90;
  robotics_Joint lobj_89;
  robotics_Joint lobj_88;
  robotics_Joint lobj_87;
  robotics_Joint lobj_86;
  robotics_Joint lobj_85;
  robotics_Joint lobj_84;
  robotics_Joint lobj_83;
  robotics_Joint lobj_82;
  c_robotics_manip_internal_Rigid lobj_81;
  c_robotics_manip_internal_Rigid lobj_80;
  c_robotics_manip_internal_Rigid lobj_79;
  c_robotics_manip_internal_Rigid lobj_78;
  c_robotics_manip_internal_Rigid lobj_77;
  c_robotics_manip_internal_Rigid lobj_76;
  c_robotics_manip_internal_Rigid lobj_75;
  c_robotics_manip_internal_Rigid lobj_74;
  c_robotics_manip_internal_Rigid lobj_73;
  c_robotics_manip_internal_Rigid lobj_72;
  c_robotics_manip_internal_Rigid lobj_71;
  c_robotics_manip_internal_Rigid lobj_70;
  c_robotics_manip_internal_Rigid lobj_69;
  c_robotics_manip_internal_Rigid lobj_68;
  c_robotics_manip_internal_Rigid lobj_67;
  d_robotics_manip_internal_Rigid lobj_66;
  d_robotics_manip_internal_Rigid lobj_65;
  c_robotics_core_internal_Damped lobj_64;
  robotics_Joint lobj_63;
  robotics_Joint lobj_62;
  c_robotics_manip_internal_Rigid lobj_61;
  robotics_Joint lobj_60;
  robotics_Joint lobj_59;
  c_robotics_manip_internal_Rigid lobj_58;
  robotics_Joint lobj_57;
  robotics_Joint lobj_56;
  c_robotics_manip_internal_Rigid lobj_55;
  robotics_Joint lobj_54;
  robotics_Joint lobj_53;
  c_robotics_manip_internal_Rigid lobj_52;
  robotics_Joint lobj_51;
  robotics_Joint lobj_50;
  c_robotics_manip_internal_Rigid lobj_49;
  robotics_Joint lobj_48;
  robotics_Joint lobj_47;
  robotics_Joint lobj_46;
  robotics_Joint lobj_45;
  robotics_Joint lobj_44;
  e_robotics_manip_internal_Rigid lobj_43;
  f_robotics_manip_internal_Rigid lobj_42;
  robotics_Joint lobj_41;
  c_robotics_manip_internal_Rigid lobj_40;
  e_robotics_manip_internal_Rigid lobj_39;
  f_robotics_manip_internal_Rigid lobj_38;
  robotics_Joint lobj_37;
  c_robotics_manip_internal_Rigid lobj_36;
  e_robotics_manip_internal_Rigid lobj_35;
  f_robotics_manip_internal_Rigid lobj_34;
  robotics_Joint lobj_33;
  c_robotics_manip_internal_Rigid lobj_32;
  e_robotics_manip_internal_Rigid lobj_31;
  f_robotics_manip_internal_Rigid lobj_30;
  robotics_Joint lobj_29;
  c_robotics_manip_internal_Rigid lobj_28;
  e_robotics_manip_internal_Rigid lobj_27;
  f_robotics_manip_internal_Rigid lobj_26;
  robotics_Joint lobj_25;
  c_robotics_manip_internal_Rigid lobj_24;
  robotics_Joint lobj_23;
  robotics_Joint lobj_22;
  robotics_Joint lobj_21;
  robotics_Joint lobj_20;
  robotics_Joint lobj_19;
  c_robotics_manip_internal_Rigid lobj_18;
  c_robotics_manip_internal_Rigid lobj_17;
  c_robotics_manip_internal_Rigid lobj_16;
  c_robotics_manip_internal_Rigid lobj_15;
  c_robotics_manip_internal_Rigid lobj_14;
  robotics_Joint lobj_13;
  d_robotics_manip_internal_Rigid lobj_12;
  robotics_InverseKinematics ik;
  robotics_Joint jnt5;
  robotics_Joint jnt4;
  robotics_Joint jnt3;
  robotics_Joint jnt2;
  robotics_Joint jnt1;
  c_struct_T initialguess[4];
  robotics_RigidBodyTree robot;
  robotics_RigidBody body1;
  robotics_RigidBody body2;
  robotics_RigidBody body3;
  robotics_RigidBody body4;
  robotics_RigidBody body5;
  c_robotics_manip_internal_Rigid *obj;
  emxArray_int8_T *A;
  emxArray_struct_T *jointAngle;
  double d0;
  double n;
  int i0;
  int i1;
  double b_r[3];
  int loop_ub;
  double dv0[16];
  int b_loop_ub;
  emxArray_real_T *b;
  double k;
  double m;
  emxArray_real_T *r0;
  emxArray_int8_T *b_I;
  int i;
  boolean_T jointAngle_data[1];
  boolean_T guard1 = false;
  robotics_Joint *joint;
  boolean_T guard2 = false;
  double pnum;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  int i2;
  boolean_T x_data_idx_0;
  int b_m;
  boolean_T y;
  int i3;
  double c_I[16];
  c_emxInitStruct_robotics_manip_(&lobj_104);
  emxInitStruct_robotics_Joint(&lobj_103);
  emxInitStruct_robotics_Joint(&lobj_102);
  emxInitStruct_robotics_Joint(&lobj_101);
  emxInitStruct_robotics_Joint(&lobj_100);
  emxInitStruct_robotics_Joint(&lobj_99);
  emxInitStruct_robotics_Joint(&lobj_98);
  emxInitStruct_robotics_Joint(&lobj_97);
  emxInitStruct_robotics_Joint(&lobj_96);
  emxInitStruct_robotics_Joint(&lobj_95);
  emxInitStruct_robotics_Joint(&lobj_94);
  emxInitStruct_robotics_Joint(&lobj_93);
  emxInitStruct_robotics_Joint(&lobj_92);
  emxInitStruct_robotics_Joint(&lobj_91);
  emxInitStruct_robotics_Joint(&lobj_90);
  emxInitStruct_robotics_Joint(&lobj_89);
  emxInitStruct_robotics_Joint(&lobj_88);
  emxInitStruct_robotics_Joint(&lobj_87);
  emxInitStruct_robotics_Joint(&lobj_86);
  emxInitStruct_robotics_Joint(&lobj_85);
  emxInitStruct_robotics_Joint(&lobj_84);
  emxInitStruct_robotics_Joint(&lobj_83);
  emxInitStruct_robotics_Joint(&lobj_82);
  d_emxInitStruct_robotics_manip_(&lobj_81);
  d_emxInitStruct_robotics_manip_(&lobj_80);
  d_emxInitStruct_robotics_manip_(&lobj_79);
  d_emxInitStruct_robotics_manip_(&lobj_78);
  d_emxInitStruct_robotics_manip_(&lobj_77);
  d_emxInitStruct_robotics_manip_(&lobj_76);
  d_emxInitStruct_robotics_manip_(&lobj_75);
  d_emxInitStruct_robotics_manip_(&lobj_74);
  d_emxInitStruct_robotics_manip_(&lobj_73);
  d_emxInitStruct_robotics_manip_(&lobj_72);
  d_emxInitStruct_robotics_manip_(&lobj_71);
  d_emxInitStruct_robotics_manip_(&lobj_70);
  d_emxInitStruct_robotics_manip_(&lobj_69);
  d_emxInitStruct_robotics_manip_(&lobj_68);
  d_emxInitStruct_robotics_manip_(&lobj_67);
  e_emxInitStruct_robotics_manip_(&lobj_66);
  e_emxInitStruct_robotics_manip_(&lobj_65);
  c_emxInitStruct_robotics_core_i(&lobj_64);
  emxInitStruct_robotics_Joint(&lobj_63);
  emxInitStruct_robotics_Joint(&lobj_62);
  d_emxInitStruct_robotics_manip_(&lobj_61);
  emxInitStruct_robotics_Joint(&lobj_60);
  emxInitStruct_robotics_Joint(&lobj_59);
  d_emxInitStruct_robotics_manip_(&lobj_58);
  emxInitStruct_robotics_Joint(&lobj_57);
  emxInitStruct_robotics_Joint(&lobj_56);
  d_emxInitStruct_robotics_manip_(&lobj_55);
  emxInitStruct_robotics_Joint(&lobj_54);
  emxInitStruct_robotics_Joint(&lobj_53);
  d_emxInitStruct_robotics_manip_(&lobj_52);
  emxInitStruct_robotics_Joint(&lobj_51);
  emxInitStruct_robotics_Joint(&lobj_50);
  d_emxInitStruct_robotics_manip_(&lobj_49);
  emxInitStruct_robotics_Joint(&lobj_48);
  emxInitStruct_robotics_Joint(&lobj_47);
  emxInitStruct_robotics_Joint(&lobj_46);
  emxInitStruct_robotics_Joint(&lobj_45);
  emxInitStruct_robotics_Joint(&lobj_44);
  f_emxInitStruct_robotics_manip_(&lobj_43);
  g_emxInitStruct_robotics_manip_(&lobj_42);
  emxInitStruct_robotics_Joint(&lobj_41);
  d_emxInitStruct_robotics_manip_(&lobj_40);
  f_emxInitStruct_robotics_manip_(&lobj_39);
  g_emxInitStruct_robotics_manip_(&lobj_38);
  emxInitStruct_robotics_Joint(&lobj_37);
  d_emxInitStruct_robotics_manip_(&lobj_36);
  f_emxInitStruct_robotics_manip_(&lobj_35);
  g_emxInitStruct_robotics_manip_(&lobj_34);
  emxInitStruct_robotics_Joint(&lobj_33);
  d_emxInitStruct_robotics_manip_(&lobj_32);
  f_emxInitStruct_robotics_manip_(&lobj_31);
  g_emxInitStruct_robotics_manip_(&lobj_30);
  emxInitStruct_robotics_Joint(&lobj_29);
  d_emxInitStruct_robotics_manip_(&lobj_28);
  f_emxInitStruct_robotics_manip_(&lobj_27);
  g_emxInitStruct_robotics_manip_(&lobj_26);
  emxInitStruct_robotics_Joint(&lobj_25);
  d_emxInitStruct_robotics_manip_(&lobj_24);
  emxInitStruct_robotics_Joint(&lobj_23);
  emxInitStruct_robotics_Joint(&lobj_22);
  emxInitStruct_robotics_Joint(&lobj_21);
  emxInitStruct_robotics_Joint(&lobj_20);
  emxInitStruct_robotics_Joint(&lobj_19);
  d_emxInitStruct_robotics_manip_(&lobj_18);
  d_emxInitStruct_robotics_manip_(&lobj_17);
  d_emxInitStruct_robotics_manip_(&lobj_16);
  d_emxInitStruct_robotics_manip_(&lobj_15);
  d_emxInitStruct_robotics_manip_(&lobj_14);
  emxInitStruct_robotics_Joint(&lobj_13);
  e_emxInitStruct_robotics_manip_(&lobj_12);
  c_emxInitStruct_robotics_Invers(&ik);
  emxInitStruct_robotics_Joint(&jnt5);
  emxInitStruct_robotics_Joint(&jnt4);
  emxInitStruct_robotics_Joint(&jnt3);
  emxInitStruct_robotics_Joint(&jnt2);
  emxInitStruct_robotics_Joint(&jnt1);
  ik.matlabCodegenIsDeleted = true;

  /*  Init Variablen und Structs */
  /* Teilkörper1 */
  /* Teilkörper2 */
  /* Teilkörper3 */
  /* Teilkörper4 */
  /* Länge des endefektors von zwei Position aus. Erste Position ist der  */
  /* Toolcenterpoint. Grappertip ist die spitze mit 2 cm abstand zum Objekt. */
  /* muss noch genauer vermessen werden. */
  /* Initialisiere initialguess und jointAngle. Somit wird vermieden das die */
  /* Variable initialguess und jointAngle dynamisch angelegt werden müssne. */
  /* Dieser Fehelr kan mit dem Matlabl Code Report Viewer angezeigt werden. */
  initialguess[0].JointName[0] = 'j';
  initialguess[1].JointName[0] = 'j';
  initialguess[2].JointName[0] = 'j';
  initialguess[3].JointName[0] = 'j';
  initialguess[0].JointName[1] = 'n';
  initialguess[1].JointName[1] = 'n';
  initialguess[2].JointName[1] = 'n';
  initialguess[3].JointName[1] = 'n';
  initialguess[0].JointName[2] = 't';
  initialguess[1].JointName[2] = 't';
  initialguess[2].JointName[2] = 't';
  initialguess[3].JointName[2] = 't';
  initialguess[0].JointName[3] = '1';
  initialguess[1].JointName[3] = '2';
  initialguess[2].JointName[3] = '3';
  initialguess[3].JointName[3] = '4';

  /* Berechne Zylinder Koordinaten in Kartesische Koordinaten um */
  /*  Erstelle Roboter Arm */
  robot.TreeInternal = RigidBodyTree_RigidBodyTree(&lobj_12, &lobj_13, &lobj_14,
    &lobj_15, &lobj_16, &lobj_17, &lobj_18, &lobj_19, &lobj_20, &lobj_21,
    &lobj_22, &lobj_23);

  /*  TODO in der Final Version wird eine identifer übergeben ob vom tool */
  /*  centerpoint oder von der Endefektor spitze ausgegangen wird. Dies wird */
  /*  dann zum Teilkörper 4 dazuaddiert. */
  /* dreh um z */
  /* dreh um y */
  /* dreh um x */
  b_RigidBody_RigidBody(&body1, &lobj_24, &lobj_25, &lobj_26, &lobj_27);
  c_Joint_Joint(&jnt1);
  Joint_set_JointAxis(&jnt1);

  /* Home Position = 0° */
  Joint_set_HomePosition(&jnt1);

  /* PositionsLimits -180°<->180° */
  Joint_set_PositionLimits(&jnt1);
  c_RigidBody_RigidBody(&body2, &lobj_28, &lobj_29, &lobj_30, &lobj_31);
  d_Joint_Joint(&jnt2);
  b_Joint_set_JointAxis(&jnt2);

  /* Home Position = -90° */
  b_Joint_set_HomePosition(&jnt2);

  /* PositionsLimits -20°<->70° */
  /* Eigenständig bestimmt PositionsLimits -20°<->90° */
  b_Joint_set_PositionLimits(&jnt2);
  d_RigidBody_RigidBody(&body3, &lobj_32, &lobj_33, &lobj_34, &lobj_35);
  e_Joint_Joint(&jnt3);
  b_Joint_set_JointAxis(&jnt3);

  /* Home Position = -90° */
  b_Joint_set_HomePosition(&jnt3);

  /* PositionsLimits -150°<->150° */
  c_Joint_set_PositionLimits(&jnt3);
  e_RigidBody_RigidBody(&body4, &lobj_36, &lobj_37, &lobj_38, &lobj_39);
  f_Joint_Joint(&jnt4);
  b_Joint_set_JointAxis(&jnt4);

  /* Home Position = -90° */
  b_Joint_set_HomePosition(&jnt4);

  /* PositionsLimits -100°<->100° */
  d_Joint_set_PositionLimits(&jnt4);
  f_RigidBody_RigidBody(&body5, &lobj_40, &lobj_41, &lobj_42, &lobj_43);
  g_Joint_Joint(&jnt5);

  /* jnt5.JointAxis = [1 0 0]; */
  /* Home Position = 0° */
  /* jnt5.HomePosition = 0; */
  /* PositionsLimits -180°<->180° */
  /* jnt5.PositionLimits = [-pi pi]; */
  Joint_setFixedTransform(&jnt1);
  b_Joint_setFixedTransform(&jnt2);
  c_Joint_setFixedTransform(&jnt3);
  c_Joint_setFixedTransform(&jnt4);
  d_Joint_setFixedTransform(&jnt5);
  obj = body1.BodyInternal;
  obj->JointInternal = Joint_copy(&jnt1, &lobj_44);
  obj = body2.BodyInternal;
  obj->JointInternal = Joint_copy(&jnt2, &lobj_45);
  obj = body3.BodyInternal;
  obj->JointInternal = Joint_copy(&jnt3, &lobj_46);
  obj = body4.BodyInternal;
  obj->JointInternal = Joint_copy(&jnt4, &lobj_47);
  obj = body5.BodyInternal;
  obj->JointInternal = Joint_copy(&jnt5, &lobj_48);
  RigidBodyTree_addBody(&robot, &body1, &lobj_49, &lobj_50, &lobj_51);
  b_RigidBodyTree_addBody(&robot, &body2, &lobj_52, &lobj_53, &lobj_54);
  c_RigidBodyTree_addBody(&robot, &body3, &lobj_55, &lobj_56, &lobj_57);
  d_RigidBodyTree_addBody(&robot, &body4, &lobj_58, &lobj_59, &lobj_60);
  e_RigidBodyTree_addBody(&robot, &body5, &lobj_61, &lobj_62, &lobj_63);

  /*  q1 q2 q2 q4 von Enkoderwerten von Grad in Rad */
  initialguess[0].JointPosition = 0.017453292519943295 * -(q1_init - 180.0);
  initialguess[1].JointPosition = 0.017453292519943295 * -(q2_init + 270.0);
  initialguess[2].JointPosition = 0.017453292519943295 * -(q3_init + 180.0);
  initialguess[3].JointPosition = 0.017453292519943295 * -(q4_init + 180.0);

  /*  Berechne inverse Kinematik für den erstellten Roboterarm */
  c_InverseKinematics_InverseKine(&ik, &robot, &lobj_64, &lobj_65, &lobj_66,
    &lobj_67, &lobj_68, &lobj_69, &lobj_70, &lobj_71, &lobj_72, &lobj_73,
    &lobj_74, &lobj_75, &lobj_76, &lobj_77, &lobj_78, &lobj_79, &lobj_80,
    &lobj_81, &lobj_82, &lobj_83, &lobj_84, &lobj_85, &lobj_86, &lobj_87,
    &lobj_88, &lobj_89, &lobj_90, &lobj_91, &lobj_92, &lobj_93, &lobj_94,
    &lobj_95, &lobj_96, &lobj_97, &lobj_98, &lobj_99, &lobj_100, &lobj_101,
    &lobj_102, &lobj_103);
  if (ik.isInitialized != 1) {
    emxInit_int8_T(&A, 2);
    ik.isSetupComplete = false;
    ik.isInitialized = 1;
    n = ik.RigidBodyTreeInternal->PositionNumber;
    i0 = A->size[0] * A->size[1];
    i1 = (int)n;
    A->size[0] = i1;
    loop_ub = (int)(2.0 * n);
    A->size[1] = loop_ub;
    emxEnsureCapacity_int8_T(A, i0);
    b_loop_ub = i1 * loop_ub;
    for (i0 = 0; i0 < b_loop_ub; i0++) {
      A->data[i0] = 0;
    }

    emxInit_real_T(&b, 1);
    i0 = b->size[0];
    b->size[0] = loop_ub;
    emxEnsureCapacity_real_T(b, i0);
    for (i0 = 0; i0 < loop_ub; i0++) {
      b->data[i0] = 0.0;
    }

    k = 1.0;
    m = 1.0;
    d0 = ik.RigidBodyTreeInternal->NumBodies;
    i0 = (int)d0;
    emxInit_real_T(&r0, 2);
    emxInit_int8_T(&b_I, 2);
    for (i = 0; i < i0; i++) {
      obj = ik.RigidBodyTreeInternal->Bodies[i];
      joint = obj->JointInternal;
      pnum = joint->PositionNumber;
      if (!d_strcmp(joint->Type)) {
        if (k > (k + pnum) - 1.0) {
          i1 = 0;
        } else {
          i1 = (int)k - 1;
        }

        if (m > (m + pnum) - 1.0) {
          i2 = 0;
        } else {
          i2 = (int)m - 1;
        }

        if (pnum < 0.0) {
          n = 0.0;
        } else {
          n = pnum;
        }

        b_m = (int)n;
        i3 = b_I->size[0] * b_I->size[1];
        b_I->size[0] = (int)n;
        b_I->size[1] = (int)n;
        emxEnsureCapacity_int8_T(b_I, i3);
        loop_ub = (int)n * (int)n;
        for (i3 = 0; i3 < loop_ub; i3++) {
          b_I->data[i3] = 0;
        }

        if ((int)n > 0) {
          for (b_loop_ub = 0; b_loop_ub < b_m; b_loop_ub++) {
            b_I->data[b_loop_ub + b_I->size[0] * b_loop_ub] = 1;
          }
        }

        loop_ub = b_I->size[1];
        for (i3 = 0; i3 < loop_ub; i3++) {
          b_loop_ub = b_I->size[0];
          for (b_m = 0; b_m < b_loop_ub; b_m++) {
            A->data[(i1 + b_m) + A->size[0] * (i2 + i3)] = b_I->data[b_m +
              b_I->size[0] * i3];
          }
        }

        if (k > (k + pnum) - 1.0) {
          i1 = 0;
        } else {
          i1 = (int)k - 1;
        }

        d0 = m + pnum;
        if (d0 > (m + 2.0 * pnum) - 1.0) {
          i2 = 0;
        } else {
          i2 = (int)d0 - 1;
        }

        if (pnum < 0.0) {
          n = 0.0;
        } else {
          n = pnum;
        }

        b_m = (int)n;
        i3 = b_I->size[0] * b_I->size[1];
        b_I->size[0] = (int)n;
        b_I->size[1] = (int)n;
        emxEnsureCapacity_int8_T(b_I, i3);
        loop_ub = (int)n * (int)n;
        for (i3 = 0; i3 < loop_ub; i3++) {
          b_I->data[i3] = 0;
        }

        if ((int)n > 0) {
          for (b_loop_ub = 0; b_loop_ub < b_m; b_loop_ub++) {
            b_I->data[b_loop_ub + b_I->size[0] * b_loop_ub] = 1;
          }
        }

        loop_ub = b_I->size[1];
        for (i3 = 0; i3 < loop_ub; i3++) {
          b_loop_ub = b_I->size[0];
          for (b_m = 0; b_m < b_loop_ub; b_m++) {
            A->data[(i1 + b_m) + A->size[0] * (i2 + i3)] = (signed char)-
              b_I->data[b_m + b_I->size[0] * i3];
          }
        }

        i1 = r0->size[0] * r0->size[1];
        r0->size[0] = joint->PositionLimitsInternal->size[0];
        r0->size[1] = 2;
        emxEnsureCapacity_real_T(r0, i1);
        loop_ub = joint->PositionLimitsInternal->size[0] *
          joint->PositionLimitsInternal->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          r0->data[i1] = joint->PositionLimitsInternal->data[i1];
        }

        b->data[(int)m - 1] = r0->data[1];
        i1 = r0->size[0] * r0->size[1];
        r0->size[0] = joint->PositionLimitsInternal->size[0];
        r0->size[1] = 2;
        emxEnsureCapacity_real_T(r0, i1);
        loop_ub = joint->PositionLimitsInternal->size[0] *
          joint->PositionLimitsInternal->size[1];
        for (i1 = 0; i1 < loop_ub; i1++) {
          r0->data[i1] = joint->PositionLimitsInternal->data[i1];
        }

        b->data[(int)(unsigned int)m] = -r0->data[0];
        m += 2.0 * pnum;
      }

      k += pnum;
    }

    emxFree_int8_T(&b_I);
    i0 = ik.Solver->ConstraintMatrix->size[0] * ik.Solver->
      ConstraintMatrix->size[1];
    ik.Solver->ConstraintMatrix->size[0] = A->size[0];
    ik.Solver->ConstraintMatrix->size[1] = A->size[1];
    emxEnsureCapacity_real_T(ik.Solver->ConstraintMatrix, i0);
    loop_ub = A->size[0] * A->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      ik.Solver->ConstraintMatrix->data[i0] = A->data[i0];
    }

    emxFree_int8_T(&A);
    i0 = ik.Solver->ConstraintBound->size[0];
    ik.Solver->ConstraintBound->size[0] = b->size[0];
    emxEnsureCapacity_real_T(ik.Solver->ConstraintBound, i0);
    loop_ub = b->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      ik.Solver->ConstraintBound->data[i0] = b->data[i0];
    }

    c_RigidBodyTree_get_JointPositi(ik.RigidBodyTreeInternal, r0);
    i0 = ik.Limits->size[0] * ik.Limits->size[1];
    ik.Limits->size[0] = r0->size[0];
    ik.Limits->size[1] = 2;
    emxEnsureCapacity_real_T(ik.Limits, i0);
    loop_ub = r0->size[0] * r0->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      ik.Limits->data[i0] = r0->data[i0];
    }

    emxFree_real_T(&r0);
    ik.Solver->ExtraArgs = &lobj_104;
    for (i0 = 0; i0 < 36; i0++) {
      ik.Solver->ExtraArgs->WeightMatrix[i0] = 0.0;
    }

    ik.Solver->ExtraArgs->Robot = ik.RigidBodyTreeInternal;
    memset(&c_I[0], 0, sizeof(double) << 4);
    c_I[0] = 1.0;
    c_I[5] = 1.0;
    c_I[10] = 1.0;
    c_I[15] = 1.0;
    for (i0 = 0; i0 < 16; i0++) {
      ik.Solver->ExtraArgs->Tform[i0] = c_I[i0];
    }

    ik.Solver->ExtraArgs->BodyName->size[0] = 1;
    ik.Solver->ExtraArgs->BodyName->size[1] = 0;
    i0 = ik.Solver->ExtraArgs->ErrTemp->size[0];
    ik.Solver->ExtraArgs->ErrTemp->size[0] = 6;
    emxEnsureCapacity_real_T(ik.Solver->ExtraArgs->ErrTemp, i0);
    for (i0 = 0; i0 < 6; i0++) {
      ik.Solver->ExtraArgs->ErrTemp->data[i0] = 0.0;
    }

    ik.Solver->ExtraArgs->CostTemp = 0.0;
    i0 = b->size[0];
    b->size[0] = (int)ik.RigidBodyTreeInternal->PositionNumber;
    emxEnsureCapacity_real_T(b, i0);
    loop_ub = (int)ik.RigidBodyTreeInternal->PositionNumber;
    for (i0 = 0; i0 < loop_ub; i0++) {
      b->data[i0] = 0.0;
    }

    i0 = ik.Solver->ExtraArgs->GradTemp->size[0];
    ik.Solver->ExtraArgs->GradTemp->size[0] = b->size[0];
    emxEnsureCapacity_real_T(ik.Solver->ExtraArgs->GradTemp, i0);
    loop_ub = b->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      ik.Solver->ExtraArgs->GradTemp->data[i0] = b->data[i0];
    }

    emxFree_real_T(&b);
    ik.isSetupComplete = true;
  }

  emxInit_struct_T(&jointAngle, 2);
  d0 = phi;
  b_cosd(&d0);
  n = phi;
  b_sind(&n);
  b_r[0] = r * d0;
  b_r[1] = h * n;
  b_r[2] = h;
  trvec2tform(b_r, dv0);
  InverseKinematics_stepImpl(&ik, dv0, initialguess, jointAngle);

  /*  Gelenk Q1 */
  q1_size[0] = 1;
  q1_size[1] = 1;
  q1_data[0] = 180.0 + jointAngle->data[0].JointPosition.data[0] *
    565.48667764616278;

  /*  Gelenk Q2 */
  /*  Der Ik Solver erzeugt für das Gelenk Q2 folgende winkel ausgaben. */
  /*  Nun wird der für jeden Qudranten der erzeugte Winkel umgerechnet. */
  /*  Dazu wird auf jeden Qudaraten per if anfrage speziell der Wert angepasst. */
  /*     -90°/|\-90° */
  /*     <->/ | \<-> */
  /*  -180°/  |  \ 0° */
  /*  --------|-------- */
  /*   180°\  |  /0° */
  /*     <->\ | /<-> */
  /*      90°\|/90° */
  /* für das gelenk q2 erster Quandrant 0° <-> -90° */
  jointAngle_data[0] = (jointAngle->data[1].JointPosition.data[0] < 0.0);
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (b_all(jointAngle_data)) {
    jointAngle_data[0] = (jointAngle->data[1].JointPosition.data[0] >=
                          -1.5707963267948966);
    if (b_all(jointAngle_data)) {
      q2_size[0] = 1;
      q2_size[1] = 1;
      q2_data[0] = 90.0 + -(jointAngle->data[1].JointPosition.data[0] *
                            57.295779513082323);

      /* für das gelenk q2 zweite Quandrant 0° <-> -90° */
    } else {
      guard4 = true;
    }
  } else {
    guard4 = true;
  }

  if (guard4) {
    jointAngle_data[0] = (jointAngle->data[1].JointPosition.data[0] <
                          -1.5707963267948966);
    if (b_all(jointAngle_data)) {
      jointAngle_data[0] = (jointAngle->data[1].JointPosition.data[0] >=
                            -3.1415926535897931);
      if (b_all(jointAngle_data)) {
        q2_size[0] = 1;
        q2_size[1] = 1;
        q2_data[0] = 90.0 + -(jointAngle->data[1].JointPosition.data[0] *
                              57.295779513082323);

        /* für das gelenk q2 vierter Quandrant 0° <-> -90° */
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  }

  if (guard3) {
    jointAngle_data[0] = (jointAngle->data[1].JointPosition.data[0] > 0.0);
    if (b_all(jointAngle_data)) {
      x_data_idx_0 = (jointAngle->data[1].JointPosition.data[0] <=
                      1.5707963267948966);
      y = true;
      if (!x_data_idx_0) {
        y = false;
      }

      if (y) {
        q2_size[0] = 1;
        q2_size[1] = 1;
        q2_data[0] = 90.0 - jointAngle->data[1].JointPosition.data[0] *
          57.295779513082323;

        /* für das gelenk q2 dritter Quandrant 0° <-> -90° */
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
  }

  if (guard2) {
    x_data_idx_0 = (jointAngle->data[1].JointPosition.data[0] >
                    1.5707963267948966);
    y = true;
    if (!x_data_idx_0) {
      y = false;
    }

    if (y) {
      x_data_idx_0 = (jointAngle->data[1].JointPosition.data[0] <=
                      3.1415926535897931);
      y = true;
      if (!x_data_idx_0) {
        y = false;
      }

      if (y) {
        q2_size[0] = 1;
        q2_size[1] = 1;
        q2_data[0] = 360.0 - (jointAngle->data[1].JointPosition.data[0] *
                              57.295779513082323 - 90.0);
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    q2_size[0] = 1;
    q2_size[1] = 1;
    q2_data[0] = 360.0 - (jointAngle->data[1].JointPosition.data[0] *
                          57.295779513082323 - 90.0);
  }

  /*  Gelenk q3, q4 */
  /* Hier zeigt sich im test das für jeden Quadraten der gleiche wert von 180 */
  /* aufadiert werden muss um die werte an den Encoder anzupassen. deshalb wird */
  /* keine if abfrage erzeugt. */
  /*     -90°/|\-90° */
  /*     <->/ | \<-> */
  /*  -180°/  |  \ 0° */
  /*  --------|-------- */
  /*   180°\  |  /0° */
  /*     <->\ | /<-> */
  /*      90°\|/90° */
  /* für das gelenk q3 erster Quandrant 0° <-> -90° */
  /* für das gelenk q3 zweite Quandrant -90° <-> -180° */
  /* für das gelenk q3 vierter Quandrant 0° <-> 90° */
  /* für das gelenk q3 dritter Quandrant 90° <-> 180° */
  q3_size[0] = 1;
  q3_size[1] = 1;
  q3_data[0] = 180.0 + -(jointAngle->data[2].JointPosition.data[0] *
    57.295779513082323);

  /* für das gelenk q4 erster Quandrant 0° <-> -90° */
  /* für das gelenk q4 zweite Quandrant -90° <-> -180° */
  /* für das gelenk q4 vierter Quandrant 0° <-> 90° */
  /* für das gelenk q4 dritter Quandrant 90° <-> 180° */
  q4_size[0] = 1;
  q4_size[1] = 1;
  q4_data[0] = 180.0 + -(jointAngle->data[3].JointPosition.data[0] *
    57.295779513082323);
  emxFree_struct_T(&jointAngle);
  if (!ik.matlabCodegenIsDeleted) {
    ik.matlabCodegenIsDeleted = true;
    if (ik.isInitialized == 1) {
      ik.isInitialized = 2;
    }
  }

  emxFreeStruct_robotics_Joint(&jnt1);
  emxFreeStruct_robotics_Joint(&jnt2);
  emxFreeStruct_robotics_Joint(&jnt3);
  emxFreeStruct_robotics_Joint(&jnt4);
  emxFreeStruct_robotics_Joint(&jnt5);
  c_emxFreeStruct_robotics_Invers(&ik);
  e_emxFreeStruct_robotics_manip_(&lobj_12);
  emxFreeStruct_robotics_Joint(&lobj_13);
  d_emxFreeStruct_robotics_manip_(&lobj_14);
  d_emxFreeStruct_robotics_manip_(&lobj_15);
  d_emxFreeStruct_robotics_manip_(&lobj_16);
  d_emxFreeStruct_robotics_manip_(&lobj_17);
  d_emxFreeStruct_robotics_manip_(&lobj_18);
  emxFreeStruct_robotics_Joint(&lobj_19);
  emxFreeStruct_robotics_Joint(&lobj_20);
  emxFreeStruct_robotics_Joint(&lobj_21);
  emxFreeStruct_robotics_Joint(&lobj_22);
  emxFreeStruct_robotics_Joint(&lobj_23);
  d_emxFreeStruct_robotics_manip_(&lobj_24);
  emxFreeStruct_robotics_Joint(&lobj_25);
  g_emxFreeStruct_robotics_manip_(&lobj_26);
  f_emxFreeStruct_robotics_manip_(&lobj_27);
  d_emxFreeStruct_robotics_manip_(&lobj_28);
  emxFreeStruct_robotics_Joint(&lobj_29);
  g_emxFreeStruct_robotics_manip_(&lobj_30);
  f_emxFreeStruct_robotics_manip_(&lobj_31);
  d_emxFreeStruct_robotics_manip_(&lobj_32);
  emxFreeStruct_robotics_Joint(&lobj_33);
  g_emxFreeStruct_robotics_manip_(&lobj_34);
  f_emxFreeStruct_robotics_manip_(&lobj_35);
  d_emxFreeStruct_robotics_manip_(&lobj_36);
  emxFreeStruct_robotics_Joint(&lobj_37);
  g_emxFreeStruct_robotics_manip_(&lobj_38);
  f_emxFreeStruct_robotics_manip_(&lobj_39);
  d_emxFreeStruct_robotics_manip_(&lobj_40);
  emxFreeStruct_robotics_Joint(&lobj_41);
  g_emxFreeStruct_robotics_manip_(&lobj_42);
  f_emxFreeStruct_robotics_manip_(&lobj_43);
  emxFreeStruct_robotics_Joint(&lobj_44);
  emxFreeStruct_robotics_Joint(&lobj_45);
  emxFreeStruct_robotics_Joint(&lobj_46);
  emxFreeStruct_robotics_Joint(&lobj_47);
  emxFreeStruct_robotics_Joint(&lobj_48);
  d_emxFreeStruct_robotics_manip_(&lobj_49);
  emxFreeStruct_robotics_Joint(&lobj_50);
  emxFreeStruct_robotics_Joint(&lobj_51);
  d_emxFreeStruct_robotics_manip_(&lobj_52);
  emxFreeStruct_robotics_Joint(&lobj_53);
  emxFreeStruct_robotics_Joint(&lobj_54);
  d_emxFreeStruct_robotics_manip_(&lobj_55);
  emxFreeStruct_robotics_Joint(&lobj_56);
  emxFreeStruct_robotics_Joint(&lobj_57);
  d_emxFreeStruct_robotics_manip_(&lobj_58);
  emxFreeStruct_robotics_Joint(&lobj_59);
  emxFreeStruct_robotics_Joint(&lobj_60);
  d_emxFreeStruct_robotics_manip_(&lobj_61);
  emxFreeStruct_robotics_Joint(&lobj_62);
  emxFreeStruct_robotics_Joint(&lobj_63);
  c_emxFreeStruct_robotics_core_i(&lobj_64);
  e_emxFreeStruct_robotics_manip_(&lobj_65);
  e_emxFreeStruct_robotics_manip_(&lobj_66);
  d_emxFreeStruct_robotics_manip_(&lobj_67);
  d_emxFreeStruct_robotics_manip_(&lobj_68);
  d_emxFreeStruct_robotics_manip_(&lobj_69);
  d_emxFreeStruct_robotics_manip_(&lobj_70);
  d_emxFreeStruct_robotics_manip_(&lobj_71);
  d_emxFreeStruct_robotics_manip_(&lobj_72);
  d_emxFreeStruct_robotics_manip_(&lobj_73);
  d_emxFreeStruct_robotics_manip_(&lobj_74);
  d_emxFreeStruct_robotics_manip_(&lobj_75);
  d_emxFreeStruct_robotics_manip_(&lobj_76);
  d_emxFreeStruct_robotics_manip_(&lobj_77);
  d_emxFreeStruct_robotics_manip_(&lobj_78);
  d_emxFreeStruct_robotics_manip_(&lobj_79);
  d_emxFreeStruct_robotics_manip_(&lobj_80);
  d_emxFreeStruct_robotics_manip_(&lobj_81);
  emxFreeStruct_robotics_Joint(&lobj_82);
  emxFreeStruct_robotics_Joint(&lobj_83);
  emxFreeStruct_robotics_Joint(&lobj_84);
  emxFreeStruct_robotics_Joint(&lobj_85);
  emxFreeStruct_robotics_Joint(&lobj_86);
  emxFreeStruct_robotics_Joint(&lobj_87);
  emxFreeStruct_robotics_Joint(&lobj_88);
  emxFreeStruct_robotics_Joint(&lobj_89);
  emxFreeStruct_robotics_Joint(&lobj_90);
  emxFreeStruct_robotics_Joint(&lobj_91);
  emxFreeStruct_robotics_Joint(&lobj_92);
  emxFreeStruct_robotics_Joint(&lobj_93);
  emxFreeStruct_robotics_Joint(&lobj_94);
  emxFreeStruct_robotics_Joint(&lobj_95);
  emxFreeStruct_robotics_Joint(&lobj_96);
  emxFreeStruct_robotics_Joint(&lobj_97);
  emxFreeStruct_robotics_Joint(&lobj_98);
  emxFreeStruct_robotics_Joint(&lobj_99);
  emxFreeStruct_robotics_Joint(&lobj_100);
  emxFreeStruct_robotics_Joint(&lobj_101);
  emxFreeStruct_robotics_Joint(&lobj_102);
  emxFreeStruct_robotics_Joint(&lobj_103);
  c_emxFreeStruct_robotics_manip_(&lobj_104);
}

/*
 * File trailer for IK_solver.c
 *
 * [EOF]
 */
