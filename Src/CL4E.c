/*
 * CL4E.c
 *
 *  Created on: 23.07.2019
 *      Author: Sven
 */

#include "CL4E.h"


canOpen_typeDef_SDOprimitive CL4E_initSDOs[INIT_SDO_COUNT] =
{
          {0x6060, 0 , 0x03, 1}		//1
         ,{0x60FF, 0 , 0x00000000, 4}		//2
         ,{0x6040, 0 , 0x0000, 2}		//3
         ,{0x6040, 0, 0x06, 2}             // 4 ready to swicth on
         ,{0x6040, 0, 0x07, 2}             // 5 switch on
         ,{0x6040, 0, 0x0F, 2}             // 6 OE
         ,{0x6040, 0, 0x03, 2}             // 7 quick stop

//         ,{0x6040, 0 , 0x1F, 2}		//3
//         ,{0x6040, 0 , 0x0F, 2}		//4
//         ,{0x6040, 0 , 0x00, 2}		//5
//         ,{0x6040, 0 , 0x06, 2}		//6
//         ,{0x6040, 0 , 0x07, 2}		//7
//         ,{0x6040, 0 , 0x0F, 2}		//8


         /* ATTENTION!
          * always check number of SDOs INIT_SDO_COUNT!
          * must match entries here
          */
};

canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor1[INIT_SDO_COUNT] =
{
          {0x6060, 0 , 0x03, 1}		//1
         ,{0x60FF, 0 , 0x00000000, 4}		//2
         ,{0x6040, 0 , 0x0000, 2}		//3
          ,{0x6040, 0, 0x06, 2}             // 4 ready to swicth on
          ,{0x6040, 0, 0x07, 2}             // 5 switch on
          ,{0x6040, 0, 0x0F, 2}             // 6 OE
          ,{0x6040, 0, 0x03, 2}             // 7 quick stop
};
canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor2[INIT_SDO_COUNT] =
{
          {0x6060, 0 , 0x03, 1}		//1
         ,{0x60FF, 0 , 0x00000000, 4}		//2
         ,{0x6040, 0 , 0x0000, 2}		//3
          ,{0x6040, 0, 0x06, 2}             // 4 ready to swicth on
          ,{0x6040, 0, 0x07, 2}             // 5 switch on
          ,{0x6040, 0, 0x0F, 2}             // 6 OE
          ,{0x6040, 0, 0x03, 2}             // 7 quick stop
};
canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor3[INIT_SDO_COUNT] =
{
          {0x6060, 0 , 0x03, 1}		//1
         ,{0x60FF, 0 , 0x00000000, 4}		//2
         ,{0x6040, 0 , 0x0000, 2}		//3
          ,{0x6040, 0, 0x06, 2}             // 4 ready to swicth on
          ,{0x6040, 0, 0x07, 2}             // 5 switch on
          ,{0x6040, 0, 0x0F, 2}             // 6 OE
          ,{0x6040, 0, 0x03, 2}             // 7 quick stop
};
canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor4[INIT_SDO_COUNT] =
{
          {0x6060, 0 , 0x03, 1}		//1
         ,{0x60FF, 0 , 0x00000000, 4}		//2
         ,{0x6040, 0 , 0x0000, 2}		//3
          ,{0x6040, 0, 0x06, 2}             // 4 ready to swicth on
          ,{0x6040, 0, 0x07, 2}             // 5 switch on
          ,{0x6040, 0, 0x0F, 2}             // 6 OE
          ,{0x6040, 0, 0x03, 2}             // 7 quick stop
};
canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor5[INIT_SDO_COUNT] =
{
          {0x6060, 0 , 0x03, 1}		//1
         ,{0x60FF, 0 , 0x00000000, 4}		//2
         ,{0x6040, 0 , 0x0000, 2}		//3
          ,{0x6040, 0, 0x06, 2}             // 4 ready to swicth on
          ,{0x6040, 0, 0x07, 2}             // 5 switch on
          ,{0x6040, 0, 0x0F, 2}             // 6 OE
          ,{0x6040, 0, 0x03, 2}             // 7 quick stop
};
canOpen_typeDef_SDOprimitive CL4E_initSDOsMotor6[INIT_SDO_COUNT] =
{
          {0x6060, 0 , 0x03, 1}		//1
         ,{0x60FF, 0 , 0x00000000, 4}		//2
         ,{0x6040, 0 , 0x0000, 2}		//3
          ,{0x6040, 0, 0x06, 2}             // 4 ready to swicth on
          ,{0x6040, 0, 0x07, 2}             // 5 switch on
          ,{0x6040, 0, 0x0F, 2}             // 6 OE
          ,{0x6040, 0, 0x03, 2}             // 7 quick stop
};

