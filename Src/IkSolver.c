/* =============================================================================
 * PROJEKT:	 FRoST_HorizonXIX_Firmware
 * DATEI:	 IkSolver.c
 * AUTOR:	 dannn
 * ERSTELLT: 14.08.2019
 * =============================================================================
 */


#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/IK_solver_terminate.h"
#include "IK_solver/IK_solver_initialize.h"


/* _____________________________(GLOBALE VARIABLES)____________________________
 */
double q1_data[1] = {0};
int q1_size[2];
double q2_data[1] = {180};
int q2_size[2];
double q3_data[1] = {180};
int q3_size[2];
double q4_data[1] = {180};
int q4_size[2];


/* _____________________________(LOKALE DATENTYPEN)____________________________
 */

/* _____________________________(LOKALE VARIABLEN)_____________________________
 */

/* _______________________(PROTOTYPEN LOKALER FUNKTIONEN)______________________
 */

/* ____________________(IMPLEMENTATION GLOBALER FUNKTIONEN)____________________
 */
void IkSolver_init ( void )
{
	IK_solver_initialize();
	// init with random values
	IK_solver(1.3, 0.9, 0, 0, 180, 180, 180, q1_data, q1_size, q2_data, q2_size, q3_data, q3_size, q4_data, q4_size);
}

void IkSolver_deInit ( void )
{
	IK_solver_terminate();
}

void IkSolver_getAngle ( double r, double h, double phi, double
		q1_init, double q2_init, double q3_init, double q4_init,
		double q1_ret[1], double q2_ret[1], double q3_ret[1], double q4_ret[1])
{
	IK_solver(r, h, phi, q1_init, q2_init, q3_init, q4_init, q1_ret, q1_size, q2_ret, q2_size, q3_ret,
				q3_size, q4_ret, q4_size);
}

/* ____________________(IMPLEMENTATION LOKALER FUNKTIONEN)_____________________
 */
