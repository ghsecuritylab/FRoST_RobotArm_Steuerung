/* =============================================================================
 * PROJEKT:	 FRoST_HorizonXIX_Firmware
 * DATEI:	 IkSolver.h
 * AUTOR:	 dannn
 * ERSTELLT: 14.08.2019
 * =============================================================================
 */

#ifndef IKSOLVER_H_
#define IKSOLVER_H_


/* ___________________________(GLOBALE DATENYPEN)___________________________
 */

/* ___________________________(GLOBALE VARIABLES)___________________________
 */

/* ___________________________(GLOBALE FUNKTIONEN)___________________________
 */

extern void IkSolver_init ( void );

extern void IkSolver_deInit ( void );

extern void IkSolver_getAngle ( double r, double h, double phi, double
		q1_init, double q2_init, double q3_init, double q4_init,
		double q1_ret[1], double q2_ret[1], double q3_ret[1], double q4_ret[1]);



#endif /* IKSOLVER_H_ */
