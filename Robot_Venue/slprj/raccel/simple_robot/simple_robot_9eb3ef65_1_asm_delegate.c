#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "sm_CTarget.h"
void simple_robot_9eb3ef65_1_setTargets ( const RuntimeDerivedValuesBundle *
rtdv , CTarget * targets ) { ( void ) rtdv ; ( void ) targets ; } void
simple_robot_9eb3ef65_1_resetAsmStateVector ( const void * mech , double *
state ) { double xx [ 1 ] ; ( void ) mech ; xx [ 0 ] = 0.0 ; state [ 0 ] = xx
[ 0 ] ; state [ 1 ] = xx [ 0 ] ; state [ 2 ] = xx [ 0 ] ; state [ 3 ] = 1.0 ;
state [ 4 ] = xx [ 0 ] ; state [ 5 ] = xx [ 0 ] ; state [ 6 ] = xx [ 0 ] ;
state [ 7 ] = xx [ 0 ] ; state [ 8 ] = xx [ 0 ] ; state [ 9 ] = xx [ 0 ] ;
state [ 10 ] = xx [ 0 ] ; state [ 11 ] = xx [ 0 ] ; state [ 12 ] = xx [ 0 ] ;
state [ 13 ] = xx [ 0 ] ; state [ 14 ] = xx [ 0 ] ; state [ 15 ] = xx [ 0 ] ;
state [ 16 ] = xx [ 0 ] ; state [ 17 ] = xx [ 0 ] ; state [ 18 ] = xx [ 0 ] ;
state [ 19 ] = xx [ 0 ] ; state [ 20 ] = xx [ 0 ] ; state [ 21 ] = xx [ 0 ] ;
state [ 22 ] = xx [ 0 ] ; state [ 23 ] = xx [ 0 ] ; state [ 24 ] = xx [ 0 ] ;
state [ 25 ] = xx [ 0 ] ; state [ 26 ] = xx [ 0 ] ; state [ 27 ] = xx [ 0 ] ;
state [ 28 ] = xx [ 0 ] ; state [ 29 ] = xx [ 0 ] ; state [ 30 ] = xx [ 0 ] ;
state [ 31 ] = xx [ 0 ] ; state [ 32 ] = xx [ 0 ] ; state [ 33 ] = xx [ 0 ] ;
state [ 34 ] = xx [ 0 ] ; state [ 35 ] = xx [ 0 ] ; state [ 36 ] = xx [ 0 ] ;
} void simple_robot_9eb3ef65_1_initializeTrackedAngleState ( const void *
mech , const RuntimeDerivedValuesBundle * rtdv , const int * modeVector ,
const double * motionData , double * state , void * neDiagMgr0 ) {
NeuDiagnosticManager * neDiagMgr = ( NeuDiagnosticManager * ) neDiagMgr0 ; (
void ) mech ; ( void ) rtdv ; ( void ) modeVector ; ( void ) motionData ; (
void ) state ; ( void ) neDiagMgr ; } void
simple_robot_9eb3ef65_1_computeDiscreteState ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , double * state ) { const double * rtdvd =
rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; (
void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; } void
simple_robot_9eb3ef65_1_adjustPosition ( const void * mech , const double *
dofDeltas , double * state ) { double xx [ 11 ] ; ( void ) mech ; xx [ 0 ] =
state [ 3 ] ; xx [ 1 ] = state [ 4 ] ; xx [ 2 ] = state [ 5 ] ; xx [ 3 ] =
state [ 6 ] ; xx [ 4 ] = dofDeltas [ 3 ] ; xx [ 5 ] = dofDeltas [ 4 ] ; xx [
6 ] = dofDeltas [ 5 ] ; pm_math_Quaternion_compDeriv_ra ( xx + 0 , xx + 4 ,
xx + 7 ) ; xx [ 0 ] = state [ 3 ] + xx [ 7 ] ; xx [ 1 ] = state [ 4 ] + xx [
8 ] ; xx [ 2 ] = state [ 5 ] + xx [ 9 ] ; xx [ 3 ] = state [ 6 ] + xx [ 10 ]
; xx [ 4 ] = sqrt ( xx [ 0 ] * xx [ 0 ] + xx [ 1 ] * xx [ 1 ] + xx [ 2 ] * xx
[ 2 ] + xx [ 3 ] * xx [ 3 ] ) ; xx [ 5 ] = 1.0e-64 ; if ( xx [ 5 ] > xx [ 4 ]
) xx [ 4 ] = xx [ 5 ] ; state [ 0 ] = state [ 0 ] + dofDeltas [ 0 ] ; state [
1 ] = state [ 1 ] + dofDeltas [ 1 ] ; state [ 2 ] = state [ 2 ] + dofDeltas [
2 ] ; state [ 3 ] = xx [ 0 ] / xx [ 4 ] ; state [ 4 ] = xx [ 1 ] / xx [ 4 ] ;
state [ 5 ] = xx [ 2 ] / xx [ 4 ] ; state [ 6 ] = xx [ 3 ] / xx [ 4 ] ; state
[ 13 ] = state [ 13 ] + dofDeltas [ 6 ] ; state [ 15 ] = state [ 15 ] +
dofDeltas [ 7 ] ; state [ 17 ] = state [ 17 ] + dofDeltas [ 8 ] ; state [ 19
] = state [ 19 ] + dofDeltas [ 9 ] ; state [ 21 ] = state [ 21 ] + dofDeltas
[ 10 ] ; state [ 23 ] = state [ 23 ] + dofDeltas [ 11 ] ; state [ 25 ] =
state [ 25 ] + dofDeltas [ 12 ] ; state [ 27 ] = state [ 27 ] + dofDeltas [
13 ] ; state [ 29 ] = state [ 29 ] + dofDeltas [ 14 ] ; state [ 31 ] = state
[ 31 ] + dofDeltas [ 15 ] ; state [ 33 ] = state [ 33 ] + dofDeltas [ 16 ] ;
state [ 35 ] = state [ 35 ] + dofDeltas [ 17 ] ; } static void
perturbAsmJointPrimitiveState_0_0 ( double mag , double * state ) { state [ 0
] = state [ 0 ] + mag ; } static void perturbAsmJointPrimitiveState_0_0v (
double mag , double * state ) { state [ 0 ] = state [ 0 ] + mag ; state [ 7 ]
= state [ 7 ] - 0.875 * mag ; } static void perturbAsmJointPrimitiveState_0_1
( double mag , double * state ) { state [ 1 ] = state [ 1 ] + mag ; } static
void perturbAsmJointPrimitiveState_0_1v ( double mag , double * state ) {
state [ 1 ] = state [ 1 ] + mag ; state [ 8 ] = state [ 8 ] - 0.875 * mag ; }
static void perturbAsmJointPrimitiveState_0_2 ( double mag , double * state )
{ state [ 2 ] = state [ 2 ] + mag ; } static void
perturbAsmJointPrimitiveState_0_2v ( double mag , double * state ) { state [
2 ] = state [ 2 ] + mag ; state [ 9 ] = state [ 9 ] - 0.875 * mag ; } static
void perturbAsmJointPrimitiveState_0_3 ( double mag , double * state ) {
double xx [ 15 ] ; xx [ 0 ] = 1.0 ; xx [ 1 ] = fabs ( mag ) ; xx [ 2 ] = xx [
0 ] / ( xx [ 1 ] - floor ( xx [ 1 ] ) + 1.0e-9 ) ; xx [ 1 ] = sin ( xx [ 2 ]
) ; xx [ 3 ] = 0.0 ; xx [ 4 ] = cos ( xx [ 2 ] ) ; xx [ 5 ] = sin ( 2.0 * xx
[ 2 ] ) ; xx [ 2 ] = 0.5 * mag ; xx [ 6 ] = sqrt ( xx [ 1 ] * xx [ 1 ] + xx [
4 ] * xx [ 4 ] + xx [ 5 ] * xx [ 5 ] ) ; xx [ 7 ] = xx [ 6 ] == 0.0 ? 0.0 :
xx [ 1 ] / xx [ 6 ] ; xx [ 8 ] = sin ( xx [ 2 ] ) ; xx [ 9 ] = xx [ 6 ] ==
0.0 ? 0.0 : xx [ 4 ] / xx [ 6 ] ; xx [ 10 ] = xx [ 6 ] == 0.0 ? 0.0 : xx [ 5
] / xx [ 6 ] ; xx [ 11 ] = xx [ 1 ] == xx [ 3 ] && xx [ 4 ] == xx [ 3 ] && xx
[ 5 ] == xx [ 3 ] ? xx [ 0 ] : cos ( xx [ 2 ] ) ; xx [ 12 ] = xx [ 7 ] * xx [
8 ] ; xx [ 13 ] = xx [ 9 ] * xx [ 8 ] ; xx [ 14 ] = xx [ 10 ] * xx [ 8 ] ; xx
[ 0 ] = state [ 3 ] ; xx [ 1 ] = state [ 4 ] ; xx [ 2 ] = state [ 5 ] ; xx [
3 ] = state [ 6 ] ; pm_math_Quaternion_compose_ra ( xx + 11 , xx + 0 , xx + 4
) ; state [ 3 ] = xx [ 4 ] ; state [ 4 ] = xx [ 5 ] ; state [ 5 ] = xx [ 6 ]
; state [ 6 ] = xx [ 7 ] ; } static void perturbAsmJointPrimitiveState_0_3v (
double mag , double * state ) { double xx [ 15 ] ; xx [ 0 ] = 1.0 ; xx [ 1 ]
= fabs ( mag ) ; xx [ 2 ] = xx [ 0 ] / ( xx [ 1 ] - floor ( xx [ 1 ] ) +
1.0e-9 ) ; xx [ 1 ] = sin ( xx [ 2 ] ) ; xx [ 3 ] = 0.0 ; xx [ 4 ] = cos ( xx
[ 2 ] ) ; xx [ 5 ] = sin ( 2.0 * xx [ 2 ] ) ; xx [ 2 ] = 0.5 * mag ; xx [ 6 ]
= sqrt ( xx [ 1 ] * xx [ 1 ] + xx [ 4 ] * xx [ 4 ] + xx [ 5 ] * xx [ 5 ] ) ;
xx [ 7 ] = xx [ 6 ] == 0.0 ? 0.0 : xx [ 1 ] / xx [ 6 ] ; xx [ 8 ] = sin ( xx
[ 2 ] ) ; xx [ 9 ] = xx [ 6 ] == 0.0 ? 0.0 : xx [ 4 ] / xx [ 6 ] ; xx [ 10 ]
= xx [ 6 ] == 0.0 ? 0.0 : xx [ 5 ] / xx [ 6 ] ; xx [ 11 ] = xx [ 1 ] == xx [
3 ] && xx [ 4 ] == xx [ 3 ] && xx [ 5 ] == xx [ 3 ] ? xx [ 0 ] : cos ( xx [ 2
] ) ; xx [ 12 ] = xx [ 7 ] * xx [ 8 ] ; xx [ 13 ] = xx [ 9 ] * xx [ 8 ] ; xx
[ 14 ] = xx [ 10 ] * xx [ 8 ] ; xx [ 3 ] = state [ 3 ] ; xx [ 4 ] = state [ 4
] ; xx [ 5 ] = state [ 5 ] ; xx [ 6 ] = state [ 6 ] ;
pm_math_Quaternion_compose_ra ( xx + 11 , xx + 3 , xx + 7 ) ; state [ 3 ] =
xx [ 7 ] ; state [ 4 ] = xx [ 8 ] ; state [ 5 ] = xx [ 9 ] ; state [ 6 ] = xx
[ 10 ] ; state [ 10 ] = state [ 10 ] + 1.2 * mag ; state [ 11 ] = state [ 11
] - xx [ 2 ] ; state [ 12 ] = state [ 12 ] + 0.9 * mag ; } static void
perturbAsmJointPrimitiveState_1_0 ( double mag , double * state ) { state [
13 ] = state [ 13 ] + mag ; } static void perturbAsmJointPrimitiveState_1_0v
( double mag , double * state ) { state [ 13 ] = state [ 13 ] + mag ; state [
14 ] = state [ 14 ] - 0.875 * mag ; } static void
perturbAsmJointPrimitiveState_2_0 ( double mag , double * state ) { state [
15 ] = state [ 15 ] + mag ; } static void perturbAsmJointPrimitiveState_2_0v
( double mag , double * state ) { state [ 15 ] = state [ 15 ] + mag ; state [
16 ] = state [ 16 ] - 0.875 * mag ; } static void
perturbAsmJointPrimitiveState_3_0 ( double mag , double * state ) { state [
17 ] = state [ 17 ] + mag ; } static void perturbAsmJointPrimitiveState_3_0v
( double mag , double * state ) { state [ 17 ] = state [ 17 ] + mag ; state [
18 ] = state [ 18 ] - 0.875 * mag ; } static void
perturbAsmJointPrimitiveState_4_0 ( double mag , double * state ) { state [
19 ] = state [ 19 ] + mag ; } static void perturbAsmJointPrimitiveState_4_0v
( double mag , double * state ) { state [ 19 ] = state [ 19 ] + mag ; state [
20 ] = state [ 20 ] - 0.875 * mag ; } static void
perturbAsmJointPrimitiveState_5_0 ( double mag , double * state ) { state [
21 ] = state [ 21 ] + mag ; } static void perturbAsmJointPrimitiveState_5_0v
( double mag , double * state ) { state [ 21 ] = state [ 21 ] + mag ; state [
22 ] = state [ 22 ] - 0.875 * mag ; } static void
perturbAsmJointPrimitiveState_6_0 ( double mag , double * state ) { state [
23 ] = state [ 23 ] + mag ; } static void perturbAsmJointPrimitiveState_6_0v
( double mag , double * state ) { state [ 23 ] = state [ 23 ] + mag ; state [
24 ] = state [ 24 ] - 0.875 * mag ; } static void
perturbAsmJointPrimitiveState_7_0 ( double mag , double * state ) { state [
25 ] = state [ 25 ] + mag ; } static void perturbAsmJointPrimitiveState_7_0v
( double mag , double * state ) { state [ 25 ] = state [ 25 ] + mag ; state [
26 ] = state [ 26 ] - 0.875 * mag ; } static void
perturbAsmJointPrimitiveState_8_0 ( double mag , double * state ) { state [
27 ] = state [ 27 ] + mag ; } static void perturbAsmJointPrimitiveState_8_0v
( double mag , double * state ) { state [ 27 ] = state [ 27 ] + mag ; state [
28 ] = state [ 28 ] - 0.875 * mag ; } static void
perturbAsmJointPrimitiveState_9_0 ( double mag , double * state ) { state [
29 ] = state [ 29 ] + mag ; } static void perturbAsmJointPrimitiveState_9_0v
( double mag , double * state ) { state [ 29 ] = state [ 29 ] + mag ; state [
30 ] = state [ 30 ] - 0.875 * mag ; } static void
perturbAsmJointPrimitiveState_10_0 ( double mag , double * state ) { state [
31 ] = state [ 31 ] + mag ; } static void perturbAsmJointPrimitiveState_10_0v
( double mag , double * state ) { state [ 31 ] = state [ 31 ] + mag ; state [
32 ] = state [ 32 ] - 0.875 * mag ; } static void
perturbAsmJointPrimitiveState_11_0 ( double mag , double * state ) { state [
33 ] = state [ 33 ] + mag ; } static void perturbAsmJointPrimitiveState_11_0v
( double mag , double * state ) { state [ 33 ] = state [ 33 ] + mag ; state [
34 ] = state [ 34 ] - 0.875 * mag ; } static void
perturbAsmJointPrimitiveState_12_0 ( double mag , double * state ) { state [
35 ] = state [ 35 ] + mag ; } static void perturbAsmJointPrimitiveState_12_0v
( double mag , double * state ) { state [ 35 ] = state [ 35 ] + mag ; state [
36 ] = state [ 36 ] - 0.875 * mag ; } void
simple_robot_9eb3ef65_1_perturbAsmJointPrimitiveState ( const void * mech ,
size_t stageIdx , size_t primIdx , double mag , boolean_T doPerturbVelocity ,
double * state ) { ( void ) mech ; ( void ) stageIdx ; ( void ) primIdx ; (
void ) mag ; ( void ) doPerturbVelocity ; ( void ) state ; switch ( (
stageIdx * 6 + primIdx ) * 2 + ( doPerturbVelocity ? 1 : 0 ) ) { case 0 :
perturbAsmJointPrimitiveState_0_0 ( mag , state ) ; break ; case 1 :
perturbAsmJointPrimitiveState_0_0v ( mag , state ) ; break ; case 2 :
perturbAsmJointPrimitiveState_0_1 ( mag , state ) ; break ; case 3 :
perturbAsmJointPrimitiveState_0_1v ( mag , state ) ; break ; case 4 :
perturbAsmJointPrimitiveState_0_2 ( mag , state ) ; break ; case 5 :
perturbAsmJointPrimitiveState_0_2v ( mag , state ) ; break ; case 6 :
perturbAsmJointPrimitiveState_0_3 ( mag , state ) ; break ; case 7 :
perturbAsmJointPrimitiveState_0_3v ( mag , state ) ; break ; case 12 :
perturbAsmJointPrimitiveState_1_0 ( mag , state ) ; break ; case 13 :
perturbAsmJointPrimitiveState_1_0v ( mag , state ) ; break ; case 24 :
perturbAsmJointPrimitiveState_2_0 ( mag , state ) ; break ; case 25 :
perturbAsmJointPrimitiveState_2_0v ( mag , state ) ; break ; case 36 :
perturbAsmJointPrimitiveState_3_0 ( mag , state ) ; break ; case 37 :
perturbAsmJointPrimitiveState_3_0v ( mag , state ) ; break ; case 48 :
perturbAsmJointPrimitiveState_4_0 ( mag , state ) ; break ; case 49 :
perturbAsmJointPrimitiveState_4_0v ( mag , state ) ; break ; case 60 :
perturbAsmJointPrimitiveState_5_0 ( mag , state ) ; break ; case 61 :
perturbAsmJointPrimitiveState_5_0v ( mag , state ) ; break ; case 72 :
perturbAsmJointPrimitiveState_6_0 ( mag , state ) ; break ; case 73 :
perturbAsmJointPrimitiveState_6_0v ( mag , state ) ; break ; case 84 :
perturbAsmJointPrimitiveState_7_0 ( mag , state ) ; break ; case 85 :
perturbAsmJointPrimitiveState_7_0v ( mag , state ) ; break ; case 96 :
perturbAsmJointPrimitiveState_8_0 ( mag , state ) ; break ; case 97 :
perturbAsmJointPrimitiveState_8_0v ( mag , state ) ; break ; case 108 :
perturbAsmJointPrimitiveState_9_0 ( mag , state ) ; break ; case 109 :
perturbAsmJointPrimitiveState_9_0v ( mag , state ) ; break ; case 120 :
perturbAsmJointPrimitiveState_10_0 ( mag , state ) ; break ; case 121 :
perturbAsmJointPrimitiveState_10_0v ( mag , state ) ; break ; case 132 :
perturbAsmJointPrimitiveState_11_0 ( mag , state ) ; break ; case 133 :
perturbAsmJointPrimitiveState_11_0v ( mag , state ) ; break ; case 144 :
perturbAsmJointPrimitiveState_12_0 ( mag , state ) ; break ; case 145 :
perturbAsmJointPrimitiveState_12_0v ( mag , state ) ; break ; } } static void
computePosDofBlendMatrix_0_3 ( const double * state , int partialType ,
double * matrix ) { double xx [ 20 ] ; xx [ 0 ] = 0.0 ; xx [ 1 ] = 2.0 ; xx [
2 ] = xx [ 1 ] * ( state [ 4 ] * state [ 5 ] - state [ 3 ] * state [ 6 ] ) ;
xx [ 3 ] = xx [ 2 ] * xx [ 2 ] ; xx [ 4 ] = 1.0 ; xx [ 5 ] = ( state [ 3 ] *
state [ 3 ] + state [ 4 ] * state [ 4 ] ) * xx [ 1 ] - xx [ 4 ] ; xx [ 6 ] =
xx [ 5 ] * xx [ 5 ] ; xx [ 7 ] = sqrt ( xx [ 3 ] + xx [ 6 ] ) ; xx [ 8 ] = xx
[ 7 ] == 0.0 ? 0.0 : - xx [ 2 ] / xx [ 7 ] ; xx [ 9 ] = xx [ 6 ] + xx [ 3 ] ;
xx [ 3 ] = sqrt ( xx [ 9 ] ) ; xx [ 6 ] = xx [ 3 ] == 0.0 ? 0.0 : xx [ 5 ] /
xx [ 3 ] ; xx [ 10 ] = 0.0 ; xx [ 11 ] = ( state [ 4 ] * state [ 6 ] + state
[ 3 ] * state [ 5 ] ) * xx [ 1 ] ; xx [ 1 ] = sqrt ( xx [ 9 ] + xx [ 11 ] *
xx [ 11 ] ) ; xx [ 12 ] = xx [ 1 ] == 0.0 ? 0.0 : xx [ 5 ] / xx [ 1 ] ; xx [
14 ] = xx [ 8 ] ; xx [ 15 ] = xx [ 6 ] ; xx [ 16 ] = xx [ 10 ] ; xx [ 17 ] =
xx [ 8 ] ; xx [ 18 ] = xx [ 8 ] ; xx [ 19 ] = xx [ 12 ] ; xx [ 6 ] = xx [ 13
+ ( partialType ) ] ; xx [ 8 ] = xx [ 7 ] == 0.0 ? 0.0 : xx [ 5 ] / xx [ 7 ]
; xx [ 7 ] = xx [ 3 ] == 0.0 ? 0.0 : xx [ 2 ] / xx [ 3 ] ; xx [ 3 ] = xx [ 1
] == 0.0 ? 0.0 : xx [ 2 ] / xx [ 1 ] ; xx [ 13 ] = xx [ 8 ] ; xx [ 14 ] = xx
[ 7 ] ; xx [ 15 ] = xx [ 10 ] ; xx [ 16 ] = xx [ 8 ] ; xx [ 17 ] = xx [ 8 ] ;
xx [ 18 ] = xx [ 3 ] ; xx [ 2 ] = xx [ 12 + ( partialType ) ] ; xx [ 3 ] = xx
[ 1 ] == 0.0 ? 0.0 : xx [ 11 ] / xx [ 1 ] ; xx [ 13 ] = xx [ 10 ] ; xx [ 14 ]
= xx [ 10 ] ; xx [ 15 ] = xx [ 4 ] ; xx [ 16 ] = xx [ 10 ] ; xx [ 17 ] = xx [
10 ] ; xx [ 18 ] = xx [ 3 ] ; xx [ 1 ] = xx [ 12 + ( partialType ) ] ; xx [ 3
] = xx [ 11 ] * xx [ 5 ] ; xx [ 5 ] = sqrt ( xx [ 9 ] * xx [ 9 ] + xx [ 3 ] *
xx [ 3 ] ) ; xx [ 7 ] = xx [ 5 ] == 0.0 ? 0.0 : xx [ 9 ] / xx [ 5 ] ; xx [ 12
] = xx [ 10 ] ; xx [ 13 ] = xx [ 10 ] ; xx [ 14 ] = xx [ 10 ] ; xx [ 15 ] =
xx [ 7 ] ; xx [ 16 ] = xx [ 10 ] ; xx [ 17 ] = xx [ 10 ] ; xx [ 7 ] = xx [ 11
+ ( partialType ) ] ; xx [ 12 ] = xx [ 10 ] ; xx [ 13 ] = xx [ 10 ] ; xx [ 14
] = xx [ 10 ] ; xx [ 15 ] = xx [ 10 ] ; xx [ 16 ] = xx [ 10 ] ; xx [ 17 ] =
xx [ 10 ] ; xx [ 8 ] = xx [ 11 + ( partialType ) ] ; xx [ 9 ] = xx [ 5 ] ==
0.0 ? 0.0 : xx [ 3 ] / xx [ 5 ] ; xx [ 12 ] = xx [ 4 ] ; xx [ 13 ] = xx [ 4 ]
; xx [ 14 ] = xx [ 10 ] ; xx [ 15 ] = xx [ 9 ] ; xx [ 16 ] = xx [ 10 ] ; xx [
17 ] = xx [ 10 ] ; xx [ 0 ] = xx [ 11 + ( partialType ) ] ; matrix [ 0 ] = xx
[ 6 ] ; matrix [ 1 ] = xx [ 2 ] ; matrix [ 2 ] = xx [ 1 ] ; matrix [ 3 ] = xx
[ 7 ] ; matrix [ 4 ] = xx [ 8 ] ; matrix [ 5 ] = xx [ 0 ] ; matrix [ 6 ] = xx
[ 8 ] ; matrix [ 7 ] = xx [ 8 ] ; matrix [ 8 ] = xx [ 8 ] ; } void
simple_robot_9eb3ef65_1_computePosDofBlendMatrix ( const void * mech , size_t
stageIdx , size_t primIdx , const double * state , int partialType , double *
matrix ) { ( void ) mech ; ( void ) stageIdx ; ( void ) primIdx ; ( void )
state ; ( void ) partialType ; ( void ) matrix ; switch ( ( stageIdx * 6 +
primIdx ) ) { case 3 : computePosDofBlendMatrix_0_3 ( state , partialType ,
matrix ) ; break ; } } static void computeVelDofBlendMatrix_0_3 ( const
double * state , int partialType , double * matrix ) { double xx [ 15 ] ; (
void ) state ; xx [ 0 ] = 0.0 ; xx [ 1 ] = 0.0 ; xx [ 2 ] = 1.0 ; xx [ 4 ] =
xx [ 1 ] ; xx [ 5 ] = xx [ 2 ] ; xx [ 6 ] = xx [ 1 ] ; xx [ 7 ] = xx [ 2 ] ;
xx [ 8 ] = xx [ 1 ] ; xx [ 9 ] = xx [ 2 ] ; xx [ 10 ] = xx [ 3 + (
partialType ) ] ; xx [ 4 ] = xx [ 2 ] ; xx [ 5 ] = xx [ 1 ] ; xx [ 6 ] = xx [
1 ] ; xx [ 7 ] = xx [ 1 ] ; xx [ 8 ] = xx [ 2 ] ; xx [ 9 ] = xx [ 1 ] ; xx [
11 ] = xx [ 3 + ( partialType ) ] ; xx [ 4 ] = xx [ 1 ] ; xx [ 5 ] = xx [ 1 ]
; xx [ 6 ] = xx [ 2 ] ; xx [ 7 ] = xx [ 1 ] ; xx [ 8 ] = xx [ 1 ] ; xx [ 9 ]
= xx [ 1 ] ; xx [ 12 ] = xx [ 3 + ( partialType ) ] ; xx [ 4 ] = xx [ 1 ] ;
xx [ 5 ] = xx [ 1 ] ; xx [ 6 ] = xx [ 1 ] ; xx [ 7 ] = xx [ 1 ] ; xx [ 8 ] =
xx [ 1 ] ; xx [ 9 ] = xx [ 1 ] ; xx [ 13 ] = xx [ 3 + ( partialType ) ] ; xx
[ 4 ] = xx [ 1 ] ; xx [ 5 ] = xx [ 1 ] ; xx [ 6 ] = xx [ 1 ] ; xx [ 7 ] = xx
[ 2 ] ; xx [ 8 ] = xx [ 1 ] ; xx [ 9 ] = xx [ 1 ] ; xx [ 14 ] = xx [ 3 + (
partialType ) ] ; xx [ 4 ] = xx [ 2 ] ; xx [ 5 ] = xx [ 2 ] ; xx [ 6 ] = xx [
1 ] ; xx [ 7 ] = xx [ 1 ] ; xx [ 8 ] = xx [ 1 ] ; xx [ 9 ] = xx [ 1 ] ; xx [
0 ] = xx [ 3 + ( partialType ) ] ; matrix [ 0 ] = xx [ 10 ] ; matrix [ 1 ] =
xx [ 11 ] ; matrix [ 2 ] = xx [ 12 ] ; matrix [ 3 ] = xx [ 13 ] ; matrix [ 4
] = xx [ 14 ] ; matrix [ 5 ] = xx [ 0 ] ; matrix [ 6 ] = xx [ 13 ] ; matrix [
7 ] = xx [ 13 ] ; matrix [ 8 ] = xx [ 13 ] ; } void
simple_robot_9eb3ef65_1_computeVelDofBlendMatrix ( const void * mech , size_t
stageIdx , size_t primIdx , const double * state , int partialType , double *
matrix ) { ( void ) mech ; ( void ) stageIdx ; ( void ) primIdx ; ( void )
state ; ( void ) partialType ; ( void ) matrix ; switch ( ( stageIdx * 6 +
primIdx ) ) { case 3 : computeVelDofBlendMatrix_0_3 ( state , partialType ,
matrix ) ; break ; } } static void projectPartiallyTargetedPos_0_3 ( const
double * origState , int partialType , double * state ) { boolean_T bb [ 1 ]
; int ii [ 3 ] ; double xx [ 24 ] ; xx [ 0 ] = state [ 4 ] * state [ 6 ] ; xx
[ 1 ] = state [ 3 ] * state [ 5 ] ; xx [ 2 ] = 2.0 ; xx [ 3 ] = ( xx [ 0 ] +
xx [ 1 ] ) * xx [ 2 ] ; xx [ 4 ] = fabs ( xx [ 3 ] ) > 1.0 ? atan2 ( xx [ 3 ]
, 0.0 ) : asin ( xx [ 3 ] ) ; xx [ 5 ] = origState [ 4 ] * origState [ 6 ] ;
xx [ 6 ] = origState [ 3 ] * origState [ 5 ] ; xx [ 7 ] = ( xx [ 5 ] + xx [ 6
] ) * xx [ 2 ] ; xx [ 8 ] = fabs ( xx [ 7 ] ) > 1.0 ? atan2 ( xx [ 7 ] , 0.0
) : asin ( xx [ 7 ] ) ; xx [ 9 ] = xx [ 4 ] ; xx [ 10 ] = xx [ 4 ] ; xx [ 11
] = xx [ 8 ] ; xx [ 12 ] = xx [ 8 ] ; xx [ 13 ] = xx [ 4 ] ; xx [ 14 ] = xx [
4 ] ; xx [ 15 ] = xx [ 8 ] ; xx [ 16 ] = xx [ 9 + ( partialType ) ] ; xx [ 9
] = cos ( xx [ 16 ] ) ; xx [ 10 ] = 0.99999999999999 ; xx [ 11 ] = fabs ( xx
[ 3 ] ) - xx [ 10 ] ; if ( xx [ 11 ] < 0.0 ) ii [ 0 ] = - 1 ; else if ( xx [
11 ] > 0.0 ) ii [ 0 ] = + 1 ; else ii [ 0 ] = 0 ; ii [ 1 ] = ii [ 0 ] ; if (
0 > ii [ 1 ] ) ii [ 1 ] = 0 ; bb [ 0 ] = ! ( ii [ 1 ] == 1 ) ; xx [ 3 ] =
state [ 4 ] * state [ 5 ] ; xx [ 11 ] = state [ 3 ] * state [ 6 ] ; xx [ 12 ]
= state [ 3 ] * state [ 3 ] ; xx [ 13 ] = 1.0 ; xx [ 14 ] = ( xx [ 12 ] +
state [ 4 ] * state [ 4 ] ) * xx [ 2 ] - xx [ 13 ] ; xx [ 15 ] = - ( xx [ 2 ]
* ( xx [ 3 ] - xx [ 11 ] ) ) ; xx [ 14 ] = ( xx [ 15 ] == 0.0 && xx [ 14 ] ==
0.0 ) ? 0.0 : atan2 ( xx [ 15 ] , xx [ 14 ] ) ; if ( xx [ 4 ] < 0.0 ) xx [ 15
] = - 1.0 ; else if ( xx [ 4 ] > 0.0 ) xx [ 15 ] = + 1.0 ; else xx [ 15 ] =
0.0 ; xx [ 4 ] = ( xx [ 12 ] + state [ 6 ] * state [ 6 ] ) * xx [ 2 ] - xx [
13 ] ; xx [ 17 ] = - ( xx [ 2 ] * ( state [ 5 ] * state [ 6 ] - state [ 3 ] *
state [ 4 ] ) ) ; xx [ 4 ] = ( xx [ 17 ] == 0.0 && xx [ 4 ] == 0.0 ) ? 0.0 :
atan2 ( xx [ 17 ] , xx [ 4 ] ) ; xx [ 12 ] = 0.5 ; xx [ 17 ] = - ( xx [ 2 ] *
( xx [ 0 ] - xx [ 1 ] ) * xx [ 15 ] ) ; xx [ 18 ] = ( xx [ 3 ] + xx [ 11 ] )
* xx [ 2 ] * xx [ 15 ] ; xx [ 17 ] = ( xx [ 18 ] == 0.0 && xx [ 17 ] == 0.0 )
? 0.0 : atan2 ( xx [ 18 ] , xx [ 17 ] ) ; xx [ 0 ] = bb [ 0 ] ? xx [ 4 ] : xx
[ 12 ] * xx [ 17 ] ; xx [ 1 ] = bb [ 0 ] ? xx [ 14 ] : xx [ 15 ] * xx [ 0 ] ;
xx [ 3 ] = fabs ( xx [ 7 ] ) - xx [ 10 ] ; if ( xx [ 3 ] < 0.0 ) ii [ 0 ] = -
1 ; else if ( xx [ 3 ] > 0.0 ) ii [ 0 ] = + 1 ; else ii [ 0 ] = 0 ; ii [ 1 ]
= ii [ 0 ] ; if ( 0 > ii [ 1 ] ) ii [ 1 ] = 0 ; bb [ 0 ] = ! ( ii [ 1 ] == 1
) ; xx [ 3 ] = origState [ 4 ] * origState [ 5 ] ; xx [ 4 ] = origState [ 3 ]
* origState [ 6 ] ; xx [ 7 ] = origState [ 3 ] * origState [ 3 ] ; xx [ 10 ]
= ( xx [ 7 ] + origState [ 4 ] * origState [ 4 ] ) * xx [ 2 ] - xx [ 13 ] ;
xx [ 11 ] = - ( xx [ 2 ] * ( xx [ 3 ] - xx [ 4 ] ) ) ; xx [ 10 ] = ( xx [ 11
] == 0.0 && xx [ 10 ] == 0.0 ) ? 0.0 : atan2 ( xx [ 11 ] , xx [ 10 ] ) ; if (
xx [ 8 ] < 0.0 ) xx [ 11 ] = - 1.0 ; else if ( xx [ 8 ] > 0.0 ) xx [ 11 ] = +
1.0 ; else xx [ 11 ] = 0.0 ; xx [ 8 ] = ( xx [ 7 ] + origState [ 6 ] *
origState [ 6 ] ) * xx [ 2 ] - xx [ 13 ] ; xx [ 14 ] = - ( xx [ 2 ] * (
origState [ 5 ] * origState [ 6 ] - origState [ 3 ] * origState [ 4 ] ) ) ;
xx [ 8 ] = ( xx [ 14 ] == 0.0 && xx [ 8 ] == 0.0 ) ? 0.0 : atan2 ( xx [ 14 ]
, xx [ 8 ] ) ; xx [ 7 ] = - ( xx [ 2 ] * ( xx [ 5 ] - xx [ 6 ] ) * xx [ 11 ]
) ; xx [ 13 ] = ( xx [ 3 ] + xx [ 4 ] ) * xx [ 2 ] * xx [ 11 ] ; xx [ 7 ] = (
xx [ 13 ] == 0.0 && xx [ 7 ] == 0.0 ) ? 0.0 : atan2 ( xx [ 13 ] , xx [ 7 ] )
; xx [ 2 ] = bb [ 0 ] ? xx [ 8 ] : xx [ 12 ] * xx [ 7 ] ; xx [ 3 ] = bb [ 0 ]
? xx [ 10 ] : xx [ 11 ] * xx [ 2 ] ; xx [ 17 ] = xx [ 1 ] ; xx [ 18 ] = xx [
1 ] ; xx [ 19 ] = xx [ 1 ] ; xx [ 20 ] = xx [ 1 ] ; xx [ 21 ] = xx [ 3 ] ; xx
[ 22 ] = xx [ 3 ] ; xx [ 23 ] = xx [ 3 ] ; xx [ 1 ] = xx [ 17 + ( partialType
) ] ; xx [ 3 ] = cos ( xx [ 1 ] ) ; xx [ 4 ] = sin ( xx [ 1 ] ) ; xx [ 1 ] =
sin ( xx [ 16 ] ) ; xx [ 10 ] = xx [ 0 ] ; xx [ 11 ] = xx [ 2 ] ; xx [ 12 ] =
xx [ 0 ] ; xx [ 13 ] = xx [ 2 ] ; xx [ 14 ] = xx [ 0 ] ; xx [ 15 ] = xx [ 2 ]
; xx [ 16 ] = xx [ 0 ] ; xx [ 0 ] = xx [ 10 + ( partialType ) ] ; xx [ 2 ] =
cos ( xx [ 0 ] ) ; xx [ 5 ] = sin ( xx [ 0 ] ) ; xx [ 0 ] = xx [ 3 ] * xx [ 5
] ; xx [ 6 ] = xx [ 3 ] * xx [ 2 ] ; xx [ 10 ] = xx [ 9 ] * xx [ 3 ] ; xx [
11 ] = - ( xx [ 9 ] * xx [ 4 ] ) ; xx [ 12 ] = xx [ 1 ] ; xx [ 13 ] = xx [ 2
] * xx [ 4 ] + xx [ 0 ] * xx [ 1 ] ; xx [ 14 ] = xx [ 6 ] - xx [ 1 ] * xx [ 5
] * xx [ 4 ] ; xx [ 15 ] = - ( xx [ 9 ] * xx [ 5 ] ) ; xx [ 16 ] = xx [ 4 ] *
xx [ 5 ] - xx [ 6 ] * xx [ 1 ] ; xx [ 17 ] = xx [ 0 ] + xx [ 2 ] * xx [ 1 ] *
xx [ 4 ] ; xx [ 18 ] = xx [ 9 ] * xx [ 2 ] ;
pm_math_Quaternion_Matrix3x3Ctor_ra ( xx + 10 , xx + 0 ) ; state [ 3 ] = xx [
0 ] ; state [ 4 ] = xx [ 1 ] ; state [ 5 ] = xx [ 2 ] ; state [ 6 ] = xx [ 3
] ; } void simple_robot_9eb3ef65_1_projectPartiallyTargetedPos ( const void *
mech , size_t stageIdx , size_t primIdx , const double * origState , int
partialType , double * state ) { ( void ) mech ; ( void ) stageIdx ; ( void )
primIdx ; ( void ) origState ; ( void ) partialType ; ( void ) state ; switch
( ( stageIdx * 6 + primIdx ) ) { case 3 : projectPartiallyTargetedPos_0_3 (
origState , partialType , state ) ; break ; } } void
simple_robot_9eb3ef65_1_propagateMotion ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , const double * state , double *
motionData ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; double xx [ 315 ] ; ( void ) mech ; (
void ) rtdvd ; ( void ) rtdvi ; xx [ 0 ] = state [ 3 ] ; xx [ 1 ] = state [ 4
] ; xx [ 2 ] = state [ 5 ] ; xx [ 3 ] = state [ 6 ] ; xx [ 4 ] =
0.6422553040725784 ; xx [ 5 ] = 0.295817721562857 ; xx [ 6 ] = xx [ 4 ] ; xx
[ 7 ] = xx [ 5 ] ; xx [ 8 ] = xx [ 4 ] ; xx [ 9 ] = xx [ 5 ] ;
pm_math_Quaternion_composeInverse_ra ( xx + 0 , xx + 6 , xx + 10 ) ; xx [ 0 ]
= 4.952999999999999e-3 ; xx [ 1 ] = xx [ 0 ] * xx [ 12 ] ; xx [ 2 ] = xx [ 0
] * xx [ 13 ] ; xx [ 3 ] = 2.0 ; xx [ 4 ] = state [ 0 ] - ( ( xx [ 12 ] * xx
[ 1 ] + xx [ 13 ] * xx [ 2 ] ) * xx [ 3 ] - xx [ 0 ] ) ; xx [ 5 ] = state [ 1
] + ( xx [ 10 ] * xx [ 2 ] + xx [ 11 ] * xx [ 1 ] ) * xx [ 3 ] ; xx [ 14 ] =
state [ 2 ] - xx [ 3 ] * ( xx [ 10 ] * xx [ 1 ] - xx [ 11 ] * xx [ 2 ] ) ; xx
[ 1 ] = 5.02846706808957e-3 ; xx [ 2 ] = 0.5 ; xx [ 15 ] = xx [ 2 ] * state [
13 ] ; xx [ 16 ] = cos ( xx [ 15 ] ) ; xx [ 17 ] = 0.9379742042928239 ; xx [
18 ] = 0.02902651212139928 ; xx [ 19 ] = sin ( xx [ 15 ] ) ; xx [ 15 ] = xx [
18 ] * xx [ 19 ] ; xx [ 20 ] = 0.01361592839523582 ; xx [ 21 ] =
0.999578642025762 ; xx [ 22 ] = xx [ 21 ] * xx [ 19 ] ; xx [ 19 ] = xx [ 1 ]
* xx [ 16 ] + xx [ 17 ] * xx [ 15 ] - xx [ 20 ] * xx [ 22 ] ; xx [ 23 ] = -
xx [ 19 ] ; xx [ 24 ] = 0.3464010870279208 ; xx [ 25 ] = xx [ 24 ] * xx [ 15
] - ( xx [ 1 ] * xx [ 22 ] + xx [ 20 ] * xx [ 16 ] ) ; xx [ 26 ] = xx [ 1 ] *
xx [ 15 ] - xx [ 17 ] * xx [ 16 ] + xx [ 24 ] * xx [ 22 ] ; xx [ 27 ] = xx [
24 ] * xx [ 16 ] + xx [ 20 ] * xx [ 15 ] + xx [ 17 ] * xx [ 22 ] ; xx [ 15 ]
= 0.02135192530809139 ; xx [ 16 ] = 0.05562318494169338 ; xx [ 20 ] = xx [ 16
] * xx [ 27 ] ; xx [ 22 ] = xx [ 15 ] * xx [ 27 ] ; xx [ 24 ] = xx [ 16 ] *
xx [ 25 ] + xx [ 26 ] * xx [ 15 ] ; xx [ 28 ] = xx [ 20 ] ; xx [ 29 ] = xx [
22 ] ; xx [ 30 ] = - xx [ 24 ] ; pm_math_Vector3_cross_ra ( xx + 25 , xx + 28
, xx + 31 ) ; xx [ 28 ] = xx [ 0 ] + xx [ 15 ] + xx [ 3 ] * ( xx [ 31 ] - xx
[ 20 ] * xx [ 19 ] ) ; xx [ 20 ] = - xx [ 28 ] ; xx [ 29 ] = ( xx [ 32 ] - xx
[ 22 ] * xx [ 19 ] ) * xx [ 3 ] - xx [ 16 ] ; xx [ 22 ] = - xx [ 29 ] ; xx [
30 ] = ( xx [ 19 ] * xx [ 24 ] + xx [ 33 ] ) * xx [ 3 ] ; xx [ 19 ] = - xx [
30 ] ; xx [ 24 ] = 0.6271461567375358 ; xx [ 31 ] = 0.3835306436794506 ; xx [
32 ] = 0.3186539101717888 ; xx [ 33 ] = 0.5983741546728057 ; xx [ 34 ] = - xx
[ 24 ] ; xx [ 35 ] = - xx [ 31 ] ; xx [ 36 ] = - xx [ 32 ] ; xx [ 37 ] = xx [
33 ] ; xx [ 38 ] = xx [ 2 ] * state [ 15 ] ; xx [ 39 ] = 0.09547264031652658
; xx [ 40 ] = sin ( xx [ 38 ] ) ; xx [ 41 ] = 0.9948427668622 ; xx [ 42 ] =
0.03424681259555507 ; xx [ 43 ] = cos ( xx [ 38 ] ) ; xx [ 44 ] = xx [ 39 ] *
xx [ 40 ] ; xx [ 45 ] = xx [ 41 ] * xx [ 40 ] ; xx [ 46 ] = - ( xx [ 42 ] *
xx [ 40 ] ) ; pm_math_Quaternion_compose_ra ( xx + 34 , xx + 43 , xx + 47 ) ;
xx [ 34 ] = 0.03736275114177646 ; xx [ 35 ] = 0.1168511605538739 ; xx [ 36 ]
= 0.08181278075467377 ; pm_math_Quaternion_xform_ra ( xx + 47 , xx + 34 , xx
+ 43 ) ; xx [ 34 ] = 4.244406610027192e-3 - xx [ 43 ] ; xx [ 35 ] =
0.06404981182865317 - xx [ 44 ] ; xx [ 36 ] = - xx [ 45 ] ; xx [ 37 ] =
0.6153577354298877 ; xx [ 38 ] = xx [ 2 ] * state [ 17 ] ; xx [ 40 ] = cos (
xx [ 38 ] ) ; xx [ 43 ] = 0.3459611930551978 ; xx [ 44 ] = 0.9953376248861994
; xx [ 46 ] = sin ( xx [ 38 ] ) ; xx [ 38 ] = xx [ 44 ] * xx [ 46 ] ; xx [ 51
] = 0.3585189984711593 ; xx [ 52 ] = 0.09645212535708714 ; xx [ 53 ] = xx [
52 ] * xx [ 46 ] ; xx [ 46 ] = xx [ 37 ] * xx [ 40 ] + xx [ 43 ] * xx [ 38 ]
- xx [ 51 ] * xx [ 53 ] ; xx [ 54 ] = - xx [ 46 ] ; xx [ 55 ] =
0.6108271752972861 ; xx [ 56 ] = xx [ 55 ] * xx [ 40 ] - xx [ 43 ] * xx [ 53
] - xx [ 51 ] * xx [ 38 ] ; xx [ 57 ] = xx [ 43 ] * xx [ 40 ] - xx [ 37 ] *
xx [ 38 ] + xx [ 55 ] * xx [ 53 ] ; xx [ 43 ] = xx [ 37 ] * xx [ 53 ] + xx [
51 ] * xx [ 40 ] + xx [ 55 ] * xx [ 38 ] ; xx [ 58 ] = xx [ 56 ] ; xx [ 59 ]
= xx [ 57 ] ; xx [ 60 ] = xx [ 43 ] ; xx [ 37 ] = 0.01043798038823734 ; xx [
38 ] = 0.03286610935552547 ; xx [ 40 ] = xx [ 43 ] * xx [ 37 ] - xx [ 38 ] *
xx [ 57 ] ; xx [ 51 ] = xx [ 38 ] * xx [ 56 ] ; xx [ 53 ] = xx [ 37 ] * xx [
56 ] ; xx [ 61 ] = xx [ 40 ] ; xx [ 62 ] = xx [ 51 ] ; xx [ 63 ] = - xx [ 53
] ; pm_math_Vector3_cross_ra ( xx + 58 , xx + 61 , xx + 64 ) ; xx [ 58 ] =
0.01858416102255839 - ( xx [ 64 ] - xx [ 40 ] * xx [ 46 ] ) * xx [ 3 ] ; xx [
40 ] = 0.02394326065483052 - ( xx [ 3 ] * ( xx [ 65 ] - xx [ 51 ] * xx [ 46 ]
) - xx [ 37 ] ) ; xx [ 37 ] = 0.04694548396048914 - ( ( xx [ 53 ] * xx [ 46 ]
+ xx [ 66 ] ) * xx [ 3 ] - xx [ 38 ] ) ; xx [ 38 ] = 0.3264823273303549 ; xx
[ 46 ] = xx [ 2 ] * state [ 19 ] ; xx [ 51 ] = cos ( xx [ 46 ] ) ; xx [ 53 ]
= 0.6272234768732394 ; xx [ 59 ] = sin ( xx [ 46 ] ) ; xx [ 46 ] = xx [ 38 ]
* xx [ 51 ] + xx [ 53 ] * xx [ 59 ] ; xx [ 60 ] = - xx [ 46 ] ; xx [ 61 ] =
xx [ 38 ] * xx [ 59 ] - xx [ 53 ] * xx [ 51 ] ; xx [ 53 ] = 0.655789040106415
; xx [ 62 ] = 0.2644631068340282 ; xx [ 63 ] = xx [ 53 ] * xx [ 51 ] - xx [
62 ] * xx [ 59 ] ; xx [ 64 ] = xx [ 62 ] * xx [ 51 ] + xx [ 53 ] * xx [ 59 ]
; xx [ 51 ] = 4.953000000000018e-3 ; xx [ 59 ] = xx [ 51 ] * xx [ 63 ] ; xx [
65 ] = xx [ 64 ] * xx [ 51 ] ; xx [ 66 ] = - ( xx [ 51 ] - ( xx [ 59 ] * xx [
63 ] + xx [ 64 ] * xx [ 65 ] ) * xx [ 3 ] ) ; xx [ 67 ] = 0.02511221626888745
; xx [ 68 ] = - ( xx [ 67 ] + ( xx [ 59 ] * xx [ 61 ] - xx [ 46 ] * xx [ 65 ]
) * xx [ 3 ] ) ; xx [ 69 ] = 0.03042104860947363 - xx [ 3 ] * ( xx [ 65 ] *
xx [ 61 ] + xx [ 46 ] * xx [ 59 ] ) ; xx [ 46 ] = 0.615357735429888 ; xx [ 59
] = xx [ 2 ] * state [ 21 ] ; xx [ 65 ] = cos ( xx [ 59 ] ) ; xx [ 70 ] =
0.3459611930551978 ; xx [ 71 ] = sin ( xx [ 59 ] ) ; xx [ 59 ] = xx [ 44 ] *
xx [ 71 ] ; xx [ 72 ] = 0.3585189984711592 ; xx [ 73 ] = 0.0964521253570878 ;
xx [ 74 ] = xx [ 73 ] * xx [ 71 ] ; xx [ 71 ] = xx [ 46 ] * xx [ 65 ] + xx [
70 ] * xx [ 59 ] - xx [ 72 ] * xx [ 74 ] ; xx [ 75 ] = - xx [ 71 ] ; xx [ 76
] = xx [ 55 ] * xx [ 65 ] - xx [ 70 ] * xx [ 74 ] - xx [ 72 ] * xx [ 59 ] ;
xx [ 77 ] = xx [ 70 ] * xx [ 65 ] - xx [ 46 ] * xx [ 59 ] + xx [ 55 ] * xx [
74 ] ; xx [ 70 ] = xx [ 46 ] * xx [ 74 ] + xx [ 72 ] * xx [ 65 ] + xx [ 55 ]
* xx [ 59 ] ; xx [ 78 ] = xx [ 76 ] ; xx [ 79 ] = xx [ 77 ] ; xx [ 80 ] = xx
[ 70 ] ; xx [ 46 ] = 0.01043798038823735 ; xx [ 55 ] = 0.03286610935552551 ;
xx [ 59 ] = xx [ 70 ] * xx [ 46 ] - xx [ 55 ] * xx [ 77 ] ; xx [ 65 ] = xx [
55 ] * xx [ 76 ] ; xx [ 72 ] = xx [ 46 ] * xx [ 76 ] ; xx [ 81 ] = xx [ 59 ]
; xx [ 82 ] = xx [ 65 ] ; xx [ 83 ] = - xx [ 72 ] ; pm_math_Vector3_cross_ra
( xx + 78 , xx + 81 , xx + 84 ) ; xx [ 74 ] = 0.01161227146344391 - ( xx [ 84
] - xx [ 59 ] * xx [ 71 ] ) * xx [ 3 ] ; xx [ 59 ] = - ( 0.04870513239528159
+ xx [ 3 ] * ( xx [ 85 ] - xx [ 65 ] * xx [ 71 ] ) - xx [ 46 ] ) ; xx [ 46 ]
= 0.04944635745027942 - ( ( xx [ 72 ] * xx [ 71 ] + xx [ 86 ] ) * xx [ 3 ] -
xx [ 55 ] ) ; xx [ 55 ] = xx [ 2 ] * state [ 23 ] ; xx [ 65 ] = cos ( xx [ 55
] ) ; xx [ 71 ] = 0.6272234768732393 ; xx [ 72 ] = sin ( xx [ 55 ] ) ; xx [
55 ] = xx [ 38 ] * xx [ 65 ] + xx [ 71 ] * xx [ 72 ] ; xx [ 78 ] = - xx [ 55
] ; xx [ 79 ] = xx [ 38 ] * xx [ 72 ] - xx [ 71 ] * xx [ 65 ] ; xx [ 38 ] =
xx [ 53 ] * xx [ 65 ] - xx [ 62 ] * xx [ 72 ] ; xx [ 71 ] = xx [ 62 ] * xx [
65 ] + xx [ 53 ] * xx [ 72 ] ; xx [ 53 ] = xx [ 51 ] * xx [ 38 ] ; xx [ 62 ]
= xx [ 71 ] * xx [ 51 ] ; xx [ 65 ] = - ( xx [ 51 ] - ( xx [ 53 ] * xx [ 38 ]
+ xx [ 71 ] * xx [ 62 ] ) * xx [ 3 ] ) ; xx [ 72 ] = - ( xx [ 67 ] + ( xx [
53 ] * xx [ 79 ] - xx [ 55 ] * xx [ 62 ] ) * xx [ 3 ] ) ; xx [ 67 ] =
0.03042104860947364 - xx [ 3 ] * ( xx [ 62 ] * xx [ 79 ] + xx [ 55 ] * xx [
53 ] ) ; xx [ 53 ] = 0.5799181594307252 ; xx [ 55 ] = xx [ 2 ] * state [ 25 ]
; xx [ 62 ] = sin ( xx [ 55 ] ) ; xx [ 80 ] = xx [ 21 ] * xx [ 62 ] ; xx [ 81
] = 0.6442506183571239 ; xx [ 82 ] = xx [ 18 ] * xx [ 62 ] ; xx [ 62 ] =
0.3296521996834055 ; xx [ 83 ] = cos ( xx [ 55 ] ) ; xx [ 55 ] = xx [ 53 ] *
xx [ 80 ] + xx [ 81 ] * xx [ 82 ] - xx [ 62 ] * xx [ 83 ] ; xx [ 84 ] =
0.3741196283982388 ; xx [ 85 ] = xx [ 62 ] * xx [ 80 ] + xx [ 53 ] * xx [ 83
] - xx [ 84 ] * xx [ 82 ] ; xx [ 86 ] = xx [ 62 ] * xx [ 82 ] + xx [ 81 ] *
xx [ 83 ] + xx [ 84 ] * xx [ 80 ] ; xx [ 87 ] = - xx [ 86 ] ; xx [ 88 ] = xx
[ 84 ] * xx [ 83 ] + xx [ 53 ] * xx [ 82 ] - xx [ 81 ] * xx [ 80 ] ; xx [ 80
] = 4.244406610027125e-3 ; xx [ 89 ] = xx [ 85 ] ; xx [ 90 ] = xx [ 87 ] ; xx
[ 91 ] = xx [ 88 ] ; xx [ 82 ] = 0.06404981182865323 ; xx [ 83 ] = xx [ 82 ]
* xx [ 88 ] ; xx [ 92 ] = xx [ 80 ] * xx [ 88 ] ; xx [ 93 ] = xx [ 82 ] * xx
[ 85 ] + xx [ 86 ] * xx [ 80 ] ; xx [ 94 ] = - xx [ 83 ] ; xx [ 95 ] = xx [
92 ] ; xx [ 96 ] = xx [ 93 ] ; pm_math_Vector3_cross_ra ( xx + 89 , xx + 94 ,
xx + 97 ) ; xx [ 86 ] = 0.01262769948857074 - ( xx [ 80 ] + xx [ 3 ] * ( xx [
97 ] - xx [ 83 ] * xx [ 55 ] ) ) ; xx [ 80 ] = 0.1408927034847847 + xx [ 82 ]
+ ( xx [ 92 ] * xx [ 55 ] + xx [ 98 ] ) * xx [ 3 ] ; xx [ 82 ] = - xx [ 80 ]
; xx [ 83 ] = 0.09068544496193037 - ( xx [ 93 ] * xx [ 55 ] + xx [ 99 ] ) *
xx [ 3 ] ; xx [ 89 ] = 0.9379742042928237 ; xx [ 90 ] = xx [ 2 ] * state [ 27
] ; xx [ 91 ] = cos ( xx [ 90 ] ) ; xx [ 92 ] = 0.3464010870279209 ; xx [ 93
] = sin ( xx [ 90 ] ) ; xx [ 90 ] = xx [ 89 ] * xx [ 91 ] + xx [ 92 ] * xx [
93 ] ; xx [ 94 ] = - xx [ 90 ] ; xx [ 95 ] = xx [ 89 ] * xx [ 93 ] - xx [ 92
] * xx [ 91 ] ; xx [ 89 ] = 0.01361592839523579 ; xx [ 92 ] = xx [ 1 ] * xx [
91 ] - xx [ 89 ] * xx [ 93 ] ; xx [ 96 ] = xx [ 89 ] * xx [ 91 ] + xx [ 1 ] *
xx [ 93 ] ; xx [ 91 ] = xx [ 51 ] * xx [ 92 ] ; xx [ 93 ] = xx [ 96 ] * xx [
51 ] ; xx [ 97 ] = xx [ 15 ] - ( xx [ 51 ] - ( xx [ 91 ] * xx [ 92 ] + xx [
96 ] * xx [ 93 ] ) * xx [ 3 ] ) ; xx [ 51 ] = - ( xx [ 16 ] + ( xx [ 91 ] *
xx [ 95 ] - xx [ 90 ] * xx [ 93 ] ) * xx [ 3 ] ) ; xx [ 16 ] = - ( xx [ 3 ] *
( xx [ 93 ] * xx [ 95 ] + xx [ 90 ] * xx [ 91 ] ) ) ; xx [ 90 ] = xx [ 2 ] *
state [ 29 ] ; xx [ 91 ] = sin ( xx [ 90 ] ) ; xx [ 93 ] = xx [ 21 ] * xx [
91 ] ; xx [ 98 ] = xx [ 18 ] * xx [ 91 ] ; xx [ 91 ] = cos ( xx [ 90 ] ) ; xx
[ 90 ] = xx [ 53 ] * xx [ 93 ] + xx [ 81 ] * xx [ 98 ] - xx [ 62 ] * xx [ 91
] ; xx [ 99 ] = xx [ 62 ] * xx [ 93 ] + xx [ 53 ] * xx [ 91 ] - xx [ 84 ] *
xx [ 98 ] ; xx [ 100 ] = xx [ 62 ] * xx [ 98 ] + xx [ 81 ] * xx [ 91 ] + xx [
84 ] * xx [ 93 ] ; xx [ 62 ] = - xx [ 100 ] ; xx [ 101 ] = xx [ 84 ] * xx [
91 ] + xx [ 53 ] * xx [ 98 ] - xx [ 81 ] * xx [ 93 ] ; xx [ 53 ] =
4.244406610027171e-3 ; xx [ 102 ] = xx [ 99 ] ; xx [ 103 ] = xx [ 62 ] ; xx [
104 ] = xx [ 101 ] ; xx [ 81 ] = 0.06404981182865327 ; xx [ 84 ] = xx [ 81 ]
* xx [ 101 ] ; xx [ 91 ] = xx [ 53 ] * xx [ 101 ] ; xx [ 93 ] = xx [ 81 ] *
xx [ 99 ] + xx [ 100 ] * xx [ 53 ] ; xx [ 105 ] = - xx [ 84 ] ; xx [ 106 ] =
xx [ 91 ] ; xx [ 107 ] = xx [ 93 ] ; pm_math_Vector3_cross_ra ( xx + 102 , xx
+ 105 , xx + 108 ) ; xx [ 98 ] = 0.09519342531851044 + xx [ 53 ] + xx [ 3 ] *
( xx [ 108 ] - xx [ 84 ] * xx [ 90 ] ) ; xx [ 53 ] = - xx [ 98 ] ; xx [ 84 ]
= 0.136912200903329 + xx [ 81 ] + ( xx [ 91 ] * xx [ 90 ] + xx [ 109 ] ) * xx
[ 3 ] ; xx [ 81 ] = - xx [ 84 ] ; xx [ 91 ] = 0.09426587708939985 + ( xx [ 93
] * xx [ 90 ] + xx [ 110 ] ) * xx [ 3 ] ; xx [ 93 ] = - xx [ 91 ] ; xx [ 100
] = 0.9379742042928236 ; xx [ 102 ] = xx [ 2 ] * state [ 31 ] ; xx [ 103 ] =
cos ( xx [ 102 ] ) ; xx [ 104 ] = 0.3464010870279211 ; xx [ 105 ] = sin ( xx
[ 102 ] ) ; xx [ 102 ] = xx [ 100 ] * xx [ 103 ] + xx [ 104 ] * xx [ 105 ] ;
xx [ 106 ] = - xx [ 102 ] ; xx [ 107 ] = xx [ 100 ] * xx [ 105 ] - xx [ 104 ]
* xx [ 103 ] ; xx [ 100 ] = 5.028467068089626e-3 ; xx [ 104 ] =
0.01361592839523584 ; xx [ 108 ] = xx [ 100 ] * xx [ 103 ] - xx [ 104 ] * xx
[ 105 ] ; xx [ 109 ] = xx [ 104 ] * xx [ 103 ] + xx [ 100 ] * xx [ 105 ] ; xx
[ 100 ] = 4.953000000000063e-3 ; xx [ 103 ] = xx [ 100 ] * xx [ 108 ] ; xx [
104 ] = xx [ 109 ] * xx [ 100 ] ; xx [ 105 ] = xx [ 15 ] - ( xx [ 100 ] - (
xx [ 103 ] * xx [ 108 ] + xx [ 109 ] * xx [ 104 ] ) * xx [ 3 ] ) ; xx [ 100 ]
= - ( 0.05562318494169332 + ( xx [ 103 ] * xx [ 107 ] - xx [ 102 ] * xx [ 104
] ) * xx [ 3 ] ) ; xx [ 110 ] = - ( xx [ 3 ] * ( xx [ 104 ] * xx [ 107 ] + xx
[ 102 ] * xx [ 103 ] ) ) ; xx [ 102 ] = xx [ 2 ] * state [ 33 ] ; xx [ 103 ]
= cos ( xx [ 102 ] ) ; xx [ 104 ] = sin ( xx [ 102 ] ) ; xx [ 102 ] = xx [ 18
] * xx [ 104 ] ; xx [ 111 ] = xx [ 21 ] * xx [ 104 ] ; xx [ 104 ] = xx [ 24 ]
* xx [ 103 ] + xx [ 32 ] * xx [ 102 ] - xx [ 31 ] * xx [ 111 ] ; xx [ 112 ] =
- xx [ 104 ] ; xx [ 113 ] = xx [ 24 ] * xx [ 111 ] + xx [ 31 ] * xx [ 103 ] +
xx [ 33 ] * xx [ 102 ] ; xx [ 114 ] = xx [ 32 ] * xx [ 103 ] - xx [ 24 ] * xx
[ 102 ] + xx [ 33 ] * xx [ 111 ] ; xx [ 24 ] = xx [ 31 ] * xx [ 102 ] - xx [
33 ] * xx [ 103 ] + xx [ 32 ] * xx [ 111 ] ; xx [ 31 ] = 4.244406610027172e-3
; xx [ 115 ] = xx [ 113 ] ; xx [ 116 ] = xx [ 114 ] ; xx [ 117 ] = xx [ 24 ]
; xx [ 32 ] = 0.06404981182865328 ; xx [ 33 ] = xx [ 32 ] * xx [ 24 ] ; xx [
102 ] = xx [ 31 ] * xx [ 24 ] ; xx [ 103 ] = xx [ 32 ] * xx [ 113 ] - xx [
114 ] * xx [ 31 ] ; xx [ 118 ] = - xx [ 33 ] ; xx [ 119 ] = xx [ 102 ] ; xx [
120 ] = xx [ 103 ] ; pm_math_Vector3_cross_ra ( xx + 115 , xx + 118 , xx +
121 ) ; xx [ 111 ] = 0.07045837366530477 + xx [ 31 ] + xx [ 3 ] * ( xx [ 121
] + xx [ 33 ] * xx [ 104 ] ) ; xx [ 31 ] = - xx [ 111 ] ; xx [ 33 ] =
0.1208316631353296 - ( xx [ 32 ] + ( xx [ 122 ] - xx [ 102 ] * xx [ 104 ] ) *
xx [ 3 ] ) ; xx [ 32 ] = 0.1031385412966565 + ( xx [ 123 ] - xx [ 103 ] * xx
[ 104 ] ) * xx [ 3 ] ; xx [ 102 ] = - xx [ 32 ] ; xx [ 103 ] = xx [ 2 ] *
state [ 35 ] ; xx [ 2 ] = cos ( xx [ 103 ] ) ; xx [ 104 ] = sin ( xx [ 103 ]
) ; xx [ 103 ] = xx [ 1 ] * xx [ 2 ] + xx [ 89 ] * xx [ 104 ] ; xx [ 115 ] =
- xx [ 103 ] ; xx [ 116 ] = xx [ 89 ] * xx [ 2 ] - xx [ 1 ] * xx [ 104 ] ; xx
[ 1 ] = 0.3464010870279207 ; xx [ 89 ] = xx [ 17 ] * xx [ 2 ] - xx [ 1 ] * xx
[ 104 ] ; xx [ 117 ] = xx [ 1 ] * xx [ 2 ] + xx [ 17 ] * xx [ 104 ] ; xx [ 1
] = - xx [ 117 ] ; xx [ 2 ] = xx [ 89 ] * xx [ 0 ] ; xx [ 17 ] = xx [ 0 ] *
xx [ 117 ] ; xx [ 104 ] = xx [ 15 ] - ( ( xx [ 89 ] * xx [ 2 ] + xx [ 17 ] *
xx [ 117 ] ) * xx [ 3 ] - xx [ 0 ] ) ; xx [ 15 ] = ( xx [ 17 ] * xx [ 103 ] +
xx [ 116 ] * xx [ 2 ] ) * xx [ 3 ] - 0.05562318494169335 ; xx [ 117 ] = - (
xx [ 3 ] * ( xx [ 116 ] * xx [ 17 ] - xx [ 2 ] * xx [ 103 ] ) ) ; xx [ 118 ]
= xx [ 23 ] ; xx [ 119 ] = xx [ 25 ] ; xx [ 120 ] = xx [ 26 ] ; xx [ 121 ] =
xx [ 27 ] ; pm_math_Quaternion_compose_ra ( xx + 10 , xx + 118 , xx + 122 ) ;
xx [ 126 ] = xx [ 20 ] ; xx [ 127 ] = xx [ 22 ] ; xx [ 128 ] = xx [ 19 ] ;
pm_math_Quaternion_xform_ra ( xx + 10 , xx + 126 , xx + 129 ) ; xx [ 2 ] = xx
[ 129 ] + xx [ 4 ] ; xx [ 3 ] = xx [ 130 ] + xx [ 5 ] ; xx [ 17 ] = xx [ 131
] + xx [ 14 ] ; pm_math_Quaternion_compose_ra ( xx + 122 , xx + 47 , xx + 129
) ; pm_math_Quaternion_xform_ra ( xx + 122 , xx + 34 , xx + 133 ) ; xx [ 103
] = xx [ 133 ] + xx [ 2 ] ; xx [ 136 ] = xx [ 134 ] + xx [ 3 ] ; xx [ 133 ] =
xx [ 135 ] + xx [ 17 ] ; xx [ 137 ] = xx [ 90 ] ; xx [ 138 ] = xx [ 99 ] ; xx
[ 139 ] = xx [ 62 ] ; xx [ 140 ] = xx [ 101 ] ; pm_math_Quaternion_compose_ra
( xx + 129 , xx + 137 , xx + 141 ) ; xx [ 145 ] = xx [ 53 ] ; xx [ 146 ] = xx
[ 81 ] ; xx [ 147 ] = xx [ 93 ] ; pm_math_Quaternion_xform_ra ( xx + 129 , xx
+ 145 , xx + 148 ) ; xx [ 134 ] = xx [ 148 ] + xx [ 103 ] ; xx [ 135 ] = xx [
149 ] + xx [ 136 ] ; xx [ 148 ] = xx [ 150 ] + xx [ 133 ] ;
pm_math_Quaternion_compose_ra ( xx + 141 , xx + 106 , xx + 149 ) ; xx [ 153 ]
= xx [ 105 ] ; xx [ 154 ] = xx [ 100 ] ; xx [ 155 ] = xx [ 110 ] ;
pm_math_Quaternion_xform_ra ( xx + 141 , xx + 153 , xx + 156 ) ;
pm_math_Quaternion_compose_ra ( xx + 137 , xx + 106 , xx + 159 ) ;
pm_math_Quaternion_xform_ra ( xx + 137 , xx + 153 , xx + 163 ) ; xx [ 166 ] =
xx [ 163 ] - xx [ 98 ] ; xx [ 98 ] = xx [ 164 ] - xx [ 84 ] ; xx [ 84 ] = xx
[ 165 ] - xx [ 91 ] ; pm_math_Quaternion_compose_ra ( xx + 47 , xx + 159 , xx
+ 167 ) ; xx [ 163 ] = xx [ 166 ] ; xx [ 164 ] = xx [ 98 ] ; xx [ 165 ] = xx
[ 84 ] ; pm_math_Quaternion_xform_ra ( xx + 47 , xx + 163 , xx + 171 ) ; xx [
91 ] = xx [ 171 ] + xx [ 34 ] ; xx [ 163 ] = xx [ 172 ] + xx [ 35 ] ; xx [
164 ] = xx [ 173 ] - xx [ 45 ] ; pm_math_Quaternion_compose_ra ( xx + 118 ,
xx + 167 , xx + 171 ) ; xx [ 175 ] = xx [ 91 ] ; xx [ 176 ] = xx [ 163 ] ; xx
[ 177 ] = xx [ 164 ] ; pm_math_Quaternion_xform_ra ( xx + 118 , xx + 175 , xx
+ 178 ) ; xx [ 181 ] = xx [ 55 ] ; xx [ 182 ] = xx [ 85 ] ; xx [ 183 ] = xx [
87 ] ; xx [ 184 ] = xx [ 88 ] ; pm_math_Quaternion_compose_ra ( xx + 129 , xx
+ 181 , xx + 185 ) ; xx [ 175 ] = xx [ 86 ] ; xx [ 176 ] = xx [ 82 ] ; xx [
177 ] = xx [ 83 ] ; pm_math_Quaternion_xform_ra ( xx + 129 , xx + 175 , xx +
189 ) ; xx [ 165 ] = xx [ 189 ] + xx [ 103 ] ; xx [ 192 ] = xx [ 190 ] + xx [
136 ] ; xx [ 189 ] = xx [ 191 ] + xx [ 133 ] ; xx [ 193 ] = xx [ 94 ] ; xx [
194 ] = xx [ 95 ] ; xx [ 195 ] = xx [ 92 ] ; xx [ 196 ] = xx [ 96 ] ;
pm_math_Quaternion_compose_ra ( xx + 185 , xx + 193 , xx + 197 ) ; xx [ 201 ]
= xx [ 97 ] ; xx [ 202 ] = xx [ 51 ] ; xx [ 203 ] = xx [ 16 ] ;
pm_math_Quaternion_xform_ra ( xx + 185 , xx + 201 , xx + 204 ) ;
pm_math_Quaternion_compose_ra ( xx + 181 , xx + 193 , xx + 207 ) ;
pm_math_Quaternion_xform_ra ( xx + 181 , xx + 201 , xx + 211 ) ; xx [ 190 ] =
xx [ 211 ] + xx [ 86 ] ; xx [ 191 ] = xx [ 212 ] - xx [ 80 ] ; xx [ 80 ] = xx
[ 213 ] + xx [ 83 ] ; pm_math_Quaternion_compose_ra ( xx + 47 , xx + 207 , xx
+ 211 ) ; xx [ 215 ] = xx [ 190 ] ; xx [ 216 ] = xx [ 191 ] ; xx [ 217 ] = xx
[ 80 ] ; pm_math_Quaternion_xform_ra ( xx + 47 , xx + 215 , xx + 218 ) ; xx [
215 ] = xx [ 218 ] + xx [ 34 ] ; xx [ 216 ] = xx [ 219 ] + xx [ 35 ] ; xx [
217 ] = xx [ 220 ] - xx [ 45 ] ; pm_math_Quaternion_compose_ra ( xx + 118 ,
xx + 211 , xx + 218 ) ; pm_math_Quaternion_xform_ra ( xx + 118 , xx + 215 ,
xx + 222 ) ; xx [ 225 ] = xx [ 112 ] ; xx [ 226 ] = xx [ 113 ] ; xx [ 227 ] =
xx [ 114 ] ; xx [ 228 ] = xx [ 24 ] ; pm_math_Quaternion_compose_ra ( xx +
129 , xx + 225 , xx + 229 ) ; xx [ 233 ] = xx [ 31 ] ; xx [ 234 ] = xx [ 33 ]
; xx [ 235 ] = xx [ 102 ] ; pm_math_Quaternion_xform_ra ( xx + 129 , xx + 233
, xx + 236 ) ; xx [ 239 ] = xx [ 236 ] + xx [ 103 ] ; xx [ 240 ] = xx [ 237 ]
+ xx [ 136 ] ; xx [ 236 ] = xx [ 238 ] + xx [ 133 ] ; xx [ 241 ] = xx [ 115 ]
; xx [ 242 ] = xx [ 116 ] ; xx [ 243 ] = xx [ 89 ] ; xx [ 244 ] = xx [ 1 ] ;
pm_math_Quaternion_compose_ra ( xx + 229 , xx + 241 , xx + 245 ) ; xx [ 249 ]
= xx [ 104 ] ; xx [ 250 ] = xx [ 15 ] ; xx [ 251 ] = xx [ 117 ] ;
pm_math_Quaternion_xform_ra ( xx + 229 , xx + 249 , xx + 252 ) ;
pm_math_Quaternion_compose_ra ( xx + 225 , xx + 241 , xx + 255 ) ;
pm_math_Quaternion_xform_ra ( xx + 225 , xx + 249 , xx + 259 ) ; xx [ 237 ] =
xx [ 259 ] - xx [ 111 ] ; xx [ 111 ] = xx [ 260 ] + xx [ 33 ] ; xx [ 238 ] =
xx [ 261 ] - xx [ 32 ] ; pm_math_Quaternion_compose_ra ( xx + 47 , xx + 255 ,
xx + 259 ) ; xx [ 263 ] = xx [ 237 ] ; xx [ 264 ] = xx [ 111 ] ; xx [ 265 ] =
xx [ 238 ] ; pm_math_Quaternion_xform_ra ( xx + 47 , xx + 263 , xx + 266 ) ;
xx [ 32 ] = xx [ 266 ] + xx [ 34 ] ; xx [ 263 ] = xx [ 267 ] + xx [ 35 ] ; xx
[ 264 ] = xx [ 268 ] - xx [ 45 ] ; pm_math_Quaternion_compose_ra ( xx + 118 ,
xx + 259 , xx + 265 ) ; xx [ 269 ] = xx [ 32 ] ; xx [ 270 ] = xx [ 263 ] ; xx
[ 271 ] = xx [ 264 ] ; pm_math_Quaternion_xform_ra ( xx + 118 , xx + 269 , xx
+ 272 ) ; xx [ 269 ] = state [ 10 ] ; xx [ 270 ] = state [ 11 ] ; xx [ 271 ]
= state [ 12 ] ; pm_math_Quaternion_xform_ra ( xx + 6 , xx + 269 , xx + 275 )
; xx [ 6 ] = state [ 7 ] ; xx [ 7 ] = state [ 8 ] ; xx [ 8 ] = state [ 9 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 10 , xx + 6 , xx + 269 ) ; xx [ 6 ]
= xx [ 270 ] + xx [ 0 ] * xx [ 277 ] ; xx [ 7 ] = xx [ 271 ] - xx [ 0 ] * xx
[ 276 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 118 , xx + 275 , xx + 278
) ; xx [ 0 ] = xx [ 278 ] + xx [ 21 ] * state [ 14 ] ; xx [ 8 ] = xx [ 279 ]
- xx [ 18 ] * state [ 14 ] ; pm_math_Vector3_cross_ra ( xx + 275 , xx + 126 ,
xx + 281 ) ; xx [ 126 ] = xx [ 281 ] + xx [ 269 ] ; xx [ 127 ] = xx [ 282 ] +
xx [ 6 ] ; xx [ 128 ] = xx [ 283 ] + xx [ 7 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 118 , xx + 126 , xx + 281 ) ; xx [
9 ] = xx [ 283 ] + 0.05497997575039515 * state [ 14 ] ; xx [ 118 ] = xx [ 0 ]
; xx [ 119 ] = xx [ 8 ] ; xx [ 120 ] = xx [ 280 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 47 , xx + 118 , xx + 126 ) ; xx [
45 ] = xx [ 126 ] + xx [ 39 ] * state [ 16 ] ; xx [ 39 ] = xx [ 127 ] + xx [
41 ] * state [ 16 ] ; xx [ 41 ] = xx [ 128 ] - xx [ 42 ] * state [ 16 ] ;
pm_math_Vector3_cross_ra ( xx + 118 , xx + 34 , xx + 126 ) ; xx [ 118 ] = xx
[ 126 ] + xx [ 281 ] ; xx [ 119 ] = xx [ 127 ] + xx [ 282 ] ; xx [ 120 ] = xx
[ 128 ] + xx [ 9 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 47 , xx + 118
, xx + 126 ) ; xx [ 42 ] = xx [ 126 ] - 0.08539263296773184 * state [ 16 ] ;
xx [ 118 ] = xx [ 127 ] + 9.090437326692596e-3 * state [ 16 ] ; xx [ 119 ] =
xx [ 128 ] + 0.02601397390134001 * state [ 16 ] ; xx [ 283 ] = xx [ 54 ] ; xx
[ 284 ] = xx [ 56 ] ; xx [ 285 ] = xx [ 57 ] ; xx [ 286 ] = xx [ 43 ] ; xx [
126 ] = xx [ 45 ] ; xx [ 127 ] = xx [ 39 ] ; xx [ 128 ] = xx [ 41 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 283 , xx + 126 , xx + 287 ) ; xx [
120 ] = xx [ 288 ] + xx [ 44 ] * state [ 18 ] ; xx [ 121 ] = xx [ 289 ] - xx
[ 52 ] * state [ 18 ] ; xx [ 288 ] = xx [ 58 ] ; xx [ 289 ] = xx [ 40 ] ; xx
[ 290 ] = xx [ 37 ] ; pm_math_Vector3_cross_ra ( xx + 126 , xx + 288 , xx +
291 ) ; xx [ 288 ] = xx [ 291 ] + xx [ 42 ] ; xx [ 289 ] = xx [ 292 ] + xx [
118 ] ; xx [ 290 ] = xx [ 293 ] + xx [ 119 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 283 , xx + 288 , xx + 291 ) ; xx [
52 ] = xx [ 291 ] + 0.0337196406180599 * state [ 18 ] ; xx [ 283 ] = xx [ 60
] ; xx [ 284 ] = xx [ 61 ] ; xx [ 285 ] = xx [ 63 ] ; xx [ 286 ] = xx [ 64 ]
; xx [ 288 ] = xx [ 287 ] ; xx [ 289 ] = xx [ 120 ] ; xx [ 290 ] = xx [ 121 ]
; pm_math_Quaternion_inverseXform_ra ( xx + 283 , xx + 288 , xx + 294 ) ; xx
[ 297 ] = xx [ 66 ] ; xx [ 298 ] = xx [ 68 ] ; xx [ 299 ] = xx [ 69 ] ;
pm_math_Vector3_cross_ra ( xx + 288 , xx + 297 , xx + 300 ) ; xx [ 288 ] = xx
[ 300 ] + xx [ 52 ] ; xx [ 289 ] = xx [ 301 ] + xx [ 292 ] ; xx [ 290 ] = xx
[ 302 ] + xx [ 293 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 283 , xx +
288 , xx + 297 ) ; xx [ 283 ] = xx [ 75 ] ; xx [ 284 ] = xx [ 76 ] ; xx [ 285
] = xx [ 77 ] ; xx [ 286 ] = xx [ 70 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 283 , xx + 126 , xx + 288 ) ; xx [ 270 ] = xx [ 289 ] + xx [ 44 ] *
state [ 22 ] ; xx [ 44 ] = xx [ 290 ] - xx [ 73 ] * state [ 22 ] ; xx [ 289 ]
= xx [ 74 ] ; xx [ 290 ] = xx [ 59 ] ; xx [ 291 ] = xx [ 46 ] ;
pm_math_Vector3_cross_ra ( xx + 126 , xx + 289 , xx + 300 ) ; xx [ 289 ] = xx
[ 300 ] + xx [ 42 ] ; xx [ 290 ] = xx [ 301 ] + xx [ 118 ] ; xx [ 291 ] = xx
[ 302 ] + xx [ 119 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 283 , xx +
289 , xx + 300 ) ; xx [ 73 ] = xx [ 300 ] + 0.03371964061805995 * state [ 22
] ; xx [ 283 ] = xx [ 78 ] ; xx [ 284 ] = xx [ 79 ] ; xx [ 285 ] = xx [ 38 ]
; xx [ 286 ] = xx [ 71 ] ; xx [ 289 ] = xx [ 288 ] ; xx [ 290 ] = xx [ 270 ]
; xx [ 291 ] = xx [ 44 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 283 , xx
+ 289 , xx + 303 ) ; xx [ 306 ] = xx [ 65 ] ; xx [ 307 ] = xx [ 72 ] ; xx [
308 ] = xx [ 67 ] ; pm_math_Vector3_cross_ra ( xx + 289 , xx + 306 , xx + 309
) ; xx [ 289 ] = xx [ 309 ] + xx [ 73 ] ; xx [ 290 ] = xx [ 310 ] + xx [ 301
] ; xx [ 291 ] = xx [ 311 ] + xx [ 302 ] ; pm_math_Quaternion_inverseXform_ra
( xx + 283 , xx + 289 , xx + 306 ) ; pm_math_Quaternion_inverseXform_ra ( xx
+ 181 , xx + 126 , xx + 283 ) ; xx [ 271 ] = xx [ 283 ] - xx [ 21 ] * state [
26 ] ; xx [ 278 ] = xx [ 284 ] + xx [ 18 ] * state [ 26 ] ;
pm_math_Vector3_cross_ra ( xx + 126 , xx + 175 , xx + 289 ) ; xx [ 175 ] = xx
[ 289 ] + xx [ 42 ] ; xx [ 176 ] = xx [ 290 ] + xx [ 118 ] ; xx [ 177 ] = xx
[ 291 ] + xx [ 119 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 181 , xx +
175 , xx + 289 ) ; xx [ 175 ] = xx [ 291 ] + 0.06414602424960487 * state [ 26
] ; xx [ 181 ] = xx [ 271 ] ; xx [ 182 ] = xx [ 278 ] ; xx [ 183 ] = xx [ 285
] ; pm_math_Quaternion_inverseXform_ra ( xx + 193 , xx + 181 , xx + 309 ) ;
pm_math_Vector3_cross_ra ( xx + 181 , xx + 201 , xx + 312 ) ; xx [ 181 ] = xx
[ 312 ] + xx [ 289 ] ; xx [ 182 ] = xx [ 313 ] + xx [ 290 ] ; xx [ 183 ] = xx
[ 314 ] + xx [ 175 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 193 , xx +
181 , xx + 201 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 137 , xx + 126 ,
xx + 181 ) ; xx [ 176 ] = xx [ 181 ] - xx [ 21 ] * state [ 30 ] ; xx [ 177 ]
= xx [ 182 ] + xx [ 18 ] * state [ 30 ] ; pm_math_Vector3_cross_ra ( xx + 126
, xx + 145 , xx + 193 ) ; xx [ 145 ] = xx [ 193 ] + xx [ 42 ] ; xx [ 146 ] =
xx [ 194 ] + xx [ 118 ] ; xx [ 147 ] = xx [ 195 ] + xx [ 119 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 137 , xx + 145 , xx + 193 ) ; xx [
137 ] = xx [ 195 ] + 0.06414602424960492 * state [ 30 ] ; xx [ 138 ] = xx [
176 ] ; xx [ 139 ] = xx [ 177 ] ; xx [ 140 ] = xx [ 183 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 106 , xx + 138 , xx + 145 ) ;
pm_math_Vector3_cross_ra ( xx + 138 , xx + 153 , xx + 312 ) ; xx [ 138 ] = xx
[ 312 ] + xx [ 193 ] ; xx [ 139 ] = xx [ 313 ] + xx [ 194 ] ; xx [ 140 ] = xx
[ 314 ] + xx [ 137 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 106 , xx +
138 , xx + 153 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 225 , xx + 126 ,
xx + 138 ) ; xx [ 181 ] = xx [ 138 ] - xx [ 21 ] * state [ 34 ] ; xx [ 21 ] =
xx [ 139 ] + xx [ 18 ] * state [ 34 ] ; pm_math_Vector3_cross_ra ( xx + 126 ,
xx + 233 , xx + 312 ) ; xx [ 126 ] = xx [ 312 ] + xx [ 42 ] ; xx [ 127 ] = xx
[ 313 ] + xx [ 118 ] ; xx [ 128 ] = xx [ 314 ] + xx [ 119 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 225 , xx + 126 , xx + 233 ) ; xx [
18 ] = xx [ 235 ] + 0.06414602424960493 * state [ 34 ] ; xx [ 126 ] = xx [
181 ] ; xx [ 127 ] = xx [ 21 ] ; xx [ 128 ] = xx [ 140 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 241 , xx + 126 , xx + 225 ) ;
pm_math_Vector3_cross_ra ( xx + 126 , xx + 249 , xx + 312 ) ; xx [ 126 ] = xx
[ 312 ] + xx [ 233 ] ; xx [ 127 ] = xx [ 313 ] + xx [ 234 ] ; xx [ 128 ] = xx
[ 314 ] + xx [ 18 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 241 , xx +
126 , xx + 249 ) ; motionData [ 0 ] = xx [ 10 ] ; motionData [ 1 ] = xx [ 11
] ; motionData [ 2 ] = xx [ 12 ] ; motionData [ 3 ] = xx [ 13 ] ; motionData
[ 4 ] = xx [ 4 ] ; motionData [ 5 ] = xx [ 5 ] ; motionData [ 6 ] = xx [ 14 ]
; motionData [ 7 ] = xx [ 23 ] ; motionData [ 8 ] = xx [ 25 ] ; motionData [
9 ] = xx [ 26 ] ; motionData [ 10 ] = xx [ 27 ] ; motionData [ 11 ] = xx [ 20
] ; motionData [ 12 ] = xx [ 22 ] ; motionData [ 13 ] = xx [ 19 ] ;
motionData [ 14 ] = xx [ 47 ] ; motionData [ 15 ] = xx [ 48 ] ; motionData [
16 ] = xx [ 49 ] ; motionData [ 17 ] = xx [ 50 ] ; motionData [ 18 ] = xx [
34 ] ; motionData [ 19 ] = xx [ 35 ] ; motionData [ 20 ] = xx [ 36 ] ;
motionData [ 21 ] = xx [ 54 ] ; motionData [ 22 ] = xx [ 56 ] ; motionData [
23 ] = xx [ 57 ] ; motionData [ 24 ] = xx [ 43 ] ; motionData [ 25 ] = xx [
58 ] ; motionData [ 26 ] = xx [ 40 ] ; motionData [ 27 ] = xx [ 37 ] ;
motionData [ 28 ] = xx [ 60 ] ; motionData [ 29 ] = xx [ 61 ] ; motionData [
30 ] = xx [ 63 ] ; motionData [ 31 ] = xx [ 64 ] ; motionData [ 32 ] = xx [
66 ] ; motionData [ 33 ] = xx [ 68 ] ; motionData [ 34 ] = xx [ 69 ] ;
motionData [ 35 ] = xx [ 75 ] ; motionData [ 36 ] = xx [ 76 ] ; motionData [
37 ] = xx [ 77 ] ; motionData [ 38 ] = xx [ 70 ] ; motionData [ 39 ] = xx [
74 ] ; motionData [ 40 ] = xx [ 59 ] ; motionData [ 41 ] = xx [ 46 ] ;
motionData [ 42 ] = xx [ 78 ] ; motionData [ 43 ] = xx [ 79 ] ; motionData [
44 ] = xx [ 38 ] ; motionData [ 45 ] = xx [ 71 ] ; motionData [ 46 ] = xx [
65 ] ; motionData [ 47 ] = xx [ 72 ] ; motionData [ 48 ] = xx [ 67 ] ;
motionData [ 49 ] = xx [ 55 ] ; motionData [ 50 ] = xx [ 85 ] ; motionData [
51 ] = xx [ 87 ] ; motionData [ 52 ] = xx [ 88 ] ; motionData [ 53 ] = xx [
86 ] ; motionData [ 54 ] = xx [ 82 ] ; motionData [ 55 ] = xx [ 83 ] ;
motionData [ 56 ] = xx [ 94 ] ; motionData [ 57 ] = xx [ 95 ] ; motionData [
58 ] = xx [ 92 ] ; motionData [ 59 ] = xx [ 96 ] ; motionData [ 60 ] = xx [
97 ] ; motionData [ 61 ] = xx [ 51 ] ; motionData [ 62 ] = xx [ 16 ] ;
motionData [ 63 ] = xx [ 90 ] ; motionData [ 64 ] = xx [ 99 ] ; motionData [
65 ] = xx [ 62 ] ; motionData [ 66 ] = xx [ 101 ] ; motionData [ 67 ] = xx [
53 ] ; motionData [ 68 ] = xx [ 81 ] ; motionData [ 69 ] = xx [ 93 ] ;
motionData [ 70 ] = xx [ 106 ] ; motionData [ 71 ] = xx [ 107 ] ; motionData
[ 72 ] = xx [ 108 ] ; motionData [ 73 ] = xx [ 109 ] ; motionData [ 74 ] = xx
[ 105 ] ; motionData [ 75 ] = xx [ 100 ] ; motionData [ 76 ] = xx [ 110 ] ;
motionData [ 77 ] = xx [ 112 ] ; motionData [ 78 ] = xx [ 113 ] ; motionData
[ 79 ] = xx [ 114 ] ; motionData [ 80 ] = xx [ 24 ] ; motionData [ 81 ] = xx
[ 31 ] ; motionData [ 82 ] = xx [ 33 ] ; motionData [ 83 ] = xx [ 102 ] ;
motionData [ 84 ] = xx [ 115 ] ; motionData [ 85 ] = xx [ 116 ] ; motionData
[ 86 ] = xx [ 89 ] ; motionData [ 87 ] = xx [ 1 ] ; motionData [ 88 ] = xx [
104 ] ; motionData [ 89 ] = xx [ 15 ] ; motionData [ 90 ] = xx [ 117 ] ;
motionData [ 91 ] = xx [ 122 ] ; motionData [ 92 ] = xx [ 123 ] ; motionData
[ 93 ] = xx [ 124 ] ; motionData [ 94 ] = xx [ 125 ] ; motionData [ 95 ] = xx
[ 2 ] ; motionData [ 96 ] = xx [ 3 ] ; motionData [ 97 ] = xx [ 17 ] ;
motionData [ 98 ] = xx [ 129 ] ; motionData [ 99 ] = xx [ 130 ] ; motionData
[ 100 ] = xx [ 131 ] ; motionData [ 101 ] = xx [ 132 ] ; motionData [ 102 ] =
xx [ 103 ] ; motionData [ 103 ] = xx [ 136 ] ; motionData [ 104 ] = xx [ 133
] ; motionData [ 105 ] = xx [ 141 ] ; motionData [ 106 ] = xx [ 142 ] ;
motionData [ 107 ] = xx [ 143 ] ; motionData [ 108 ] = xx [ 144 ] ;
motionData [ 109 ] = xx [ 134 ] ; motionData [ 110 ] = xx [ 135 ] ;
motionData [ 111 ] = xx [ 148 ] ; motionData [ 112 ] = xx [ 149 ] ;
motionData [ 113 ] = xx [ 150 ] ; motionData [ 114 ] = xx [ 151 ] ;
motionData [ 115 ] = xx [ 152 ] ; motionData [ 116 ] = xx [ 156 ] + xx [ 134
] ; motionData [ 117 ] = xx [ 157 ] + xx [ 135 ] ; motionData [ 118 ] = xx [
158 ] + xx [ 148 ] ; motionData [ 119 ] = xx [ 159 ] ; motionData [ 120 ] =
xx [ 160 ] ; motionData [ 121 ] = xx [ 161 ] ; motionData [ 122 ] = xx [ 162
] ; motionData [ 123 ] = xx [ 166 ] ; motionData [ 124 ] = xx [ 98 ] ;
motionData [ 125 ] = xx [ 84 ] ; motionData [ 126 ] = xx [ 167 ] ; motionData
[ 127 ] = xx [ 168 ] ; motionData [ 128 ] = xx [ 169 ] ; motionData [ 129 ] =
xx [ 170 ] ; motionData [ 130 ] = xx [ 91 ] ; motionData [ 131 ] = xx [ 163 ]
; motionData [ 132 ] = xx [ 164 ] ; motionData [ 133 ] = xx [ 171 ] ;
motionData [ 134 ] = xx [ 172 ] ; motionData [ 135 ] = xx [ 173 ] ;
motionData [ 136 ] = xx [ 174 ] ; motionData [ 137 ] = xx [ 178 ] - xx [ 28 ]
; motionData [ 138 ] = xx [ 179 ] - xx [ 29 ] ; motionData [ 139 ] = xx [ 180
] - xx [ 30 ] ; motionData [ 140 ] = xx [ 185 ] ; motionData [ 141 ] = xx [
186 ] ; motionData [ 142 ] = xx [ 187 ] ; motionData [ 143 ] = xx [ 188 ] ;
motionData [ 144 ] = xx [ 165 ] ; motionData [ 145 ] = xx [ 192 ] ;
motionData [ 146 ] = xx [ 189 ] ; motionData [ 147 ] = xx [ 197 ] ;
motionData [ 148 ] = xx [ 198 ] ; motionData [ 149 ] = xx [ 199 ] ;
motionData [ 150 ] = xx [ 200 ] ; motionData [ 151 ] = xx [ 204 ] + xx [ 165
] ; motionData [ 152 ] = xx [ 205 ] + xx [ 192 ] ; motionData [ 153 ] = xx [
206 ] + xx [ 189 ] ; motionData [ 154 ] = xx [ 207 ] ; motionData [ 155 ] =
xx [ 208 ] ; motionData [ 156 ] = xx [ 209 ] ; motionData [ 157 ] = xx [ 210
] ; motionData [ 158 ] = xx [ 190 ] ; motionData [ 159 ] = xx [ 191 ] ;
motionData [ 160 ] = xx [ 80 ] ; motionData [ 161 ] = xx [ 211 ] ; motionData
[ 162 ] = xx [ 212 ] ; motionData [ 163 ] = xx [ 213 ] ; motionData [ 164 ] =
xx [ 214 ] ; motionData [ 165 ] = xx [ 215 ] ; motionData [ 166 ] = xx [ 216
] ; motionData [ 167 ] = xx [ 217 ] ; motionData [ 168 ] = xx [ 218 ] ;
motionData [ 169 ] = xx [ 219 ] ; motionData [ 170 ] = xx [ 220 ] ;
motionData [ 171 ] = xx [ 221 ] ; motionData [ 172 ] = xx [ 222 ] - xx [ 28 ]
; motionData [ 173 ] = xx [ 223 ] - xx [ 29 ] ; motionData [ 174 ] = xx [ 224
] - xx [ 30 ] ; motionData [ 175 ] = xx [ 229 ] ; motionData [ 176 ] = xx [
230 ] ; motionData [ 177 ] = xx [ 231 ] ; motionData [ 178 ] = xx [ 232 ] ;
motionData [ 179 ] = xx [ 239 ] ; motionData [ 180 ] = xx [ 240 ] ;
motionData [ 181 ] = xx [ 236 ] ; motionData [ 182 ] = xx [ 245 ] ;
motionData [ 183 ] = xx [ 246 ] ; motionData [ 184 ] = xx [ 247 ] ;
motionData [ 185 ] = xx [ 248 ] ; motionData [ 186 ] = xx [ 252 ] + xx [ 239
] ; motionData [ 187 ] = xx [ 253 ] + xx [ 240 ] ; motionData [ 188 ] = xx [
254 ] + xx [ 236 ] ; motionData [ 189 ] = xx [ 255 ] ; motionData [ 190 ] =
xx [ 256 ] ; motionData [ 191 ] = xx [ 257 ] ; motionData [ 192 ] = xx [ 258
] ; motionData [ 193 ] = xx [ 237 ] ; motionData [ 194 ] = xx [ 111 ] ;
motionData [ 195 ] = xx [ 238 ] ; motionData [ 196 ] = xx [ 259 ] ;
motionData [ 197 ] = xx [ 260 ] ; motionData [ 198 ] = xx [ 261 ] ;
motionData [ 199 ] = xx [ 262 ] ; motionData [ 200 ] = xx [ 32 ] ; motionData
[ 201 ] = xx [ 263 ] ; motionData [ 202 ] = xx [ 264 ] ; motionData [ 203 ] =
xx [ 265 ] ; motionData [ 204 ] = xx [ 266 ] ; motionData [ 205 ] = xx [ 267
] ; motionData [ 206 ] = xx [ 268 ] ; motionData [ 207 ] = xx [ 272 ] - xx [
28 ] ; motionData [ 208 ] = xx [ 273 ] - xx [ 29 ] ; motionData [ 209 ] = xx
[ 274 ] - xx [ 30 ] ; motionData [ 210 ] = xx [ 275 ] ; motionData [ 211 ] =
xx [ 276 ] ; motionData [ 212 ] = xx [ 277 ] ; motionData [ 213 ] = xx [ 269
] ; motionData [ 214 ] = xx [ 6 ] ; motionData [ 215 ] = xx [ 7 ] ;
motionData [ 216 ] = xx [ 0 ] ; motionData [ 217 ] = xx [ 8 ] ; motionData [
218 ] = xx [ 280 ] ; motionData [ 219 ] = xx [ 281 ] ; motionData [ 220 ] =
xx [ 282 ] ; motionData [ 221 ] = xx [ 9 ] ; motionData [ 222 ] = xx [ 45 ] ;
motionData [ 223 ] = xx [ 39 ] ; motionData [ 224 ] = xx [ 41 ] ; motionData
[ 225 ] = xx [ 42 ] ; motionData [ 226 ] = xx [ 118 ] ; motionData [ 227 ] =
xx [ 119 ] ; motionData [ 228 ] = xx [ 287 ] ; motionData [ 229 ] = xx [ 120
] ; motionData [ 230 ] = xx [ 121 ] ; motionData [ 231 ] = xx [ 52 ] ;
motionData [ 232 ] = xx [ 292 ] ; motionData [ 233 ] = xx [ 293 ] ;
motionData [ 234 ] = xx [ 294 ] - state [ 20 ] ; motionData [ 235 ] = xx [
295 ] ; motionData [ 236 ] = xx [ 296 ] ; motionData [ 237 ] = xx [ 297 ] ;
motionData [ 238 ] = xx [ 298 ] ; motionData [ 239 ] = xx [ 299 ] ;
motionData [ 240 ] = xx [ 288 ] ; motionData [ 241 ] = xx [ 270 ] ;
motionData [ 242 ] = xx [ 44 ] ; motionData [ 243 ] = xx [ 73 ] ; motionData
[ 244 ] = xx [ 301 ] ; motionData [ 245 ] = xx [ 302 ] ; motionData [ 246 ] =
xx [ 303 ] - state [ 24 ] ; motionData [ 247 ] = xx [ 304 ] ; motionData [
248 ] = xx [ 305 ] ; motionData [ 249 ] = xx [ 306 ] ; motionData [ 250 ] =
xx [ 307 ] + 1.099786928193684e-18 * state [ 24 ] ; motionData [ 251 ] = xx [
308 ] ; motionData [ 252 ] = xx [ 271 ] ; motionData [ 253 ] = xx [ 278 ] ;
motionData [ 254 ] = xx [ 285 ] ; motionData [ 255 ] = xx [ 289 ] ;
motionData [ 256 ] = xx [ 290 ] ; motionData [ 257 ] = xx [ 175 ] ;
motionData [ 258 ] = xx [ 309 ] - state [ 28 ] ; motionData [ 259 ] = xx [
310 ] ; motionData [ 260 ] = xx [ 311 ] ; motionData [ 261 ] = xx [ 201 ] ;
motionData [ 262 ] = xx [ 202 ] ; motionData [ 263 ] = xx [ 203 ] ;
motionData [ 264 ] = xx [ 176 ] ; motionData [ 265 ] = xx [ 177 ] ;
motionData [ 266 ] = xx [ 183 ] ; motionData [ 267 ] = xx [ 193 ] ;
motionData [ 268 ] = xx [ 194 ] ; motionData [ 269 ] = xx [ 137 ] ;
motionData [ 270 ] = xx [ 145 ] - state [ 32 ] ; motionData [ 271 ] = xx [
146 ] ; motionData [ 272 ] = xx [ 147 ] ; motionData [ 273 ] = xx [ 153 ] ;
motionData [ 274 ] = xx [ 154 ] + 1.099786928193694e-18 * state [ 32 ] ;
motionData [ 275 ] = xx [ 155 ] ; motionData [ 276 ] = xx [ 181 ] ;
motionData [ 277 ] = xx [ 21 ] ; motionData [ 278 ] = xx [ 140 ] ; motionData
[ 279 ] = xx [ 233 ] ; motionData [ 280 ] = xx [ 234 ] ; motionData [ 281 ] =
xx [ 18 ] ; motionData [ 282 ] = xx [ 225 ] + state [ 36 ] ; motionData [ 283
] = xx [ 226 ] ; motionData [ 284 ] = xx [ 227 ] ; motionData [ 285 ] = xx [
249 ] ; motionData [ 286 ] = xx [ 250 ] - 1.09978692819368e-18 * state [ 36 ]
; motionData [ 287 ] = xx [ 251 ] ; } static size_t computeAssemblyError_0 (
const RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * motionData , double * error ) { const double *
rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts .
mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void )
modeVector ; ( void ) motionData ; ( void ) error ; return 0 ; } static
size_t computeAssemblyError_1 ( const RuntimeDerivedValuesBundle * rtdv ,
const double * state , const int * modeVector , const double * motionData ,
double * error ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const
int * rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; (
void ) state ; ( void ) modeVector ; ( void ) motionData ; ( void ) error ;
return 0 ; } static size_t computeAssemblyError_2 ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * motionData , double * error ) { const double *
rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts .
mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void )
modeVector ; ( void ) motionData ; ( void ) error ; return 0 ; } size_t
simple_robot_9eb3ef65_1_computeAssemblyError ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , size_t constraintIdx , const double *
state , const int * modeVector , const double * motionData , double * error )
{ ( void ) mech ; ( void ) rtdv ; ( void ) state ; ( void ) modeVector ; (
void ) motionData ; ( void ) error ; switch ( constraintIdx ) { case 0 :
return computeAssemblyError_0 ( rtdv , state , modeVector , motionData ,
error ) ; case 1 : return computeAssemblyError_1 ( rtdv , state , modeVector
, motionData , error ) ; case 2 : return computeAssemblyError_2 ( rtdv ,
state , modeVector , motionData , error ) ; } return 0 ; } static size_t
computeAssemblyJacobian_0 ( const RuntimeDerivedValuesBundle * rtdv , const
double * state , const int * modeVector , const double * motionData , double
* J ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi
= rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) state
; ( void ) modeVector ; ( void ) motionData ; ( void ) J ; return 0 ; }
static size_t computeAssemblyJacobian_1 ( const RuntimeDerivedValuesBundle *
rtdv , const double * state , const int * modeVector , const double *
motionData , double * J ) { const double * rtdvd = rtdv -> mDoubles . mValues
; const int * rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void )
rtdvi ; ( void ) state ; ( void ) modeVector ; ( void ) motionData ; ( void )
J ; return 0 ; } static size_t computeAssemblyJacobian_2 ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * motionData , double * J ) { const double * rtdvd
= rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ;
( void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void ) modeVector ; (
void ) motionData ; ( void ) J ; return 0 ; } size_t
simple_robot_9eb3ef65_1_computeAssemblyJacobian ( const void * mech , const
RuntimeDerivedValuesBundle * rtdv , size_t constraintIdx , boolean_T
forVelocitySatisfaction , const double * state , const int * modeVector ,
const double * motionData , double * J ) { ( void ) mech ; ( void ) rtdv ; (
void ) state ; ( void ) modeVector ; ( void ) forVelocitySatisfaction ; (
void ) motionData ; ( void ) J ; switch ( constraintIdx ) { case 0 : return
computeAssemblyJacobian_0 ( rtdv , state , modeVector , motionData , J ) ;
case 1 : return computeAssemblyJacobian_1 ( rtdv , state , modeVector ,
motionData , J ) ; case 2 : return computeAssemblyJacobian_2 ( rtdv , state ,
modeVector , motionData , J ) ; } return 0 ; } size_t
simple_robot_9eb3ef65_1_computeFullAssemblyJacobian ( const void * mech ,
const RuntimeDerivedValuesBundle * rtdv , const double * state , const int *
modeVector , const double * motionData , double * J ) { const double * rtdvd
= rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ;
( void ) mech ; ( void ) rtdvd ; ( void ) rtdvi ; ( void ) state ; ( void )
modeVector ; ( void ) motionData ; ( void ) J ; return 0 ; } static int
isInKinematicSingularity_0 ( const RuntimeDerivedValuesBundle * rtdv , const
int * modeVector , const double * motionData ) { const double * rtdvd = rtdv
-> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void
) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; ( void ) motionData ; return
0 ; } static int isInKinematicSingularity_1 ( const
RuntimeDerivedValuesBundle * rtdv , const int * modeVector , const double *
motionData ) { const double * rtdvd = rtdv -> mDoubles . mValues ; const int
* rtdvi = rtdv -> mInts . mValues ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) modeVector ; ( void ) motionData ; return 0 ; } static int
isInKinematicSingularity_2 ( const RuntimeDerivedValuesBundle * rtdv , const
int * modeVector , const double * motionData ) { const double * rtdvd = rtdv
-> mDoubles . mValues ; const int * rtdvi = rtdv -> mInts . mValues ; ( void
) rtdvd ; ( void ) rtdvi ; ( void ) modeVector ; ( void ) motionData ; return
0 ; } int simple_robot_9eb3ef65_1_isInKinematicSingularity ( const void *
mech , const RuntimeDerivedValuesBundle * rtdv , size_t constraintIdx , const
int * modeVector , const double * motionData ) { ( void ) mech ; ( void )
rtdv ; ( void ) modeVector ; ( void ) motionData ; switch ( constraintIdx ) {
case 0 : return isInKinematicSingularity_0 ( rtdv , modeVector , motionData )
; case 1 : return isInKinematicSingularity_1 ( rtdv , modeVector , motionData
) ; case 2 : return isInKinematicSingularity_2 ( rtdv , modeVector ,
motionData ) ; } return 0 ; } PmfMessageId
simple_robot_9eb3ef65_1_convertStateVector ( const void * asmMech , const
RuntimeDerivedValuesBundle * rtdv , const void * simMech , const double *
asmState , const int * asmModeVector , const int * simModeVector , double *
simState , void * neDiagMgr0 ) { const double * rtdvd = rtdv -> mDoubles .
mValues ; const int * rtdvi = rtdv -> mInts . mValues ; NeuDiagnosticManager
* neDiagMgr = ( NeuDiagnosticManager * ) neDiagMgr0 ; ( void ) asmMech ; (
void ) rtdvd ; ( void ) rtdvi ; ( void ) simMech ; ( void ) asmModeVector ; (
void ) simModeVector ; ( void ) neDiagMgr ; simState [ 0 ] = asmState [ 0 ] ;
simState [ 1 ] = asmState [ 1 ] ; simState [ 2 ] = asmState [ 2 ] ; simState
[ 3 ] = asmState [ 3 ] ; simState [ 4 ] = asmState [ 4 ] ; simState [ 5 ] =
asmState [ 5 ] ; simState [ 6 ] = asmState [ 6 ] ; simState [ 7 ] = asmState
[ 7 ] ; simState [ 8 ] = asmState [ 8 ] ; simState [ 9 ] = asmState [ 9 ] ;
simState [ 10 ] = asmState [ 10 ] ; simState [ 11 ] = asmState [ 11 ] ;
simState [ 12 ] = asmState [ 12 ] ; simState [ 13 ] = asmState [ 13 ] ;
simState [ 14 ] = asmState [ 14 ] ; simState [ 15 ] = asmState [ 15 ] ;
simState [ 16 ] = asmState [ 16 ] ; simState [ 17 ] = asmState [ 17 ] ;
simState [ 18 ] = asmState [ 18 ] ; simState [ 19 ] = asmState [ 19 ] ;
simState [ 20 ] = asmState [ 20 ] ; simState [ 21 ] = asmState [ 21 ] ;
simState [ 22 ] = asmState [ 22 ] ; simState [ 23 ] = asmState [ 23 ] ;
simState [ 24 ] = asmState [ 24 ] ; simState [ 25 ] = asmState [ 25 ] ;
simState [ 26 ] = asmState [ 26 ] ; simState [ 27 ] = asmState [ 27 ] ;
simState [ 28 ] = asmState [ 28 ] ; simState [ 29 ] = asmState [ 29 ] ;
simState [ 30 ] = asmState [ 30 ] ; simState [ 31 ] = asmState [ 31 ] ;
simState [ 32 ] = asmState [ 32 ] ; simState [ 33 ] = asmState [ 33 ] ;
simState [ 34 ] = asmState [ 34 ] ; simState [ 35 ] = asmState [ 35 ] ;
simState [ 36 ] = asmState [ 36 ] ; return NULL ; }
