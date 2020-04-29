#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "simple_robot_9eb3ef65_1_geometries.h"
PmfMessageId simple_robot_9eb3ef65_1_checkDynamics ( const
RuntimeDerivedValuesBundle * rtdv , const double * state , const double *
input , const double * inputDot , const double * inputDdot , const double *
discreteState , double * result , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 135 ] ; ( void ) rtdvd ; ( void ) rtdvi ; (
void ) input ; ( void ) inputDot ; ( void ) inputDdot ; ( void )
discreteState ; ( void ) neDiagMgr ; xx [ 0 ] = 0.9380730256354353 ; xx [ 1 ]
= state [ 3 ] ; xx [ 2 ] = state [ 4 ] ; xx [ 3 ] = state [ 5 ] ; xx [ 4 ] =
state [ 6 ] ; xx [ 5 ] = 0.6422553040725784 ; xx [ 6 ] = 0.295817721562857 ;
xx [ 7 ] = xx [ 5 ] ; xx [ 8 ] = xx [ 6 ] ; xx [ 9 ] = xx [ 5 ] ; xx [ 10 ] =
xx [ 6 ] ; pm_math_Quaternion_composeInverse_ra ( xx + 1 , xx + 7 , xx + 11 )
; xx [ 1 ] = 0.3464375825097215 ; xx [ 2 ] = 4.952999999999999e-3 ; xx [ 3 ]
= xx [ 2 ] * xx [ 13 ] ; xx [ 4 ] = xx [ 2 ] * xx [ 14 ] ; xx [ 5 ] = 2.0 ;
xx [ 6 ] = ( xx [ 13 ] * xx [ 3 ] + xx [ 14 ] * xx [ 4 ] ) * xx [ 5 ] ; xx [
15 ] = state [ 0 ] - xx [ 6 ] + xx [ 2 ] ; xx [ 16 ] = ( xx [ 11 ] * xx [ 4 ]
+ xx [ 12 ] * xx [ 3 ] ) * xx [ 5 ] ; xx [ 17 ] = state [ 1 ] + xx [ 16 ] ;
xx [ 18 ] = xx [ 5 ] * ( xx [ 11 ] * xx [ 3 ] - xx [ 12 ] * xx [ 4 ] ) ; xx [
3 ] = state [ 2 ] - xx [ 18 ] ; xx [ 19 ] = xx [ 0 ] * xx [ 11 ] + xx [ 1 ] *
xx [ 12 ] ; xx [ 20 ] = xx [ 0 ] * xx [ 12 ] - xx [ 1 ] * xx [ 11 ] ; xx [ 21
] = xx [ 0 ] * xx [ 13 ] - xx [ 1 ] * xx [ 14 ] ; xx [ 22 ] = xx [ 0 ] * xx [
14 ] + xx [ 1 ] * xx [ 13 ] ; xx [ 23 ] = xx [ 6 ] - xx [ 2 ] + xx [ 15 ] ;
xx [ 24 ] = xx [ 17 ] - xx [ 16 ] ; xx [ 25 ] = xx [ 18 ] + xx [ 3 ] ; xx [ 4
] = 0.0 ; xx [ 26 ] = 1.0 ; xx [ 27 ] = xx [ 4 ] ; xx [ 28 ] = xx [ 4 ] ; xx
[ 29 ] = xx [ 4 ] ; xx [ 30 ] = xx [ 4 ] ; xx [ 31 ] = xx [ 4 ] ; xx [ 32 ] =
xx [ 4 ] ; xx [ 6 ] = sm_core_compiler_computeProximityInfoCxpolyCxpoly (
simple_robot_9eb3ef65_1_geometry_0 ( rtdv ) ,
simple_robot_9eb3ef65_1_geometry_1 ( rtdv ) , ( pm_math_Transform3 * ) ( xx +
19 ) , ( pm_math_Transform3 * ) ( xx + 26 ) , ( pm_math_Vector3 * ) ( xx + 33
) , ( pm_math_Vector3 * ) ( xx + 36 ) , ( pm_math_Vector3 * ) ( xx + 39 ) , (
pm_math_Vector3 * ) ( xx + 42 ) ) ; xx [ 45 ] = xx [ 0 ] ; xx [ 46 ] = - xx [
1 ] ; xx [ 47 ] = xx [ 4 ] ; xx [ 48 ] = xx [ 4 ] ; xx [ 49 ] = - xx [ 2 ] ;
xx [ 50 ] = xx [ 4 ] ; xx [ 51 ] = xx [ 4 ] ; xx [ 52 ] = state [ 10 ] ; xx [
53 ] = state [ 11 ] ; xx [ 54 ] = state [ 12 ] ; pm_math_Quaternion_xform_ra
( xx + 7 , xx + 52 , xx + 55 ) ; xx [ 7 ] = state [ 7 ] ; xx [ 8 ] = state [
8 ] ; xx [ 9 ] = state [ 9 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 11 ,
xx + 7 , xx + 52 ) ; xx [ 7 ] = xx [ 53 ] + xx [ 2 ] * xx [ 57 ] ; xx [ 8 ] =
xx [ 54 ] - xx [ 2 ] * xx [ 56 ] ; xx [ 58 ] = xx [ 55 ] ; xx [ 59 ] = xx [
56 ] ; xx [ 60 ] = xx [ 57 ] ; xx [ 61 ] = xx [ 52 ] ; xx [ 62 ] = xx [ 7 ] ;
xx [ 63 ] = xx [ 8 ] ; xx [ 9 ] = 1.0e6 ; xx [ 10 ] = 1000.0 ; xx [ 16 ] =
1.0e-4 ; xx [ 18 ] = 0.3 ; xx [ 53 ] = 0.2119573811760597 ; xx [ 54 ] =
9.126024771145405e-4 ; sm_core_compiler_computeContactWrenches ( xx [ 6 ] ,
xx + 36 , xx + 33 , xx + 42 , xx + 39 , xx + 26 , xx + 45 , xx + 26 , xx + 19
, NULL , xx + 58 , 0 , 1 , xx [ 9 ] , xx [ 10 ] , xx [ 16 ] , xx [ 18 ] , xx
[ 53 ] , xx [ 54 ] , NULL , xx + 64 ) ; xx [ 6 ] = 5.02846706808957e-3 ; xx [
19 ] = 0.5 ; xx [ 20 ] = xx [ 19 ] * state [ 13 ] ; xx [ 21 ] = cos ( xx [ 20
] ) ; xx [ 22 ] = 0.9379742042928239 ; xx [ 23 ] = 0.02902651212139928 ; xx [
24 ] = sin ( xx [ 20 ] ) ; xx [ 20 ] = xx [ 23 ] * xx [ 24 ] ; xx [ 25 ] =
0.01361592839523582 ; xx [ 33 ] = 0.999578642025762 ; xx [ 34 ] = xx [ 33 ] *
xx [ 24 ] ; xx [ 24 ] = xx [ 6 ] * xx [ 21 ] + xx [ 22 ] * xx [ 20 ] - xx [
25 ] * xx [ 34 ] ; xx [ 35 ] = 0.3464010870279208 ; xx [ 36 ] = xx [ 35 ] *
xx [ 20 ] - ( xx [ 6 ] * xx [ 34 ] + xx [ 25 ] * xx [ 21 ] ) ; xx [ 37 ] = xx
[ 6 ] * xx [ 20 ] - xx [ 22 ] * xx [ 21 ] + xx [ 35 ] * xx [ 34 ] ; xx [ 38 ]
= xx [ 35 ] * xx [ 21 ] + xx [ 25 ] * xx [ 20 ] + xx [ 22 ] * xx [ 34 ] ; xx
[ 39 ] = - xx [ 24 ] ; xx [ 40 ] = xx [ 36 ] ; xx [ 41 ] = xx [ 37 ] ; xx [
42 ] = xx [ 38 ] ; pm_math_Quaternion_compose_ra ( xx + 11 , xx + 39 , xx +
58 ) ; xx [ 20 ] = 0.6271461567375358 ; xx [ 21 ] = 0.3835306436794506 ; xx [
25 ] = 0.3186539101717888 ; xx [ 34 ] = 0.5983741546728057 ; xx [ 70 ] = - xx
[ 20 ] ; xx [ 71 ] = - xx [ 21 ] ; xx [ 72 ] = - xx [ 25 ] ; xx [ 73 ] = xx [
34 ] ; xx [ 35 ] = xx [ 19 ] * state [ 15 ] ; xx [ 43 ] = 0.09547264031652658
; xx [ 44 ] = sin ( xx [ 35 ] ) ; xx [ 62 ] = 0.9948427668622 ; xx [ 63 ] =
0.03424681259555507 ; xx [ 74 ] = cos ( xx [ 35 ] ) ; xx [ 75 ] = xx [ 43 ] *
xx [ 44 ] ; xx [ 76 ] = xx [ 62 ] * xx [ 44 ] ; xx [ 77 ] = - ( xx [ 63 ] *
xx [ 44 ] ) ; pm_math_Quaternion_compose_ra ( xx + 70 , xx + 74 , xx + 78 ) ;
pm_math_Quaternion_compose_ra ( xx + 58 , xx + 78 , xx + 70 ) ; xx [ 35 ] =
0.5799181594307252 ; xx [ 44 ] = xx [ 19 ] * state [ 25 ] ; xx [ 74 ] = sin (
xx [ 44 ] ) ; xx [ 75 ] = xx [ 33 ] * xx [ 74 ] ; xx [ 76 ] =
0.6442506183571239 ; xx [ 77 ] = xx [ 23 ] * xx [ 74 ] ; xx [ 74 ] =
0.3296521996834055 ; xx [ 82 ] = cos ( xx [ 44 ] ) ; xx [ 44 ] = xx [ 35 ] *
xx [ 75 ] + xx [ 76 ] * xx [ 77 ] - xx [ 74 ] * xx [ 82 ] ; xx [ 83 ] =
0.3741196283982388 ; xx [ 84 ] = xx [ 74 ] * xx [ 75 ] + xx [ 35 ] * xx [ 82
] - xx [ 83 ] * xx [ 77 ] ; xx [ 85 ] = xx [ 74 ] * xx [ 77 ] + xx [ 76 ] *
xx [ 82 ] + xx [ 83 ] * xx [ 75 ] ; xx [ 86 ] = - xx [ 85 ] ; xx [ 87 ] = xx
[ 83 ] * xx [ 82 ] + xx [ 35 ] * xx [ 77 ] - xx [ 76 ] * xx [ 75 ] ; xx [ 88
] = xx [ 44 ] ; xx [ 89 ] = xx [ 84 ] ; xx [ 90 ] = xx [ 86 ] ; xx [ 91 ] =
xx [ 87 ] ; pm_math_Quaternion_compose_ra ( xx + 70 , xx + 88 , xx + 92 ) ;
xx [ 75 ] = 0.9379742042928237 ; xx [ 77 ] = xx [ 19 ] * state [ 27 ] ; xx [
82 ] = cos ( xx [ 77 ] ) ; xx [ 96 ] = 0.3464010870279209 ; xx [ 97 ] = sin (
xx [ 77 ] ) ; xx [ 77 ] = xx [ 75 ] * xx [ 82 ] + xx [ 96 ] * xx [ 97 ] ; xx
[ 98 ] = xx [ 75 ] * xx [ 97 ] - xx [ 96 ] * xx [ 82 ] ; xx [ 75 ] =
0.01361592839523579 ; xx [ 96 ] = xx [ 6 ] * xx [ 82 ] - xx [ 75 ] * xx [ 97
] ; xx [ 99 ] = xx [ 75 ] * xx [ 82 ] + xx [ 6 ] * xx [ 97 ] ; xx [ 100 ] = -
xx [ 77 ] ; xx [ 101 ] = xx [ 98 ] ; xx [ 102 ] = xx [ 96 ] ; xx [ 103 ] = xx
[ 99 ] ; pm_math_Quaternion_compose_ra ( xx + 92 , xx + 100 , xx + 104 ) ; xx
[ 82 ] = xx [ 2 ] * xx [ 106 ] ; xx [ 97 ] = xx [ 2 ] * xx [ 107 ] ; xx [ 108
] = 0.02135192530809139 ; xx [ 109 ] = 4.953000000000018e-3 ; xx [ 110 ] = xx
[ 109 ] * xx [ 96 ] ; xx [ 111 ] = xx [ 99 ] * xx [ 109 ] ; xx [ 112 ] =
0.05562318494169338 ; xx [ 113 ] = xx [ 108 ] - ( xx [ 109 ] - ( xx [ 110 ] *
xx [ 96 ] + xx [ 99 ] * xx [ 111 ] ) * xx [ 5 ] ) ; xx [ 114 ] = - ( xx [ 112
] + ( xx [ 110 ] * xx [ 98 ] - xx [ 77 ] * xx [ 111 ] ) * xx [ 5 ] ) ; xx [
115 ] = - ( xx [ 5 ] * ( xx [ 111 ] * xx [ 98 ] + xx [ 77 ] * xx [ 110 ] ) )
; pm_math_Quaternion_xform_ra ( xx + 92 , xx + 113 , xx + 109 ) ; xx [ 77 ] =
4.244406610027125e-3 ; xx [ 92 ] = xx [ 84 ] ; xx [ 93 ] = xx [ 86 ] ; xx [
94 ] = xx [ 87 ] ; xx [ 86 ] = 0.06404981182865323 ; xx [ 95 ] = xx [ 86 ] *
xx [ 87 ] ; xx [ 96 ] = xx [ 77 ] * xx [ 87 ] ; xx [ 87 ] = xx [ 86 ] * xx [
84 ] + xx [ 85 ] * xx [ 77 ] ; xx [ 116 ] = - xx [ 95 ] ; xx [ 117 ] = xx [
96 ] ; xx [ 118 ] = xx [ 87 ] ; pm_math_Vector3_cross_ra ( xx + 92 , xx + 116
, xx + 119 ) ; xx [ 92 ] = 0.01262769948857074 - ( xx [ 77 ] + xx [ 5 ] * (
xx [ 119 ] - xx [ 95 ] * xx [ 44 ] ) ) ; xx [ 93 ] = - ( 0.1408927034847847 +
xx [ 86 ] + ( xx [ 96 ] * xx [ 44 ] + xx [ 120 ] ) * xx [ 5 ] ) ; xx [ 94 ] =
0.09068544496193037 - ( xx [ 87 ] * xx [ 44 ] + xx [ 121 ] ) * xx [ 5 ] ;
pm_math_Quaternion_xform_ra ( xx + 70 , xx + 92 , xx + 84 ) ; xx [ 116 ] =
0.03736275114177646 ; xx [ 117 ] = 0.1168511605538739 ; xx [ 118 ] =
0.08181278075467377 ; pm_math_Quaternion_xform_ra ( xx + 78 , xx + 116 , xx +
119 ) ; xx [ 116 ] = 4.244406610027192e-3 - xx [ 119 ] ; xx [ 117 ] =
0.06404981182865317 - xx [ 120 ] ; xx [ 118 ] = - xx [ 121 ] ;
pm_math_Quaternion_xform_ra ( xx + 58 , xx + 116 , xx + 119 ) ; xx [ 44 ] =
xx [ 112 ] * xx [ 38 ] ; xx [ 58 ] = xx [ 108 ] * xx [ 38 ] ; xx [ 59 ] = xx
[ 112 ] * xx [ 36 ] + xx [ 37 ] * xx [ 108 ] ; xx [ 122 ] = xx [ 44 ] ; xx [
123 ] = xx [ 58 ] ; xx [ 124 ] = - xx [ 59 ] ; pm_math_Vector3_cross_ra ( xx
+ 36 , xx + 122 , xx + 125 ) ; xx [ 36 ] = - ( xx [ 2 ] + xx [ 108 ] + xx [ 5
] * ( xx [ 125 ] - xx [ 44 ] * xx [ 24 ] ) ) ; xx [ 37 ] = - ( ( xx [ 126 ] -
xx [ 58 ] * xx [ 24 ] ) * xx [ 5 ] - xx [ 112 ] ) ; xx [ 38 ] = - ( ( xx [ 24
] * xx [ 59 ] + xx [ 127 ] ) * xx [ 5 ] ) ; pm_math_Quaternion_xform_ra ( xx
+ 11 , xx + 36 , xx + 58 ) ; xx [ 11 ] = xx [ 119 ] + xx [ 58 ] + xx [ 15 ] ;
xx [ 12 ] = xx [ 120 ] + xx [ 59 ] + xx [ 17 ] ; xx [ 13 ] = xx [ 121 ] + xx
[ 60 ] + xx [ 3 ] ; xx [ 119 ] = xx [ 0 ] * xx [ 104 ] + xx [ 1 ] * xx [ 105
] ; xx [ 120 ] = xx [ 0 ] * xx [ 105 ] - xx [ 1 ] * xx [ 104 ] ; xx [ 121 ] =
xx [ 0 ] * xx [ 106 ] - xx [ 1 ] * xx [ 107 ] ; xx [ 122 ] = xx [ 0 ] * xx [
107 ] + xx [ 1 ] * xx [ 106 ] ; xx [ 123 ] = ( xx [ 106 ] * xx [ 82 ] + xx [
107 ] * xx [ 97 ] ) * xx [ 5 ] - xx [ 2 ] + xx [ 109 ] + xx [ 84 ] + xx [ 11
] ; xx [ 124 ] = xx [ 110 ] + xx [ 85 ] + xx [ 12 ] - ( xx [ 104 ] * xx [ 97
] + xx [ 105 ] * xx [ 82 ] ) * xx [ 5 ] ; xx [ 125 ] = xx [ 5 ] * ( xx [ 104
] * xx [ 82 ] - xx [ 105 ] * xx [ 97 ] ) + xx [ 111 ] + xx [ 86 ] + xx [ 13 ]
; xx [ 3 ] = sm_core_compiler_computeProximityInfoCxpolyCxpoly (
simple_robot_9eb3ef65_1_geometry_0 ( rtdv ) ,
simple_robot_9eb3ef65_1_geometry_1 ( rtdv ) , ( pm_math_Transform3 * ) ( xx +
119 ) , ( pm_math_Transform3 * ) ( xx + 26 ) , ( pm_math_Vector3 * ) ( xx +
58 ) , ( pm_math_Vector3 * ) ( xx + 84 ) , ( pm_math_Vector3 * ) ( xx + 95 )
, ( pm_math_Vector3 * ) ( xx + 104 ) ) ; pm_math_Quaternion_inverseXform_ra (
xx + 39 , xx + 55 , xx + 109 ) ; xx [ 126 ] = xx [ 109 ] + xx [ 33 ] * state
[ 14 ] ; xx [ 127 ] = xx [ 110 ] - xx [ 23 ] * state [ 14 ] ; xx [ 128 ] = xx
[ 111 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 78 , xx + 126 , xx + 109
) ; xx [ 129 ] = xx [ 109 ] + xx [ 43 ] * state [ 16 ] ; xx [ 130 ] = xx [
110 ] + xx [ 62 ] * state [ 16 ] ; xx [ 131 ] = xx [ 111 ] - xx [ 63 ] *
state [ 16 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 88 , xx + 129 , xx +
61 ) ; xx [ 109 ] = xx [ 61 ] - xx [ 33 ] * state [ 26 ] ; xx [ 110 ] = xx [
62 ] + xx [ 23 ] * state [ 26 ] ; xx [ 111 ] = xx [ 63 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 100 , xx + 109 , xx + 61 ) ;
pm_math_Vector3_cross_ra ( xx + 109 , xx + 113 , xx + 132 ) ;
pm_math_Vector3_cross_ra ( xx + 129 , xx + 92 , xx + 109 ) ;
pm_math_Vector3_cross_ra ( xx + 126 , xx + 116 , xx + 92 ) ;
pm_math_Vector3_cross_ra ( xx + 55 , xx + 36 , xx + 112 ) ; xx [ 36 ] = xx [
112 ] + xx [ 52 ] ; xx [ 37 ] = xx [ 113 ] + xx [ 7 ] ; xx [ 38 ] = xx [ 114
] + xx [ 8 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 39 , xx + 36 , xx +
55 ) ; xx [ 36 ] = xx [ 92 ] + xx [ 55 ] ; xx [ 37 ] = xx [ 93 ] + xx [ 56 ]
; xx [ 38 ] = xx [ 94 ] + xx [ 57 ] + 0.05497997575039515 * state [ 14 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 78 , xx + 36 , xx + 39 ) ; xx [ 7 ]
= xx [ 39 ] - 0.08539263296773184 * state [ 16 ] ; xx [ 8 ] = xx [ 40 ] +
9.090437326692596e-3 * state [ 16 ] ; xx [ 14 ] = xx [ 41 ] +
0.02601397390134001 * state [ 16 ] ; xx [ 36 ] = xx [ 109 ] + xx [ 7 ] ; xx [
37 ] = xx [ 110 ] + xx [ 8 ] ; xx [ 38 ] = xx [ 111 ] + xx [ 14 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 88 , xx + 36 , xx + 39 ) ; xx [ 36
] = xx [ 132 ] + xx [ 39 ] ; xx [ 37 ] = xx [ 133 ] + xx [ 40 ] ; xx [ 38 ] =
xx [ 134 ] + xx [ 41 ] + 0.06414602424960487 * state [ 26 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 100 , xx + 36 , xx + 39 ) ; xx [ 77
] = xx [ 61 ] - state [ 28 ] ; xx [ 78 ] = xx [ 62 ] ; xx [ 79 ] = xx [ 63 ]
; xx [ 80 ] = xx [ 39 ] ; xx [ 81 ] = xx [ 40 ] ; xx [ 82 ] = xx [ 41 ] ;
sm_core_compiler_computeContactWrenches ( xx [ 3 ] , xx + 84 , xx + 58 , xx +
104 , xx + 95 , xx + 26 , xx + 45 , xx + 26 , xx + 119 , NULL , xx + 77 , 0 ,
1 , xx [ 9 ] , xx [ 10 ] , xx [ 16 ] , xx [ 18 ] , xx [ 53 ] , xx [ 54 ] ,
NULL , xx + 36 ) ; xx [ 3 ] = xx [ 19 ] * state [ 29 ] ; xx [ 15 ] = sin ( xx
[ 3 ] ) ; xx [ 17 ] = xx [ 33 ] * xx [ 15 ] ; xx [ 24 ] = xx [ 23 ] * xx [ 15
] ; xx [ 15 ] = cos ( xx [ 3 ] ) ; xx [ 3 ] = xx [ 35 ] * xx [ 17 ] + xx [ 76
] * xx [ 24 ] - xx [ 74 ] * xx [ 15 ] ; xx [ 42 ] = xx [ 74 ] * xx [ 17 ] +
xx [ 35 ] * xx [ 15 ] - xx [ 83 ] * xx [ 24 ] ; xx [ 43 ] = xx [ 74 ] * xx [
24 ] + xx [ 76 ] * xx [ 15 ] + xx [ 83 ] * xx [ 17 ] ; xx [ 44 ] = - xx [ 43
] ; xx [ 52 ] = xx [ 83 ] * xx [ 15 ] + xx [ 35 ] * xx [ 24 ] - xx [ 76 ] *
xx [ 17 ] ; xx [ 55 ] = xx [ 3 ] ; xx [ 56 ] = xx [ 42 ] ; xx [ 57 ] = xx [
44 ] ; xx [ 58 ] = xx [ 52 ] ; pm_math_Quaternion_compose_ra ( xx + 70 , xx +
55 , xx + 59 ) ; xx [ 15 ] = 0.9379742042928236 ; xx [ 17 ] = xx [ 19 ] *
state [ 31 ] ; xx [ 24 ] = cos ( xx [ 17 ] ) ; xx [ 35 ] = 0.3464010870279211
; xx [ 63 ] = sin ( xx [ 17 ] ) ; xx [ 17 ] = xx [ 15 ] * xx [ 24 ] + xx [ 35
] * xx [ 63 ] ; xx [ 74 ] = xx [ 15 ] * xx [ 63 ] - xx [ 35 ] * xx [ 24 ] ;
xx [ 15 ] = 5.028467068089626e-3 ; xx [ 35 ] = 0.01361592839523584 ; xx [ 76
] = xx [ 15 ] * xx [ 24 ] - xx [ 35 ] * xx [ 63 ] ; xx [ 77 ] = xx [ 35 ] *
xx [ 24 ] + xx [ 15 ] * xx [ 63 ] ; xx [ 78 ] = - xx [ 17 ] ; xx [ 79 ] = xx
[ 74 ] ; xx [ 80 ] = xx [ 76 ] ; xx [ 81 ] = xx [ 77 ] ;
pm_math_Quaternion_compose_ra ( xx + 59 , xx + 78 , xx + 82 ) ; xx [ 15 ] =
xx [ 2 ] * xx [ 84 ] ; xx [ 24 ] = xx [ 2 ] * xx [ 85 ] ; xx [ 35 ] =
4.953000000000063e-3 ; xx [ 63 ] = xx [ 35 ] * xx [ 76 ] ; xx [ 86 ] = xx [
77 ] * xx [ 35 ] ; xx [ 87 ] = xx [ 108 ] - ( xx [ 35 ] - ( xx [ 63 ] * xx [
76 ] + xx [ 77 ] * xx [ 86 ] ) * xx [ 5 ] ) ; xx [ 88 ] = - (
0.05562318494169332 + ( xx [ 63 ] * xx [ 74 ] - xx [ 17 ] * xx [ 86 ] ) * xx
[ 5 ] ) ; xx [ 89 ] = - ( xx [ 5 ] * ( xx [ 86 ] * xx [ 74 ] + xx [ 17 ] * xx
[ 63 ] ) ) ; pm_math_Quaternion_xform_ra ( xx + 59 , xx + 87 , xx + 90 ) ; xx
[ 17 ] = 4.244406610027171e-3 ; xx [ 59 ] = xx [ 42 ] ; xx [ 60 ] = xx [ 44 ]
; xx [ 61 ] = xx [ 52 ] ; xx [ 35 ] = 0.06404981182865327 ; xx [ 44 ] = xx [
35 ] * xx [ 52 ] ; xx [ 62 ] = xx [ 17 ] * xx [ 52 ] ; xx [ 52 ] = xx [ 35 ]
* xx [ 42 ] + xx [ 43 ] * xx [ 17 ] ; xx [ 93 ] = - xx [ 44 ] ; xx [ 94 ] =
xx [ 62 ] ; xx [ 95 ] = xx [ 52 ] ; pm_math_Vector3_cross_ra ( xx + 59 , xx +
93 , xx + 96 ) ; xx [ 59 ] = - ( 0.09519342531851044 + xx [ 17 ] + xx [ 5 ] *
( xx [ 96 ] - xx [ 44 ] * xx [ 3 ] ) ) ; xx [ 60 ] = - ( 0.136912200903329 +
xx [ 35 ] + ( xx [ 62 ] * xx [ 3 ] + xx [ 97 ] ) * xx [ 5 ] ) ; xx [ 61 ] = -
( 0.09426587708939985 + ( xx [ 52 ] * xx [ 3 ] + xx [ 98 ] ) * xx [ 5 ] ) ;
pm_math_Quaternion_xform_ra ( xx + 70 , xx + 59 , xx + 42 ) ; xx [ 93 ] = xx
[ 0 ] * xx [ 82 ] + xx [ 1 ] * xx [ 83 ] ; xx [ 94 ] = xx [ 0 ] * xx [ 83 ] -
xx [ 1 ] * xx [ 82 ] ; xx [ 95 ] = xx [ 0 ] * xx [ 84 ] - xx [ 1 ] * xx [ 85
] ; xx [ 96 ] = xx [ 0 ] * xx [ 85 ] + xx [ 1 ] * xx [ 84 ] ; xx [ 97 ] = (
xx [ 84 ] * xx [ 15 ] + xx [ 85 ] * xx [ 24 ] ) * xx [ 5 ] - xx [ 2 ] + xx [
90 ] + xx [ 42 ] + xx [ 11 ] ; xx [ 98 ] = xx [ 91 ] + xx [ 43 ] + xx [ 12 ]
- ( xx [ 82 ] * xx [ 24 ] + xx [ 83 ] * xx [ 15 ] ) * xx [ 5 ] ; xx [ 99 ] =
xx [ 5 ] * ( xx [ 82 ] * xx [ 15 ] - xx [ 83 ] * xx [ 24 ] ) + xx [ 92 ] + xx
[ 44 ] + xx [ 13 ] ; xx [ 3 ] =
sm_core_compiler_computeProximityInfoCxpolyCxpoly (
simple_robot_9eb3ef65_1_geometry_0 ( rtdv ) ,
simple_robot_9eb3ef65_1_geometry_1 ( rtdv ) , ( pm_math_Transform3 * ) ( xx +
93 ) , ( pm_math_Transform3 * ) ( xx + 26 ) , ( pm_math_Vector3 * ) ( xx + 42
) , ( pm_math_Vector3 * ) ( xx + 82 ) , ( pm_math_Vector3 * ) ( xx + 90 ) , (
pm_math_Vector3 * ) ( xx + 100 ) ) ; pm_math_Quaternion_inverseXform_ra ( xx
+ 55 , xx + 129 , xx + 103 ) ; xx [ 109 ] = xx [ 103 ] - xx [ 33 ] * state [
30 ] ; xx [ 110 ] = xx [ 104 ] + xx [ 23 ] * state [ 30 ] ; xx [ 111 ] = xx [
105 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 78 , xx + 109 , xx + 103 )
; pm_math_Vector3_cross_ra ( xx + 109 , xx + 87 , xx + 112 ) ;
pm_math_Vector3_cross_ra ( xx + 129 , xx + 59 , xx + 85 ) ; xx [ 59 ] = xx [
85 ] + xx [ 7 ] ; xx [ 60 ] = xx [ 86 ] + xx [ 8 ] ; xx [ 61 ] = xx [ 87 ] +
xx [ 14 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 55 , xx + 59 , xx + 85
) ; xx [ 55 ] = xx [ 112 ] + xx [ 85 ] ; xx [ 56 ] = xx [ 113 ] + xx [ 86 ] ;
xx [ 57 ] = xx [ 114 ] + xx [ 87 ] + 0.06414602424960492 * state [ 30 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 78 , xx + 55 , xx + 58 ) ; xx [ 76
] = xx [ 103 ] - state [ 32 ] ; xx [ 77 ] = xx [ 104 ] ; xx [ 78 ] = xx [ 105
] ; xx [ 79 ] = xx [ 58 ] ; xx [ 80 ] = xx [ 59 ] + 1.099786928193694e-18 *
state [ 32 ] ; xx [ 81 ] = xx [ 60 ] ;
sm_core_compiler_computeContactWrenches ( xx [ 3 ] , xx + 82 , xx + 42 , xx +
100 , xx + 90 , xx + 26 , xx + 45 , xx + 26 , xx + 93 , NULL , xx + 76 , 0 ,
1 , xx [ 9 ] , xx [ 10 ] , xx [ 16 ] , xx [ 18 ] , xx [ 53 ] , xx [ 54 ] ,
NULL , xx + 55 ) ; xx [ 3 ] = xx [ 19 ] * state [ 33 ] ; xx [ 15 ] = cos ( xx
[ 3 ] ) ; xx [ 17 ] = sin ( xx [ 3 ] ) ; xx [ 3 ] = xx [ 23 ] * xx [ 17 ] ;
xx [ 24 ] = xx [ 33 ] * xx [ 17 ] ; xx [ 17 ] = xx [ 20 ] * xx [ 15 ] + xx [
25 ] * xx [ 3 ] - xx [ 21 ] * xx [ 24 ] ; xx [ 35 ] = xx [ 20 ] * xx [ 24 ] +
xx [ 21 ] * xx [ 15 ] + xx [ 34 ] * xx [ 3 ] ; xx [ 42 ] = xx [ 25 ] * xx [
15 ] - xx [ 20 ] * xx [ 3 ] + xx [ 34 ] * xx [ 24 ] ; xx [ 20 ] = xx [ 21 ] *
xx [ 3 ] - xx [ 34 ] * xx [ 15 ] + xx [ 25 ] * xx [ 24 ] ; xx [ 76 ] = - xx [
17 ] ; xx [ 77 ] = xx [ 35 ] ; xx [ 78 ] = xx [ 42 ] ; xx [ 79 ] = xx [ 20 ]
; pm_math_Quaternion_compose_ra ( xx + 70 , xx + 76 , xx + 80 ) ; xx [ 3 ] =
xx [ 19 ] * state [ 35 ] ; xx [ 15 ] = cos ( xx [ 3 ] ) ; xx [ 19 ] = sin (
xx [ 3 ] ) ; xx [ 3 ] = xx [ 6 ] * xx [ 15 ] + xx [ 75 ] * xx [ 19 ] ; xx [
21 ] = xx [ 75 ] * xx [ 15 ] - xx [ 6 ] * xx [ 19 ] ; xx [ 6 ] =
0.3464010870279207 ; xx [ 24 ] = xx [ 22 ] * xx [ 15 ] - xx [ 6 ] * xx [ 19 ]
; xx [ 25 ] = xx [ 6 ] * xx [ 15 ] + xx [ 22 ] * xx [ 19 ] ; xx [ 84 ] = - xx
[ 3 ] ; xx [ 85 ] = xx [ 21 ] ; xx [ 86 ] = xx [ 24 ] ; xx [ 87 ] = - xx [ 25
] ; pm_math_Quaternion_compose_ra ( xx + 80 , xx + 84 , xx + 88 ) ; xx [ 6 ]
= xx [ 2 ] * xx [ 90 ] ; xx [ 15 ] = xx [ 2 ] * xx [ 91 ] ; xx [ 19 ] = xx [
24 ] * xx [ 2 ] ; xx [ 22 ] = xx [ 2 ] * xx [ 25 ] ; xx [ 61 ] = xx [ 108 ] -
( ( xx [ 24 ] * xx [ 19 ] + xx [ 22 ] * xx [ 25 ] ) * xx [ 5 ] - xx [ 2 ] ) ;
xx [ 62 ] = ( xx [ 22 ] * xx [ 3 ] + xx [ 21 ] * xx [ 19 ] ) * xx [ 5 ] -
0.05562318494169335 ; xx [ 63 ] = - ( xx [ 5 ] * ( xx [ 21 ] * xx [ 22 ] - xx
[ 19 ] * xx [ 3 ] ) ) ; pm_math_Quaternion_xform_ra ( xx + 80 , xx + 61 , xx
+ 92 ) ; xx [ 3 ] = 4.244406610027172e-3 ; xx [ 80 ] = xx [ 35 ] ; xx [ 81 ]
= xx [ 42 ] ; xx [ 82 ] = xx [ 20 ] ; xx [ 19 ] = 0.06404981182865328 ; xx [
21 ] = xx [ 19 ] * xx [ 20 ] ; xx [ 22 ] = xx [ 3 ] * xx [ 20 ] ; xx [ 20 ] =
xx [ 19 ] * xx [ 35 ] - xx [ 42 ] * xx [ 3 ] ; xx [ 42 ] = - xx [ 21 ] ; xx [
43 ] = xx [ 22 ] ; xx [ 44 ] = xx [ 20 ] ; pm_math_Vector3_cross_ra ( xx + 80
, xx + 42 , xx + 95 ) ; xx [ 42 ] = - ( 0.07045837366530477 + xx [ 3 ] + xx [
5 ] * ( xx [ 95 ] + xx [ 21 ] * xx [ 17 ] ) ) ; xx [ 43 ] =
0.1208316631353296 - ( xx [ 19 ] + ( xx [ 96 ] - xx [ 22 ] * xx [ 17 ] ) * xx
[ 5 ] ) ; xx [ 44 ] = - ( 0.1031385412966565 + ( xx [ 97 ] - xx [ 20 ] * xx [
17 ] ) * xx [ 5 ] ) ; pm_math_Quaternion_xform_ra ( xx + 70 , xx + 42 , xx +
19 ) ; xx [ 95 ] = xx [ 0 ] * xx [ 88 ] + xx [ 1 ] * xx [ 89 ] ; xx [ 96 ] =
xx [ 0 ] * xx [ 89 ] - xx [ 1 ] * xx [ 88 ] ; xx [ 97 ] = xx [ 0 ] * xx [ 90
] - xx [ 1 ] * xx [ 91 ] ; xx [ 98 ] = xx [ 0 ] * xx [ 91 ] + xx [ 1 ] * xx [
90 ] ; xx [ 99 ] = ( xx [ 90 ] * xx [ 6 ] + xx [ 91 ] * xx [ 15 ] ) * xx [ 5
] - xx [ 2 ] + xx [ 92 ] + xx [ 19 ] + xx [ 11 ] ; xx [ 100 ] = xx [ 93 ] +
xx [ 20 ] + xx [ 12 ] - ( xx [ 88 ] * xx [ 15 ] + xx [ 89 ] * xx [ 6 ] ) * xx
[ 5 ] ; xx [ 101 ] = xx [ 5 ] * ( xx [ 88 ] * xx [ 6 ] - xx [ 89 ] * xx [ 15
] ) + xx [ 94 ] + xx [ 21 ] + xx [ 13 ] ; xx [ 0 ] =
sm_core_compiler_computeProximityInfoCxpolyCxpoly (
simple_robot_9eb3ef65_1_geometry_0 ( rtdv ) ,
simple_robot_9eb3ef65_1_geometry_1 ( rtdv ) , ( pm_math_Transform3 * ) ( xx +
95 ) , ( pm_math_Transform3 * ) ( xx + 26 ) , ( pm_math_Vector3 * ) ( xx + 1
) , ( pm_math_Vector3 * ) ( xx + 11 ) , ( pm_math_Vector3 * ) ( xx + 19 ) , (
pm_math_Vector3 * ) ( xx + 70 ) ) ; pm_math_Quaternion_inverseXform_ra ( xx +
76 , xx + 129 , xx + 73 ) ; xx [ 80 ] = xx [ 73 ] - xx [ 33 ] * state [ 34 ]
; xx [ 81 ] = xx [ 74 ] + xx [ 23 ] * state [ 34 ] ; xx [ 82 ] = xx [ 75 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 84 , xx + 80 , xx + 22 ) ;
pm_math_Vector3_cross_ra ( xx + 80 , xx + 61 , xx + 33 ) ;
pm_math_Vector3_cross_ra ( xx + 129 , xx + 42 , xx + 61 ) ; xx [ 42 ] = xx [
61 ] + xx [ 7 ] ; xx [ 43 ] = xx [ 62 ] + xx [ 8 ] ; xx [ 44 ] = xx [ 63 ] +
xx [ 14 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 76 , xx + 42 , xx + 5 )
; xx [ 42 ] = xx [ 33 ] + xx [ 5 ] ; xx [ 43 ] = xx [ 34 ] + xx [ 6 ] ; xx [
44 ] = xx [ 35 ] + xx [ 7 ] + 0.06414602424960493 * state [ 34 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 84 , xx + 42 , xx + 5 ) ; xx [ 73 ]
= xx [ 22 ] + state [ 36 ] ; xx [ 74 ] = xx [ 23 ] ; xx [ 75 ] = xx [ 24 ] ;
xx [ 76 ] = xx [ 5 ] ; xx [ 77 ] = xx [ 6 ] - 1.09978692819368e-18 * state [
36 ] ; xx [ 78 ] = xx [ 7 ] ; sm_core_compiler_computeContactWrenches ( xx [
0 ] , xx + 11 , xx + 1 , xx + 70 , xx + 19 , xx + 26 , xx + 45 , xx + 26 , xx
+ 95 , NULL , xx + 73 , 0 , 1 , xx [ 9 ] , xx [ 10 ] , xx [ 16 ] , xx [ 18 ]
, xx [ 53 ] , xx [ 54 ] , NULL , xx + 79 ) ; xx [ 0 ] = xx [ 64 ] + xx [ 36 ]
+ xx [ 55 ] + xx [ 79 ] ; xx [ 1 ] = xx [ 65 ] + xx [ 37 ] + xx [ 56 ] + xx [
80 ] ; xx [ 2 ] = xx [ 66 ] + xx [ 38 ] + xx [ 57 ] + xx [ 81 ] ; xx [ 3 ] =
xx [ 67 ] + xx [ 39 ] + xx [ 58 ] + xx [ 82 ] ; xx [ 4 ] = xx [ 68 ] + xx [
40 ] + xx [ 59 ] + xx [ 83 ] ; xx [ 5 ] = xx [ 69 ] + xx [ 41 ] + xx [ 60 ] +
xx [ 84 ] ; result [ 0 ] = xx [ 0 ] * xx [ 0 ] + xx [ 1 ] * xx [ 1 ] + xx [ 2
] * xx [ 2 ] + xx [ 3 ] * xx [ 3 ] + xx [ 4 ] * xx [ 4 ] + xx [ 5 ] * xx [ 5
] ; return NULL ; }
