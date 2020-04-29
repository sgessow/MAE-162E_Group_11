#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "simple_robot_9eb3ef65_1_geometries.h"
PmfMessageId simple_robot_9eb3ef65_1_compDerivs ( const
RuntimeDerivedValuesBundle * rtdv , const int * eqnEnableFlags , const double
* state , const int * modeVector , const double * input , const double *
inputDot , const double * inputDdot , const double * discreteState , double *
deriv , double * errorResult , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; int ii [ 2 ] ; double xx [ 737 ] ; ( void ) rtdvd ; ( void
) rtdvi ; ( void ) eqnEnableFlags ; ( void ) modeVector ; ( void ) input ; (
void ) inputDot ; ( void ) inputDdot ; ( void ) discreteState ; ( void )
neDiagMgr ; xx [ 0 ] = state [ 3 ] ; xx [ 1 ] = state [ 4 ] ; xx [ 2 ] =
state [ 5 ] ; xx [ 3 ] = state [ 6 ] ; xx [ 4 ] = state [ 10 ] ; xx [ 5 ] =
state [ 11 ] ; xx [ 6 ] = state [ 12 ] ; pm_math_Quaternion_compDeriv_ra ( xx
+ 0 , xx + 4 , xx + 7 ) ; xx [ 11 ] = 1.0 ; xx [ 12 ] = 0.6422553040725784 ;
xx [ 13 ] = 0.295817721562857 ; xx [ 14 ] = xx [ 12 ] ; xx [ 15 ] = xx [ 13 ]
; xx [ 16 ] = xx [ 12 ] ; xx [ 17 ] = xx [ 13 ] ;
pm_math_Quaternion_composeInverse_ra ( xx + 0 , xx + 14 , xx + 18 ) ; xx [ 0
] = xx [ 20 ] * xx [ 20 ] ; xx [ 1 ] = xx [ 21 ] * xx [ 21 ] ; xx [ 2 ] = 2.0
; xx [ 3 ] = xx [ 11 ] - ( xx [ 0 ] + xx [ 1 ] ) * xx [ 2 ] ; xx [ 12 ] = xx
[ 19 ] * xx [ 20 ] ; xx [ 13 ] = xx [ 18 ] * xx [ 21 ] ; xx [ 22 ] = xx [ 2 ]
* ( xx [ 12 ] - xx [ 13 ] ) ; xx [ 23 ] = xx [ 18 ] * xx [ 20 ] ; xx [ 24 ] =
xx [ 19 ] * xx [ 21 ] ; xx [ 25 ] = ( xx [ 23 ] + xx [ 24 ] ) * xx [ 2 ] ; xx
[ 26 ] = xx [ 3 ] ; xx [ 27 ] = xx [ 22 ] ; xx [ 28 ] = xx [ 25 ] ; xx [ 29 ]
= 0.0133 ; xx [ 30 ] = 5.02846706808957e-3 ; xx [ 31 ] = 0.5 ; xx [ 32 ] = xx
[ 31 ] * state [ 13 ] ; xx [ 33 ] = cos ( xx [ 32 ] ) ; xx [ 34 ] =
0.9379742042928239 ; xx [ 35 ] = 0.02902651212139928 ; xx [ 36 ] = sin ( xx [
32 ] ) ; xx [ 32 ] = xx [ 35 ] * xx [ 36 ] ; xx [ 37 ] = 0.01361592839523582
; xx [ 38 ] = 0.999578642025762 ; xx [ 39 ] = xx [ 38 ] * xx [ 36 ] ; xx [ 36
] = xx [ 30 ] * xx [ 33 ] + xx [ 34 ] * xx [ 32 ] - xx [ 37 ] * xx [ 39 ] ;
xx [ 40 ] = xx [ 36 ] * xx [ 36 ] ; xx [ 41 ] = 0.3464010870279208 ; xx [ 42
] = xx [ 41 ] * xx [ 32 ] - ( xx [ 30 ] * xx [ 39 ] + xx [ 37 ] * xx [ 33 ] )
; xx [ 43 ] = xx [ 30 ] * xx [ 32 ] - xx [ 34 ] * xx [ 33 ] + xx [ 41 ] * xx
[ 39 ] ; xx [ 44 ] = xx [ 43 ] * xx [ 42 ] ; xx [ 45 ] = xx [ 41 ] * xx [ 33
] + xx [ 37 ] * xx [ 32 ] + xx [ 34 ] * xx [ 39 ] ; xx [ 32 ] = xx [ 45 ] *
xx [ 36 ] ; xx [ 33 ] = xx [ 45 ] * xx [ 42 ] ; xx [ 37 ] = xx [ 43 ] * xx [
36 ] ; xx [ 39 ] = xx [ 43 ] * xx [ 45 ] ; xx [ 41 ] = xx [ 42 ] * xx [ 36 ]
; xx [ 46 ] = ( xx [ 40 ] + xx [ 42 ] * xx [ 42 ] ) * xx [ 2 ] - xx [ 11 ] ;
xx [ 47 ] = xx [ 2 ] * ( xx [ 44 ] + xx [ 32 ] ) ; xx [ 48 ] = ( xx [ 33 ] -
xx [ 37 ] ) * xx [ 2 ] ; xx [ 49 ] = ( xx [ 44 ] - xx [ 32 ] ) * xx [ 2 ] ;
xx [ 50 ] = ( xx [ 40 ] + xx [ 43 ] * xx [ 43 ] ) * xx [ 2 ] - xx [ 11 ] ; xx
[ 51 ] = xx [ 2 ] * ( xx [ 39 ] + xx [ 41 ] ) ; xx [ 52 ] = xx [ 2 ] * ( xx [
33 ] + xx [ 37 ] ) ; xx [ 53 ] = ( xx [ 39 ] - xx [ 41 ] ) * xx [ 2 ] ; xx [
54 ] = ( xx [ 40 ] + xx [ 45 ] * xx [ 45 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 32
] = 0.6271461567375358 ; xx [ 33 ] = 0.3835306436794506 ; xx [ 37 ] =
0.3186539101717888 ; xx [ 39 ] = 0.5983741546728057 ; xx [ 55 ] = - xx [ 32 ]
; xx [ 56 ] = - xx [ 33 ] ; xx [ 57 ] = - xx [ 37 ] ; xx [ 58 ] = xx [ 39 ] ;
xx [ 40 ] = xx [ 31 ] * state [ 15 ] ; xx [ 41 ] = 0.09547264031652658 ; xx [
44 ] = sin ( xx [ 40 ] ) ; xx [ 59 ] = 0.9948427668622 ; xx [ 60 ] =
0.03424681259555507 ; xx [ 61 ] = cos ( xx [ 40 ] ) ; xx [ 62 ] = xx [ 41 ] *
xx [ 44 ] ; xx [ 63 ] = xx [ 59 ] * xx [ 44 ] ; xx [ 64 ] = - ( xx [ 60 ] *
xx [ 44 ] ) ; pm_math_Quaternion_compose_ra ( xx + 55 , xx + 61 , xx + 65 ) ;
xx [ 40 ] = xx [ 65 ] * xx [ 65 ] ; xx [ 44 ] = xx [ 66 ] * xx [ 67 ] ; xx [
55 ] = xx [ 65 ] * xx [ 68 ] ; xx [ 56 ] = xx [ 66 ] * xx [ 68 ] ; xx [ 57 ]
= xx [ 65 ] * xx [ 67 ] ; xx [ 58 ] = xx [ 67 ] * xx [ 68 ] ; xx [ 61 ] = xx
[ 65 ] * xx [ 66 ] ; xx [ 69 ] = ( xx [ 40 ] + xx [ 66 ] * xx [ 66 ] ) * xx [
2 ] - xx [ 11 ] ; xx [ 70 ] = xx [ 2 ] * ( xx [ 44 ] - xx [ 55 ] ) ; xx [ 71
] = ( xx [ 56 ] + xx [ 57 ] ) * xx [ 2 ] ; xx [ 72 ] = ( xx [ 44 ] + xx [ 55
] ) * xx [ 2 ] ; xx [ 73 ] = ( xx [ 40 ] + xx [ 67 ] * xx [ 67 ] ) * xx [ 2 ]
- xx [ 11 ] ; xx [ 74 ] = xx [ 2 ] * ( xx [ 58 ] - xx [ 61 ] ) ; xx [ 75 ] =
xx [ 2 ] * ( xx [ 56 ] - xx [ 57 ] ) ; xx [ 76 ] = ( xx [ 58 ] + xx [ 61 ] )
* xx [ 2 ] ; xx [ 77 ] = ( xx [ 40 ] + xx [ 68 ] * xx [ 68 ] ) * xx [ 2 ] -
xx [ 11 ] ; xx [ 40 ] = xx [ 31 ] * state [ 33 ] ; xx [ 44 ] = cos ( xx [ 40
] ) ; xx [ 55 ] = sin ( xx [ 40 ] ) ; xx [ 40 ] = xx [ 35 ] * xx [ 55 ] ; xx
[ 56 ] = xx [ 38 ] * xx [ 55 ] ; xx [ 55 ] = xx [ 32 ] * xx [ 44 ] + xx [ 37
] * xx [ 40 ] - xx [ 33 ] * xx [ 56 ] ; xx [ 57 ] = xx [ 55 ] * xx [ 55 ] ;
xx [ 58 ] = xx [ 32 ] * xx [ 56 ] + xx [ 33 ] * xx [ 44 ] + xx [ 39 ] * xx [
40 ] ; xx [ 61 ] = xx [ 37 ] * xx [ 44 ] - xx [ 32 ] * xx [ 40 ] + xx [ 39 ]
* xx [ 56 ] ; xx [ 32 ] = xx [ 61 ] * xx [ 58 ] ; xx [ 62 ] = xx [ 33 ] * xx
[ 40 ] - xx [ 39 ] * xx [ 44 ] + xx [ 37 ] * xx [ 56 ] ; xx [ 33 ] = xx [ 62
] * xx [ 55 ] ; xx [ 37 ] = xx [ 58 ] * xx [ 62 ] ; xx [ 39 ] = xx [ 61 ] *
xx [ 55 ] ; xx [ 40 ] = xx [ 61 ] * xx [ 62 ] ; xx [ 44 ] = xx [ 58 ] * xx [
55 ] ; xx [ 78 ] = ( xx [ 57 ] + xx [ 58 ] * xx [ 58 ] ) * xx [ 2 ] - xx [ 11
] ; xx [ 79 ] = xx [ 2 ] * ( xx [ 32 ] + xx [ 33 ] ) ; xx [ 80 ] = ( xx [ 37
] - xx [ 39 ] ) * xx [ 2 ] ; xx [ 81 ] = ( xx [ 32 ] - xx [ 33 ] ) * xx [ 2 ]
; xx [ 82 ] = ( xx [ 57 ] + xx [ 61 ] * xx [ 61 ] ) * xx [ 2 ] - xx [ 11 ] ;
xx [ 83 ] = xx [ 2 ] * ( xx [ 40 ] + xx [ 44 ] ) ; xx [ 84 ] = xx [ 2 ] * (
xx [ 37 ] + xx [ 39 ] ) ; xx [ 85 ] = ( xx [ 40 ] - xx [ 44 ] ) * xx [ 2 ] ;
xx [ 86 ] = ( xx [ 57 ] + xx [ 62 ] * xx [ 62 ] ) * xx [ 2 ] - xx [ 11 ] ; xx
[ 32 ] = xx [ 31 ] * state [ 35 ] ; xx [ 33 ] = cos ( xx [ 32 ] ) ; xx [ 37 ]
= 0.01361592839523579 ; xx [ 39 ] = sin ( xx [ 32 ] ) ; xx [ 32 ] = xx [ 30 ]
* xx [ 33 ] + xx [ 37 ] * xx [ 39 ] ; xx [ 40 ] = xx [ 32 ] * xx [ 32 ] ; xx
[ 44 ] = xx [ 37 ] * xx [ 33 ] - xx [ 30 ] * xx [ 39 ] ; xx [ 56 ] = ( xx [
40 ] + xx [ 44 ] * xx [ 44 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 57 ] =
0.3464010870279207 ; xx [ 63 ] = xx [ 34 ] * xx [ 33 ] - xx [ 57 ] * xx [ 39
] ; xx [ 64 ] = xx [ 63 ] * xx [ 44 ] ; xx [ 87 ] = xx [ 57 ] * xx [ 33 ] +
xx [ 34 ] * xx [ 39 ] ; xx [ 33 ] = xx [ 32 ] * xx [ 87 ] ; xx [ 34 ] = xx [
2 ] * ( xx [ 64 ] - xx [ 33 ] ) ; xx [ 39 ] = xx [ 44 ] * xx [ 87 ] ; xx [ 57
] = xx [ 63 ] * xx [ 32 ] ; xx [ 88 ] = ( xx [ 39 ] + xx [ 57 ] ) * xx [ 2 ]
; xx [ 89 ] = ( xx [ 64 ] + xx [ 33 ] ) * xx [ 2 ] ; xx [ 33 ] = ( xx [ 40 ]
+ xx [ 63 ] * xx [ 63 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 64 ] = xx [ 44 ] * xx
[ 32 ] ; xx [ 90 ] = xx [ 63 ] * xx [ 87 ] ; xx [ 91 ] = xx [ 2 ] * ( xx [ 64
] - xx [ 90 ] ) ; xx [ 92 ] = xx [ 2 ] * ( xx [ 57 ] - xx [ 39 ] ) ; xx [ 39
] = ( xx [ 90 ] + xx [ 64 ] ) * xx [ 2 ] ; xx [ 57 ] = ( xx [ 40 ] + xx [ 87
] * xx [ 87 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 93 ] = xx [ 56 ] ; xx [ 94 ] =
xx [ 34 ] ; xx [ 95 ] = - xx [ 88 ] ; xx [ 96 ] = xx [ 89 ] ; xx [ 97 ] = xx
[ 33 ] ; xx [ 98 ] = xx [ 91 ] ; xx [ 99 ] = xx [ 92 ] ; xx [ 100 ] = - xx [
39 ] ; xx [ 101 ] = xx [ 57 ] ; xx [ 40 ] = 1.462716614497594e-20 ; xx [ 64 ]
= 2.490170000000001e-6 ; memcpy ( xx + 90 , xx + 64 , 1 * sizeof ( double ) )
; ii [ 0 ] = factorSymmetricPosDef ( xx + 90 , 1 , xx + 102 ) ; if ( ii [ 0 ]
!= 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'simple_robot/Revolute6' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } memcpy ( xx + 102 , xx + 90 , 1 * sizeof ( double ) ) ; xx
[ 103 ] = xx [ 40 ] / xx [ 102 ] ; xx [ 104 ] = xx [ 29 ] - xx [ 40 ] * xx [
103 ] ; xx [ 105 ] = xx [ 29 ] * xx [ 56 ] ; xx [ 106 ] = xx [ 29 ] * xx [ 89
] ; xx [ 107 ] = xx [ 29 ] * xx [ 92 ] ; xx [ 108 ] = xx [ 34 ] * xx [ 104 ]
; xx [ 109 ] = xx [ 104 ] * xx [ 33 ] ; xx [ 110 ] = - ( xx [ 39 ] * xx [ 104
] ) ; xx [ 111 ] = - ( xx [ 29 ] * xx [ 88 ] ) ; xx [ 112 ] = xx [ 29 ] * xx
[ 91 ] ; xx [ 113 ] = xx [ 29 ] * xx [ 57 ] ; pm_math_Matrix3x3_compose_ra (
xx + 93 , xx + 105 , xx + 114 ) ; xx [ 104 ] = 0.06414602424960493 ; xx [ 105
] = 0.02135192530809139 ; xx [ 106 ] = 4.952999999999999e-3 ; xx [ 107 ] = xx
[ 63 ] * xx [ 106 ] ; xx [ 108 ] = xx [ 106 ] * xx [ 87 ] ; xx [ 109 ] = xx [
105 ] - ( ( xx [ 63 ] * xx [ 107 ] + xx [ 108 ] * xx [ 87 ] ) * xx [ 2 ] - xx
[ 106 ] ) ; xx [ 110 ] = ( xx [ 108 ] * xx [ 32 ] + xx [ 44 ] * xx [ 107 ] )
* xx [ 2 ] - 0.05562318494169335 ; xx [ 111 ] = - ( xx [ 2 ] * ( xx [ 44 ] *
xx [ 108 ] - xx [ 107 ] * xx [ 32 ] ) ) ; pm_math_Matrix3x3_postCross_ra ( xx
+ 114 , xx + 109 , xx + 123 ) ; xx [ 107 ] = xx [ 64 ] * xx [ 103 ] ; xx [
108 ] = xx [ 107 ] * xx [ 34 ] ; xx [ 112 ] = xx [ 89 ] * xx [ 108 ] ; xx [
113 ] = xx [ 124 ] - xx [ 112 ] ; xx [ 132 ] = xx [ 108 ] * xx [ 56 ] ; xx [
133 ] = xx [ 123 ] - xx [ 132 ] ; xx [ 134 ] = xx [ 104 ] * xx [ 116 ] - ( xx
[ 113 ] * xx [ 35 ] - xx [ 133 ] * xx [ 38 ] ) ; xx [ 135 ] =
7.797265349058053e-7 ; xx [ 136 ] = xx [ 64 ] / xx [ 102 ] ; xx [ 137 ] = xx
[ 64 ] - xx [ 64 ] * xx [ 136 ] ; xx [ 138 ] = 1.356230000000001e-6 ; xx [
139 ] = 1.35721e-6 ; xx [ 140 ] = xx [ 137 ] * xx [ 56 ] ; xx [ 141 ] = xx [
89 ] * xx [ 137 ] ; xx [ 142 ] = xx [ 92 ] * xx [ 137 ] ; xx [ 143 ] = xx [
138 ] * xx [ 34 ] ; xx [ 144 ] = xx [ 138 ] * xx [ 33 ] ; xx [ 145 ] = - ( xx
[ 138 ] * xx [ 39 ] ) ; xx [ 146 ] = - ( xx [ 139 ] * xx [ 88 ] ) ; xx [ 147
] = xx [ 139 ] * xx [ 91 ] ; xx [ 148 ] = xx [ 139 ] * xx [ 57 ] ;
pm_math_Matrix3x3_compose_ra ( xx + 93 , xx + 140 , xx + 149 ) ; xx [ 34 ] =
xx [ 107 ] * xx [ 33 ] ; xx [ 33 ] = xx [ 34 ] * xx [ 56 ] ; xx [ 57 ] = xx [
39 ] * xx [ 107 ] ; xx [ 39 ] = xx [ 57 ] * xx [ 56 ] ; xx [ 56 ] = xx [ 89 ]
* xx [ 34 ] ; xx [ 88 ] = xx [ 89 ] * xx [ 57 ] ; xx [ 89 ] = xx [ 92 ] * xx
[ 108 ] ; xx [ 91 ] = xx [ 92 ] * xx [ 34 ] ; xx [ 34 ] = xx [ 92 ] * xx [ 57
] ; xx [ 92 ] = xx [ 132 ] ; xx [ 93 ] = xx [ 33 ] ; xx [ 94 ] = - xx [ 39 ]
; xx [ 95 ] = xx [ 112 ] ; xx [ 96 ] = xx [ 56 ] ; xx [ 97 ] = - xx [ 88 ] ;
xx [ 98 ] = xx [ 89 ] ; xx [ 99 ] = xx [ 91 ] ; xx [ 100 ] = - xx [ 34 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 92 , xx + 109 , xx + 140 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 123 , xx + 109 , xx + 92 ) ; xx [ 57 ] =
xx [ 135 ] + xx [ 153 ] - xx [ 144 ] - xx [ 144 ] - xx [ 96 ] ; xx [ 101 ] =
xx [ 152 ] - xx [ 143 ] - xx [ 141 ] - xx [ 95 ] ; xx [ 107 ] = xx [ 88 ] +
xx [ 130 ] ; xx [ 88 ] = xx [ 57 ] * xx [ 35 ] - xx [ 38 ] * xx [ 101 ] - xx
[ 107 ] * xx [ 104 ] ; xx [ 108 ] = xx [ 150 ] - xx [ 141 ] - xx [ 143 ] - xx
[ 93 ] ; xx [ 112 ] = 4.409631249368538e-5 ; xx [ 123 ] = xx [ 112 ] + xx [
149 ] - xx [ 140 ] - xx [ 140 ] - xx [ 92 ] ; xx [ 124 ] = xx [ 39 ] + xx [
129 ] ; xx [ 39 ] = xx [ 35 ] * xx [ 108 ] - xx [ 123 ] * xx [ 38 ] - xx [
124 ] * xx [ 104 ] ; xx [ 129 ] = 0.02887999847592921 ; xx [ 130 ] = xx [ 129
] + xx [ 122 ] ; xx [ 122 ] = xx [ 130 ] * xx [ 104 ] - ( xx [ 107 ] * xx [
35 ] - xx [ 124 ] * xx [ 38 ] ) ; xx [ 132 ] = xx [ 35 ] * xx [ 88 ] - xx [
38 ] * xx [ 39 ] + xx [ 104 ] * xx [ 122 ] ; ii [ 1 ] = factorSymmetricPosDef
( xx + 132 , 1 , xx + 137 ) ; if ( ii [ 1 ] != 0 ) { return
sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'simple_robot/Revolute4' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 137 ] = xx [ 134 ] / xx [ 132 ] ; xx [ 158 ] = xx [
127 ] - xx [ 56 ] ; xx [ 56 ] = xx [ 126 ] - xx [ 33 ] ; xx [ 33 ] = xx [ 104
] * xx [ 119 ] - ( xx [ 158 ] * xx [ 35 ] - xx [ 56 ] * xx [ 38 ] ) ; xx [
126 ] = xx [ 137 ] * xx [ 33 ] ; xx [ 127 ] = xx [ 137 ] * xx [ 122 ] ; xx [
159 ] = xx [ 33 ] / xx [ 132 ] ; xx [ 160 ] = xx [ 159 ] * xx [ 122 ] ; xx [
161 ] = xx [ 122 ] / xx [ 132 ] ; xx [ 162 ] = xx [ 114 ] - xx [ 137 ] * xx [
134 ] + xx [ 129 ] ; xx [ 163 ] = xx [ 115 ] - xx [ 126 ] ; xx [ 164 ] = xx [
116 ] - xx [ 127 ] ; xx [ 165 ] = xx [ 117 ] - xx [ 126 ] ; xx [ 166 ] = xx [
118 ] - xx [ 159 ] * xx [ 33 ] + xx [ 129 ] ; xx [ 167 ] = xx [ 119 ] - xx [
160 ] ; xx [ 168 ] = xx [ 120 ] - xx [ 127 ] ; xx [ 169 ] = xx [ 121 ] - xx [
160 ] ; xx [ 170 ] = xx [ 130 ] - xx [ 161 ] * xx [ 122 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 162 , xx + 78 , xx + 171 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 78 , xx + 171 , xx + 162 ) ; xx [ 126 ] =
0.5799181594307252 ; xx [ 127 ] = xx [ 31 ] * state [ 29 ] ; xx [ 160 ] = sin
( xx [ 127 ] ) ; xx [ 171 ] = xx [ 38 ] * xx [ 160 ] ; xx [ 172 ] =
0.6442506183571239 ; xx [ 173 ] = xx [ 35 ] * xx [ 160 ] ; xx [ 160 ] =
0.3296521996834055 ; xx [ 174 ] = cos ( xx [ 127 ] ) ; xx [ 127 ] = xx [ 126
] * xx [ 171 ] + xx [ 172 ] * xx [ 173 ] - xx [ 160 ] * xx [ 174 ] ; xx [ 175
] = xx [ 127 ] * xx [ 127 ] ; xx [ 176 ] = 0.3741196283982388 ; xx [ 177 ] =
xx [ 160 ] * xx [ 171 ] + xx [ 126 ] * xx [ 174 ] - xx [ 176 ] * xx [ 173 ] ;
xx [ 178 ] = xx [ 160 ] * xx [ 173 ] + xx [ 172 ] * xx [ 174 ] + xx [ 176 ] *
xx [ 171 ] ; xx [ 179 ] = xx [ 178 ] * xx [ 177 ] ; xx [ 180 ] = xx [ 176 ] *
xx [ 174 ] + xx [ 126 ] * xx [ 173 ] - xx [ 172 ] * xx [ 171 ] ; xx [ 171 ] =
xx [ 180 ] * xx [ 127 ] ; xx [ 173 ] = xx [ 177 ] * xx [ 180 ] ; xx [ 174 ] =
xx [ 178 ] * xx [ 127 ] ; xx [ 181 ] = xx [ 178 ] * xx [ 180 ] ; xx [ 182 ] =
xx [ 177 ] * xx [ 127 ] ; xx [ 183 ] = ( xx [ 175 ] + xx [ 177 ] * xx [ 177 ]
) * xx [ 2 ] - xx [ 11 ] ; xx [ 184 ] = - ( xx [ 2 ] * ( xx [ 179 ] + xx [
171 ] ) ) ; xx [ 185 ] = ( xx [ 173 ] - xx [ 174 ] ) * xx [ 2 ] ; xx [ 186 ]
= ( xx [ 171 ] - xx [ 179 ] ) * xx [ 2 ] ; xx [ 187 ] = ( xx [ 175 ] + xx [
178 ] * xx [ 178 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 188 ] = - ( xx [ 2 ] * (
xx [ 181 ] + xx [ 182 ] ) ) ; xx [ 189 ] = xx [ 2 ] * ( xx [ 173 ] + xx [ 174
] ) ; xx [ 190 ] = ( xx [ 182 ] - xx [ 181 ] ) * xx [ 2 ] ; xx [ 191 ] = ( xx
[ 175 ] + xx [ 180 ] * xx [ 180 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 171 ] =
0.9379742042928236 ; xx [ 173 ] = xx [ 31 ] * state [ 31 ] ; xx [ 174 ] = cos
( xx [ 173 ] ) ; xx [ 175 ] = 0.3464010870279211 ; xx [ 179 ] = sin ( xx [
173 ] ) ; xx [ 173 ] = xx [ 171 ] * xx [ 174 ] + xx [ 175 ] * xx [ 179 ] ; xx
[ 181 ] = xx [ 173 ] * xx [ 173 ] ; xx [ 182 ] = xx [ 171 ] * xx [ 179 ] - xx
[ 175 ] * xx [ 174 ] ; xx [ 171 ] = ( xx [ 181 ] + xx [ 182 ] * xx [ 182 ] )
* xx [ 2 ] - xx [ 11 ] ; xx [ 175 ] = 5.028467068089626e-3 ; xx [ 192 ] =
0.01361592839523584 ; xx [ 193 ] = xx [ 175 ] * xx [ 174 ] - xx [ 192 ] * xx
[ 179 ] ; xx [ 194 ] = xx [ 193 ] * xx [ 182 ] ; xx [ 195 ] = xx [ 192 ] * xx
[ 174 ] + xx [ 175 ] * xx [ 179 ] ; xx [ 174 ] = xx [ 195 ] * xx [ 173 ] ; xx
[ 175 ] = xx [ 2 ] * ( xx [ 194 ] + xx [ 174 ] ) ; xx [ 179 ] = xx [ 195 ] *
xx [ 182 ] ; xx [ 192 ] = xx [ 173 ] * xx [ 193 ] ; xx [ 196 ] = ( xx [ 179 ]
- xx [ 192 ] ) * xx [ 2 ] ; xx [ 197 ] = ( xx [ 194 ] - xx [ 174 ] ) * xx [ 2
] ; xx [ 174 ] = ( xx [ 181 ] + xx [ 193 ] * xx [ 193 ] ) * xx [ 2 ] - xx [
11 ] ; xx [ 194 ] = xx [ 195 ] * xx [ 193 ] ; xx [ 198 ] = xx [ 173 ] * xx [
182 ] ; xx [ 199 ] = xx [ 2 ] * ( xx [ 194 ] + xx [ 198 ] ) ; xx [ 200 ] = xx
[ 2 ] * ( xx [ 179 ] + xx [ 192 ] ) ; xx [ 179 ] = ( xx [ 194 ] - xx [ 198 ]
) * xx [ 2 ] ; xx [ 192 ] = ( xx [ 181 ] + xx [ 195 ] * xx [ 195 ] ) * xx [ 2
] - xx [ 11 ] ; xx [ 201 ] = xx [ 171 ] ; xx [ 202 ] = xx [ 175 ] ; xx [ 203
] = xx [ 196 ] ; xx [ 204 ] = xx [ 197 ] ; xx [ 205 ] = xx [ 174 ] ; xx [ 206
] = xx [ 199 ] ; xx [ 207 ] = xx [ 200 ] ; xx [ 208 ] = xx [ 179 ] ; xx [ 209
] = xx [ 192 ] ; xx [ 181 ] = 1.462716614497613e-20 ; if ( ii [ 0 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'simple_robot/Revolute9' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } memcpy ( xx + 194 , xx + 90 , 1 * sizeof ( double ) ) ; xx
[ 198 ] = xx [ 181 ] / xx [ 194 ] ; xx [ 210 ] = xx [ 29 ] - xx [ 181 ] * xx
[ 198 ] ; xx [ 211 ] = xx [ 29 ] * xx [ 171 ] ; xx [ 212 ] = xx [ 29 ] * xx [
197 ] ; xx [ 213 ] = xx [ 29 ] * xx [ 200 ] ; xx [ 214 ] = xx [ 175 ] * xx [
210 ] ; xx [ 215 ] = xx [ 210 ] * xx [ 174 ] ; xx [ 216 ] = xx [ 179 ] * xx [
210 ] ; xx [ 217 ] = xx [ 29 ] * xx [ 196 ] ; xx [ 218 ] = xx [ 29 ] * xx [
199 ] ; xx [ 219 ] = xx [ 29 ] * xx [ 192 ] ; pm_math_Matrix3x3_compose_ra (
xx + 201 , xx + 211 , xx + 220 ) ; xx [ 210 ] = 0.06414602424960492 ; xx [
211 ] = 4.953000000000063e-3 ; xx [ 212 ] = xx [ 211 ] * xx [ 193 ] ; xx [
213 ] = xx [ 195 ] * xx [ 211 ] ; xx [ 214 ] = xx [ 105 ] - ( xx [ 211 ] - (
xx [ 212 ] * xx [ 193 ] + xx [ 195 ] * xx [ 213 ] ) * xx [ 2 ] ) ; xx [ 215 ]
= - ( 0.05562318494169332 + ( xx [ 212 ] * xx [ 182 ] - xx [ 173 ] * xx [ 213
] ) * xx [ 2 ] ) ; xx [ 216 ] = - ( xx [ 2 ] * ( xx [ 213 ] * xx [ 182 ] + xx
[ 173 ] * xx [ 212 ] ) ) ; pm_math_Matrix3x3_postCross_ra ( xx + 220 , xx +
214 , xx + 229 ) ; xx [ 211 ] = xx [ 64 ] * xx [ 198 ] ; xx [ 212 ] = xx [
211 ] * xx [ 175 ] ; xx [ 213 ] = xx [ 197 ] * xx [ 212 ] ; xx [ 217 ] = xx [
230 ] - xx [ 213 ] ; xx [ 218 ] = xx [ 212 ] * xx [ 171 ] ; xx [ 219 ] = xx [
229 ] - xx [ 218 ] ; xx [ 238 ] = xx [ 210 ] * xx [ 222 ] - ( xx [ 217 ] * xx
[ 35 ] - xx [ 219 ] * xx [ 38 ] ) ; xx [ 239 ] = xx [ 64 ] / xx [ 194 ] ; xx
[ 240 ] = xx [ 64 ] - xx [ 64 ] * xx [ 239 ] ; xx [ 241 ] = xx [ 240 ] * xx [
171 ] ; xx [ 242 ] = xx [ 197 ] * xx [ 240 ] ; xx [ 243 ] = xx [ 200 ] * xx [
240 ] ; xx [ 244 ] = xx [ 138 ] * xx [ 175 ] ; xx [ 245 ] = xx [ 138 ] * xx [
174 ] ; xx [ 246 ] = xx [ 138 ] * xx [ 179 ] ; xx [ 247 ] = xx [ 139 ] * xx [
196 ] ; xx [ 248 ] = xx [ 139 ] * xx [ 199 ] ; xx [ 249 ] = xx [ 139 ] * xx [
192 ] ; pm_math_Matrix3x3_compose_ra ( xx + 201 , xx + 241 , xx + 250 ) ; xx
[ 175 ] = xx [ 211 ] * xx [ 174 ] ; xx [ 174 ] = xx [ 175 ] * xx [ 171 ] ; xx
[ 192 ] = xx [ 179 ] * xx [ 211 ] ; xx [ 179 ] = xx [ 192 ] * xx [ 171 ] ; xx
[ 171 ] = xx [ 197 ] * xx [ 175 ] ; xx [ 196 ] = xx [ 197 ] * xx [ 192 ] ; xx
[ 197 ] = xx [ 200 ] * xx [ 212 ] ; xx [ 199 ] = xx [ 200 ] * xx [ 175 ] ; xx
[ 175 ] = xx [ 200 ] * xx [ 192 ] ; xx [ 200 ] = xx [ 218 ] ; xx [ 201 ] = xx
[ 174 ] ; xx [ 202 ] = xx [ 179 ] ; xx [ 203 ] = xx [ 213 ] ; xx [ 204 ] = xx
[ 171 ] ; xx [ 205 ] = xx [ 196 ] ; xx [ 206 ] = xx [ 197 ] ; xx [ 207 ] = xx
[ 199 ] ; xx [ 208 ] = xx [ 175 ] ; pm_math_Matrix3x3_postCross_ra ( xx + 200
, xx + 214 , xx + 240 ) ; pm_math_Matrix3x3_preCross_ra ( xx + 229 , xx + 214
, xx + 200 ) ; xx [ 192 ] = xx [ 135 ] + xx [ 254 ] - xx [ 244 ] - xx [ 244 ]
- xx [ 204 ] ; xx [ 209 ] = xx [ 253 ] - xx [ 243 ] - xx [ 241 ] - xx [ 203 ]
; xx [ 211 ] = xx [ 236 ] - xx [ 196 ] ; xx [ 196 ] = xx [ 192 ] * xx [ 35 ]
- xx [ 38 ] * xx [ 209 ] - xx [ 211 ] * xx [ 210 ] ; xx [ 212 ] = xx [ 251 ]
- xx [ 241 ] - xx [ 243 ] - xx [ 201 ] ; xx [ 213 ] = xx [ 112 ] + xx [ 250 ]
- xx [ 240 ] - xx [ 240 ] - xx [ 200 ] ; xx [ 218 ] = xx [ 235 ] - xx [ 179 ]
; xx [ 179 ] = xx [ 35 ] * xx [ 212 ] - xx [ 213 ] * xx [ 38 ] - xx [ 218 ] *
xx [ 210 ] ; xx [ 229 ] = xx [ 129 ] + xx [ 228 ] ; xx [ 228 ] = xx [ 229 ] *
xx [ 210 ] - ( xx [ 211 ] * xx [ 35 ] - xx [ 218 ] * xx [ 38 ] ) ; xx [ 230 ]
= xx [ 35 ] * xx [ 196 ] - xx [ 38 ] * xx [ 179 ] + xx [ 210 ] * xx [ 228 ] ;
ii [ 1 ] = factorSymmetricPosDef ( xx + 230 , 1 , xx + 235 ) ; if ( ii [ 1 ]
!= 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'simple_robot/Revolute3' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 235 ] = xx [ 238 ] / xx [ 230 ] ; xx [ 236 ] = xx [
233 ] - xx [ 171 ] ; xx [ 171 ] = xx [ 232 ] - xx [ 174 ] ; xx [ 174 ] = xx [
210 ] * xx [ 225 ] - ( xx [ 236 ] * xx [ 35 ] - xx [ 171 ] * xx [ 38 ] ) ; xx
[ 232 ] = xx [ 235 ] * xx [ 174 ] ; xx [ 233 ] = xx [ 235 ] * xx [ 228 ] ; xx
[ 249 ] = xx [ 174 ] / xx [ 230 ] ; xx [ 259 ] = xx [ 249 ] * xx [ 228 ] ; xx
[ 260 ] = xx [ 228 ] / xx [ 230 ] ; xx [ 261 ] = xx [ 220 ] - xx [ 235 ] * xx
[ 238 ] + xx [ 129 ] ; xx [ 262 ] = xx [ 221 ] - xx [ 232 ] ; xx [ 263 ] = xx
[ 222 ] - xx [ 233 ] ; xx [ 264 ] = xx [ 223 ] - xx [ 232 ] ; xx [ 265 ] = xx
[ 224 ] - xx [ 249 ] * xx [ 174 ] + xx [ 129 ] ; xx [ 266 ] = xx [ 225 ] - xx
[ 259 ] ; xx [ 267 ] = xx [ 226 ] - xx [ 233 ] ; xx [ 268 ] = xx [ 227 ] - xx
[ 259 ] ; xx [ 269 ] = xx [ 229 ] - xx [ 260 ] * xx [ 228 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 261 , xx + 183 , xx + 270 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 183 , xx + 270 , xx + 261 ) ; xx [ 232 ]
= xx [ 31 ] * state [ 25 ] ; xx [ 233 ] = sin ( xx [ 232 ] ) ; xx [ 259 ] =
xx [ 38 ] * xx [ 233 ] ; xx [ 270 ] = xx [ 35 ] * xx [ 233 ] ; xx [ 233 ] =
cos ( xx [ 232 ] ) ; xx [ 232 ] = xx [ 126 ] * xx [ 259 ] + xx [ 172 ] * xx [
270 ] - xx [ 160 ] * xx [ 233 ] ; xx [ 271 ] = xx [ 232 ] * xx [ 232 ] ; xx [
272 ] = xx [ 160 ] * xx [ 259 ] + xx [ 126 ] * xx [ 233 ] - xx [ 176 ] * xx [
270 ] ; xx [ 273 ] = xx [ 160 ] * xx [ 270 ] + xx [ 172 ] * xx [ 233 ] + xx [
176 ] * xx [ 259 ] ; xx [ 160 ] = xx [ 273 ] * xx [ 272 ] ; xx [ 274 ] = xx [
176 ] * xx [ 233 ] + xx [ 126 ] * xx [ 270 ] - xx [ 172 ] * xx [ 259 ] ; xx [
126 ] = xx [ 274 ] * xx [ 232 ] ; xx [ 172 ] = xx [ 272 ] * xx [ 274 ] ; xx [
176 ] = xx [ 273 ] * xx [ 232 ] ; xx [ 233 ] = xx [ 273 ] * xx [ 274 ] ; xx [
259 ] = xx [ 272 ] * xx [ 232 ] ; xx [ 275 ] = ( xx [ 271 ] + xx [ 272 ] * xx
[ 272 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 276 ] = - ( xx [ 2 ] * ( xx [ 160 ] +
xx [ 126 ] ) ) ; xx [ 277 ] = ( xx [ 172 ] - xx [ 176 ] ) * xx [ 2 ] ; xx [
278 ] = ( xx [ 126 ] - xx [ 160 ] ) * xx [ 2 ] ; xx [ 279 ] = ( xx [ 271 ] +
xx [ 273 ] * xx [ 273 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 280 ] = - ( xx [ 2 ]
* ( xx [ 233 ] + xx [ 259 ] ) ) ; xx [ 281 ] = xx [ 2 ] * ( xx [ 172 ] + xx [
176 ] ) ; xx [ 282 ] = ( xx [ 259 ] - xx [ 233 ] ) * xx [ 2 ] ; xx [ 283 ] =
( xx [ 271 ] + xx [ 274 ] * xx [ 274 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 126 ]
= 0.9379742042928237 ; xx [ 160 ] = xx [ 31 ] * state [ 27 ] ; xx [ 172 ] =
cos ( xx [ 160 ] ) ; xx [ 176 ] = 0.3464010870279209 ; xx [ 233 ] = sin ( xx
[ 160 ] ) ; xx [ 160 ] = xx [ 126 ] * xx [ 172 ] + xx [ 176 ] * xx [ 233 ] ;
xx [ 259 ] = xx [ 160 ] * xx [ 160 ] ; xx [ 270 ] = xx [ 126 ] * xx [ 233 ] -
xx [ 176 ] * xx [ 172 ] ; xx [ 126 ] = ( xx [ 259 ] + xx [ 270 ] * xx [ 270 ]
) * xx [ 2 ] - xx [ 11 ] ; xx [ 176 ] = xx [ 30 ] * xx [ 172 ] - xx [ 37 ] *
xx [ 233 ] ; xx [ 271 ] = xx [ 176 ] * xx [ 270 ] ; xx [ 284 ] = xx [ 37 ] *
xx [ 172 ] + xx [ 30 ] * xx [ 233 ] ; xx [ 30 ] = xx [ 284 ] * xx [ 160 ] ;
xx [ 37 ] = xx [ 2 ] * ( xx [ 271 ] + xx [ 30 ] ) ; xx [ 172 ] = xx [ 284 ] *
xx [ 270 ] ; xx [ 233 ] = xx [ 160 ] * xx [ 176 ] ; xx [ 285 ] = ( xx [ 172 ]
- xx [ 233 ] ) * xx [ 2 ] ; xx [ 286 ] = ( xx [ 271 ] - xx [ 30 ] ) * xx [ 2
] ; xx [ 30 ] = ( xx [ 259 ] + xx [ 176 ] * xx [ 176 ] ) * xx [ 2 ] - xx [ 11
] ; xx [ 271 ] = xx [ 284 ] * xx [ 176 ] ; xx [ 287 ] = xx [ 160 ] * xx [ 270
] ; xx [ 288 ] = xx [ 2 ] * ( xx [ 271 ] + xx [ 287 ] ) ; xx [ 289 ] = xx [ 2
] * ( xx [ 172 ] + xx [ 233 ] ) ; xx [ 172 ] = ( xx [ 271 ] - xx [ 287 ] ) *
xx [ 2 ] ; xx [ 233 ] = ( xx [ 259 ] + xx [ 284 ] * xx [ 284 ] ) * xx [ 2 ] -
xx [ 11 ] ; xx [ 290 ] = xx [ 126 ] ; xx [ 291 ] = xx [ 37 ] ; xx [ 292 ] =
xx [ 285 ] ; xx [ 293 ] = xx [ 286 ] ; xx [ 294 ] = xx [ 30 ] ; xx [ 295 ] =
xx [ 288 ] ; xx [ 296 ] = xx [ 289 ] ; xx [ 297 ] = xx [ 172 ] ; xx [ 298 ] =
xx [ 233 ] ; xx [ 299 ] = xx [ 29 ] * xx [ 126 ] ; xx [ 300 ] = xx [ 29 ] *
xx [ 286 ] ; xx [ 301 ] = xx [ 29 ] * xx [ 289 ] ; xx [ 302 ] = xx [ 29 ] *
xx [ 37 ] ; xx [ 303 ] = xx [ 29 ] * xx [ 30 ] ; xx [ 304 ] = xx [ 29 ] * xx
[ 172 ] ; xx [ 305 ] = xx [ 29 ] * xx [ 285 ] ; xx [ 306 ] = xx [ 29 ] * xx [
288 ] ; xx [ 307 ] = xx [ 29 ] * xx [ 233 ] ; pm_math_Matrix3x3_compose_ra (
xx + 290 , xx + 299 , xx + 308 ) ; xx [ 259 ] = 0.06414602424960487 ; xx [
271 ] = 4.953000000000018e-3 ; xx [ 287 ] = xx [ 271 ] * xx [ 176 ] ; xx [
299 ] = xx [ 284 ] * xx [ 271 ] ; xx [ 300 ] = 0.05562318494169338 ; xx [ 301
] = xx [ 105 ] - ( xx [ 271 ] - ( xx [ 287 ] * xx [ 176 ] + xx [ 284 ] * xx [
299 ] ) * xx [ 2 ] ) ; xx [ 302 ] = - ( xx [ 300 ] + ( xx [ 287 ] * xx [ 270
] - xx [ 160 ] * xx [ 299 ] ) * xx [ 2 ] ) ; xx [ 303 ] = - ( xx [ 2 ] * ( xx
[ 299 ] * xx [ 270 ] + xx [ 160 ] * xx [ 287 ] ) ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 308 , xx + 301 , xx + 317 ) ; xx [ 287
] = xx [ 259 ] * xx [ 310 ] - ( xx [ 35 ] * xx [ 318 ] - xx [ 38 ] * xx [ 317
] ) ; if ( ii [ 0 ] != 0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'simple_robot/Revolute8' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } memcpy ( xx + 299 , xx + 90 , 1 * sizeof ( double ) ) ; xx
[ 304 ] = xx [ 64 ] / xx [ 299 ] ; xx [ 305 ] = xx [ 64 ] - xx [ 64 ] * xx [
304 ] ; xx [ 326 ] = xx [ 305 ] * xx [ 126 ] ; xx [ 327 ] = xx [ 286 ] * xx [
305 ] ; xx [ 328 ] = xx [ 289 ] * xx [ 305 ] ; xx [ 329 ] = xx [ 138 ] * xx [
37 ] ; xx [ 330 ] = xx [ 138 ] * xx [ 30 ] ; xx [ 331 ] = xx [ 138 ] * xx [
172 ] ; xx [ 332 ] = xx [ 139 ] * xx [ 285 ] ; xx [ 333 ] = xx [ 139 ] * xx [
288 ] ; xx [ 334 ] = xx [ 139 ] * xx [ 233 ] ; pm_math_Matrix3x3_compose_ra (
xx + 290 , xx + 326 , xx + 335 ) ; pm_math_Matrix3x3_preCross_ra ( xx + 317 ,
xx + 301 , xx + 288 ) ; xx [ 30 ] = xx [ 135 ] + xx [ 339 ] - xx [ 292 ] ; xx
[ 37 ] = xx [ 338 ] - xx [ 291 ] ; xx [ 126 ] = xx [ 30 ] * xx [ 35 ] - xx [
38 ] * xx [ 37 ] - xx [ 259 ] * xx [ 324 ] ; xx [ 172 ] = xx [ 336 ] - xx [
289 ] ; xx [ 233 ] = xx [ 112 ] + xx [ 335 ] - xx [ 288 ] ; xx [ 285 ] = xx [
35 ] * xx [ 172 ] - xx [ 233 ] * xx [ 38 ] - xx [ 259 ] * xx [ 323 ] ; xx [
286 ] = xx [ 129 ] + xx [ 316 ] ; xx [ 297 ] = xx [ 286 ] * xx [ 259 ] - ( xx
[ 35 ] * xx [ 324 ] - xx [ 38 ] * xx [ 323 ] ) ; xx [ 298 ] = xx [ 35 ] * xx
[ 126 ] - xx [ 38 ] * xx [ 285 ] + xx [ 259 ] * xx [ 297 ] ; ii [ 1 ] =
factorSymmetricPosDef ( xx + 298 , 1 , xx + 305 ) ; if ( ii [ 1 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'simple_robot/Revolute2' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 305 ] = xx [ 287 ] / xx [ 298 ] ; xx [ 306 ] = xx [
259 ] * xx [ 313 ] - ( xx [ 35 ] * xx [ 321 ] - xx [ 38 ] * xx [ 320 ] ) ; xx
[ 307 ] = xx [ 305 ] * xx [ 306 ] ; xx [ 316 ] = xx [ 305 ] * xx [ 297 ] ; xx
[ 326 ] = xx [ 306 ] / xx [ 298 ] ; xx [ 327 ] = xx [ 326 ] * xx [ 297 ] ; xx
[ 328 ] = xx [ 297 ] / xx [ 298 ] ; xx [ 344 ] = xx [ 308 ] - xx [ 305 ] * xx
[ 287 ] + xx [ 129 ] ; xx [ 345 ] = xx [ 309 ] - xx [ 307 ] ; xx [ 346 ] = xx
[ 310 ] - xx [ 316 ] ; xx [ 347 ] = xx [ 311 ] - xx [ 307 ] ; xx [ 348 ] = xx
[ 312 ] - xx [ 326 ] * xx [ 306 ] + xx [ 129 ] ; xx [ 349 ] = xx [ 313 ] - xx
[ 327 ] ; xx [ 350 ] = xx [ 314 ] - xx [ 316 ] ; xx [ 351 ] = xx [ 315 ] - xx
[ 327 ] ; xx [ 352 ] = xx [ 286 ] - xx [ 328 ] * xx [ 297 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 344 , xx + 275 , xx + 353 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 275 , xx + 353 , xx + 344 ) ; xx [ 307 ]
= 0.615357735429888 ; xx [ 316 ] = xx [ 31 ] * state [ 21 ] ; xx [ 327 ] =
cos ( xx [ 316 ] ) ; xx [ 329 ] = 0.3459611930551978 ; xx [ 330 ] =
0.9953376248861994 ; xx [ 331 ] = sin ( xx [ 316 ] ) ; xx [ 316 ] = xx [ 330
] * xx [ 331 ] ; xx [ 332 ] = 0.3585189984711592 ; xx [ 333 ] =
0.0964521253570878 ; xx [ 334 ] = xx [ 333 ] * xx [ 331 ] ; xx [ 331 ] = xx [
307 ] * xx [ 327 ] + xx [ 329 ] * xx [ 316 ] - xx [ 332 ] * xx [ 334 ] ; xx [
353 ] = xx [ 331 ] * xx [ 331 ] ; xx [ 354 ] = 0.6108271752972861 ; xx [ 355
] = xx [ 354 ] * xx [ 327 ] - xx [ 329 ] * xx [ 334 ] - xx [ 332 ] * xx [ 316
] ; xx [ 356 ] = xx [ 329 ] * xx [ 327 ] - xx [ 307 ] * xx [ 316 ] + xx [ 354
] * xx [ 334 ] ; xx [ 329 ] = xx [ 355 ] * xx [ 356 ] ; xx [ 357 ] = xx [ 307
] * xx [ 334 ] + xx [ 332 ] * xx [ 327 ] + xx [ 354 ] * xx [ 316 ] ; xx [ 307
] = xx [ 357 ] * xx [ 331 ] ; xx [ 316 ] = xx [ 357 ] * xx [ 355 ] ; xx [ 327
] = xx [ 356 ] * xx [ 331 ] ; xx [ 332 ] = xx [ 357 ] * xx [ 356 ] ; xx [ 334
] = xx [ 355 ] * xx [ 331 ] ; xx [ 358 ] = ( xx [ 353 ] + xx [ 355 ] * xx [
355 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 359 ] = xx [ 2 ] * ( xx [ 329 ] + xx [
307 ] ) ; xx [ 360 ] = ( xx [ 316 ] - xx [ 327 ] ) * xx [ 2 ] ; xx [ 361 ] =
( xx [ 329 ] - xx [ 307 ] ) * xx [ 2 ] ; xx [ 362 ] = ( xx [ 353 ] + xx [ 356
] * xx [ 356 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 363 ] = xx [ 2 ] * ( xx [ 332
] + xx [ 334 ] ) ; xx [ 364 ] = xx [ 2 ] * ( xx [ 316 ] + xx [ 327 ] ) ; xx [
365 ] = ( xx [ 332 ] - xx [ 334 ] ) * xx [ 2 ] ; xx [ 366 ] = ( xx [ 353 ] +
xx [ 357 ] * xx [ 357 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 307 ] =
8.132207341052626e-3 ; xx [ 316 ] = 0.3264823273303549 ; xx [ 327 ] = xx [ 31
] * state [ 23 ] ; xx [ 329 ] = cos ( xx [ 327 ] ) ; xx [ 332 ] =
0.6272234768732393 ; xx [ 334 ] = sin ( xx [ 327 ] ) ; xx [ 327 ] = xx [ 316
] * xx [ 329 ] + xx [ 332 ] * xx [ 334 ] ; xx [ 353 ] = xx [ 327 ] * xx [ 327
] ; xx [ 367 ] = xx [ 316 ] * xx [ 334 ] - xx [ 332 ] * xx [ 329 ] ; xx [ 332
] = ( xx [ 353 ] + xx [ 367 ] * xx [ 367 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [
368 ] = 0.655789040106415 ; xx [ 369 ] = 0.2644631068340282 ; xx [ 370 ] = xx
[ 368 ] * xx [ 329 ] - xx [ 369 ] * xx [ 334 ] ; xx [ 371 ] = xx [ 370 ] * xx
[ 367 ] ; xx [ 372 ] = xx [ 369 ] * xx [ 329 ] + xx [ 368 ] * xx [ 334 ] ; xx
[ 329 ] = xx [ 372 ] * xx [ 327 ] ; xx [ 334 ] = xx [ 2 ] * ( xx [ 371 ] + xx
[ 329 ] ) ; xx [ 373 ] = xx [ 372 ] * xx [ 367 ] ; xx [ 374 ] = xx [ 327 ] *
xx [ 370 ] ; xx [ 375 ] = ( xx [ 373 ] - xx [ 374 ] ) * xx [ 2 ] ; xx [ 376 ]
= ( xx [ 371 ] - xx [ 329 ] ) * xx [ 2 ] ; xx [ 329 ] = ( xx [ 353 ] + xx [
370 ] * xx [ 370 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 371 ] = xx [ 372 ] * xx [
370 ] ; xx [ 377 ] = xx [ 327 ] * xx [ 367 ] ; xx [ 378 ] = xx [ 2 ] * ( xx [
371 ] + xx [ 377 ] ) ; xx [ 379 ] = xx [ 2 ] * ( xx [ 373 ] + xx [ 374 ] ) ;
xx [ 373 ] = ( xx [ 371 ] - xx [ 377 ] ) * xx [ 2 ] ; xx [ 371 ] = ( xx [ 353
] + xx [ 372 ] * xx [ 372 ] ) * xx [ 2 ] - xx [ 11 ] ; xx [ 380 ] = xx [ 332
] ; xx [ 381 ] = xx [ 334 ] ; xx [ 382 ] = xx [ 375 ] ; xx [ 383 ] = xx [ 376
] ; xx [ 384 ] = xx [ 329 ] ; xx [ 385 ] = xx [ 378 ] ; xx [ 386 ] = xx [ 379
] ; xx [ 387 ] = xx [ 373 ] ; xx [ 388 ] = xx [ 371 ] ; xx [ 353 ] =
1.4627166144976e-20 ; if ( ii [ 0 ] != 0 ) { return
sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'simple_robot/Revolute11' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } memcpy ( xx + 374 , xx + 90 , 1 * sizeof ( double ) ) ; xx
[ 377 ] = xx [ 353 ] / xx [ 374 ] ; xx [ 389 ] = xx [ 29 ] - xx [ 353 ] * xx
[ 377 ] ; xx [ 390 ] = xx [ 29 ] * xx [ 332 ] ; xx [ 391 ] = xx [ 29 ] * xx [
376 ] ; xx [ 392 ] = xx [ 29 ] * xx [ 379 ] ; xx [ 393 ] = xx [ 334 ] * xx [
389 ] ; xx [ 394 ] = xx [ 389 ] * xx [ 329 ] ; xx [ 395 ] = xx [ 373 ] * xx [
389 ] ; xx [ 396 ] = xx [ 29 ] * xx [ 375 ] ; xx [ 397 ] = xx [ 29 ] * xx [
378 ] ; xx [ 398 ] = xx [ 29 ] * xx [ 371 ] ; pm_math_Matrix3x3_compose_ra (
xx + 380 , xx + 390 , xx + 399 ) ; xx [ 389 ] = xx [ 307 ] + xx [ 399 ] ; xx
[ 390 ] = 0.03371964061805995 ; xx [ 391 ] = xx [ 271 ] * xx [ 370 ] ; xx [
392 ] = xx [ 372 ] * xx [ 271 ] ; xx [ 393 ] = 0.02511221626888745 ; xx [ 394
] = - ( xx [ 271 ] - ( xx [ 391 ] * xx [ 370 ] + xx [ 372 ] * xx [ 392 ] ) *
xx [ 2 ] ) ; xx [ 395 ] = - ( xx [ 393 ] + ( xx [ 391 ] * xx [ 367 ] - xx [
327 ] * xx [ 392 ] ) * xx [ 2 ] ) ; xx [ 396 ] = 0.03042104860947364 - xx [ 2
] * ( xx [ 392 ] * xx [ 367 ] + xx [ 327 ] * xx [ 391 ] ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 399 , xx + 394 , xx + 408 ) ; xx [ 391
] = xx [ 64 ] * xx [ 377 ] ; xx [ 392 ] = xx [ 391 ] * xx [ 334 ] ; xx [ 397
] = xx [ 376 ] * xx [ 392 ] ; xx [ 398 ] = xx [ 409 ] - xx [ 397 ] ; xx [ 399
] = xx [ 379 ] * xx [ 392 ] ; xx [ 417 ] = xx [ 410 ] - xx [ 399 ] ; xx [ 418
] = xx [ 389 ] * xx [ 390 ] - ( xx [ 398 ] * xx [ 330 ] - xx [ 417 ] * xx [
333 ] ) ; xx [ 419 ] = 4.22636105797123e-6 ; xx [ 420 ] = xx [ 64 ] / xx [
374 ] ; xx [ 421 ] = xx [ 64 ] - xx [ 64 ] * xx [ 420 ] ; xx [ 422 ] = xx [
421 ] * xx [ 332 ] ; xx [ 423 ] = xx [ 376 ] * xx [ 421 ] ; xx [ 424 ] = xx [
379 ] * xx [ 421 ] ; xx [ 425 ] = xx [ 138 ] * xx [ 334 ] ; xx [ 426 ] = xx [
138 ] * xx [ 329 ] ; xx [ 427 ] = xx [ 138 ] * xx [ 373 ] ; xx [ 428 ] = xx [
139 ] * xx [ 375 ] ; xx [ 429 ] = xx [ 139 ] * xx [ 378 ] ; xx [ 430 ] = xx [
139 ] * xx [ 371 ] ; pm_math_Matrix3x3_compose_ra ( xx + 380 , xx + 422 , xx
+ 431 ) ; xx [ 334 ] = xx [ 392 ] * xx [ 332 ] ; xx [ 371 ] = xx [ 391 ] * xx
[ 329 ] ; xx [ 329 ] = xx [ 371 ] * xx [ 332 ] ; xx [ 375 ] = xx [ 373 ] * xx
[ 391 ] ; xx [ 373 ] = xx [ 375 ] * xx [ 332 ] ; xx [ 332 ] = xx [ 376 ] * xx
[ 371 ] ; xx [ 378 ] = xx [ 376 ] * xx [ 375 ] ; xx [ 376 ] = xx [ 379 ] * xx
[ 371 ] ; xx [ 371 ] = xx [ 379 ] * xx [ 375 ] ; xx [ 379 ] = xx [ 334 ] ; xx
[ 380 ] = xx [ 329 ] ; xx [ 381 ] = xx [ 373 ] ; xx [ 382 ] = xx [ 397 ] ; xx
[ 383 ] = xx [ 332 ] ; xx [ 384 ] = xx [ 378 ] ; xx [ 385 ] = xx [ 399 ] ; xx
[ 386 ] = xx [ 376 ] ; xx [ 387 ] = xx [ 371 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 379 , xx + 394 , xx + 421 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 408 , xx + 394 , xx + 379 ) ; xx [ 375 ]
= xx [ 419 ] + xx [ 435 ] - xx [ 425 ] - xx [ 425 ] - xx [ 383 ] ; xx [ 388 ]
= xx [ 436 ] - xx [ 426 ] - xx [ 428 ] - xx [ 384 ] ; xx [ 391 ] = xx [ 375 ]
* xx [ 330 ] - xx [ 333 ] * xx [ 388 ] - xx [ 398 ] * xx [ 390 ] ; xx [ 392 ]
= xx [ 438 ] - xx [ 428 ] - xx [ 426 ] - xx [ 386 ] ; xx [ 397 ] =
3.521379421456925e-7 ; xx [ 399 ] = xx [ 397 ] + xx [ 439 ] - xx [ 429 ] - xx
[ 429 ] - xx [ 387 ] ; xx [ 409 ] = xx [ 330 ] * xx [ 392 ] - xx [ 399 ] * xx
[ 333 ] - xx [ 417 ] * xx [ 390 ] ; xx [ 410 ] = xx [ 330 ] * xx [ 391 ] - xx
[ 333 ] * xx [ 409 ] + xx [ 390 ] * xx [ 418 ] ; ii [ 1 ] =
factorSymmetricPosDef ( xx + 410 , 1 , xx + 430 ) ; if ( ii [ 1 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'simple_robot/Revolute' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 430 ] = xx [ 418 ] / xx [ 410 ] ; xx [ 440 ] = xx [
412 ] - xx [ 332 ] ; xx [ 332 ] = xx [ 413 ] - xx [ 376 ] ; xx [ 376 ] = xx [
390 ] * xx [ 402 ] - ( xx [ 440 ] * xx [ 330 ] - xx [ 332 ] * xx [ 333 ] ) ;
xx [ 412 ] = xx [ 430 ] * xx [ 376 ] ; xx [ 413 ] = xx [ 415 ] - xx [ 378 ] ;
xx [ 378 ] = xx [ 416 ] - xx [ 371 ] ; xx [ 371 ] = xx [ 390 ] * xx [ 405 ] -
( xx [ 413 ] * xx [ 330 ] - xx [ 378 ] * xx [ 333 ] ) ; xx [ 415 ] = xx [ 430
] * xx [ 371 ] ; xx [ 416 ] = xx [ 376 ] / xx [ 410 ] ; xx [ 441 ] = xx [ 416
] * xx [ 371 ] ; xx [ 442 ] = xx [ 371 ] / xx [ 410 ] ; xx [ 443 ] = xx [ 389
] - xx [ 430 ] * xx [ 418 ] ; xx [ 444 ] = xx [ 400 ] - xx [ 412 ] ; xx [ 445
] = xx [ 401 ] - xx [ 415 ] ; xx [ 446 ] = xx [ 402 ] - xx [ 412 ] ; xx [ 447
] = xx [ 403 ] - xx [ 416 ] * xx [ 376 ] + xx [ 307 ] ; xx [ 448 ] = xx [ 404
] - xx [ 441 ] ; xx [ 449 ] = xx [ 405 ] - xx [ 415 ] ; xx [ 450 ] = xx [ 406
] - xx [ 441 ] ; xx [ 451 ] = xx [ 407 ] - xx [ 442 ] * xx [ 371 ] + xx [ 307
] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 443 , xx + 358 , xx + 452 )
; pm_math_Matrix3x3_compose_ra ( xx + 358 , xx + 452 , xx + 443 ) ; xx [ 412
] = 1.841270583852046 ; xx [ 415 ] = 0.6153577354298877 ; xx [ 441 ] = xx [
31 ] * state [ 17 ] ; xx [ 452 ] = cos ( xx [ 441 ] ) ; xx [ 453 ] =
0.3459611930551978 ; xx [ 454 ] = sin ( xx [ 441 ] ) ; xx [ 441 ] = xx [ 330
] * xx [ 454 ] ; xx [ 455 ] = 0.3585189984711593 ; xx [ 456 ] =
0.09645212535708714 ; xx [ 457 ] = xx [ 456 ] * xx [ 454 ] ; xx [ 454 ] = xx
[ 415 ] * xx [ 452 ] + xx [ 453 ] * xx [ 441 ] - xx [ 455 ] * xx [ 457 ] ; xx
[ 458 ] = xx [ 454 ] * xx [ 454 ] ; xx [ 459 ] = xx [ 354 ] * xx [ 452 ] - xx
[ 453 ] * xx [ 457 ] - xx [ 455 ] * xx [ 441 ] ; xx [ 460 ] = xx [ 453 ] * xx
[ 452 ] - xx [ 415 ] * xx [ 441 ] + xx [ 354 ] * xx [ 457 ] ; xx [ 453 ] = xx
[ 459 ] * xx [ 460 ] ; xx [ 461 ] = xx [ 415 ] * xx [ 457 ] + xx [ 455 ] * xx
[ 452 ] + xx [ 354 ] * xx [ 441 ] ; xx [ 354 ] = xx [ 461 ] * xx [ 454 ] ; xx
[ 415 ] = xx [ 461 ] * xx [ 459 ] ; xx [ 441 ] = xx [ 460 ] * xx [ 454 ] ; xx
[ 452 ] = xx [ 461 ] * xx [ 460 ] ; xx [ 455 ] = xx [ 459 ] * xx [ 454 ] ; xx
[ 462 ] = ( xx [ 458 ] + xx [ 459 ] * xx [ 459 ] ) * xx [ 2 ] - xx [ 11 ] ;
xx [ 463 ] = xx [ 2 ] * ( xx [ 453 ] + xx [ 354 ] ) ; xx [ 464 ] = ( xx [ 415
] - xx [ 441 ] ) * xx [ 2 ] ; xx [ 465 ] = ( xx [ 453 ] - xx [ 354 ] ) * xx [
2 ] ; xx [ 466 ] = ( xx [ 458 ] + xx [ 460 ] * xx [ 460 ] ) * xx [ 2 ] - xx [
11 ] ; xx [ 467 ] = xx [ 2 ] * ( xx [ 452 ] + xx [ 455 ] ) ; xx [ 468 ] = xx
[ 2 ] * ( xx [ 415 ] + xx [ 441 ] ) ; xx [ 469 ] = ( xx [ 452 ] - xx [ 455 ]
) * xx [ 2 ] ; xx [ 470 ] = ( xx [ 458 ] + xx [ 461 ] * xx [ 461 ] ) * xx [ 2
] - xx [ 11 ] ; xx [ 354 ] = xx [ 31 ] * state [ 19 ] ; xx [ 31 ] = cos ( xx
[ 354 ] ) ; xx [ 415 ] = 0.6272234768732394 ; xx [ 441 ] = sin ( xx [ 354 ] )
; xx [ 354 ] = xx [ 316 ] * xx [ 31 ] + xx [ 415 ] * xx [ 441 ] ; xx [ 452 ]
= xx [ 354 ] * xx [ 354 ] ; xx [ 453 ] = xx [ 316 ] * xx [ 441 ] - xx [ 415 ]
* xx [ 31 ] ; xx [ 316 ] = ( xx [ 452 ] + xx [ 453 ] * xx [ 453 ] ) * xx [ 2
] - xx [ 11 ] ; xx [ 415 ] = xx [ 368 ] * xx [ 31 ] - xx [ 369 ] * xx [ 441 ]
; xx [ 455 ] = xx [ 415 ] * xx [ 453 ] ; xx [ 457 ] = xx [ 369 ] * xx [ 31 ]
+ xx [ 368 ] * xx [ 441 ] ; xx [ 31 ] = xx [ 457 ] * xx [ 354 ] ; xx [ 368 ]
= xx [ 2 ] * ( xx [ 455 ] + xx [ 31 ] ) ; xx [ 369 ] = xx [ 457 ] * xx [ 453
] ; xx [ 441 ] = xx [ 354 ] * xx [ 415 ] ; xx [ 458 ] = ( xx [ 369 ] - xx [
441 ] ) * xx [ 2 ] ; xx [ 471 ] = ( xx [ 455 ] - xx [ 31 ] ) * xx [ 2 ] ; xx
[ 31 ] = ( xx [ 452 ] + xx [ 415 ] * xx [ 415 ] ) * xx [ 2 ] - xx [ 11 ] ; xx
[ 455 ] = xx [ 457 ] * xx [ 415 ] ; xx [ 472 ] = xx [ 354 ] * xx [ 453 ] ; xx
[ 473 ] = xx [ 2 ] * ( xx [ 455 ] + xx [ 472 ] ) ; xx [ 474 ] = xx [ 2 ] * (
xx [ 369 ] + xx [ 441 ] ) ; xx [ 369 ] = ( xx [ 455 ] - xx [ 472 ] ) * xx [ 2
] ; xx [ 441 ] = ( xx [ 452 ] + xx [ 457 ] * xx [ 457 ] ) * xx [ 2 ] - xx [
11 ] ; xx [ 475 ] = xx [ 316 ] ; xx [ 476 ] = xx [ 368 ] ; xx [ 477 ] = xx [
458 ] ; xx [ 478 ] = xx [ 471 ] ; xx [ 479 ] = xx [ 31 ] ; xx [ 480 ] = xx [
473 ] ; xx [ 481 ] = xx [ 474 ] ; xx [ 482 ] = xx [ 369 ] ; xx [ 483 ] = xx [
441 ] ; xx [ 484 ] = xx [ 29 ] * xx [ 316 ] ; xx [ 485 ] = xx [ 29 ] * xx [
471 ] ; xx [ 486 ] = xx [ 29 ] * xx [ 474 ] ; xx [ 487 ] = xx [ 29 ] * xx [
368 ] ; xx [ 488 ] = xx [ 29 ] * xx [ 31 ] ; xx [ 489 ] = xx [ 29 ] * xx [
369 ] ; xx [ 490 ] = xx [ 29 ] * xx [ 458 ] ; xx [ 491 ] = xx [ 29 ] * xx [
473 ] ; xx [ 492 ] = xx [ 29 ] * xx [ 441 ] ; pm_math_Matrix3x3_compose_ra (
xx + 475 , xx + 484 , xx + 493 ) ; xx [ 452 ] = xx [ 307 ] + xx [ 493 ] ; xx
[ 455 ] = 0.0337196406180599 ; xx [ 472 ] = xx [ 271 ] * xx [ 415 ] ; xx [
484 ] = xx [ 457 ] * xx [ 271 ] ; xx [ 485 ] = - ( xx [ 271 ] - ( xx [ 472 ]
* xx [ 415 ] + xx [ 457 ] * xx [ 484 ] ) * xx [ 2 ] ) ; xx [ 486 ] = - ( xx [
393 ] + ( xx [ 472 ] * xx [ 453 ] - xx [ 354 ] * xx [ 484 ] ) * xx [ 2 ] ) ;
xx [ 487 ] = 0.03042104860947363 - xx [ 2 ] * ( xx [ 484 ] * xx [ 453 ] + xx
[ 354 ] * xx [ 472 ] ) ; pm_math_Matrix3x3_postCross_ra ( xx + 493 , xx + 485
, xx + 502 ) ; xx [ 271 ] = xx [ 452 ] * xx [ 455 ] - ( xx [ 330 ] * xx [ 503
] - xx [ 456 ] * xx [ 504 ] ) ; if ( ii [ 0 ] != 0 ) { return
sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'simple_robot/Revolute10' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 393 ] = xx [ 64 ] / xx [ 90 ] ; xx [ 472 ] = xx [ 64 ]
- xx [ 64 ] * xx [ 393 ] ; xx [ 511 ] = xx [ 472 ] * xx [ 316 ] ; xx [ 512 ]
= xx [ 471 ] * xx [ 472 ] ; xx [ 513 ] = xx [ 474 ] * xx [ 472 ] ; xx [ 514 ]
= xx [ 138 ] * xx [ 368 ] ; xx [ 515 ] = xx [ 138 ] * xx [ 31 ] ; xx [ 516 ]
= xx [ 138 ] * xx [ 369 ] ; xx [ 517 ] = xx [ 139 ] * xx [ 458 ] ; xx [ 518 ]
= xx [ 139 ] * xx [ 473 ] ; xx [ 519 ] = xx [ 139 ] * xx [ 441 ] ;
pm_math_Matrix3x3_compose_ra ( xx + 475 , xx + 511 , xx + 520 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 502 , xx + 485 , xx + 471 ) ; xx [ 31 ]
= xx [ 419 ] + xx [ 524 ] - xx [ 475 ] ; xx [ 316 ] = xx [ 525 ] - xx [ 476 ]
; xx [ 368 ] = xx [ 31 ] * xx [ 330 ] - xx [ 456 ] * xx [ 316 ] - xx [ 455 ]
* xx [ 503 ] ; xx [ 369 ] = xx [ 527 ] - xx [ 478 ] ; xx [ 441 ] = xx [ 397 ]
+ xx [ 528 ] - xx [ 479 ] ; xx [ 458 ] = xx [ 330 ] * xx [ 369 ] - xx [ 441 ]
* xx [ 456 ] - xx [ 455 ] * xx [ 504 ] ; xx [ 480 ] = xx [ 330 ] * xx [ 368 ]
- xx [ 456 ] * xx [ 458 ] + xx [ 455 ] * xx [ 271 ] ; ii [ 0 ] =
factorSymmetricPosDef ( xx + 480 , 1 , xx + 481 ) ; if ( ii [ 0 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'simple_robot/Revolute1' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 481 ] = xx [ 271 ] / xx [ 480 ] ; xx [ 482 ] = xx [
455 ] * xx [ 496 ] - ( xx [ 330 ] * xx [ 506 ] - xx [ 456 ] * xx [ 507 ] ) ;
xx [ 483 ] = xx [ 481 ] * xx [ 482 ] ; xx [ 484 ] = xx [ 455 ] * xx [ 499 ] -
( xx [ 330 ] * xx [ 509 ] - xx [ 456 ] * xx [ 510 ] ) ; xx [ 488 ] = xx [ 481
] * xx [ 484 ] ; xx [ 489 ] = xx [ 482 ] / xx [ 480 ] ; xx [ 490 ] = xx [ 489
] * xx [ 484 ] ; xx [ 491 ] = xx [ 484 ] / xx [ 480 ] ; xx [ 511 ] = xx [ 452
] - xx [ 481 ] * xx [ 271 ] ; xx [ 512 ] = xx [ 494 ] - xx [ 483 ] ; xx [ 513
] = xx [ 495 ] - xx [ 488 ] ; xx [ 514 ] = xx [ 496 ] - xx [ 483 ] ; xx [ 515
] = xx [ 497 ] - xx [ 489 ] * xx [ 482 ] + xx [ 307 ] ; xx [ 516 ] = xx [ 498
] - xx [ 490 ] ; xx [ 517 ] = xx [ 499 ] - xx [ 488 ] ; xx [ 518 ] = xx [ 500
] - xx [ 490 ] ; xx [ 519 ] = xx [ 501 ] - xx [ 491 ] * xx [ 484 ] + xx [ 307
] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 511 , xx + 462 , xx + 529 )
; pm_math_Matrix3x3_compose_ra ( xx + 462 , xx + 529 , xx + 511 ) ; xx [ 483
] = xx [ 162 ] + xx [ 261 ] + xx [ 344 ] + xx [ 443 ] + xx [ 412 ] + xx [ 511
] ; xx [ 488 ] = xx [ 125 ] - xx [ 89 ] ; xx [ 89 ] = xx [ 156 ] - xx [ 147 ]
- xx [ 145 ] - xx [ 99 ] ; xx [ 125 ] = xx [ 155 ] - xx [ 146 ] - xx [ 142 ]
- xx [ 98 ] ; xx [ 490 ] = xx [ 34 ] + xx [ 131 ] ; xx [ 34 ] = xx [ 35 ] *
xx [ 89 ] - xx [ 38 ] * xx [ 125 ] - xx [ 490 ] * xx [ 104 ] ; xx [ 131 ] =
xx [ 128 ] - xx [ 91 ] ; xx [ 529 ] = - ( xx [ 133 ] + xx [ 137 ] * xx [ 39 ]
) ; xx [ 530 ] = - ( xx [ 56 ] + xx [ 159 ] * xx [ 39 ] ) ; xx [ 531 ] = - (
xx [ 124 ] + xx [ 161 ] * xx [ 39 ] ) ; xx [ 532 ] = - ( xx [ 113 ] + xx [
137 ] * xx [ 88 ] ) ; xx [ 533 ] = - ( xx [ 158 ] + xx [ 159 ] * xx [ 88 ] )
; xx [ 534 ] = - ( xx [ 107 ] + xx [ 161 ] * xx [ 88 ] ) ; xx [ 535 ] = - (
xx [ 488 ] + xx [ 137 ] * xx [ 34 ] ) ; xx [ 536 ] = - ( xx [ 131 ] + xx [
159 ] * xx [ 34 ] ) ; xx [ 537 ] = - ( xx [ 490 ] + xx [ 161 ] * xx [ 34 ] )
; pm_math_Matrix3x3_composeTranspose_ra ( xx + 529 , xx + 78 , xx + 538 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 78 , xx + 538 , xx + 529 ) ; xx [ 91 ] =
4.244406610027172e-3 ; xx [ 538 ] = xx [ 58 ] ; xx [ 539 ] = xx [ 61 ] ; xx [
540 ] = xx [ 62 ] ; xx [ 128 ] = 0.06404981182865328 ; xx [ 492 ] = xx [ 128
] * xx [ 62 ] ; xx [ 493 ] = xx [ 91 ] * xx [ 62 ] ; xx [ 541 ] = xx [ 128 ]
* xx [ 58 ] - xx [ 61 ] * xx [ 91 ] ; xx [ 542 ] = - xx [ 492 ] ; xx [ 543 ]
= xx [ 493 ] ; xx [ 544 ] = xx [ 541 ] ; pm_math_Vector3_cross_ra ( xx + 538
, xx + 542 , xx + 545 ) ; xx [ 538 ] = - ( 0.07045837366530477 + xx [ 91 ] +
xx [ 2 ] * ( xx [ 545 ] + xx [ 492 ] * xx [ 55 ] ) ) ; xx [ 539 ] =
0.1208316631353296 - ( xx [ 128 ] + ( xx [ 546 ] - xx [ 493 ] * xx [ 55 ] ) *
xx [ 2 ] ) ; xx [ 540 ] = - ( 0.1031385412966565 + ( xx [ 547 ] - xx [ 541 ]
* xx [ 55 ] ) * xx [ 2 ] ) ; pm_math_Matrix3x3_postCross_ra ( xx + 162 , xx +
538 , xx + 541 ) ; xx [ 91 ] = xx [ 231 ] - xx [ 197 ] ; xx [ 128 ] = xx [
257 ] - xx [ 247 ] - xx [ 245 ] - xx [ 207 ] ; xx [ 162 ] = xx [ 256 ] - xx [
246 ] - xx [ 242 ] - xx [ 206 ] ; xx [ 197 ] = xx [ 237 ] - xx [ 175 ] ; xx [
175 ] = xx [ 35 ] * xx [ 128 ] - xx [ 38 ] * xx [ 162 ] - xx [ 197 ] * xx [
210 ] ; xx [ 231 ] = xx [ 234 ] - xx [ 199 ] ; xx [ 550 ] = - ( xx [ 219 ] +
xx [ 235 ] * xx [ 179 ] ) ; xx [ 551 ] = - ( xx [ 171 ] + xx [ 249 ] * xx [
179 ] ) ; xx [ 552 ] = - ( xx [ 218 ] + xx [ 260 ] * xx [ 179 ] ) ; xx [ 553
] = - ( xx [ 217 ] + xx [ 235 ] * xx [ 196 ] ) ; xx [ 554 ] = - ( xx [ 236 ]
+ xx [ 249 ] * xx [ 196 ] ) ; xx [ 555 ] = - ( xx [ 211 ] + xx [ 260 ] * xx [
196 ] ) ; xx [ 556 ] = - ( xx [ 91 ] + xx [ 235 ] * xx [ 175 ] ) ; xx [ 557 ]
= - ( xx [ 231 ] + xx [ 249 ] * xx [ 175 ] ) ; xx [ 558 ] = - ( xx [ 197 ] +
xx [ 260 ] * xx [ 175 ] ) ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 550
, xx + 183 , xx + 559 ) ; pm_math_Matrix3x3_compose_ra ( xx + 183 , xx + 559
, xx + 550 ) ; xx [ 199 ] = 4.244406610027171e-3 ; xx [ 234 ] = - xx [ 178 ]
; xx [ 559 ] = xx [ 177 ] ; xx [ 560 ] = xx [ 234 ] ; xx [ 561 ] = xx [ 180 ]
; xx [ 237 ] = 0.06404981182865327 ; xx [ 492 ] = xx [ 237 ] * xx [ 180 ] ;
xx [ 493 ] = xx [ 199 ] * xx [ 180 ] ; xx [ 562 ] = xx [ 237 ] * xx [ 177 ] +
xx [ 178 ] * xx [ 199 ] ; xx [ 563 ] = - xx [ 492 ] ; xx [ 564 ] = xx [ 493 ]
; xx [ 565 ] = xx [ 562 ] ; pm_math_Vector3_cross_ra ( xx + 559 , xx + 563 ,
xx + 566 ) ; xx [ 559 ] = - ( 0.09519342531851044 + xx [ 199 ] + xx [ 2 ] * (
xx [ 566 ] - xx [ 492 ] * xx [ 127 ] ) ) ; xx [ 560 ] = - ( 0.136912200903329
+ xx [ 237 ] + ( xx [ 493 ] * xx [ 127 ] + xx [ 567 ] ) * xx [ 2 ] ) ; xx [
561 ] = - ( 0.09426587708939985 + ( xx [ 562 ] * xx [ 127 ] + xx [ 568 ] ) *
xx [ 2 ] ) ; pm_math_Matrix3x3_postCross_ra ( xx + 261 , xx + 559 , xx + 562
) ; xx [ 178 ] = xx [ 342 ] - xx [ 295 ] ; xx [ 199 ] = xx [ 341 ] - xx [ 294
] ; xx [ 237 ] = xx [ 35 ] * xx [ 178 ] - xx [ 38 ] * xx [ 199 ] - xx [ 259 ]
* xx [ 325 ] ; xx [ 571 ] = - ( xx [ 317 ] + xx [ 305 ] * xx [ 285 ] ) ; xx [
572 ] = - ( xx [ 320 ] + xx [ 326 ] * xx [ 285 ] ) ; xx [ 573 ] = - ( xx [
323 ] + xx [ 328 ] * xx [ 285 ] ) ; xx [ 574 ] = - ( xx [ 318 ] + xx [ 305 ]
* xx [ 126 ] ) ; xx [ 575 ] = - ( xx [ 321 ] + xx [ 326 ] * xx [ 126 ] ) ; xx
[ 576 ] = - ( xx [ 324 ] + xx [ 328 ] * xx [ 126 ] ) ; xx [ 577 ] = - ( xx [
319 ] + xx [ 305 ] * xx [ 237 ] ) ; xx [ 578 ] = - ( xx [ 322 ] + xx [ 326 ]
* xx [ 237 ] ) ; xx [ 579 ] = - ( xx [ 325 ] + xx [ 328 ] * xx [ 237 ] ) ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 571 , xx + 275 , xx + 580 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 275 , xx + 580 , xx + 571 ) ; xx [ 261 ]
= 4.244406610027125e-3 ; xx [ 492 ] = - xx [ 273 ] ; xx [ 580 ] = xx [ 272 ]
; xx [ 581 ] = xx [ 492 ] ; xx [ 582 ] = xx [ 274 ] ; xx [ 493 ] =
0.06404981182865323 ; xx [ 583 ] = xx [ 493 ] * xx [ 274 ] ; xx [ 584 ] = xx
[ 261 ] * xx [ 274 ] ; xx [ 585 ] = xx [ 493 ] * xx [ 272 ] + xx [ 273 ] * xx
[ 261 ] ; xx [ 586 ] = - xx [ 583 ] ; xx [ 587 ] = xx [ 584 ] ; xx [ 588 ] =
xx [ 585 ] ; pm_math_Vector3_cross_ra ( xx + 580 , xx + 586 , xx + 589 ) ; xx
[ 580 ] = 0.01262769948857074 - ( xx [ 261 ] + xx [ 2 ] * ( xx [ 589 ] - xx [
583 ] * xx [ 232 ] ) ) ; xx [ 581 ] = - ( 0.1408927034847847 + xx [ 493 ] + (
xx [ 584 ] * xx [ 232 ] + xx [ 590 ] ) * xx [ 2 ] ) ; xx [ 582 ] =
0.09068544496193037 - ( xx [ 585 ] * xx [ 232 ] + xx [ 591 ] ) * xx [ 2 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 344 , xx + 580 , xx + 583 ) ; xx [ 261
] = xx [ 408 ] - xx [ 334 ] ; xx [ 273 ] = xx [ 432 ] - xx [ 422 ] - xx [ 424
] - xx [ 380 ] ; xx [ 334 ] = xx [ 433 ] - xx [ 423 ] - xx [ 427 ] - xx [ 381
] ; xx [ 344 ] = xx [ 330 ] * xx [ 273 ] - xx [ 333 ] * xx [ 334 ] - xx [ 261
] * xx [ 390 ] ; xx [ 408 ] = xx [ 411 ] - xx [ 329 ] ; xx [ 329 ] = xx [ 414
] - xx [ 373 ] ; xx [ 592 ] = - ( xx [ 261 ] + xx [ 430 ] * xx [ 344 ] ) ; xx
[ 593 ] = - ( xx [ 408 ] + xx [ 416 ] * xx [ 344 ] ) ; xx [ 594 ] = - ( xx [
329 ] + xx [ 442 ] * xx [ 344 ] ) ; xx [ 595 ] = - ( xx [ 398 ] + xx [ 430 ]
* xx [ 391 ] ) ; xx [ 596 ] = - ( xx [ 440 ] + xx [ 416 ] * xx [ 391 ] ) ; xx
[ 597 ] = - ( xx [ 413 ] + xx [ 442 ] * xx [ 391 ] ) ; xx [ 598 ] = - ( xx [
417 ] + xx [ 430 ] * xx [ 409 ] ) ; xx [ 599 ] = - ( xx [ 332 ] + xx [ 416 ]
* xx [ 409 ] ) ; xx [ 600 ] = - ( xx [ 378 ] + xx [ 442 ] * xx [ 409 ] ) ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 592 , xx + 358 , xx + 601 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 358 , xx + 601 , xx + 592 ) ; xx [ 373 ]
= 0.01043798038823735 ; xx [ 411 ] = 0.03286610935552551 ; xx [ 414 ] = xx [
357 ] * xx [ 373 ] - xx [ 411 ] * xx [ 356 ] ; xx [ 493 ] = xx [ 411 ] * xx [
355 ] ; xx [ 601 ] = xx [ 373 ] * xx [ 355 ] ; xx [ 602 ] = xx [ 414 ] ; xx [
603 ] = xx [ 493 ] ; xx [ 604 ] = - xx [ 601 ] ; pm_math_Vector3_cross_ra (
xx + 355 , xx + 602 , xx + 605 ) ; xx [ 602 ] = 0.01161227146344391 - ( xx [
605 ] - xx [ 414 ] * xx [ 331 ] ) * xx [ 2 ] ; xx [ 603 ] = - (
0.04870513239528159 + xx [ 2 ] * ( xx [ 606 ] - xx [ 493 ] * xx [ 331 ] ) -
xx [ 373 ] ) ; xx [ 604 ] = 0.04944635745027942 - ( ( xx [ 601 ] * xx [ 331 ]
+ xx [ 607 ] ) * xx [ 2 ] - xx [ 411 ] ) ; pm_math_Matrix3x3_postCross_ra (
xx + 443 , xx + 602 , xx + 605 ) ; xx [ 373 ] = xx [ 521 ] - xx [ 472 ] ; xx
[ 411 ] = xx [ 522 ] - xx [ 473 ] ; xx [ 414 ] = xx [ 330 ] * xx [ 373 ] - xx
[ 456 ] * xx [ 411 ] - xx [ 455 ] * xx [ 502 ] ; xx [ 614 ] = - ( xx [ 502 ]
+ xx [ 481 ] * xx [ 414 ] ) ; xx [ 615 ] = - ( xx [ 505 ] + xx [ 489 ] * xx [
414 ] ) ; xx [ 616 ] = - ( xx [ 508 ] + xx [ 491 ] * xx [ 414 ] ) ; xx [ 617
] = - ( xx [ 503 ] + xx [ 481 ] * xx [ 368 ] ) ; xx [ 618 ] = - ( xx [ 506 ]
+ xx [ 489 ] * xx [ 368 ] ) ; xx [ 619 ] = - ( xx [ 509 ] + xx [ 491 ] * xx [
368 ] ) ; xx [ 620 ] = - ( xx [ 504 ] + xx [ 481 ] * xx [ 458 ] ) ; xx [ 621
] = - ( xx [ 507 ] + xx [ 489 ] * xx [ 458 ] ) ; xx [ 622 ] = - ( xx [ 510 ]
+ xx [ 491 ] * xx [ 458 ] ) ; pm_math_Matrix3x3_composeTranspose_ra ( xx +
614 , xx + 462 , xx + 623 ) ; pm_math_Matrix3x3_compose_ra ( xx + 462 , xx +
623 , xx + 614 ) ; xx [ 443 ] = 0.01043798038823734 ; xx [ 493 ] =
0.03286610935552547 ; xx [ 601 ] = xx [ 461 ] * xx [ 443 ] - xx [ 493 ] * xx
[ 460 ] ; xx [ 623 ] = xx [ 493 ] * xx [ 459 ] ; xx [ 624 ] = xx [ 443 ] * xx
[ 459 ] ; xx [ 625 ] = xx [ 601 ] ; xx [ 626 ] = xx [ 623 ] ; xx [ 627 ] = -
xx [ 624 ] ; pm_math_Vector3_cross_ra ( xx + 459 , xx + 625 , xx + 628 ) ; xx
[ 625 ] = 0.01858416102255839 - ( xx [ 628 ] - xx [ 601 ] * xx [ 454 ] ) * xx
[ 2 ] ; xx [ 626 ] = 0.02394326065483052 - ( xx [ 2 ] * ( xx [ 629 ] - xx [
623 ] * xx [ 454 ] ) - xx [ 443 ] ) ; xx [ 627 ] = 0.04694548396048914 - ( (
xx [ 624 ] * xx [ 454 ] + xx [ 630 ] ) * xx [ 2 ] - xx [ 493 ] ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 511 , xx + 625 , xx + 628 ) ; xx [ 443
] = xx [ 529 ] - xx [ 541 ] + xx [ 550 ] - xx [ 562 ] + xx [ 571 ] - xx [ 583
] + xx [ 592 ] - xx [ 605 ] + xx [ 614 ] - xx [ 628 ] ; xx [ 493 ] = xx [ 530
] - xx [ 544 ] + xx [ 551 ] - xx [ 565 ] + xx [ 572 ] - xx [ 586 ] + xx [ 593
] - xx [ 608 ] + xx [ 615 ] - xx [ 631 ] ; xx [ 511 ] = xx [ 531 ] - xx [ 547
] + xx [ 552 ] - xx [ 568 ] + xx [ 573 ] - xx [ 589 ] + xx [ 594 ] - xx [ 611
] + xx [ 616 ] - xx [ 634 ] ; xx [ 601 ] = xx [ 532 ] - xx [ 542 ] + xx [ 553
] - xx [ 563 ] + xx [ 574 ] - xx [ 584 ] + xx [ 595 ] - xx [ 606 ] + xx [ 617
] - xx [ 629 ] ; xx [ 623 ] = xx [ 533 ] - xx [ 545 ] + xx [ 554 ] - xx [ 566
] + xx [ 575 ] - xx [ 587 ] + xx [ 596 ] - xx [ 609 ] + xx [ 618 ] - xx [ 632
] ; xx [ 624 ] = xx [ 534 ] - xx [ 548 ] + xx [ 555 ] - xx [ 569 ] + xx [ 576
] - xx [ 590 ] + xx [ 597 ] - xx [ 612 ] + xx [ 619 ] - xx [ 635 ] ; xx [ 637
] = xx [ 535 ] - xx [ 543 ] + xx [ 556 ] - xx [ 564 ] + xx [ 577 ] - xx [ 585
] + xx [ 598 ] - xx [ 607 ] + xx [ 620 ] - xx [ 630 ] ; xx [ 638 ] = xx [ 536
] - xx [ 546 ] + xx [ 557 ] - xx [ 567 ] + xx [ 578 ] - xx [ 588 ] + xx [ 599
] - xx [ 610 ] + xx [ 621 ] - xx [ 633 ] ; xx [ 639 ] = xx [ 537 ] - xx [ 549
] + xx [ 558 ] - xx [ 570 ] + xx [ 579 ] - xx [ 591 ] + xx [ 600 ] - xx [ 613
] + xx [ 622 ] - xx [ 636 ] ; xx [ 640 ] = xx [ 443 ] ; xx [ 641 ] = xx [ 493
] ; xx [ 642 ] = xx [ 511 ] ; xx [ 643 ] = xx [ 601 ] ; xx [ 644 ] = xx [ 623
] ; xx [ 645 ] = xx [ 624 ] ; xx [ 646 ] = xx [ 637 ] ; xx [ 647 ] = xx [ 638
] ; xx [ 648 ] = xx [ 639 ] ; xx [ 649 ] = xx [ 41 ] ; xx [ 650 ] = xx [ 59 ]
; xx [ 651 ] = - xx [ 60 ] ; pm_math_Matrix3x3_transposeXform_ra ( xx + 640 ,
xx + 649 , xx + 652 ) ; xx [ 655 ] = xx [ 163 ] + xx [ 262 ] + xx [ 345 ] +
xx [ 444 ] + xx [ 512 ] ; xx [ 163 ] = xx [ 164 ] + xx [ 263 ] + xx [ 346 ] +
xx [ 445 ] + xx [ 513 ] ; xx [ 164 ] = xx [ 165 ] + xx [ 264 ] + xx [ 347 ] +
xx [ 446 ] + xx [ 514 ] ; xx [ 165 ] = xx [ 166 ] + xx [ 265 ] + xx [ 348 ] +
xx [ 447 ] + xx [ 412 ] + xx [ 515 ] ; xx [ 166 ] = xx [ 167 ] + xx [ 266 ] +
xx [ 349 ] + xx [ 448 ] + xx [ 516 ] ; xx [ 167 ] = xx [ 168 ] + xx [ 267 ] +
xx [ 350 ] + xx [ 449 ] + xx [ 517 ] ; xx [ 168 ] = xx [ 169 ] + xx [ 268 ] +
xx [ 351 ] + xx [ 450 ] + xx [ 518 ] ; xx [ 169 ] = xx [ 170 ] + xx [ 269 ] +
xx [ 352 ] + xx [ 451 ] + xx [ 412 ] + xx [ 519 ] ; xx [ 656 ] = xx [ 483 ] ;
xx [ 657 ] = xx [ 655 ] ; xx [ 658 ] = xx [ 163 ] ; xx [ 659 ] = xx [ 164 ] ;
xx [ 660 ] = xx [ 165 ] ; xx [ 661 ] = xx [ 166 ] ; xx [ 662 ] = xx [ 167 ] ;
xx [ 663 ] = xx [ 168 ] ; xx [ 664 ] = xx [ 169 ] ; xx [ 170 ] =
0.08539263296773184 ; xx [ 262 ] = 9.090437326692596e-3 ; xx [ 263 ] =
0.02601397390134001 ; xx [ 264 ] = - xx [ 170 ] ; xx [ 265 ] = xx [ 262 ] ;
xx [ 266 ] = xx [ 263 ] ; pm_math_Matrix3x3_xform_ra ( xx + 656 , xx + 264 ,
xx + 267 ) ; xx [ 345 ] = xx [ 652 ] + xx [ 267 ] ; xx [ 346 ] = xx [ 39 ] /
xx [ 132 ] ; xx [ 347 ] = xx [ 346 ] * xx [ 88 ] ; xx [ 348 ] = xx [ 151 ] -
xx [ 142 ] - xx [ 146 ] - xx [ 94 ] ; xx [ 349 ] = xx [ 346 ] * xx [ 34 ] ;
xx [ 350 ] = xx [ 88 ] / xx [ 132 ] ; xx [ 351 ] = xx [ 154 ] - xx [ 145 ] -
xx [ 147 ] - xx [ 97 ] ; xx [ 352 ] = xx [ 350 ] * xx [ 34 ] ; xx [ 92 ] = xx
[ 157 ] - xx [ 148 ] - xx [ 148 ] - xx [ 100 ] ; xx [ 93 ] = xx [ 34 ] / xx [
132 ] ; xx [ 94 ] = 4.412533750799012e-5 ; xx [ 140 ] = xx [ 123 ] - xx [ 346
] * xx [ 39 ] ; xx [ 141 ] = xx [ 108 ] - xx [ 347 ] ; xx [ 142 ] = xx [ 348
] - xx [ 349 ] ; xx [ 143 ] = xx [ 101 ] - xx [ 347 ] ; xx [ 144 ] = xx [ 57
] - xx [ 350 ] * xx [ 88 ] ; xx [ 145 ] = xx [ 351 ] - xx [ 352 ] ; xx [ 146
] = xx [ 125 ] - xx [ 349 ] ; xx [ 147 ] = xx [ 89 ] - xx [ 352 ] ; xx [ 148
] = xx [ 92 ] - xx [ 93 ] * xx [ 34 ] + xx [ 94 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 140 , xx + 78 , xx + 149 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 78 , xx + 149 , xx + 140 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 529 , xx + 538 , xx + 78 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 541 , xx + 538 , xx + 149 ) ; xx [ 95 ]
= xx [ 179 ] / xx [ 230 ] ; xx [ 96 ] = xx [ 95 ] * xx [ 196 ] ; xx [ 97 ] =
xx [ 252 ] - xx [ 242 ] - xx [ 246 ] - xx [ 202 ] ; xx [ 98 ] = xx [ 95 ] *
xx [ 175 ] ; xx [ 99 ] = xx [ 196 ] / xx [ 230 ] ; xx [ 100 ] = xx [ 255 ] -
xx [ 245 ] - xx [ 247 ] - xx [ 205 ] ; xx [ 347 ] = xx [ 99 ] * xx [ 175 ] ;
xx [ 200 ] = xx [ 258 ] - xx [ 248 ] - xx [ 248 ] - xx [ 208 ] ; xx [ 201 ] =
xx [ 175 ] / xx [ 230 ] ; xx [ 240 ] = xx [ 213 ] - xx [ 95 ] * xx [ 179 ] ;
xx [ 241 ] = xx [ 212 ] - xx [ 96 ] ; xx [ 242 ] = xx [ 97 ] - xx [ 98 ] ; xx
[ 243 ] = xx [ 209 ] - xx [ 96 ] ; xx [ 244 ] = xx [ 192 ] - xx [ 99 ] * xx [
196 ] ; xx [ 245 ] = xx [ 100 ] - xx [ 347 ] ; xx [ 246 ] = xx [ 162 ] - xx [
98 ] ; xx [ 247 ] = xx [ 128 ] - xx [ 347 ] ; xx [ 248 ] = xx [ 200 ] - xx [
201 ] * xx [ 175 ] + xx [ 94 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx +
240 , xx + 183 , xx + 250 ) ; pm_math_Matrix3x3_compose_ra ( xx + 183 , xx +
250 , xx + 240 ) ; pm_math_Matrix3x3_postCross_ra ( xx + 550 , xx + 559 , xx
+ 183 ) ; pm_math_Matrix3x3_preCross_ra ( xx + 562 , xx + 559 , xx + 250 ) ;
xx [ 96 ] = xx [ 285 ] / xx [ 298 ] ; xx [ 98 ] = xx [ 96 ] * xx [ 126 ] ; xx
[ 202 ] = xx [ 337 ] - xx [ 290 ] ; xx [ 203 ] = xx [ 96 ] * xx [ 237 ] ; xx
[ 204 ] = xx [ 126 ] / xx [ 298 ] ; xx [ 205 ] = xx [ 340 ] - xx [ 293 ] ; xx
[ 206 ] = xx [ 204 ] * xx [ 237 ] ; xx [ 207 ] = xx [ 343 ] - xx [ 296 ] ; xx
[ 208 ] = xx [ 237 ] / xx [ 298 ] ; xx [ 288 ] = xx [ 233 ] - xx [ 96 ] * xx
[ 285 ] ; xx [ 289 ] = xx [ 172 ] - xx [ 98 ] ; xx [ 290 ] = xx [ 202 ] - xx
[ 203 ] ; xx [ 291 ] = xx [ 37 ] - xx [ 98 ] ; xx [ 292 ] = xx [ 30 ] - xx [
204 ] * xx [ 126 ] ; xx [ 293 ] = xx [ 205 ] - xx [ 206 ] ; xx [ 294 ] = xx [
199 ] - xx [ 203 ] ; xx [ 295 ] = xx [ 178 ] - xx [ 206 ] ; xx [ 296 ] = xx [
207 ] - xx [ 208 ] * xx [ 237 ] + xx [ 94 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 288 , xx + 275 , xx + 335 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 275 , xx + 335 , xx + 288 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 571 , xx + 580 , xx + 275 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 583 , xx + 580 , xx + 335 ) ; xx [ 98 ]
= xx [ 431 ] - xx [ 421 ] - xx [ 421 ] - xx [ 379 ] ; xx [ 203 ] = xx [ 344 ]
/ xx [ 410 ] ; xx [ 206 ] = 4.378816592285135e-6 ; xx [ 347 ] = xx [ 203 ] *
xx [ 391 ] ; xx [ 349 ] = xx [ 203 ] * xx [ 409 ] ; xx [ 352 ] = xx [ 434 ] -
xx [ 424 ] - xx [ 422 ] - xx [ 382 ] ; xx [ 412 ] = xx [ 391 ] / xx [ 410 ] ;
xx [ 421 ] = xx [ 412 ] * xx [ 409 ] ; xx [ 379 ] = xx [ 437 ] - xx [ 427 ] -
xx [ 423 ] - xx [ 385 ] ; xx [ 380 ] = xx [ 409 ] / xx [ 410 ] ; xx [ 431 ] =
xx [ 98 ] - xx [ 203 ] * xx [ 344 ] + xx [ 206 ] ; xx [ 432 ] = xx [ 273 ] -
xx [ 347 ] ; xx [ 433 ] = xx [ 334 ] - xx [ 349 ] ; xx [ 434 ] = xx [ 352 ] -
xx [ 347 ] ; xx [ 435 ] = xx [ 375 ] - xx [ 412 ] * xx [ 391 ] ; xx [ 436 ] =
xx [ 388 ] - xx [ 421 ] ; xx [ 437 ] = xx [ 379 ] - xx [ 349 ] ; xx [ 438 ] =
xx [ 392 ] - xx [ 421 ] ; xx [ 439 ] = xx [ 399 ] - xx [ 380 ] * xx [ 409 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 431 , xx + 358 , xx + 421 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 358 , xx + 421 , xx + 431 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 592 , xx + 602 , xx + 358 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 605 , xx + 602 , xx + 421 ) ; xx [ 347 ]
= 0.01075186134385027 ; xx [ 349 ] = xx [ 520 ] - xx [ 471 ] ; xx [ 381 ] =
xx [ 414 ] / xx [ 480 ] ; xx [ 382 ] = xx [ 381 ] * xx [ 368 ] ; xx [ 383 ] =
xx [ 381 ] * xx [ 458 ] ; xx [ 384 ] = xx [ 523 ] - xx [ 474 ] ; xx [ 385 ] =
xx [ 368 ] / xx [ 480 ] ; xx [ 386 ] = xx [ 385 ] * xx [ 458 ] ; xx [ 387 ] =
xx [ 526 ] - xx [ 477 ] ; xx [ 444 ] = xx [ 458 ] / xx [ 480 ] ; xx [ 471 ] =
xx [ 349 ] - xx [ 381 ] * xx [ 414 ] + xx [ 206 ] ; xx [ 472 ] = xx [ 373 ] -
xx [ 382 ] ; xx [ 473 ] = xx [ 411 ] - xx [ 383 ] ; xx [ 474 ] = xx [ 384 ] -
xx [ 382 ] ; xx [ 475 ] = xx [ 31 ] - xx [ 385 ] * xx [ 368 ] ; xx [ 476 ] =
xx [ 316 ] - xx [ 386 ] ; xx [ 477 ] = xx [ 387 ] - xx [ 383 ] ; xx [ 478 ] =
xx [ 369 ] - xx [ 386 ] ; xx [ 479 ] = xx [ 441 ] - xx [ 444 ] * xx [ 458 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 471 , xx + 462 , xx + 512 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 462 , xx + 512 , xx + 471 ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 614 , xx + 625 , xx + 462 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 628 , xx + 625 , xx + 512 ) ; xx [ 382 ]
= xx [ 140 ] - xx [ 78 ] - xx [ 78 ] - xx [ 149 ] + xx [ 240 ] - xx [ 183 ] -
xx [ 183 ] - xx [ 250 ] + xx [ 288 ] - xx [ 275 ] - xx [ 275 ] - xx [ 335 ] +
xx [ 431 ] - xx [ 358 ] - xx [ 358 ] - xx [ 421 ] + xx [ 347 ] + xx [ 471 ] -
xx [ 462 ] - xx [ 462 ] - xx [ 512 ] ; xx [ 383 ] = xx [ 141 ] - xx [ 79 ] -
xx [ 81 ] - xx [ 150 ] + xx [ 241 ] - xx [ 184 ] - xx [ 186 ] - xx [ 251 ] +
xx [ 289 ] - xx [ 276 ] - xx [ 278 ] - xx [ 336 ] + xx [ 432 ] - xx [ 359 ] -
xx [ 361 ] - xx [ 422 ] + xx [ 472 ] - xx [ 463 ] - xx [ 465 ] - xx [ 513 ] ;
xx [ 386 ] = xx [ 142 ] - xx [ 80 ] - xx [ 84 ] - xx [ 151 ] + xx [ 242 ] -
xx [ 185 ] - xx [ 189 ] - xx [ 252 ] + xx [ 290 ] - xx [ 277 ] - xx [ 281 ] -
xx [ 337 ] + xx [ 433 ] - xx [ 360 ] - xx [ 364 ] - xx [ 423 ] + xx [ 473 ] -
xx [ 464 ] - xx [ 468 ] - xx [ 514 ] ; xx [ 445 ] = xx [ 143 ] - xx [ 81 ] -
xx [ 79 ] - xx [ 152 ] + xx [ 243 ] - xx [ 186 ] - xx [ 184 ] - xx [ 253 ] +
xx [ 291 ] - xx [ 278 ] - xx [ 276 ] - xx [ 338 ] + xx [ 434 ] - xx [ 361 ] -
xx [ 359 ] - xx [ 424 ] + xx [ 474 ] - xx [ 465 ] - xx [ 463 ] - xx [ 515 ] ;
xx [ 446 ] = 0.02353791429300479 ; xx [ 447 ] = xx [ 144 ] - xx [ 82 ] - xx [
82 ] - xx [ 153 ] + xx [ 244 ] - xx [ 187 ] - xx [ 187 ] - xx [ 254 ] + xx [
292 ] - xx [ 279 ] - xx [ 279 ] - xx [ 339 ] + xx [ 435 ] - xx [ 362 ] - xx [
362 ] - xx [ 425 ] + xx [ 446 ] + xx [ 475 ] - xx [ 466 ] - xx [ 466 ] - xx [
516 ] ; xx [ 448 ] = xx [ 145 ] - xx [ 83 ] - xx [ 85 ] - xx [ 154 ] + xx [
245 ] - xx [ 188 ] - xx [ 190 ] - xx [ 255 ] + xx [ 293 ] - xx [ 280 ] - xx [
282 ] - xx [ 340 ] + xx [ 436 ] - xx [ 363 ] - xx [ 365 ] - xx [ 426 ] + xx [
476 ] - xx [ 467 ] - xx [ 469 ] - xx [ 517 ] ; xx [ 449 ] = xx [ 146 ] - xx [
84 ] - xx [ 80 ] - xx [ 155 ] + xx [ 246 ] - xx [ 189 ] - xx [ 185 ] - xx [
256 ] + xx [ 294 ] - xx [ 281 ] - xx [ 277 ] - xx [ 341 ] + xx [ 437 ] - xx [
364 ] - xx [ 360 ] - xx [ 427 ] + xx [ 477 ] - xx [ 468 ] - xx [ 464 ] - xx [
518 ] ; xx [ 450 ] = xx [ 147 ] - xx [ 85 ] - xx [ 83 ] - xx [ 156 ] + xx [
247 ] - xx [ 190 ] - xx [ 188 ] - xx [ 257 ] + xx [ 295 ] - xx [ 282 ] - xx [
280 ] - xx [ 342 ] + xx [ 438 ] - xx [ 365 ] - xx [ 363 ] - xx [ 428 ] + xx [
478 ] - xx [ 469 ] - xx [ 467 ] - xx [ 519 ] ; xx [ 78 ] =
0.01509428966950729 ; xx [ 79 ] = xx [ 148 ] - xx [ 86 ] - xx [ 86 ] - xx [
157 ] + xx [ 248 ] - xx [ 191 ] - xx [ 191 ] - xx [ 258 ] + xx [ 296 ] - xx [
283 ] - xx [ 283 ] - xx [ 343 ] + xx [ 439 ] - xx [ 366 ] - xx [ 366 ] - xx [
429 ] + xx [ 78 ] + xx [ 479 ] - xx [ 470 ] - xx [ 470 ] - xx [ 520 ] ; xx [
140 ] = xx [ 382 ] ; xx [ 141 ] = xx [ 383 ] ; xx [ 142 ] = xx [ 386 ] ; xx [
143 ] = xx [ 445 ] ; xx [ 144 ] = xx [ 447 ] ; xx [ 145 ] = xx [ 448 ] ; xx [
146 ] = xx [ 449 ] ; xx [ 147 ] = xx [ 450 ] ; xx [ 148 ] = xx [ 79 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 140 , xx + 649 , xx + 80 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 640 , xx + 264 , xx + 83 ) ; xx [ 86 ] = xx
[ 80 ] + xx [ 83 ] ; xx [ 149 ] = xx [ 81 ] + xx [ 84 ] ; xx [ 80 ] = xx [ 82
] + xx [ 85 ] ; xx [ 81 ] = xx [ 86 ] ; xx [ 82 ] = xx [ 149 ] ; xx [ 83 ] =
xx [ 80 ] ; xx [ 84 ] = xx [ 653 ] + xx [ 268 ] ; xx [ 85 ] = xx [ 654 ] + xx
[ 269 ] ; xx [ 150 ] = xx [ 345 ] ; xx [ 151 ] = xx [ 84 ] ; xx [ 152 ] = xx
[ 85 ] ; xx [ 153 ] = pm_math_Vector3_dot_ra ( xx + 649 , xx + 81 ) +
pm_math_Vector3_dot_ra ( xx + 264 , xx + 150 ) ; ii [ 0 ] =
factorSymmetricPosDef ( xx + 153 , 1 , xx + 81 ) ; if ( ii [ 0 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
"'simple_robot/Revolute5' has a degenerate mass distribution on its base side."
, neDiagMgr ) ; } xx [ 81 ] = xx [ 345 ] / xx [ 153 ] ; xx [ 82 ] = xx [ 84 ]
* xx [ 81 ] ; xx [ 83 ] = xx [ 85 ] * xx [ 81 ] ; xx [ 150 ] = xx [ 84 ] / xx
[ 153 ] ; xx [ 151 ] = xx [ 85 ] * xx [ 150 ] ; xx [ 152 ] = xx [ 85 ] / xx [
153 ] ; xx [ 183 ] = xx [ 483 ] - xx [ 345 ] * xx [ 81 ] ; xx [ 184 ] = xx [
655 ] - xx [ 82 ] ; xx [ 185 ] = xx [ 163 ] - xx [ 83 ] ; xx [ 186 ] = xx [
164 ] - xx [ 82 ] ; xx [ 187 ] = xx [ 165 ] - xx [ 84 ] * xx [ 150 ] ; xx [
188 ] = xx [ 166 ] - xx [ 151 ] ; xx [ 189 ] = xx [ 167 ] - xx [ 83 ] ; xx [
190 ] = xx [ 168 ] - xx [ 151 ] ; xx [ 191 ] = xx [ 169 ] - xx [ 85 ] * xx [
152 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 183 , xx + 69 , xx + 240
) ; pm_math_Matrix3x3_compose_ra ( xx + 69 , xx + 240 , xx + 183 ) ; xx [ 240
] = xx [ 443 ] - xx [ 86 ] * xx [ 81 ] ; xx [ 241 ] = xx [ 493 ] - xx [ 86 ]
* xx [ 150 ] ; xx [ 242 ] = xx [ 511 ] - xx [ 86 ] * xx [ 152 ] ; xx [ 243 ]
= xx [ 601 ] - xx [ 149 ] * xx [ 81 ] ; xx [ 244 ] = xx [ 623 ] - xx [ 149 ]
* xx [ 150 ] ; xx [ 245 ] = xx [ 624 ] - xx [ 149 ] * xx [ 152 ] ; xx [ 246 ]
= xx [ 637 ] - xx [ 80 ] * xx [ 81 ] ; xx [ 247 ] = xx [ 638 ] - xx [ 80 ] *
xx [ 150 ] ; xx [ 248 ] = xx [ 639 ] - xx [ 80 ] * xx [ 152 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 240 , xx + 69 , xx + 250 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 69 , xx + 250 , xx + 240 ) ; xx [ 154 ] =
0.03736275114177646 ; xx [ 155 ] = 0.1168511605538739 ; xx [ 156 ] =
0.08181278075467377 ; pm_math_Quaternion_xform_ra ( xx + 65 , xx + 154 , xx +
163 ) ; xx [ 154 ] = 4.244406610027192e-3 - xx [ 163 ] ; xx [ 155 ] =
0.06404981182865317 - xx [ 164 ] ; xx [ 156 ] = - xx [ 165 ] ;
pm_math_Matrix3x3_postCross_ra ( xx + 183 , xx + 154 , xx + 250 ) ; xx [ 82 ]
= xx [ 240 ] - xx [ 250 ] ; xx [ 83 ] = xx [ 243 ] - xx [ 251 ] ; xx [ 151 ]
= 0.05497997575039515 ; xx [ 157 ] = xx [ 38 ] * xx [ 82 ] - xx [ 35 ] * xx [
83 ] + xx [ 151 ] * xx [ 185 ] ; xx [ 163 ] = xx [ 86 ] / xx [ 153 ] ; xx [
164 ] = xx [ 149 ] * xx [ 163 ] ; xx [ 165 ] = xx [ 80 ] * xx [ 163 ] ; xx [
166 ] = xx [ 149 ] / xx [ 153 ] ; xx [ 167 ] = xx [ 80 ] * xx [ 166 ] ; xx [
168 ] = xx [ 80 ] / xx [ 153 ] ; xx [ 275 ] = xx [ 382 ] - xx [ 86 ] * xx [
163 ] ; xx [ 276 ] = xx [ 383 ] - xx [ 164 ] ; xx [ 277 ] = xx [ 386 ] - xx [
165 ] ; xx [ 278 ] = xx [ 445 ] - xx [ 164 ] ; xx [ 279 ] = xx [ 447 ] - xx [
149 ] * xx [ 166 ] ; xx [ 280 ] = xx [ 448 ] - xx [ 167 ] ; xx [ 281 ] = xx [
449 ] - xx [ 165 ] ; xx [ 282 ] = xx [ 450 ] - xx [ 167 ] ; xx [ 283 ] = xx [
79 ] - xx [ 80 ] * xx [ 168 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx +
275 , xx + 69 , xx + 288 ) ; pm_math_Matrix3x3_compose_ra ( xx + 69 , xx +
288 , xx + 275 ) ; pm_math_Matrix3x3_postCross_ra ( xx + 240 , xx + 154 , xx
+ 69 ) ; pm_math_Matrix3x3_preCross_ra ( xx + 250 , xx + 154 , xx + 288 ) ;
xx [ 79 ] = xx [ 112 ] + xx [ 275 ] - xx [ 69 ] - xx [ 69 ] - xx [ 288 ] ; xx
[ 164 ] = xx [ 276 ] - xx [ 70 ] - xx [ 72 ] - xx [ 289 ] ; xx [ 165 ] = xx [
242 ] - xx [ 256 ] ; xx [ 167 ] = xx [ 79 ] * xx [ 38 ] - xx [ 35 ] * xx [
164 ] + xx [ 151 ] * xx [ 165 ] ; xx [ 169 ] = xx [ 278 ] - xx [ 72 ] - xx [
70 ] - xx [ 291 ] ; xx [ 240 ] = xx [ 135 ] + xx [ 279 ] - xx [ 73 ] - xx [
73 ] - xx [ 292 ] ; xx [ 242 ] = xx [ 245 ] - xx [ 257 ] ; xx [ 243 ] = xx [
38 ] * xx [ 169 ] - xx [ 240 ] * xx [ 35 ] + xx [ 151 ] * xx [ 242 ] ; xx [
245 ] = xx [ 129 ] + xx [ 191 ] ; xx [ 191 ] = xx [ 38 ] * xx [ 165 ] - xx [
35 ] * xx [ 242 ] + xx [ 245 ] * xx [ 151 ] ; xx [ 250 ] = xx [ 167 ] * xx [
38 ] - xx [ 243 ] * xx [ 35 ] + xx [ 191 ] * xx [ 151 ] ; ii [ 0 ] =
factorSymmetricPosDef ( xx + 250 , 1 , xx + 251 ) ; if ( ii [ 0 ] != 0 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
"'simple_robot/Revolute7' has a degenerate mass distribution on its base side."
, neDiagMgr ) ; } xx [ 251 ] = xx [ 157 ] / xx [ 250 ] ; xx [ 256 ] = xx [
241 ] - xx [ 253 ] ; xx [ 241 ] = xx [ 244 ] - xx [ 254 ] ; xx [ 244 ] = xx [
38 ] * xx [ 256 ] - xx [ 35 ] * xx [ 241 ] + xx [ 151 ] * xx [ 188 ] ; xx [
253 ] = xx [ 244 ] * xx [ 251 ] ; xx [ 254 ] = xx [ 191 ] * xx [ 251 ] ; xx [
257 ] = xx [ 244 ] / xx [ 250 ] ; xx [ 267 ] = xx [ 191 ] * xx [ 257 ] ; xx [
268 ] = xx [ 191 ] / xx [ 250 ] ; xx [ 335 ] = xx [ 183 ] - xx [ 157 ] * xx [
251 ] + xx [ 129 ] ; xx [ 336 ] = xx [ 184 ] - xx [ 253 ] ; xx [ 337 ] = xx [
185 ] - xx [ 254 ] ; xx [ 338 ] = xx [ 186 ] - xx [ 253 ] ; xx [ 339 ] = xx [
187 ] - xx [ 244 ] * xx [ 257 ] + xx [ 129 ] ; xx [ 340 ] = xx [ 188 ] - xx [
267 ] ; xx [ 341 ] = xx [ 189 ] - xx [ 254 ] ; xx [ 342 ] = xx [ 190 ] - xx [
267 ] ; xx [ 343 ] = xx [ 245 ] - xx [ 191 ] * xx [ 268 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 335 , xx + 46 , xx + 358 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 46 , xx + 358 , xx + 335 ) ; xx [ 253 ] =
xx [ 29 ] + xx [ 339 ] ; xx [ 254 ] = xx [ 29 ] + xx [ 343 ] ; xx [ 358 ] =
xx [ 29 ] + xx [ 335 ] ; xx [ 359 ] = xx [ 336 ] ; xx [ 360 ] = xx [ 337 ] ;
xx [ 361 ] = xx [ 338 ] ; xx [ 362 ] = xx [ 253 ] ; xx [ 363 ] = xx [ 340 ] ;
xx [ 364 ] = xx [ 341 ] ; xx [ 365 ] = xx [ 342 ] ; xx [ 366 ] = xx [ 254 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 358 , xx + 26 , xx + 421 ) ; xx [ 267 ] = (
xx [ 13 ] + xx [ 12 ] ) * xx [ 2 ] ; xx [ 12 ] = xx [ 19 ] * xx [ 19 ] ; xx [
13 ] = xx [ 11 ] - ( xx [ 1 ] + xx [ 12 ] ) * xx [ 2 ] ; xx [ 1 ] = xx [ 20 ]
* xx [ 21 ] ; xx [ 269 ] = xx [ 18 ] * xx [ 19 ] ; xx [ 382 ] = xx [ 2 ] * (
xx [ 1 ] - xx [ 269 ] ) ; xx [ 424 ] = xx [ 267 ] ; xx [ 425 ] = xx [ 13 ] ;
xx [ 426 ] = xx [ 382 ] ; pm_math_Matrix3x3_xform_ra ( xx + 358 , xx + 424 ,
xx + 427 ) ; xx [ 383 ] = pm_math_Vector3_dot_ra ( xx + 26 , xx + 427 ) ; xx
[ 386 ] = xx [ 2 ] * ( xx [ 24 ] - xx [ 23 ] ) ; xx [ 23 ] = ( xx [ 269 ] +
xx [ 1 ] ) * xx [ 2 ] ; xx [ 1 ] = xx [ 11 ] - ( xx [ 12 ] + xx [ 0 ] ) * xx
[ 2 ] ; xx [ 431 ] = xx [ 386 ] ; xx [ 432 ] = xx [ 23 ] ; xx [ 433 ] = xx [
1 ] ; pm_math_Matrix3x3_xform_ra ( xx + 358 , xx + 431 , xx + 434 ) ; xx [ 0
] = pm_math_Vector3_dot_ra ( xx + 26 , xx + 434 ) ; xx [ 12 ] =
0.7599620028496401 ; xx [ 24 ] = xx [ 246 ] - xx [ 252 ] ; xx [ 246 ] = xx [
281 ] - xx [ 75 ] - xx [ 71 ] - xx [ 294 ] ; xx [ 252 ] = xx [ 282 ] - xx [
76 ] - xx [ 74 ] - xx [ 295 ] ; xx [ 269 ] = xx [ 248 ] - xx [ 258 ] ; xx [
248 ] = xx [ 38 ] * xx [ 246 ] - xx [ 35 ] * xx [ 252 ] + xx [ 151 ] * xx [
269 ] ; xx [ 258 ] = xx [ 247 ] - xx [ 255 ] ; xx [ 462 ] = xx [ 82 ] - xx [
167 ] * xx [ 251 ] ; xx [ 463 ] = xx [ 256 ] - xx [ 167 ] * xx [ 257 ] ; xx [
464 ] = xx [ 165 ] - xx [ 167 ] * xx [ 268 ] ; xx [ 465 ] = xx [ 83 ] - xx [
243 ] * xx [ 251 ] ; xx [ 466 ] = xx [ 241 ] - xx [ 243 ] * xx [ 257 ] ; xx [
467 ] = xx [ 242 ] - xx [ 243 ] * xx [ 268 ] ; xx [ 468 ] = xx [ 24 ] - xx [
248 ] * xx [ 251 ] ; xx [ 469 ] = xx [ 258 ] - xx [ 248 ] * xx [ 257 ] ; xx [
470 ] = xx [ 269 ] - xx [ 248 ] * xx [ 268 ] ;
pm_math_Matrix3x3_composeTranspose_ra ( xx + 462 , xx + 46 , xx + 471 ) ;
pm_math_Matrix3x3_compose_ra ( xx + 46 , xx + 471 , xx + 462 ) ; xx [ 437 ] =
xx [ 42 ] ; xx [ 438 ] = xx [ 43 ] ; xx [ 439 ] = xx [ 45 ] ; xx [ 247 ] = xx
[ 300 ] * xx [ 45 ] ; xx [ 255 ] = xx [ 105 ] * xx [ 45 ] ; xx [ 443 ] = xx [
300 ] * xx [ 42 ] + xx [ 43 ] * xx [ 105 ] ; xx [ 447 ] = xx [ 247 ] ; xx [
448 ] = xx [ 255 ] ; xx [ 449 ] = - xx [ 443 ] ; pm_math_Vector3_cross_ra (
xx + 437 , xx + 447 , xx + 471 ) ; xx [ 437 ] = - ( xx [ 106 ] + xx [ 105 ] +
xx [ 2 ] * ( xx [ 471 ] - xx [ 247 ] * xx [ 36 ] ) ) ; xx [ 438 ] = - ( ( xx
[ 472 ] - xx [ 255 ] * xx [ 36 ] ) * xx [ 2 ] - xx [ 300 ] ) ; xx [ 439 ] = -
( ( xx [ 36 ] * xx [ 443 ] + xx [ 473 ] ) * xx [ 2 ] ) ;
pm_math_Matrix3x3_postCross_ra ( xx + 335 , xx + 437 , xx + 471 ) ; xx [ 105
] = xx [ 465 ] - xx [ 472 ] ; xx [ 247 ] = 0.6499675024374402 ; xx [ 255 ] =
xx [ 468 ] - xx [ 473 ] ; xx [ 300 ] = 3.21928903957264e-3 ; xx [ 335 ] =
3.764091800114267e-3 ; xx [ 338 ] = xx [ 12 ] * xx [ 105 ] - xx [ 247 ] * xx
[ 255 ] - ( xx [ 300 ] * xx [ 336 ] + xx [ 335 ] * xx [ 337 ] ) ; xx [ 339 ]
= xx [ 466 ] - xx [ 475 ] ; xx [ 341 ] = xx [ 469 ] - xx [ 476 ] ; xx [ 343 ]
= xx [ 12 ] * xx [ 339 ] - xx [ 247 ] * xx [ 341 ] - ( xx [ 253 ] * xx [ 300
] + xx [ 335 ] * xx [ 340 ] ) ; xx [ 443 ] = xx [ 467 ] - xx [ 478 ] ; xx [
445 ] = xx [ 470 ] - xx [ 479 ] ; xx [ 447 ] = xx [ 12 ] * xx [ 443 ] - xx [
247 ] * xx [ 445 ] - ( xx [ 300 ] * xx [ 342 ] + xx [ 254 ] * xx [ 335 ] ) ;
xx [ 448 ] = xx [ 338 ] ; xx [ 449 ] = xx [ 343 ] ; xx [ 450 ] = xx [ 447 ] ;
xx [ 451 ] = pm_math_Vector3_dot_ra ( xx + 26 , xx + 448 ) ; xx [ 483 ] =
0.6499675024374401 ; xx [ 493 ] = 3.21928903957264e-3 ; xx [ 511 ] = xx [ 483
] * xx [ 105 ] + xx [ 12 ] * xx [ 255 ] + xx [ 335 ] * xx [ 336 ] - xx [ 493
] * xx [ 337 ] ; xx [ 336 ] = xx [ 483 ] * xx [ 339 ] + xx [ 12 ] * xx [ 341
] + xx [ 253 ] * xx [ 335 ] - xx [ 493 ] * xx [ 340 ] ; xx [ 253 ] = xx [ 483
] * xx [ 443 ] + xx [ 12 ] * xx [ 445 ] + xx [ 335 ] * xx [ 342 ] - xx [ 254
] * xx [ 493 ] ; xx [ 512 ] = xx [ 511 ] ; xx [ 513 ] = xx [ 336 ] ; xx [ 514
] = xx [ 253 ] ; xx [ 254 ] = pm_math_Vector3_dot_ra ( xx + 26 , xx + 512 ) ;
xx [ 337 ] = xx [ 462 ] - xx [ 471 ] ; xx [ 340 ] = xx [ 463 ] - xx [ 474 ] ;
xx [ 342 ] = xx [ 464 ] - xx [ 477 ] ; xx [ 515 ] = xx [ 337 ] ; xx [ 516 ] =
xx [ 340 ] ; xx [ 517 ] = xx [ 342 ] ; xx [ 518 ] = pm_math_Vector3_dot_ra (
xx + 26 , xx + 515 ) ; xx [ 519 ] = pm_math_Vector3_dot_ra ( xx + 424 , xx +
434 ) ; xx [ 520 ] = pm_math_Vector3_dot_ra ( xx + 424 , xx + 448 ) ; xx [
521 ] = pm_math_Vector3_dot_ra ( xx + 424 , xx + 512 ) ; xx [ 522 ] =
pm_math_Vector3_dot_ra ( xx + 424 , xx + 515 ) ; xx [ 523 ] =
pm_math_Vector3_dot_ra ( xx + 431 , xx + 448 ) ; xx [ 448 ] =
pm_math_Vector3_dot_ra ( xx + 431 , xx + 512 ) ; xx [ 449 ] =
pm_math_Vector3_dot_ra ( xx + 431 , xx + 515 ) ; xx [ 450 ] = xx [ 167 ] / xx
[ 250 ] ; xx [ 512 ] = xx [ 243 ] * xx [ 450 ] ; xx [ 513 ] = xx [ 277 ] - xx
[ 71 ] - xx [ 75 ] - xx [ 290 ] ; xx [ 514 ] = xx [ 248 ] * xx [ 450 ] ; xx [
515 ] = xx [ 243 ] / xx [ 250 ] ; xx [ 516 ] = xx [ 280 ] - xx [ 74 ] - xx [
76 ] - xx [ 293 ] ; xx [ 517 ] = xx [ 248 ] * xx [ 515 ] ; xx [ 69 ] = xx [
283 ] - xx [ 77 ] - xx [ 77 ] - xx [ 296 ] ; xx [ 70 ] = xx [ 248 ] / xx [
250 ] ; xx [ 275 ] = xx [ 79 ] - xx [ 167 ] * xx [ 450 ] ; xx [ 276 ] = xx [
164 ] - xx [ 512 ] ; xx [ 277 ] = xx [ 513 ] - xx [ 514 ] ; xx [ 278 ] = xx [
169 ] - xx [ 512 ] ; xx [ 279 ] = xx [ 240 ] - xx [ 243 ] * xx [ 515 ] ; xx [
280 ] = xx [ 516 ] - xx [ 517 ] ; xx [ 281 ] = xx [ 246 ] - xx [ 514 ] ; xx [
282 ] = xx [ 252 ] - xx [ 517 ] ; xx [ 283 ] = xx [ 69 ] - xx [ 248 ] * xx [
70 ] + xx [ 94 ] ; pm_math_Matrix3x3_composeTranspose_ra ( xx + 275 , xx + 46
, xx + 288 ) ; pm_math_Matrix3x3_compose_ra ( xx + 46 , xx + 288 , xx + 275 )
; pm_math_Matrix3x3_postCross_ra ( xx + 462 , xx + 437 , xx + 46 ) ;
pm_math_Matrix3x3_preCross_ra ( xx + 471 , xx + 437 , xx + 288 ) ; xx [ 71 ]
= xx [ 138 ] + xx [ 279 ] - xx [ 50 ] - xx [ 50 ] - xx [ 292 ] ; xx [ 72 ] =
xx [ 280 ] - xx [ 51 ] - xx [ 53 ] - xx [ 293 ] ; xx [ 73 ] = xx [ 71 ] * xx
[ 12 ] - xx [ 247 ] * xx [ 72 ] - ( xx [ 300 ] * xx [ 339 ] + xx [ 335 ] * xx
[ 443 ] ) ; xx [ 74 ] = xx [ 282 ] - xx [ 53 ] - xx [ 51 ] - xx [ 295 ] ; xx
[ 75 ] = xx [ 139 ] + xx [ 283 ] - xx [ 54 ] - xx [ 54 ] - xx [ 296 ] ; xx [
76 ] = xx [ 12 ] * xx [ 74 ] - xx [ 75 ] * xx [ 247 ] - ( xx [ 300 ] * xx [
341 ] + xx [ 335 ] * xx [ 445 ] ) ; xx [ 77 ] = xx [ 71 ] * xx [ 483 ] + xx [
12 ] * xx [ 72 ] + xx [ 335 ] * xx [ 339 ] - xx [ 493 ] * xx [ 443 ] ; xx [
71 ] = xx [ 483 ] * xx [ 74 ] + xx [ 75 ] * xx [ 12 ] + xx [ 335 ] * xx [ 341
] - xx [ 493 ] * xx [ 445 ] ; xx [ 72 ] = xx [ 77 ] * xx [ 12 ] - xx [ 71 ] *
xx [ 247 ] - ( xx [ 336 ] * xx [ 300 ] + xx [ 253 ] * xx [ 335 ] ) ; xx [ 74
] = xx [ 278 ] - xx [ 49 ] - xx [ 47 ] - xx [ 291 ] ; xx [ 75 ] = xx [ 281 ]
- xx [ 52 ] - xx [ 48 ] - xx [ 294 ] ; xx [ 462 ] = xx [ 300 ] * xx [ 340 ] +
xx [ 335 ] * xx [ 342 ] ; xx [ 463 ] = xx [ 12 ] * xx [ 74 ] - xx [ 247 ] *
xx [ 75 ] - xx [ 462 ] ; xx [ 464 ] = xx [ 335 ] * xx [ 340 ] - xx [ 493 ] *
xx [ 342 ] ; xx [ 465 ] = xx [ 483 ] * xx [ 74 ] + xx [ 12 ] * xx [ 75 ] + xx
[ 464 ] ; xx [ 50 ] = xx [ 64 ] + xx [ 275 ] - xx [ 46 ] - xx [ 46 ] - xx [
288 ] ; xx [ 665 ] = pm_math_Vector3_dot_ra ( xx + 26 , xx + 421 ) ; xx [ 666
] = xx [ 383 ] ; xx [ 667 ] = xx [ 0 ] ; xx [ 668 ] = xx [ 451 ] ; xx [ 669 ]
= xx [ 254 ] ; xx [ 670 ] = xx [ 518 ] ; xx [ 671 ] = xx [ 383 ] ; xx [ 672 ]
= pm_math_Vector3_dot_ra ( xx + 424 , xx + 427 ) ; xx [ 673 ] = xx [ 519 ] ;
xx [ 674 ] = xx [ 520 ] ; xx [ 675 ] = xx [ 521 ] ; xx [ 676 ] = xx [ 522 ] ;
xx [ 677 ] = xx [ 0 ] ; xx [ 678 ] = xx [ 519 ] ; xx [ 679 ] =
pm_math_Vector3_dot_ra ( xx + 431 , xx + 434 ) ; xx [ 680 ] = xx [ 523 ] ; xx
[ 681 ] = xx [ 448 ] ; xx [ 682 ] = xx [ 449 ] ; xx [ 683 ] = xx [ 451 ] ; xx
[ 684 ] = xx [ 520 ] ; xx [ 685 ] = xx [ 523 ] ; xx [ 686 ] = xx [ 73 ] * xx
[ 12 ] - xx [ 76 ] * xx [ 247 ] - ( xx [ 343 ] * xx [ 300 ] + xx [ 447 ] * xx
[ 335 ] ) ; xx [ 687 ] = xx [ 72 ] ; xx [ 688 ] = xx [ 463 ] ; xx [ 689 ] =
xx [ 254 ] ; xx [ 690 ] = xx [ 521 ] ; xx [ 691 ] = xx [ 448 ] ; xx [ 692 ] =
xx [ 72 ] ; xx [ 693 ] = xx [ 77 ] * xx [ 483 ] + xx [ 71 ] * xx [ 12 ] + xx
[ 336 ] * xx [ 335 ] - xx [ 253 ] * xx [ 493 ] ; xx [ 694 ] = xx [ 465 ] ; xx
[ 695 ] = xx [ 518 ] ; xx [ 696 ] = xx [ 522 ] ; xx [ 697 ] = xx [ 449 ] ; xx
[ 698 ] = xx [ 463 ] ; xx [ 699 ] = xx [ 465 ] ; xx [ 700 ] = xx [ 50 ] ; ii
[ 0 ] = factorSymmetricPosDef ( xx + 665 , 6 , xx + 465 ) ; if ( ii [ 0 ] !=
0 ) { return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:DegenerateMass" ,
 "'simple_robot/6-DOF Joint' has a degenerate mass distribution on its follower side."
, neDiagMgr ) ; } xx [ 465 ] = - xx [ 36 ] ; xx [ 466 ] = xx [ 42 ] ; xx [
467 ] = xx [ 43 ] ; xx [ 468 ] = xx [ 45 ] ; xx [ 469 ] = - xx [ 55 ] ; xx [
470 ] = xx [ 58 ] ; xx [ 471 ] = xx [ 61 ] ; xx [ 472 ] = xx [ 62 ] ; xx [
473 ] = - xx [ 32 ] ; xx [ 474 ] = xx [ 44 ] ; xx [ 475 ] = xx [ 63 ] ; xx [
476 ] = - xx [ 87 ] ; pm_math_Quaternion_xform_ra ( xx + 14 , xx + 4 , xx +
42 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 465 , xx + 42 , xx + 4 ) ;
xx [ 0 ] = xx [ 38 ] * state [ 14 ] ; xx [ 14 ] = xx [ 4 ] + xx [ 0 ] ; xx [
15 ] = xx [ 35 ] * state [ 14 ] ; xx [ 16 ] = xx [ 5 ] - xx [ 15 ] ; xx [ 53
] = xx [ 14 ] ; xx [ 54 ] = xx [ 16 ] ; xx [ 55 ] = xx [ 6 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 65 , xx + 53 , xx + 61 ) ; xx [ 17
] = xx [ 41 ] * state [ 16 ] ; xx [ 32 ] = xx [ 61 ] + xx [ 17 ] ; xx [ 36 ]
= xx [ 59 ] * state [ 16 ] ; xx [ 45 ] = xx [ 62 ] + xx [ 36 ] ; xx [ 46 ] =
xx [ 60 ] * state [ 16 ] ; xx [ 51 ] = xx [ 63 ] - xx [ 46 ] ; xx [ 477 ] =
xx [ 32 ] ; xx [ 478 ] = xx [ 45 ] ; xx [ 479 ] = xx [ 51 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 469 , xx + 477 , xx + 517 ) ; xx [
58 ] = xx [ 38 ] * state [ 34 ] ; xx [ 72 ] = xx [ 517 ] - xx [ 58 ] ; xx [
87 ] = xx [ 35 ] * state [ 34 ] ; xx [ 254 ] = xx [ 518 ] + xx [ 87 ] ; xx [
520 ] = xx [ 72 ] ; xx [ 521 ] = xx [ 254 ] ; xx [ 522 ] = xx [ 519 ] ;
pm_math_Vector3_cross_ra ( xx + 520 , xx + 109 , xx + 523 ) ;
pm_math_Vector3_cross_ra ( xx + 520 , xx + 523 , xx + 526 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 473 , xx + 526 , xx + 529 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 473 , xx + 520 , xx + 526 ) ; xx [
383 ] = 1.09978692819368e-18 ; xx [ 448 ] = xx [ 383 ] * state [ 36 ] ; xx [
449 ] = 0.9380730256354353 ; pm_math_Quaternion_compose_ra ( xx + 18 , xx +
465 , xx + 532 ) ; pm_math_Quaternion_compose_ra ( xx + 532 , xx + 65 , xx +
541 ) ; pm_math_Quaternion_compose_ra ( xx + 541 , xx + 469 , xx + 545 ) ;
pm_math_Quaternion_compose_ra ( xx + 545 , xx + 473 , xx + 549 ) ; xx [ 451 ]
= 0.3464375825097215 ; xx [ 463 ] = xx [ 106 ] * xx [ 551 ] ; xx [ 512 ] = xx
[ 106 ] * xx [ 552 ] ; pm_math_Quaternion_xform_ra ( xx + 545 , xx + 109 , xx
+ 553 ) ; pm_math_Quaternion_xform_ra ( xx + 541 , xx + 538 , xx + 545 ) ;
pm_math_Quaternion_xform_ra ( xx + 532 , xx + 154 , xx + 556 ) ;
pm_math_Quaternion_xform_ra ( xx + 18 , xx + 437 , xx + 532 ) ; xx [ 514 ] =
xx [ 106 ] * xx [ 20 ] ; xx [ 535 ] = xx [ 106 ] * xx [ 21 ] ; xx [ 536 ] = (
xx [ 20 ] * xx [ 514 ] + xx [ 21 ] * xx [ 535 ] ) * xx [ 2 ] ; xx [ 537 ] =
state [ 0 ] - xx [ 536 ] + xx [ 106 ] ; xx [ 548 ] = xx [ 556 ] + xx [ 532 ]
+ xx [ 537 ] ; xx [ 562 ] = ( xx [ 18 ] * xx [ 535 ] + xx [ 19 ] * xx [ 514 ]
) * xx [ 2 ] ; xx [ 563 ] = state [ 1 ] + xx [ 562 ] ; xx [ 564 ] = xx [ 557
] + xx [ 533 ] + xx [ 563 ] ; xx [ 532 ] = xx [ 2 ] * ( xx [ 18 ] * xx [ 514
] - xx [ 19 ] * xx [ 535 ] ) ; xx [ 514 ] = state [ 2 ] - xx [ 532 ] ; xx [
533 ] = xx [ 558 ] + xx [ 534 ] + xx [ 514 ] ; xx [ 565 ] = xx [ 449 ] * xx [
549 ] + xx [ 451 ] * xx [ 550 ] ; xx [ 566 ] = xx [ 449 ] * xx [ 550 ] - xx [
451 ] * xx [ 549 ] ; xx [ 567 ] = xx [ 449 ] * xx [ 551 ] - xx [ 451 ] * xx [
552 ] ; xx [ 568 ] = xx [ 449 ] * xx [ 552 ] + xx [ 451 ] * xx [ 551 ] ; xx [
569 ] = ( xx [ 551 ] * xx [ 463 ] + xx [ 552 ] * xx [ 512 ] ) * xx [ 2 ] - xx
[ 106 ] + xx [ 553 ] + xx [ 545 ] + xx [ 548 ] ; xx [ 570 ] = xx [ 554 ] + xx
[ 546 ] + xx [ 564 ] - ( xx [ 549 ] * xx [ 512 ] + xx [ 550 ] * xx [ 463 ] )
* xx [ 2 ] ; xx [ 571 ] = xx [ 2 ] * ( xx [ 549 ] * xx [ 463 ] - xx [ 550 ] *
xx [ 512 ] ) + xx [ 555 ] + xx [ 547 ] + xx [ 533 ] ; xx [ 463 ] = 0.0 ; xx [
549 ] = xx [ 11 ] ; xx [ 550 ] = xx [ 463 ] ; xx [ 551 ] = xx [ 463 ] ; xx [
552 ] = xx [ 463 ] ; xx [ 553 ] = xx [ 463 ] ; xx [ 554 ] = xx [ 463 ] ; xx [
555 ] = xx [ 463 ] ; xx [ 11 ] =
sm_core_compiler_computeProximityInfoCxpolyCxpoly (
simple_robot_9eb3ef65_1_geometry_0 ( rtdv ) ,
simple_robot_9eb3ef65_1_geometry_1 ( rtdv ) , ( pm_math_Transform3 * ) ( xx +
565 ) , ( pm_math_Transform3 * ) ( xx + 549 ) , ( pm_math_Vector3 * ) ( xx +
545 ) , ( pm_math_Vector3 * ) ( xx + 556 ) , ( pm_math_Vector3 * ) ( xx + 572
) , ( pm_math_Vector3 * ) ( xx + 575 ) ) ; xx [ 583 ] = xx [ 449 ] ; xx [ 584
] = - xx [ 451 ] ; xx [ 585 ] = xx [ 463 ] ; xx [ 586 ] = xx [ 463 ] ; xx [
587 ] = - xx [ 106 ] ; xx [ 588 ] = xx [ 463 ] ; xx [ 589 ] = xx [ 463 ] ; xx
[ 512 ] = xx [ 526 ] + state [ 36 ] ; pm_math_Vector3_cross_ra ( xx + 477 ,
xx + 538 , xx + 590 ) ; pm_math_Vector3_cross_ra ( xx + 53 , xx + 154 , xx +
593 ) ; pm_math_Vector3_cross_ra ( xx + 42 , xx + 437 , xx + 596 ) ; xx [ 599
] = state [ 7 ] ; xx [ 600 ] = state [ 8 ] ; xx [ 601 ] = state [ 9 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 18 , xx + 599 , xx + 605 ) ; xx [
534 ] = xx [ 606 ] + xx [ 106 ] * xx [ 44 ] ; xx [ 535 ] = xx [ 607 ] - xx [
106 ] * xx [ 43 ] ; xx [ 599 ] = xx [ 596 ] + xx [ 605 ] ; xx [ 600 ] = xx [
597 ] + xx [ 534 ] ; xx [ 601 ] = xx [ 598 ] + xx [ 535 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 465 , xx + 599 , xx + 608 ) ; xx [
578 ] = xx [ 151 ] * state [ 14 ] ; xx [ 599 ] = xx [ 593 ] + xx [ 608 ] ; xx
[ 600 ] = xx [ 594 ] + xx [ 609 ] ; xx [ 601 ] = xx [ 595 ] + xx [ 610 ] + xx
[ 578 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 65 , xx + 599 , xx + 608
) ; xx [ 579 ] = xx [ 170 ] * state [ 16 ] ; xx [ 599 ] = xx [ 608 ] - xx [
579 ] ; xx [ 600 ] = xx [ 262 ] * state [ 16 ] ; xx [ 601 ] = xx [ 609 ] + xx
[ 600 ] ; xx [ 608 ] = xx [ 263 ] * state [ 16 ] ; xx [ 609 ] = xx [ 610 ] +
xx [ 608 ] ; xx [ 610 ] = xx [ 590 ] + xx [ 599 ] ; xx [ 611 ] = xx [ 591 ] +
xx [ 601 ] ; xx [ 612 ] = xx [ 592 ] + xx [ 609 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 469 , xx + 610 , xx + 613 ) ; xx [
610 ] = xx [ 104 ] * state [ 34 ] ; xx [ 616 ] = xx [ 523 ] + xx [ 613 ] ; xx
[ 617 ] = xx [ 524 ] + xx [ 614 ] ; xx [ 618 ] = xx [ 525 ] + xx [ 615 ] + xx
[ 610 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 473 , xx + 616 , xx + 523
) ; xx [ 611 ] = xx [ 512 ] ; xx [ 612 ] = xx [ 527 ] ; xx [ 613 ] = xx [ 528
] ; xx [ 614 ] = xx [ 523 ] ; xx [ 615 ] = xx [ 524 ] - xx [ 448 ] ; xx [ 616
] = xx [ 525 ] ; xx [ 523 ] = 1.0e6 ; xx [ 524 ] = 1000.0 ; xx [ 525 ] =
1.0e-4 ; xx [ 617 ] = 0.3 ; xx [ 618 ] = 0.2119573811760597 ; xx [ 619 ] =
9.126024771145405e-4 ; sm_core_compiler_computeContactWrenches ( xx [ 11 ] ,
xx + 556 , xx + 545 , xx + 575 , xx + 572 , xx + 549 , xx + 583 , xx + 549 ,
xx + 565 , NULL , xx + 611 , 0 , 1 , xx [ 523 ] , xx [ 524 ] , xx [ 525 ] ,
xx [ 617 ] , xx [ 618 ] , xx [ 619 ] , NULL , xx + 628 ) ; xx [ 11 ] = xx [
29 ] * xx [ 530 ] - xx [ 632 ] ; xx [ 545 ] = xx [ 512 ] ; xx [ 546 ] = xx [
527 ] ; xx [ 547 ] = xx [ 528 ] ; xx [ 556 ] = xx [ 512 ] * xx [ 64 ] ; xx [
557 ] = xx [ 138 ] * xx [ 527 ] ; xx [ 558 ] = xx [ 139 ] * xx [ 528 ] ;
pm_math_Vector3_cross_ra ( xx + 545 , xx + 556 , xx + 565 ) ; xx [ 545 ] = xx
[ 565 ] - xx [ 628 ] ; xx [ 546 ] = ( xx [ 545 ] - xx [ 383 ] * xx [ 11 ] ) /
xx [ 102 ] ; xx [ 556 ] = xx [ 29 ] * ( xx [ 529 ] + ( xx [ 528 ] + xx [ 528
] ) * xx [ 448 ] ) - xx [ 631 ] ; xx [ 557 ] = xx [ 11 ] + xx [ 40 ] * xx [
546 ] ; xx [ 558 ] = ( xx [ 531 ] - ( xx [ 526 ] + xx [ 512 ] ) * xx [ 448 ]
) * xx [ 29 ] - xx [ 633 ] ; pm_math_Quaternion_xform_ra ( xx + 473 , xx +
556 , xx + 529 ) ; xx [ 568 ] = - xx [ 133 ] ; xx [ 569 ] = - xx [ 56 ] ; xx
[ 570 ] = - xx [ 124 ] ; xx [ 571 ] = - xx [ 113 ] ; xx [ 572 ] = - xx [ 158
] ; xx [ 573 ] = - xx [ 107 ] ; xx [ 574 ] = - xx [ 488 ] ; xx [ 575 ] = - xx
[ 131 ] ; xx [ 576 ] = - xx [ 490 ] ; xx [ 11 ] = xx [ 519 ] * xx [ 87 ] ; xx
[ 40 ] = xx [ 519 ] * xx [ 58 ] ; xx [ 56 ] = xx [ 517 ] * xx [ 87 ] + xx [
518 ] * xx [ 58 ] ; xx [ 556 ] = - xx [ 11 ] ; xx [ 557 ] = - xx [ 40 ] ; xx
[ 558 ] = xx [ 56 ] ; pm_math_Matrix3x3_transposeXform_ra ( xx + 568 , xx +
556 , xx + 611 ) ; xx [ 701 ] = xx [ 129 ] + xx [ 114 ] ; xx [ 702 ] = xx [
115 ] ; xx [ 703 ] = xx [ 116 ] ; xx [ 704 ] = xx [ 117 ] ; xx [ 705 ] = xx [
129 ] + xx [ 118 ] ; xx [ 706 ] = xx [ 119 ] ; xx [ 707 ] = xx [ 120 ] ; xx [
708 ] = xx [ 121 ] ; xx [ 709 ] = xx [ 130 ] ; pm_math_Vector3_cross_ra ( xx
+ 477 , xx + 590 , xx + 113 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 469
, xx + 113 , xx + 116 ) ; xx [ 58 ] = ( xx [ 518 ] + xx [ 254 ] ) * xx [ 610
] + xx [ 116 ] ; xx [ 87 ] = xx [ 117 ] - ( xx [ 517 ] + xx [ 72 ] ) * xx [
610 ] ; xx [ 113 ] = xx [ 58 ] ; xx [ 114 ] = xx [ 87 ] ; xx [ 115 ] = xx [
118 ] ; pm_math_Matrix3x3_xform_ra ( xx + 701 , xx + 113 , xx + 119 ) ; xx [
590 ] = xx [ 72 ] * xx [ 112 ] ; xx [ 591 ] = xx [ 254 ] * xx [ 135 ] ; xx [
592 ] = xx [ 94 ] * xx [ 519 ] ; pm_math_Vector3_cross_ra ( xx + 520 , xx +
590 , xx + 517 ) ; xx [ 520 ] = xx [ 545 ] - xx [ 64 ] * xx [ 546 ] ; xx [
521 ] = xx [ 566 ] - xx [ 629 ] + xx [ 138 ] * xx [ 528 ] * state [ 36 ] ; xx
[ 522 ] = xx [ 567 ] - xx [ 630 ] - xx [ 139 ] * xx [ 527 ] * state [ 36 ] ;
pm_math_Quaternion_xform_ra ( xx + 473 , xx + 520 , xx + 526 ) ;
pm_math_Vector3_cross_ra ( xx + 109 , xx + 529 , xx + 520 ) ; xx [ 628 ] = xx
[ 123 ] ; xx [ 629 ] = xx [ 108 ] ; xx [ 630 ] = xx [ 348 ] ; xx [ 631 ] = xx
[ 101 ] ; xx [ 632 ] = xx [ 57 ] ; xx [ 633 ] = xx [ 351 ] ; xx [ 634 ] = xx
[ 125 ] ; xx [ 635 ] = xx [ 89 ] ; xx [ 636 ] = xx [ 94 ] + xx [ 92 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 628 , xx + 556 , xx + 123 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 568 , xx + 113 , xx + 556 ) ; xx [ 57 ] =
xx [ 518 ] + xx [ 527 ] + xx [ 521 ] + xx [ 124 ] + xx [ 557 ] ; xx [ 72 ] =
xx [ 517 ] + xx [ 526 ] + xx [ 520 ] + xx [ 123 ] + xx [ 556 ] ; xx [ 89 ] =
xx [ 531 ] + xx [ 613 ] + xx [ 121 ] ; xx [ 92 ] = ( xx [ 57 ] * xx [ 35 ] -
xx [ 72 ] * xx [ 38 ] + xx [ 89 ] * xx [ 104 ] ) / xx [ 132 ] ; xx [ 113 ] =
xx [ 529 ] + xx [ 611 ] + xx [ 119 ] - xx [ 92 ] * xx [ 134 ] ; xx [ 114 ] =
xx [ 530 ] + xx [ 612 ] + xx [ 120 ] - xx [ 92 ] * xx [ 33 ] ; xx [ 115 ] =
xx [ 89 ] - xx [ 92 ] * xx [ 122 ] ; pm_math_Quaternion_xform_ra ( xx + 469 ,
xx + 113 , xx + 119 ) ; xx [ 113 ] = xx [ 127 ] ; xx [ 114 ] = xx [ 177 ] ;
xx [ 115 ] = xx [ 234 ] ; xx [ 116 ] = xx [ 180 ] ; xx [ 130 ] = - xx [ 173 ]
; xx [ 131 ] = xx [ 182 ] ; xx [ 132 ] = xx [ 193 ] ; xx [ 133 ] = xx [ 195 ]
; pm_math_Quaternion_inverseXform_ra ( xx + 113 , xx + 477 , xx + 529 ) ; xx
[ 33 ] = xx [ 38 ] * state [ 30 ] ; xx [ 89 ] = xx [ 529 ] - xx [ 33 ] ; xx [
101 ] = xx [ 35 ] * state [ 30 ] ; xx [ 102 ] = xx [ 530 ] + xx [ 101 ] ; xx
[ 565 ] = xx [ 89 ] ; xx [ 566 ] = xx [ 102 ] ; xx [ 567 ] = xx [ 531 ] ;
pm_math_Vector3_cross_ra ( xx + 565 , xx + 214 , xx + 568 ) ;
pm_math_Vector3_cross_ra ( xx + 565 , xx + 568 , xx + 571 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 130 , xx + 571 , xx + 574 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 130 , xx + 565 , xx + 571 ) ; xx [
107 ] = 1.099786928193694e-18 ; xx [ 108 ] = xx [ 107 ] * state [ 32 ] ;
pm_math_Quaternion_compose_ra ( xx + 541 , xx + 113 , xx + 610 ) ;
pm_math_Quaternion_compose_ra ( xx + 610 , xx + 130 , xx + 620 ) ; xx [ 117 ]
= xx [ 106 ] * xx [ 622 ] ; xx [ 122 ] = xx [ 106 ] * xx [ 623 ] ;
pm_math_Quaternion_xform_ra ( xx + 610 , xx + 214 , xx + 590 ) ;
pm_math_Quaternion_xform_ra ( xx + 541 , xx + 559 , xx + 610 ) ; xx [ 628 ] =
xx [ 449 ] * xx [ 620 ] + xx [ 451 ] * xx [ 621 ] ; xx [ 629 ] = xx [ 449 ] *
xx [ 621 ] - xx [ 451 ] * xx [ 620 ] ; xx [ 630 ] = xx [ 449 ] * xx [ 622 ] -
xx [ 451 ] * xx [ 623 ] ; xx [ 631 ] = xx [ 449 ] * xx [ 623 ] + xx [ 451 ] *
xx [ 622 ] ; xx [ 632 ] = ( xx [ 622 ] * xx [ 117 ] + xx [ 623 ] * xx [ 122 ]
) * xx [ 2 ] - xx [ 106 ] + xx [ 590 ] + xx [ 610 ] + xx [ 548 ] ; xx [ 633 ]
= xx [ 591 ] + xx [ 611 ] + xx [ 564 ] - ( xx [ 620 ] * xx [ 122 ] + xx [ 621
] * xx [ 117 ] ) * xx [ 2 ] ; xx [ 634 ] = xx [ 2 ] * ( xx [ 620 ] * xx [ 117
] - xx [ 621 ] * xx [ 122 ] ) + xx [ 592 ] + xx [ 612 ] + xx [ 533 ] ; xx [
117 ] = sm_core_compiler_computeProximityInfoCxpolyCxpoly (
simple_robot_9eb3ef65_1_geometry_0 ( rtdv ) ,
simple_robot_9eb3ef65_1_geometry_1 ( rtdv ) , ( pm_math_Transform3 * ) ( xx +
628 ) , ( pm_math_Transform3 * ) ( xx + 549 ) , ( pm_math_Vector3 * ) ( xx +
590 ) , ( pm_math_Vector3 * ) ( xx + 610 ) , ( pm_math_Vector3 * ) ( xx + 613
) , ( pm_math_Vector3 * ) ( xx + 620 ) ) ; xx [ 122 ] = xx [ 571 ] - state [
32 ] ; pm_math_Vector3_cross_ra ( xx + 477 , xx + 559 , xx + 635 ) ; xx [ 652
] = xx [ 635 ] + xx [ 599 ] ; xx [ 653 ] = xx [ 636 ] + xx [ 601 ] ; xx [ 654
] = xx [ 637 ] + xx [ 609 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 113 ,
xx + 652 , xx + 701 ) ; xx [ 127 ] = xx [ 210 ] * state [ 30 ] ; xx [ 652 ] =
xx [ 568 ] + xx [ 701 ] ; xx [ 653 ] = xx [ 569 ] + xx [ 702 ] ; xx [ 654 ] =
xx [ 570 ] + xx [ 703 ] + xx [ 127 ] ; pm_math_Quaternion_inverseXform_ra (
xx + 130 , xx + 652 , xx + 568 ) ; xx [ 701 ] = xx [ 122 ] ; xx [ 702 ] = xx
[ 572 ] ; xx [ 703 ] = xx [ 573 ] ; xx [ 704 ] = xx [ 568 ] ; xx [ 705 ] = xx
[ 569 ] + xx [ 108 ] ; xx [ 706 ] = xx [ 570 ] ;
sm_core_compiler_computeContactWrenches ( xx [ 117 ] , xx + 610 , xx + 590 ,
xx + 620 , xx + 613 , xx + 549 , xx + 583 , xx + 549 , xx + 628 , NULL , xx +
701 , 0 , 1 , xx [ 523 ] , xx [ 524 ] , xx [ 525 ] , xx [ 617 ] , xx [ 618 ]
, xx [ 619 ] , NULL , xx + 707 ) ; xx [ 117 ] = xx [ 29 ] * xx [ 575 ] - xx [
711 ] ; xx [ 568 ] = xx [ 122 ] ; xx [ 569 ] = xx [ 572 ] ; xx [ 570 ] = xx [
573 ] ; xx [ 590 ] = xx [ 64 ] * xx [ 122 ] ; xx [ 591 ] = xx [ 138 ] * xx [
572 ] ; xx [ 592 ] = xx [ 139 ] * xx [ 573 ] ; pm_math_Vector3_cross_ra ( xx
+ 568 , xx + 590 , xx + 610 ) ; xx [ 134 ] = xx [ 610 ] - xx [ 707 ] ; xx [
158 ] = ( xx [ 107 ] * xx [ 117 ] - xx [ 134 ] ) / xx [ 194 ] ; xx [ 193 ] =
xx [ 29 ] * ( xx [ 574 ] - ( xx [ 573 ] + xx [ 573 ] ) * xx [ 108 ] ) - xx [
710 ] ; xx [ 194 ] = xx [ 117 ] - xx [ 181 ] * xx [ 158 ] ; xx [ 195 ] = ( (
xx [ 571 ] + xx [ 122 ] ) * xx [ 108 ] + xx [ 576 ] ) * xx [ 29 ] - xx [ 712
] ; pm_math_Quaternion_xform_ra ( xx + 130 , xx + 193 , xx + 180 ) ; xx [ 713
] = - xx [ 219 ] ; xx [ 714 ] = - xx [ 171 ] ; xx [ 715 ] = - xx [ 218 ] ; xx
[ 716 ] = - xx [ 217 ] ; xx [ 717 ] = - xx [ 236 ] ; xx [ 718 ] = - xx [ 211
] ; xx [ 719 ] = - xx [ 91 ] ; xx [ 720 ] = - xx [ 231 ] ; xx [ 721 ] = - xx
[ 197 ] ; xx [ 91 ] = xx [ 531 ] * xx [ 101 ] ; xx [ 107 ] = xx [ 531 ] * xx
[ 33 ] ; xx [ 108 ] = xx [ 529 ] * xx [ 101 ] + xx [ 530 ] * xx [ 33 ] ; xx [
193 ] = - xx [ 91 ] ; xx [ 194 ] = - xx [ 107 ] ; xx [ 195 ] = xx [ 108 ] ;
pm_math_Matrix3x3_transposeXform_ra ( xx + 713 , xx + 193 , xx + 217 ) ; xx [
722 ] = xx [ 129 ] + xx [ 220 ] ; xx [ 723 ] = xx [ 221 ] ; xx [ 724 ] = xx [
222 ] ; xx [ 725 ] = xx [ 223 ] ; xx [ 726 ] = xx [ 129 ] + xx [ 224 ] ; xx [
727 ] = xx [ 225 ] ; xx [ 728 ] = xx [ 226 ] ; xx [ 729 ] = xx [ 227 ] ; xx [
730 ] = xx [ 229 ] ; pm_math_Vector3_cross_ra ( xx + 477 , xx + 635 , xx +
220 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 113 , xx + 220 , xx + 223 )
; xx [ 33 ] = ( xx [ 530 ] + xx [ 102 ] ) * xx [ 127 ] + xx [ 223 ] ; xx [
101 ] = xx [ 224 ] - ( xx [ 529 ] + xx [ 89 ] ) * xx [ 127 ] ; xx [ 220 ] =
xx [ 33 ] ; xx [ 221 ] = xx [ 101 ] ; xx [ 222 ] = xx [ 225 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 722 , xx + 220 , xx + 568 ) ; xx [ 574 ] =
xx [ 89 ] * xx [ 112 ] ; xx [ 575 ] = xx [ 102 ] * xx [ 135 ] ; xx [ 576 ] =
xx [ 94 ] * xx [ 531 ] ; pm_math_Vector3_cross_ra ( xx + 565 , xx + 574 , xx
+ 529 ) ; xx [ 565 ] = xx [ 134 ] + xx [ 64 ] * xx [ 158 ] ; xx [ 566 ] = xx
[ 611 ] - xx [ 708 ] - xx [ 138 ] * xx [ 573 ] * state [ 32 ] ; xx [ 567 ] =
xx [ 612 ] - xx [ 709 ] + xx [ 139 ] * xx [ 572 ] * state [ 32 ] ;
pm_math_Quaternion_xform_ra ( xx + 130 , xx + 565 , xx + 571 ) ;
pm_math_Vector3_cross_ra ( xx + 214 , xx + 180 , xx + 565 ) ; xx [ 628 ] = xx
[ 213 ] ; xx [ 629 ] = xx [ 212 ] ; xx [ 630 ] = xx [ 97 ] ; xx [ 631 ] = xx
[ 209 ] ; xx [ 632 ] = xx [ 192 ] ; xx [ 633 ] = xx [ 100 ] ; xx [ 634 ] = xx
[ 162 ] ; xx [ 635 ] = xx [ 128 ] ; xx [ 636 ] = xx [ 94 ] + xx [ 200 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 628 , xx + 193 , xx + 211 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 713 , xx + 220 , xx + 192 ) ; xx [ 89 ] =
xx [ 530 ] + xx [ 572 ] + xx [ 566 ] + xx [ 212 ] + xx [ 193 ] ; xx [ 97 ] =
xx [ 529 ] + xx [ 571 ] + xx [ 565 ] + xx [ 211 ] + xx [ 192 ] ; xx [ 100 ] =
xx [ 182 ] + xx [ 219 ] + xx [ 570 ] ; xx [ 102 ] = ( xx [ 89 ] * xx [ 35 ] -
xx [ 97 ] * xx [ 38 ] + xx [ 100 ] * xx [ 210 ] ) / xx [ 230 ] ; xx [ 219 ] =
xx [ 180 ] + xx [ 217 ] + xx [ 568 ] - xx [ 102 ] * xx [ 238 ] ; xx [ 220 ] =
xx [ 181 ] + xx [ 218 ] + xx [ 569 ] - xx [ 102 ] * xx [ 174 ] ; xx [ 221 ] =
xx [ 100 ] - xx [ 102 ] * xx [ 228 ] ; pm_math_Quaternion_xform_ra ( xx + 113
, xx + 219 , xx + 180 ) ; xx [ 217 ] = xx [ 232 ] ; xx [ 218 ] = xx [ 272 ] ;
xx [ 219 ] = xx [ 492 ] ; xx [ 220 ] = xx [ 274 ] ; xx [ 221 ] = - xx [ 160 ]
; xx [ 222 ] = xx [ 270 ] ; xx [ 223 ] = xx [ 176 ] ; xx [ 224 ] = xx [ 284 ]
; pm_math_Quaternion_inverseXform_ra ( xx + 217 , xx + 477 , xx + 226 ) ; xx
[ 100 ] = xx [ 38 ] * state [ 26 ] ; xx [ 117 ] = xx [ 226 ] - xx [ 100 ] ;
xx [ 122 ] = xx [ 35 ] * state [ 26 ] ; xx [ 127 ] = xx [ 227 ] + xx [ 122 ]
; xx [ 229 ] = xx [ 117 ] ; xx [ 230 ] = xx [ 127 ] ; xx [ 231 ] = xx [ 228 ]
; pm_math_Vector3_cross_ra ( xx + 229 , xx + 301 , xx + 568 ) ;
pm_math_Vector3_cross_ra ( xx + 229 , xx + 568 , xx + 574 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 221 , xx + 574 , xx + 590 ) ;
pm_math_Quaternion_compose_ra ( xx + 541 , xx + 217 , xx + 574 ) ;
pm_math_Quaternion_compose_ra ( xx + 574 , xx + 221 , xx + 610 ) ; xx [ 128 ]
= xx [ 106 ] * xx [ 612 ] ; xx [ 134 ] = xx [ 106 ] * xx [ 613 ] ;
pm_math_Quaternion_xform_ra ( xx + 574 , xx + 301 , xx + 614 ) ;
pm_math_Quaternion_xform_ra ( xx + 541 , xx + 580 , xx + 574 ) ; xx [ 628 ] =
xx [ 449 ] * xx [ 610 ] + xx [ 451 ] * xx [ 611 ] ; xx [ 629 ] = xx [ 449 ] *
xx [ 611 ] - xx [ 451 ] * xx [ 610 ] ; xx [ 630 ] = xx [ 449 ] * xx [ 612 ] -
xx [ 451 ] * xx [ 613 ] ; xx [ 631 ] = xx [ 449 ] * xx [ 613 ] + xx [ 451 ] *
xx [ 612 ] ; xx [ 632 ] = ( xx [ 612 ] * xx [ 128 ] + xx [ 613 ] * xx [ 134 ]
) * xx [ 2 ] - xx [ 106 ] + xx [ 614 ] + xx [ 574 ] + xx [ 548 ] ; xx [ 633 ]
= xx [ 615 ] + xx [ 575 ] + xx [ 564 ] - ( xx [ 610 ] * xx [ 134 ] + xx [ 611
] * xx [ 128 ] ) * xx [ 2 ] ; xx [ 634 ] = xx [ 2 ] * ( xx [ 610 ] * xx [ 128
] - xx [ 611 ] * xx [ 134 ] ) + xx [ 616 ] + xx [ 576 ] + xx [ 533 ] ; xx [
128 ] = sm_core_compiler_computeProximityInfoCxpolyCxpoly (
simple_robot_9eb3ef65_1_geometry_0 ( rtdv ) ,
simple_robot_9eb3ef65_1_geometry_1 ( rtdv ) , ( pm_math_Transform3 * ) ( xx +
628 ) , ( pm_math_Transform3 * ) ( xx + 549 ) , ( pm_math_Vector3 * ) ( xx +
541 ) , ( pm_math_Vector3 * ) ( xx + 574 ) , ( pm_math_Vector3 * ) ( xx + 610
) , ( pm_math_Vector3 * ) ( xx + 613 ) ) ; pm_math_Quaternion_inverseXform_ra
( xx + 221 , xx + 229 , xx + 620 ) ; xx [ 134 ] = xx [ 620 ] - state [ 28 ] ;
pm_math_Vector3_cross_ra ( xx + 477 , xx + 580 , xx + 635 ) ; xx [ 652 ] = xx
[ 635 ] + xx [ 599 ] ; xx [ 653 ] = xx [ 636 ] + xx [ 601 ] ; xx [ 654 ] = xx
[ 637 ] + xx [ 609 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 217 , xx +
652 , xx + 701 ) ; xx [ 160 ] = xx [ 259 ] * state [ 26 ] ; xx [ 652 ] = xx [
568 ] + xx [ 701 ] ; xx [ 653 ] = xx [ 569 ] + xx [ 702 ] ; xx [ 654 ] = xx [
570 ] + xx [ 703 ] + xx [ 160 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
221 , xx + 652 , xx + 568 ) ; xx [ 701 ] = xx [ 134 ] ; xx [ 702 ] = xx [ 621
] ; xx [ 703 ] = xx [ 622 ] ; xx [ 704 ] = xx [ 568 ] ; xx [ 705 ] = xx [ 569
] ; xx [ 706 ] = xx [ 570 ] ; sm_core_compiler_computeContactWrenches ( xx [
128 ] , xx + 574 , xx + 541 , xx + 613 , xx + 610 , xx + 549 , xx + 583 , xx
+ 549 , xx + 628 , NULL , xx + 701 , 0 , 1 , xx [ 523 ] , xx [ 524 ] , xx [
525 ] , xx [ 617 ] , xx [ 618 ] , xx [ 619 ] , NULL , xx + 707 ) ; xx [ 541 ]
= xx [ 29 ] * xx [ 590 ] - xx [ 710 ] ; xx [ 542 ] = xx [ 29 ] * xx [ 591 ] -
xx [ 711 ] ; xx [ 543 ] = xx [ 29 ] * xx [ 592 ] - xx [ 712 ] ;
pm_math_Quaternion_xform_ra ( xx + 221 , xx + 541 , xx + 568 ) ; xx [ 713 ] =
- xx [ 317 ] ; xx [ 714 ] = - xx [ 320 ] ; xx [ 715 ] = - xx [ 323 ] ; xx [
716 ] = - xx [ 318 ] ; xx [ 717 ] = - xx [ 321 ] ; xx [ 718 ] = - xx [ 324 ]
; xx [ 719 ] = - xx [ 319 ] ; xx [ 720 ] = - xx [ 322 ] ; xx [ 721 ] = - xx [
325 ] ; xx [ 128 ] = xx [ 228 ] * xx [ 122 ] ; xx [ 162 ] = xx [ 228 ] * xx [
100 ] ; xx [ 171 ] = xx [ 226 ] * xx [ 122 ] + xx [ 227 ] * xx [ 100 ] ; xx [
317 ] = - xx [ 128 ] ; xx [ 318 ] = - xx [ 162 ] ; xx [ 319 ] = xx [ 171 ] ;
pm_math_Matrix3x3_transposeXform_ra ( xx + 713 , xx + 317 , xx + 320 ) ; xx [
722 ] = xx [ 129 ] + xx [ 308 ] ; xx [ 723 ] = xx [ 309 ] ; xx [ 724 ] = xx [
310 ] ; xx [ 725 ] = xx [ 311 ] ; xx [ 726 ] = xx [ 129 ] + xx [ 312 ] ; xx [
727 ] = xx [ 313 ] ; xx [ 728 ] = xx [ 314 ] ; xx [ 729 ] = xx [ 315 ] ; xx [
730 ] = xx [ 286 ] ; pm_math_Vector3_cross_ra ( xx + 477 , xx + 635 , xx +
308 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 217 , xx + 308 , xx + 311 )
; xx [ 308 ] = ( xx [ 227 ] + xx [ 127 ] ) * xx [ 160 ] + xx [ 311 ] ; xx [
309 ] = xx [ 312 ] - ( xx [ 226 ] + xx [ 117 ] ) * xx [ 160 ] ; xx [ 310 ] =
xx [ 313 ] ; pm_math_Matrix3x3_xform_ra ( xx + 722 , xx + 308 , xx + 311 ) ;
xx [ 323 ] = xx [ 117 ] * xx [ 112 ] ; xx [ 324 ] = xx [ 127 ] * xx [ 135 ] ;
xx [ 325 ] = xx [ 94 ] * xx [ 228 ] ; pm_math_Vector3_cross_ra ( xx + 229 ,
xx + 323 , xx + 226 ) ; xx [ 229 ] = xx [ 134 ] ; xx [ 230 ] = xx [ 621 ] ;
xx [ 231 ] = xx [ 622 ] ; xx [ 323 ] = xx [ 64 ] * xx [ 134 ] ; xx [ 324 ] =
xx [ 138 ] * xx [ 621 ] ; xx [ 325 ] = xx [ 139 ] * xx [ 622 ] ;
pm_math_Vector3_cross_ra ( xx + 229 , xx + 323 , xx + 541 ) ; xx [ 100 ] = xx
[ 541 ] - xx [ 707 ] ; xx [ 117 ] = xx [ 100 ] / xx [ 299 ] ; xx [ 229 ] = xx
[ 100 ] - xx [ 64 ] * xx [ 117 ] ; xx [ 230 ] = xx [ 542 ] - xx [ 708 ] - xx
[ 138 ] * xx [ 622 ] * state [ 28 ] ; xx [ 231 ] = xx [ 543 ] - xx [ 709 ] +
xx [ 139 ] * xx [ 621 ] * state [ 28 ] ; pm_math_Quaternion_xform_ra ( xx +
221 , xx + 229 , xx + 323 ) ; pm_math_Vector3_cross_ra ( xx + 301 , xx + 568
, xx + 229 ) ; xx [ 628 ] = xx [ 233 ] ; xx [ 629 ] = xx [ 172 ] ; xx [ 630 ]
= xx [ 202 ] ; xx [ 631 ] = xx [ 37 ] ; xx [ 632 ] = xx [ 30 ] ; xx [ 633 ] =
xx [ 205 ] ; xx [ 634 ] = xx [ 199 ] ; xx [ 635 ] = xx [ 178 ] ; xx [ 636 ] =
xx [ 94 ] + xx [ 207 ] ; pm_math_Matrix3x3_xform_ra ( xx + 628 , xx + 317 ,
xx + 172 ) ; pm_math_Matrix3x3_xform_ra ( xx + 713 , xx + 308 , xx + 176 ) ;
xx [ 30 ] = xx [ 227 ] + xx [ 324 ] + xx [ 230 ] + xx [ 173 ] + xx [ 177 ] ;
xx [ 37 ] = xx [ 226 ] + xx [ 323 ] + xx [ 229 ] + xx [ 172 ] + xx [ 176 ] ;
xx [ 100 ] = xx [ 570 ] + xx [ 322 ] + xx [ 313 ] ; xx [ 122 ] = ( xx [ 30 ]
* xx [ 35 ] - xx [ 37 ] * xx [ 38 ] + xx [ 100 ] * xx [ 259 ] ) / xx [ 298 ]
; xx [ 232 ] = xx [ 568 ] + xx [ 320 ] + xx [ 311 ] - xx [ 122 ] * xx [ 287 ]
; xx [ 233 ] = xx [ 569 ] + xx [ 321 ] + xx [ 312 ] - xx [ 122 ] * xx [ 306 ]
; xx [ 234 ] = xx [ 100 ] - xx [ 122 ] * xx [ 297 ] ;
pm_math_Quaternion_xform_ra ( xx + 217 , xx + 232 , xx + 297 ) ; xx [ 308 ] =
- xx [ 331 ] ; xx [ 309 ] = xx [ 355 ] ; xx [ 310 ] = xx [ 356 ] ; xx [ 311 ]
= xx [ 357 ] ; xx [ 312 ] = - xx [ 327 ] ; xx [ 313 ] = xx [ 367 ] ; xx [ 314
] = xx [ 370 ] ; xx [ 315 ] = xx [ 372 ] ; pm_math_Quaternion_inverseXform_ra
( xx + 308 , xx + 477 , xx + 232 ) ; xx [ 100 ] = xx [ 330 ] * state [ 22 ] ;
xx [ 127 ] = xx [ 233 ] + xx [ 100 ] ; xx [ 134 ] = xx [ 333 ] * state [ 22 ]
; xx [ 160 ] = xx [ 234 ] - xx [ 134 ] ; xx [ 301 ] = xx [ 232 ] ; xx [ 302 ]
= xx [ 127 ] ; xx [ 303 ] = xx [ 160 ] ; pm_math_Vector3_cross_ra ( xx + 301
, xx + 394 , xx + 317 ) ; pm_math_Vector3_cross_ra ( xx + 301 , xx + 317 , xx
+ 320 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 312 , xx + 320 , xx + 317
) ; pm_math_Quaternion_inverseXform_ra ( xx + 312 , xx + 301 , xx + 320 ) ;
xx [ 195 ] = 1.099786928193684e-18 ; xx [ 197 ] = xx [ 195 ] * state [ 24 ] ;
xx [ 199 ] = xx [ 29 ] * xx [ 318 ] ; xx [ 200 ] = xx [ 320 ] - state [ 24 ]
; xx [ 355 ] = xx [ 200 ] ; xx [ 356 ] = xx [ 321 ] ; xx [ 357 ] = xx [ 322 ]
; xx [ 541 ] = xx [ 64 ] * xx [ 200 ] ; xx [ 542 ] = xx [ 138 ] * xx [ 321 ]
; xx [ 543 ] = xx [ 139 ] * xx [ 322 ] ; pm_math_Vector3_cross_ra ( xx + 355
, xx + 541 , xx + 568 ) ; xx [ 202 ] = ( xx [ 195 ] * xx [ 199 ] - xx [ 568 ]
) / xx [ 374 ] ; xx [ 355 ] = xx [ 29 ] * ( xx [ 317 ] - ( xx [ 322 ] + xx [
322 ] ) * xx [ 197 ] ) ; xx [ 356 ] = xx [ 199 ] - xx [ 353 ] * xx [ 202 ] ;
xx [ 357 ] = ( ( xx [ 320 ] + xx [ 200 ] ) * xx [ 197 ] + xx [ 319 ] ) * xx [
29 ] ; pm_math_Quaternion_xform_ra ( xx + 312 , xx + 355 , xx + 317 ) ; xx [
628 ] = - xx [ 261 ] ; xx [ 629 ] = - xx [ 408 ] ; xx [ 630 ] = - xx [ 329 ]
; xx [ 631 ] = - xx [ 398 ] ; xx [ 632 ] = - xx [ 440 ] ; xx [ 633 ] = - xx [
413 ] ; xx [ 634 ] = - xx [ 417 ] ; xx [ 635 ] = - xx [ 332 ] ; xx [ 636 ] =
- xx [ 378 ] ; xx [ 195 ] = xx [ 233 ] * xx [ 134 ] + xx [ 234 ] * xx [ 100 ]
; xx [ 197 ] = xx [ 232 ] * xx [ 134 ] ; xx [ 134 ] = xx [ 232 ] * xx [ 100 ]
; xx [ 355 ] = - xx [ 195 ] ; xx [ 356 ] = xx [ 197 ] ; xx [ 357 ] = xx [ 134
] ; pm_math_Matrix3x3_transposeXform_ra ( xx + 628 , xx + 355 , xx + 541 ) ;
xx [ 701 ] = xx [ 389 ] ; xx [ 702 ] = xx [ 400 ] ; xx [ 703 ] = xx [ 401 ] ;
xx [ 704 ] = xx [ 402 ] ; xx [ 705 ] = xx [ 307 ] + xx [ 403 ] ; xx [ 706 ] =
xx [ 404 ] ; xx [ 707 ] = xx [ 405 ] ; xx [ 708 ] = xx [ 406 ] ; xx [ 709 ] =
xx [ 307 ] + xx [ 407 ] ; pm_math_Vector3_cross_ra ( xx + 477 , xx + 602 , xx
+ 400 ) ; pm_math_Vector3_cross_ra ( xx + 477 , xx + 400 , xx + 403 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 308 , xx + 403 , xx + 400 ) ; xx [
100 ] = xx [ 390 ] * state [ 22 ] ; xx [ 199 ] = ( xx [ 234 ] + xx [ 160 ] )
* xx [ 100 ] + xx [ 401 ] ; xx [ 200 ] = xx [ 402 ] - ( xx [ 233 ] + xx [ 127
] ) * xx [ 100 ] ; xx [ 401 ] = xx [ 400 ] ; xx [ 402 ] = xx [ 199 ] ; xx [
403 ] = xx [ 200 ] ; pm_math_Matrix3x3_xform_ra ( xx + 701 , xx + 401 , xx +
404 ) ; xx [ 100 ] = xx [ 317 ] + xx [ 541 ] + xx [ 404 ] ; xx [ 574 ] = xx [
206 ] * xx [ 232 ] ; xx [ 575 ] = xx [ 127 ] * xx [ 419 ] ; xx [ 576 ] = xx [
160 ] * xx [ 397 ] ; pm_math_Vector3_cross_ra ( xx + 301 , xx + 574 , xx +
232 ) ; xx [ 301 ] = xx [ 568 ] + xx [ 64 ] * xx [ 202 ] ; xx [ 302 ] = xx [
569 ] - xx [ 138 ] * xx [ 322 ] * state [ 24 ] ; xx [ 303 ] = xx [ 570 ] + xx
[ 139 ] * xx [ 321 ] * state [ 24 ] ; pm_math_Quaternion_xform_ra ( xx + 312
, xx + 301 , xx + 320 ) ; pm_math_Vector3_cross_ra ( xx + 394 , xx + 317 , xx
+ 301 ) ; xx [ 701 ] = xx [ 206 ] + xx [ 98 ] ; xx [ 702 ] = xx [ 273 ] ; xx
[ 703 ] = xx [ 334 ] ; xx [ 704 ] = xx [ 352 ] ; xx [ 705 ] = xx [ 375 ] ; xx
[ 706 ] = xx [ 388 ] ; xx [ 707 ] = xx [ 379 ] ; xx [ 708 ] = xx [ 392 ] ; xx
[ 709 ] = xx [ 399 ] ; pm_math_Matrix3x3_xform_ra ( xx + 701 , xx + 355 , xx
+ 272 ) ; pm_math_Matrix3x3_xform_ra ( xx + 628 , xx + 401 , xx + 351 ) ; xx
[ 98 ] = xx [ 233 ] + xx [ 321 ] + xx [ 302 ] + xx [ 273 ] + xx [ 352 ] ; xx
[ 127 ] = xx [ 234 ] + xx [ 322 ] + xx [ 303 ] + xx [ 274 ] + xx [ 353 ] ; xx
[ 160 ] = ( xx [ 98 ] * xx [ 330 ] - xx [ 127 ] * xx [ 333 ] + xx [ 100 ] *
xx [ 390 ] ) / xx [ 410 ] ; xx [ 355 ] = xx [ 100 ] - xx [ 160 ] * xx [ 418 ]
; xx [ 356 ] = xx [ 318 ] + xx [ 542 ] + xx [ 405 ] - xx [ 160 ] * xx [ 376 ]
; xx [ 357 ] = xx [ 319 ] + xx [ 543 ] + xx [ 406 ] - xx [ 160 ] * xx [ 371 ]
; pm_math_Quaternion_xform_ra ( xx + 308 , xx + 355 , xx + 317 ) ; xx [ 401 ]
= - xx [ 454 ] ; xx [ 402 ] = xx [ 459 ] ; xx [ 403 ] = xx [ 460 ] ; xx [ 404
] = xx [ 461 ] ; xx [ 405 ] = - xx [ 354 ] ; xx [ 406 ] = xx [ 453 ] ; xx [
407 ] = xx [ 415 ] ; xx [ 408 ] = xx [ 457 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 401 , xx + 477 , xx + 354 ) ; xx [
100 ] = xx [ 330 ] * state [ 18 ] ; xx [ 205 ] = xx [ 355 ] + xx [ 100 ] ; xx
[ 207 ] = xx [ 456 ] * state [ 18 ] ; xx [ 209 ] = xx [ 356 ] - xx [ 207 ] ;
xx [ 370 ] = xx [ 354 ] ; xx [ 371 ] = xx [ 205 ] ; xx [ 372 ] = xx [ 209 ] ;
pm_math_Vector3_cross_ra ( xx + 370 , xx + 485 , xx + 374 ) ;
pm_math_Vector3_cross_ra ( xx + 370 , xx + 374 , xx + 459 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 405 , xx + 459 , xx + 374 ) ; xx [
459 ] = xx [ 29 ] * xx [ 374 ] ; xx [ 460 ] = xx [ 29 ] * xx [ 375 ] ; xx [
461 ] = xx [ 29 ] * xx [ 376 ] ; pm_math_Quaternion_xform_ra ( xx + 405 , xx
+ 459 , xx + 374 ) ; xx [ 628 ] = - xx [ 502 ] ; xx [ 629 ] = - xx [ 505 ] ;
xx [ 630 ] = - xx [ 508 ] ; xx [ 631 ] = - xx [ 503 ] ; xx [ 632 ] = - xx [
506 ] ; xx [ 633 ] = - xx [ 509 ] ; xx [ 634 ] = - xx [ 504 ] ; xx [ 635 ] =
- xx [ 507 ] ; xx [ 636 ] = - xx [ 510 ] ; xx [ 29 ] = xx [ 355 ] * xx [ 207
] + xx [ 356 ] * xx [ 100 ] ; xx [ 236 ] = xx [ 354 ] * xx [ 207 ] ; xx [ 207
] = xx [ 354 ] * xx [ 100 ] ; xx [ 459 ] = - xx [ 29 ] ; xx [ 460 ] = xx [
236 ] ; xx [ 461 ] = xx [ 207 ] ; pm_math_Matrix3x3_transposeXform_ra ( xx +
628 , xx + 459 , xx + 502 ) ; xx [ 701 ] = xx [ 452 ] ; xx [ 702 ] = xx [ 494
] ; xx [ 703 ] = xx [ 495 ] ; xx [ 704 ] = xx [ 496 ] ; xx [ 705 ] = xx [ 307
] + xx [ 497 ] ; xx [ 706 ] = xx [ 498 ] ; xx [ 707 ] = xx [ 499 ] ; xx [ 708
] = xx [ 500 ] ; xx [ 709 ] = xx [ 307 ] + xx [ 501 ] ;
pm_math_Vector3_cross_ra ( xx + 477 , xx + 625 , xx + 452 ) ;
pm_math_Vector3_cross_ra ( xx + 477 , xx + 452 , xx + 494 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 401 , xx + 494 , xx + 452 ) ; xx [
100 ] = xx [ 455 ] * state [ 18 ] ; xx [ 494 ] = xx [ 452 ] ; xx [ 495 ] = (
xx [ 356 ] + xx [ 209 ] ) * xx [ 100 ] + xx [ 453 ] ; xx [ 496 ] = xx [ 454 ]
- ( xx [ 355 ] + xx [ 205 ] ) * xx [ 100 ] ; pm_math_Matrix3x3_xform_ra ( xx
+ 701 , xx + 494 , xx + 355 ) ; xx [ 100 ] = xx [ 374 ] + xx [ 502 ] + xx [
355 ] ; xx [ 452 ] = xx [ 206 ] * xx [ 354 ] ; xx [ 453 ] = xx [ 205 ] * xx [
419 ] ; xx [ 454 ] = xx [ 209 ] * xx [ 397 ] ; pm_math_Vector3_cross_ra ( xx
+ 370 , xx + 452 , xx + 397 ) ; pm_math_Quaternion_inverseXform_ra ( xx + 405
, xx + 370 , xx + 417 ) ; xx [ 205 ] = xx [ 417 ] - state [ 20 ] ; xx [ 370 ]
= xx [ 205 ] ; xx [ 371 ] = xx [ 418 ] ; xx [ 372 ] = xx [ 419 ] ; xx [ 452 ]
= xx [ 64 ] * xx [ 205 ] ; xx [ 453 ] = xx [ 138 ] * xx [ 418 ] ; xx [ 454 ]
= xx [ 139 ] * xx [ 419 ] ; pm_math_Vector3_cross_ra ( xx + 370 , xx + 452 ,
xx + 497 ) ; xx [ 205 ] = xx [ 497 ] / xx [ 90 ] ; xx [ 370 ] = xx [ 497 ] -
xx [ 64 ] * xx [ 205 ] ; xx [ 371 ] = xx [ 498 ] - xx [ 138 ] * xx [ 419 ] *
state [ 20 ] ; xx [ 372 ] = xx [ 499 ] + xx [ 139 ] * xx [ 418 ] * state [ 20
] ; pm_math_Quaternion_xform_ra ( xx + 405 , xx + 370 , xx + 417 ) ;
pm_math_Vector3_cross_ra ( xx + 485 , xx + 374 , xx + 370 ) ; xx [ 701 ] = xx
[ 206 ] + xx [ 349 ] ; xx [ 702 ] = xx [ 373 ] ; xx [ 703 ] = xx [ 411 ] ; xx
[ 704 ] = xx [ 384 ] ; xx [ 705 ] = xx [ 31 ] ; xx [ 706 ] = xx [ 316 ] ; xx
[ 707 ] = xx [ 387 ] ; xx [ 708 ] = xx [ 369 ] ; xx [ 709 ] = xx [ 441 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 701 , xx + 459 , xx + 387 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 628 , xx + 494 , xx + 452 ) ; xx [ 31 ] =
xx [ 398 ] + xx [ 418 ] + xx [ 371 ] + xx [ 388 ] + xx [ 453 ] ; xx [ 90 ] =
xx [ 399 ] + xx [ 419 ] + xx [ 372 ] + xx [ 389 ] + xx [ 454 ] ; xx [ 206 ] =
( xx [ 31 ] * xx [ 330 ] - xx [ 90 ] * xx [ 456 ] + xx [ 100 ] * xx [ 455 ] )
/ xx [ 480 ] ; xx [ 459 ] = xx [ 100 ] - xx [ 206 ] * xx [ 271 ] ; xx [ 460 ]
= xx [ 375 ] + xx [ 503 ] + xx [ 356 ] - xx [ 206 ] * xx [ 482 ] ; xx [ 461 ]
= xx [ 376 ] + xx [ 504 ] + xx [ 357 ] - xx [ 206 ] * xx [ 484 ] ;
pm_math_Quaternion_xform_ra ( xx + 401 , xx + 459 , xx + 354 ) ; xx [ 373 ] =
xx [ 17 ] ; xx [ 374 ] = xx [ 36 ] ; xx [ 375 ] = - xx [ 46 ] ;
pm_math_Vector3_cross_ra ( xx + 61 , xx + 373 , xx + 459 ) ;
pm_math_Matrix3x3_transposeXform_ra ( xx + 640 , xx + 459 , xx + 373 ) ; xx [
484 ] = xx [ 61 ] + xx [ 32 ] ; xx [ 485 ] = xx [ 62 ] + xx [ 45 ] ; xx [ 486
] = xx [ 63 ] + xx [ 51 ] ; xx [ 61 ] = - xx [ 579 ] ; xx [ 62 ] = xx [ 600 ]
; xx [ 63 ] = xx [ 608 ] ; pm_math_Vector3_cross_ra ( xx + 484 , xx + 61 , xx
+ 494 ) ; pm_math_Vector3_cross_ra ( xx + 53 , xx + 593 , xx + 61 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 65 , xx + 61 , xx + 484 ) ; xx [ 17
] = xx [ 494 ] + xx [ 484 ] ; xx [ 36 ] = xx [ 495 ] + xx [ 485 ] ; xx [ 46 ]
= xx [ 496 ] + xx [ 486 ] ; xx [ 61 ] = xx [ 17 ] ; xx [ 62 ] = xx [ 36 ] ;
xx [ 63 ] = xx [ 46 ] ; pm_math_Matrix3x3_xform_ra ( xx + 656 , xx + 61 , xx
+ 484 ) ; xx [ 100 ] = xx [ 119 ] + xx [ 180 ] + xx [ 297 ] + xx [ 317 ] + xx
[ 354 ] + xx [ 373 ] + xx [ 484 ] ; xx [ 494 ] = xx [ 32 ] * xx [ 347 ] ; xx
[ 495 ] = xx [ 45 ] * xx [ 446 ] ; xx [ 496 ] = xx [ 51 ] * xx [ 78 ] ;
pm_math_Vector3_cross_ra ( xx + 477 , xx + 494 , xx + 347 ) ; xx [ 477 ] = xx
[ 72 ] - xx [ 92 ] * xx [ 39 ] ; xx [ 478 ] = xx [ 57 ] - xx [ 92 ] * xx [ 88
] ; xx [ 479 ] = xx [ 519 ] + xx [ 528 ] + xx [ 522 ] + xx [ 125 ] + xx [ 558
] - xx [ 92 ] * xx [ 34 ] ; pm_math_Quaternion_xform_ra ( xx + 469 , xx + 477
, xx + 123 ) ; pm_math_Vector3_cross_ra ( xx + 538 , xx + 119 , xx + 477 ) ;
xx [ 494 ] = xx [ 97 ] - xx [ 102 ] * xx [ 179 ] ; xx [ 495 ] = xx [ 89 ] -
xx [ 102 ] * xx [ 196 ] ; xx [ 496 ] = xx [ 531 ] + xx [ 573 ] + xx [ 567 ] +
xx [ 213 ] + xx [ 194 ] - xx [ 102 ] * xx [ 175 ] ;
pm_math_Quaternion_xform_ra ( xx + 113 , xx + 494 , xx + 192 ) ;
pm_math_Vector3_cross_ra ( xx + 559 , xx + 180 , xx + 211 ) ; xx [ 175 ] = xx
[ 37 ] - xx [ 122 ] * xx [ 285 ] ; xx [ 176 ] = xx [ 30 ] - xx [ 122 ] * xx [
126 ] ; xx [ 177 ] = xx [ 228 ] + xx [ 325 ] + xx [ 231 ] + xx [ 174 ] + xx [
178 ] - xx [ 122 ] * xx [ 237 ] ; pm_math_Quaternion_xform_ra ( xx + 217 , xx
+ 175 , xx + 172 ) ; pm_math_Vector3_cross_ra ( xx + 580 , xx + 297 , xx +
175 ) ; xx [ 178 ] = xx [ 232 ] + xx [ 320 ] + xx [ 301 ] + xx [ 272 ] + xx [
351 ] - xx [ 160 ] * xx [ 344 ] ; xx [ 179 ] = xx [ 98 ] - xx [ 160 ] * xx [
391 ] ; xx [ 180 ] = xx [ 127 ] - xx [ 160 ] * xx [ 409 ] ;
pm_math_Quaternion_xform_ra ( xx + 308 , xx + 178 , xx + 226 ) ;
pm_math_Vector3_cross_ra ( xx + 602 , xx + 317 , xx + 178 ) ; xx [ 229 ] = xx
[ 397 ] + xx [ 417 ] + xx [ 370 ] + xx [ 387 ] + xx [ 452 ] - xx [ 206 ] * xx
[ 414 ] ; xx [ 230 ] = xx [ 31 ] - xx [ 206 ] * xx [ 368 ] ; xx [ 231 ] = xx
[ 90 ] - xx [ 206 ] * xx [ 458 ] ; pm_math_Quaternion_xform_ra ( xx + 401 ,
xx + 229 , xx + 30 ) ; pm_math_Vector3_cross_ra ( xx + 625 , xx + 354 , xx +
88 ) ; pm_math_Matrix3x3_xform_ra ( xx + 140 , xx + 459 , xx + 229 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 640 , xx + 61 , xx + 140 ) ; xx [ 34 ] = xx
[ 347 ] + xx [ 123 ] + xx [ 477 ] + xx [ 192 ] + xx [ 211 ] + xx [ 172 ] + xx
[ 175 ] + xx [ 226 ] + xx [ 178 ] + xx [ 30 ] + xx [ 88 ] + xx [ 229 ] + xx [
140 ] ; xx [ 37 ] = xx [ 348 ] + xx [ 124 ] + xx [ 478 ] + xx [ 193 ] + xx [
212 ] + xx [ 173 ] + xx [ 176 ] + xx [ 227 ] + xx [ 179 ] + xx [ 31 ] + xx [
89 ] + xx [ 230 ] + xx [ 141 ] ; xx [ 30 ] = xx [ 349 ] + xx [ 125 ] + xx [
479 ] + xx [ 194 ] + xx [ 213 ] + xx [ 174 ] + xx [ 177 ] + xx [ 228 ] + xx [
180 ] + xx [ 32 ] + xx [ 90 ] + xx [ 231 ] + xx [ 142 ] ; xx [ 61 ] = xx [ 34
] ; xx [ 62 ] = xx [ 37 ] ; xx [ 63 ] = xx [ 30 ] ; xx [ 31 ] = xx [ 120 ] +
xx [ 181 ] + xx [ 298 ] + xx [ 318 ] + xx [ 355 ] + xx [ 374 ] + xx [ 485 ] ;
xx [ 32 ] = xx [ 121 ] + xx [ 182 ] + xx [ 299 ] + xx [ 319 ] + xx [ 356 ] +
xx [ 375 ] + xx [ 486 ] ; xx [ 88 ] = xx [ 100 ] ; xx [ 89 ] = xx [ 31 ] ; xx
[ 90 ] = xx [ 32 ] ; xx [ 39 ] = ( pm_math_Vector3_dot_ra ( xx + 649 , xx +
61 ) + pm_math_Vector3_dot_ra ( xx + 264 , xx + 88 ) ) / xx [ 153 ] ; xx [ 61
] = xx [ 100 ] - xx [ 345 ] * xx [ 39 ] ; xx [ 62 ] = xx [ 31 ] - xx [ 84 ] *
xx [ 39 ] ; xx [ 63 ] = xx [ 32 ] - xx [ 85 ] * xx [ 39 ] ;
pm_math_Quaternion_xform_ra ( xx + 65 , xx + 61 , xx + 88 ) ; xx [ 140 ] = xx
[ 82 ] ; xx [ 141 ] = xx [ 256 ] ; xx [ 142 ] = xx [ 165 ] ; xx [ 143 ] = xx
[ 83 ] ; xx [ 144 ] = xx [ 241 ] ; xx [ 145 ] = xx [ 242 ] ; xx [ 146 ] = xx
[ 24 ] ; xx [ 147 ] = xx [ 258 ] ; xx [ 148 ] = xx [ 269 ] ; xx [ 24 ] = xx [
6 ] * xx [ 15 ] ; xx [ 31 ] = xx [ 6 ] * xx [ 0 ] ; xx [ 32 ] = xx [ 4 ] * xx
[ 15 ] + xx [ 5 ] * xx [ 0 ] ; xx [ 61 ] = xx [ 24 ] ; xx [ 62 ] = xx [ 31 ]
; xx [ 63 ] = - xx [ 32 ] ; pm_math_Matrix3x3_transposeXform_ra ( xx + 140 ,
xx + 61 , xx + 82 ) ; xx [ 172 ] = xx [ 129 ] + xx [ 183 ] ; xx [ 173 ] = xx
[ 184 ] ; xx [ 174 ] = xx [ 185 ] ; xx [ 175 ] = xx [ 186 ] ; xx [ 176 ] = xx
[ 129 ] + xx [ 187 ] ; xx [ 177 ] = xx [ 188 ] ; xx [ 178 ] = xx [ 189 ] ; xx
[ 179 ] = xx [ 190 ] ; xx [ 180 ] = xx [ 245 ] ; pm_math_Vector3_cross_ra (
xx + 42 , xx + 596 , xx + 119 ) ; pm_math_Quaternion_inverseXform_ra ( xx +
465 , xx + 119 , xx + 123 ) ; xx [ 0 ] = ( xx [ 5 ] + xx [ 16 ] ) * xx [ 578
] + xx [ 123 ] ; xx [ 5 ] = xx [ 124 ] - ( xx [ 4 ] + xx [ 14 ] ) * xx [ 578
] ; xx [ 119 ] = xx [ 0 ] ; xx [ 120 ] = xx [ 5 ] ; xx [ 121 ] = xx [ 125 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 172 , xx + 119 , xx + 181 ) ; xx [ 172 ] =
xx [ 14 ] * xx [ 112 ] ; xx [ 173 ] = xx [ 16 ] * xx [ 135 ] ; xx [ 174 ] =
xx [ 94 ] * xx [ 6 ] ; pm_math_Vector3_cross_ra ( xx + 53 , xx + 172 , xx +
14 ) ; xx [ 53 ] = xx [ 34 ] - xx [ 86 ] * xx [ 39 ] ; xx [ 54 ] = xx [ 37 ]
- xx [ 149 ] * xx [ 39 ] ; xx [ 55 ] = xx [ 30 ] - xx [ 80 ] * xx [ 39 ] ;
pm_math_Quaternion_xform_ra ( xx + 65 , xx + 53 , xx + 172 ) ;
pm_math_Vector3_cross_ra ( xx + 154 , xx + 88 , xx + 53 ) ; xx [ 226 ] = xx [
79 ] ; xx [ 227 ] = xx [ 164 ] ; xx [ 228 ] = xx [ 513 ] ; xx [ 229 ] = xx [
169 ] ; xx [ 230 ] = xx [ 240 ] ; xx [ 231 ] = xx [ 516 ] ; xx [ 232 ] = xx [
246 ] ; xx [ 233 ] = xx [ 252 ] ; xx [ 234 ] = xx [ 94 ] + xx [ 69 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 226 , xx + 61 , xx + 78 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 140 , xx + 119 , xx + 61 ) ; xx [ 4 ] = xx
[ 14 ] + xx [ 172 ] + xx [ 53 ] + xx [ 78 ] + xx [ 61 ] ; xx [ 6 ] = xx [ 15
] + xx [ 173 ] + xx [ 54 ] + xx [ 79 ] + xx [ 62 ] ; xx [ 30 ] = xx [ 90 ] +
xx [ 84 ] + xx [ 183 ] ; xx [ 34 ] = ( xx [ 4 ] * xx [ 38 ] - xx [ 6 ] * xx [
35 ] + xx [ 30 ] * xx [ 151 ] ) / xx [ 250 ] ; xx [ 84 ] = xx [ 88 ] + xx [
82 ] + xx [ 181 ] - xx [ 157 ] * xx [ 34 ] ; xx [ 85 ] = xx [ 89 ] + xx [ 83
] + xx [ 182 ] - xx [ 244 ] * xx [ 34 ] ; xx [ 86 ] = xx [ 30 ] - xx [ 191 ]
* xx [ 34 ] ; pm_math_Quaternion_xform_ra ( xx + 465 , xx + 84 , xx + 88 ) ;
xx [ 140 ] = xx [ 449 ] * xx [ 18 ] + xx [ 451 ] * xx [ 19 ] ; xx [ 141 ] =
xx [ 449 ] * xx [ 19 ] - xx [ 451 ] * xx [ 18 ] ; xx [ 142 ] = xx [ 449 ] *
xx [ 20 ] - xx [ 451 ] * xx [ 21 ] ; xx [ 143 ] = xx [ 449 ] * xx [ 21 ] + xx
[ 451 ] * xx [ 20 ] ; xx [ 144 ] = xx [ 536 ] - xx [ 106 ] + xx [ 537 ] ; xx
[ 145 ] = xx [ 563 ] - xx [ 562 ] ; xx [ 146 ] = xx [ 532 ] + xx [ 514 ] ; xx
[ 30 ] = sm_core_compiler_computeProximityInfoCxpolyCxpoly (
simple_robot_9eb3ef65_1_geometry_0 ( rtdv ) ,
simple_robot_9eb3ef65_1_geometry_1 ( rtdv ) , ( pm_math_Transform3 * ) ( xx +
140 ) , ( pm_math_Transform3 * ) ( xx + 549 ) , ( pm_math_Vector3 * ) ( xx +
82 ) , ( pm_math_Vector3 * ) ( xx + 119 ) , ( pm_math_Vector3 * ) ( xx + 147
) , ( pm_math_Vector3 * ) ( xx + 175 ) ) ; xx [ 178 ] = xx [ 42 ] ; xx [ 179
] = xx [ 43 ] ; xx [ 180 ] = xx [ 44 ] ; xx [ 181 ] = xx [ 605 ] ; xx [ 182 ]
= xx [ 534 ] ; xx [ 183 ] = xx [ 535 ] ;
sm_core_compiler_computeContactWrenches ( xx [ 30 ] , xx + 119 , xx + 82 , xx
+ 175 , xx + 147 , xx + 549 , xx + 583 , xx + 549 , xx + 140 , NULL , xx +
178 , 0 , 1 , xx [ 523 ] , xx [ 524 ] , xx [ 525 ] , xx [ 617 ] , xx [ 618 ]
, xx [ 619 ] , NULL , xx + 184 ) ; xx [ 82 ] = xx [ 605 ] ; xx [ 83 ] = xx [
534 ] ; xx [ 84 ] = xx [ 535 ] ; pm_math_Vector3_cross_ra ( xx + 42 , xx + 82
, xx + 119 ) ; xx [ 82 ] = - xx [ 605 ] ; xx [ 83 ] = - xx [ 606 ] ; xx [ 84
] = - xx [ 607 ] ; pm_math_Vector3_cross_ra ( xx + 42 , xx + 82 , xx + 140 )
; xx [ 30 ] = xx [ 119 ] + xx [ 140 ] ; xx [ 37 ] = xx [ 121 ] + xx [ 142 ] ;
xx [ 82 ] = xx [ 30 ] ; xx [ 83 ] = xx [ 120 ] + xx [ 141 ] ; xx [ 84 ] = xx
[ 37 ] ; pm_math_Matrix3x3_xform_ra ( xx + 358 , xx + 82 , xx + 142 ) ; xx [
45 ] = xx [ 89 ] - xx [ 188 ] + xx [ 143 ] ; xx [ 51 ] = xx [ 90 ] - xx [ 189
] + xx [ 144 ] ; xx [ 143 ] = xx [ 88 ] - xx [ 187 ] + xx [ 142 ] ; xx [ 144
] = xx [ 45 ] ; xx [ 145 ] = xx [ 51 ] ; xx [ 146 ] = xx [ 64 ] * xx [ 42 ] ;
xx [ 147 ] = xx [ 138 ] * xx [ 43 ] ; xx [ 148 ] = xx [ 139 ] * xx [ 44 ] ;
pm_math_Vector3_cross_ra ( xx + 42 , xx + 146 , xx + 138 ) ; xx [ 42 ] = xx [
4 ] - xx [ 167 ] * xx [ 34 ] ; xx [ 43 ] = xx [ 6 ] - xx [ 243 ] * xx [ 34 ]
; xx [ 44 ] = xx [ 16 ] + xx [ 174 ] + xx [ 55 ] + xx [ 80 ] + xx [ 63 ] - xx
[ 248 ] * xx [ 34 ] ; pm_math_Quaternion_xform_ra ( xx + 465 , xx + 42 , xx +
14 ) ; pm_math_Vector3_cross_ra ( xx + 437 , xx + 88 , xx + 42 ) ; xx [ 172 ]
= xx [ 337 ] ; xx [ 173 ] = xx [ 340 ] ; xx [ 174 ] = xx [ 342 ] ; xx [ 175 ]
= xx [ 105 ] ; xx [ 176 ] = xx [ 339 ] ; xx [ 177 ] = xx [ 443 ] ; xx [ 178 ]
= xx [ 255 ] ; xx [ 179 ] = xx [ 341 ] ; xx [ 180 ] = xx [ 445 ] ;
pm_math_Matrix3x3_xform_ra ( xx + 172 , xx + 82 , xx + 53 ) ; xx [ 4 ] = xx [
139 ] - xx [ 185 ] + xx [ 15 ] + xx [ 43 ] + xx [ 54 ] ; xx [ 6 ] = xx [ 140
] - xx [ 186 ] + xx [ 16 ] + xx [ 44 ] + xx [ 55 ] ; xx [ 185 ] = -
pm_math_Vector3_dot_ra ( xx + 26 , xx + 143 ) ; xx [ 186 ] = -
pm_math_Vector3_dot_ra ( xx + 424 , xx + 143 ) ; xx [ 187 ] = -
pm_math_Vector3_dot_ra ( xx + 431 , xx + 143 ) ; xx [ 188 ] = - ( xx [ 4 ] *
xx [ 12 ] - xx [ 6 ] * xx [ 247 ] - ( xx [ 45 ] * xx [ 300 ] + xx [ 51 ] * xx
[ 335 ] ) ) ; xx [ 189 ] = - ( xx [ 4 ] * xx [ 483 ] + xx [ 6 ] * xx [ 12 ] +
xx [ 45 ] * xx [ 335 ] - xx [ 51 ] * xx [ 493 ] ) ; xx [ 190 ] = - ( xx [ 138
] - xx [ 184 ] + xx [ 14 ] + xx [ 42 ] + xx [ 53 ] ) ; solveSymmetricPosDef (
xx + 665 , xx + 185 , 6 , 1 , xx + 142 , xx + 226 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 172 , xx + 26 , xx + 14 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 172 , xx + 424 , xx + 26 ) ;
pm_math_Matrix3x3_xform_ra ( xx + 172 , xx + 431 , xx + 42 ) ; xx [ 4 ] = xx
[ 276 ] - xx [ 47 ] - xx [ 49 ] - xx [ 289 ] ; xx [ 6 ] = xx [ 277 ] - xx [
48 ] - xx [ 52 ] - xx [ 290 ] ; xx [ 628 ] = xx [ 14 ] ; xx [ 629 ] = xx [ 26
] ; xx [ 630 ] = xx [ 42 ] ; xx [ 631 ] = xx [ 12 ] * xx [ 4 ] - xx [ 247 ] *
xx [ 6 ] - xx [ 462 ] ; xx [ 632 ] = xx [ 483 ] * xx [ 4 ] + xx [ 12 ] * xx [
6 ] + xx [ 464 ] ; xx [ 633 ] = xx [ 50 ] ; xx [ 634 ] = xx [ 15 ] ; xx [ 635
] = xx [ 27 ] ; xx [ 636 ] = xx [ 43 ] ; xx [ 637 ] = xx [ 73 ] ; xx [ 638 ]
= xx [ 77 ] ; xx [ 639 ] = xx [ 74 ] ; xx [ 640 ] = xx [ 16 ] ; xx [ 641 ] =
xx [ 28 ] ; xx [ 642 ] = xx [ 44 ] ; xx [ 643 ] = xx [ 76 ] ; xx [ 644 ] = xx
[ 71 ] ; xx [ 645 ] = xx [ 75 ] ; xx [ 646 ] = xx [ 421 ] ; xx [ 647 ] = xx [
427 ] ; xx [ 648 ] = xx [ 434 ] ; xx [ 649 ] = xx [ 338 ] ; xx [ 650 ] = xx [
511 ] ; xx [ 651 ] = xx [ 337 ] ; xx [ 652 ] = xx [ 422 ] ; xx [ 653 ] = xx [
428 ] ; xx [ 654 ] = xx [ 435 ] ; xx [ 655 ] = xx [ 343 ] ; xx [ 656 ] = xx [
336 ] ; xx [ 657 ] = xx [ 340 ] ; xx [ 658 ] = xx [ 423 ] ; xx [ 659 ] = xx [
429 ] ; xx [ 660 ] = xx [ 436 ] ; xx [ 661 ] = xx [ 447 ] ; xx [ 662 ] = xx [
253 ] ; xx [ 663 ] = xx [ 342 ] ; solveSymmetricPosDef ( xx + 665 , xx + 628
, 6 , 6 , xx + 701 , xx + 47 ) ; xx [ 14 ] = xx [ 719 ] ; xx [ 15 ] = xx [
725 ] ; xx [ 16 ] = xx [ 731 ] ; xx [ 4 ] = 9.806649999999999 ; xx [ 6 ] = xx
[ 4 ] * xx [ 21 ] ; xx [ 26 ] = xx [ 4 ] * xx [ 19 ] ; xx [ 27 ] = ( xx [ 18
] * xx [ 6 ] + xx [ 20 ] * xx [ 26 ] ) * xx [ 2 ] ; xx [ 28 ] = ( xx [ 21 ] *
xx [ 6 ] + xx [ 19 ] * xx [ 26 ] ) * xx [ 2 ] ; xx [ 19 ] = xx [ 2 ] * ( xx [
20 ] * xx [ 6 ] - xx [ 18 ] * xx [ 26 ] ) ; xx [ 42 ] = xx [ 27 ] ; xx [ 43 ]
= xx [ 4 ] - xx [ 28 ] ; xx [ 44 ] = xx [ 19 ] ; xx [ 2 ] = xx [ 142 ] -
pm_math_Vector3_dot_ra ( xx + 14 , xx + 42 ) ; xx [ 14 ] = xx [ 720 ] ; xx [
15 ] = xx [ 726 ] ; xx [ 16 ] = xx [ 732 ] ; xx [ 6 ] = xx [ 143 ] -
pm_math_Vector3_dot_ra ( xx + 14 , xx + 42 ) ; xx [ 14 ] = xx [ 721 ] ; xx [
15 ] = xx [ 727 ] ; xx [ 16 ] = xx [ 733 ] ; xx [ 18 ] = xx [ 144 ] -
pm_math_Vector3_dot_ra ( xx + 14 , xx + 42 ) ; xx [ 14 ] = xx [ 722 ] ; xx [
15 ] = xx [ 728 ] ; xx [ 16 ] = xx [ 734 ] ; xx [ 20 ] = xx [ 145 ] -
pm_math_Vector3_dot_ra ( xx + 14 , xx + 42 ) ; xx [ 14 ] = xx [ 723 ] ; xx [
15 ] = xx [ 729 ] ; xx [ 16 ] = xx [ 735 ] ; xx [ 21 ] = xx [ 146 ] -
pm_math_Vector3_dot_ra ( xx + 14 , xx + 42 ) ; xx [ 14 ] = xx [ 724 ] ; xx [
15 ] = xx [ 730 ] ; xx [ 16 ] = xx [ 736 ] ; xx [ 26 ] = xx [ 147 ] -
pm_math_Vector3_dot_ra ( xx + 14 , xx + 42 ) ; xx [ 14 ] = xx [ 450 ] ; xx [
15 ] = xx [ 515 ] ; xx [ 16 ] = xx [ 70 ] ; xx [ 42 ] = xx [ 26 ] ; xx [ 43 ]
= xx [ 12 ] * xx [ 20 ] + xx [ 483 ] * xx [ 21 ] ; xx [ 44 ] = xx [ 12 ] * xx
[ 21 ] - xx [ 247 ] * xx [ 20 ] ; pm_math_Quaternion_inverseXform_ra ( xx +
465 , xx + 42 , xx + 47 ) ; xx [ 50 ] = xx [ 251 ] ; xx [ 51 ] = xx [ 257 ] ;
xx [ 52 ] = xx [ 268 ] ; pm_math_Vector3_cross_ra ( xx + 42 , xx + 437 , xx +
53 ) ; xx [ 42 ] = xx [ 27 ] + xx [ 3 ] * xx [ 2 ] + xx [ 267 ] * xx [ 6 ] +
xx [ 386 ] * xx [ 18 ] + xx [ 30 ] + xx [ 53 ] ; xx [ 43 ] = xx [ 22 ] * xx [
2 ] - xx [ 28 ] + xx [ 13 ] * xx [ 6 ] + xx [ 23 ] * xx [ 18 ] - xx [ 300 ] *
xx [ 20 ] + xx [ 335 ] * xx [ 21 ] + xx [ 120 ] + xx [ 141 ] + xx [ 54 ] + xx
[ 4 ] ; xx [ 44 ] = xx [ 19 ] + xx [ 25 ] * xx [ 2 ] + xx [ 382 ] * xx [ 6 ]
+ xx [ 1 ] * xx [ 18 ] - xx [ 335 ] * xx [ 20 ] - xx [ 493 ] * xx [ 21 ] + xx
[ 37 ] + xx [ 55 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 465 , xx + 42
, xx + 53 ) ; xx [ 1 ] = xx [ 34 ] + pm_math_Vector3_dot_ra ( xx + 14 , xx +
47 ) + pm_math_Vector3_dot_ra ( xx + 50 , xx + 53 ) ; xx [ 12 ] = xx [ 163 ]
; xx [ 13 ] = xx [ 166 ] ; xx [ 14 ] = xx [ 168 ] ; xx [ 42 ] = xx [ 47 ] -
xx [ 1 ] * xx [ 38 ] + xx [ 24 ] ; xx [ 43 ] = xx [ 48 ] + xx [ 1 ] * xx [ 35
] + xx [ 31 ] ; xx [ 44 ] = xx [ 49 ] - xx [ 32 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 65 , xx + 42 , xx + 22 ) ; xx [ 30
] = xx [ 81 ] ; xx [ 31 ] = xx [ 150 ] ; xx [ 32 ] = xx [ 152 ] ;
pm_math_Vector3_cross_ra ( xx + 42 , xx + 154 , xx + 47 ) ; xx [ 42 ] = xx [
53 ] + xx [ 0 ] + xx [ 47 ] ; xx [ 43 ] = xx [ 54 ] + xx [ 5 ] + xx [ 48 ] ;
xx [ 44 ] = xx [ 55 ] - xx [ 1 ] * xx [ 151 ] + xx [ 125 ] + xx [ 49 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 65 , xx + 42 , xx + 3 ) ; xx [ 0 ]
= xx [ 39 ] + pm_math_Vector3_dot_ra ( xx + 12 , xx + 22 ) +
pm_math_Vector3_dot_ra ( xx + 30 , xx + 3 ) ; xx [ 12 ] = xx [ 381 ] ; xx [
13 ] = xx [ 385 ] ; xx [ 14 ] = xx [ 444 ] ; xx [ 30 ] = xx [ 22 ] - xx [ 0 ]
* xx [ 41 ] + xx [ 459 ] ; xx [ 31 ] = xx [ 23 ] - xx [ 0 ] * xx [ 59 ] + xx
[ 460 ] ; xx [ 32 ] = xx [ 24 ] + xx [ 0 ] * xx [ 60 ] + xx [ 461 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 401 , xx + 30 , xx + 22 ) ; xx [ 41
] = xx [ 481 ] ; xx [ 42 ] = xx [ 489 ] ; xx [ 43 ] = xx [ 491 ] ; xx [ 15 ]
= xx [ 3 ] + xx [ 0 ] * xx [ 170 ] + xx [ 17 ] ; pm_math_Vector3_cross_ra (
xx + 30 , xx + 625 , xx + 47 ) ; xx [ 3 ] = xx [ 4 ] - xx [ 0 ] * xx [ 262 ]
+ xx [ 36 ] ; xx [ 4 ] = xx [ 5 ] - xx [ 0 ] * xx [ 263 ] + xx [ 46 ] ; xx [
44 ] = xx [ 15 ] + xx [ 47 ] ; xx [ 45 ] = xx [ 3 ] + xx [ 48 ] ; xx [ 46 ] =
xx [ 4 ] + xx [ 49 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 401 , xx +
44 , xx + 47 ) ; xx [ 5 ] = xx [ 206 ] + pm_math_Vector3_dot_ra ( xx + 12 ,
xx + 22 ) + pm_math_Vector3_dot_ra ( xx + 41 , xx + 47 ) ; xx [ 12 ] = xx [
22 ] - xx [ 29 ] ; xx [ 13 ] = xx [ 23 ] - xx [ 5 ] * xx [ 330 ] + xx [ 236 ]
; xx [ 14 ] = xx [ 24 ] + xx [ 5 ] * xx [ 456 ] + xx [ 207 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 405 , xx + 12 , xx + 22 ) ; xx [ 12
] = xx [ 203 ] ; xx [ 13 ] = xx [ 412 ] ; xx [ 14 ] = xx [ 380 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 308 , xx + 30 , xx + 23 ) ; xx [ 27
] = xx [ 430 ] ; xx [ 28 ] = xx [ 416 ] ; xx [ 29 ] = xx [ 442 ] ;
pm_math_Vector3_cross_ra ( xx + 30 , xx + 602 , xx + 41 ) ; xx [ 44 ] = xx [
15 ] + xx [ 41 ] ; xx [ 45 ] = xx [ 3 ] + xx [ 42 ] ; xx [ 46 ] = xx [ 4 ] +
xx [ 43 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 308 , xx + 44 , xx + 41
) ; xx [ 16 ] = xx [ 160 ] + pm_math_Vector3_dot_ra ( xx + 12 , xx + 23 ) +
pm_math_Vector3_dot_ra ( xx + 27 , xx + 41 ) ; xx [ 12 ] = xx [ 23 ] - xx [
195 ] ; xx [ 13 ] = xx [ 24 ] - xx [ 16 ] * xx [ 330 ] + xx [ 197 ] ; xx [ 14
] = xx [ 25 ] + xx [ 16 ] * xx [ 333 ] + xx [ 134 ] ;
pm_math_Vector3_cross_ra ( xx + 12 , xx + 394 , xx + 23 ) ; xx [ 27 ] = xx [
41 ] - xx [ 16 ] * xx [ 390 ] + xx [ 400 ] + xx [ 23 ] ; xx [ 28 ] = xx [ 42
] + xx [ 199 ] + xx [ 24 ] ; xx [ 29 ] = xx [ 43 ] + xx [ 200 ] + xx [ 25 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 312 , xx + 27 , xx + 23 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 312 , xx + 12 , xx + 27 ) ; xx [ 12
] = xx [ 96 ] ; xx [ 13 ] = xx [ 204 ] ; xx [ 14 ] = xx [ 208 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 217 , xx + 30 , xx + 41 ) ; xx [ 44
] = xx [ 305 ] ; xx [ 45 ] = xx [ 326 ] ; xx [ 46 ] = xx [ 328 ] ;
pm_math_Vector3_cross_ra ( xx + 30 , xx + 580 , xx + 47 ) ; xx [ 50 ] = xx [
15 ] + xx [ 47 ] ; xx [ 51 ] = xx [ 3 ] + xx [ 48 ] ; xx [ 52 ] = xx [ 4 ] +
xx [ 49 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 217 , xx + 50 , xx + 47
) ; xx [ 17 ] = xx [ 122 ] + pm_math_Vector3_dot_ra ( xx + 12 , xx + 41 ) +
pm_math_Vector3_dot_ra ( xx + 44 , xx + 47 ) ; xx [ 12 ] = xx [ 41 ] + xx [
17 ] * xx [ 38 ] - xx [ 128 ] ; xx [ 13 ] = xx [ 42 ] - xx [ 17 ] * xx [ 35 ]
- xx [ 162 ] ; xx [ 14 ] = xx [ 43 ] + xx [ 171 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 221 , xx + 12 , xx + 41 ) ; xx [ 12
] = xx [ 95 ] ; xx [ 13 ] = xx [ 99 ] ; xx [ 14 ] = xx [ 201 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 113 , xx + 30 , xx + 42 ) ; xx [ 45
] = xx [ 235 ] ; xx [ 46 ] = xx [ 249 ] ; xx [ 47 ] = xx [ 260 ] ;
pm_math_Vector3_cross_ra ( xx + 30 , xx + 559 , xx + 48 ) ; xx [ 51 ] = xx [
15 ] + xx [ 48 ] ; xx [ 52 ] = xx [ 3 ] + xx [ 49 ] ; xx [ 53 ] = xx [ 4 ] +
xx [ 50 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 113 , xx + 51 , xx + 48
) ; xx [ 19 ] = xx [ 102 ] + pm_math_Vector3_dot_ra ( xx + 12 , xx + 42 ) +
pm_math_Vector3_dot_ra ( xx + 45 , xx + 48 ) ; xx [ 12 ] = xx [ 42 ] + xx [
19 ] * xx [ 38 ] - xx [ 91 ] ; xx [ 13 ] = xx [ 43 ] - xx [ 19 ] * xx [ 35 ]
- xx [ 107 ] ; xx [ 14 ] = xx [ 44 ] + xx [ 108 ] ; pm_math_Vector3_cross_ra
( xx + 12 , xx + 214 , xx + 42 ) ; xx [ 45 ] = xx [ 48 ] + xx [ 33 ] + xx [
42 ] ; xx [ 46 ] = xx [ 49 ] + xx [ 101 ] + xx [ 43 ] ; xx [ 47 ] = xx [ 50 ]
- xx [ 19 ] * xx [ 210 ] + xx [ 225 ] + xx [ 44 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 130 , xx + 45 , xx + 42 ) ;
pm_math_Quaternion_inverseXform_ra ( xx + 130 , xx + 12 , xx + 44 ) ; xx [ 12
] = xx [ 346 ] ; xx [ 13 ] = xx [ 350 ] ; xx [ 14 ] = xx [ 93 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 469 , xx + 30 , xx + 45 ) ; xx [ 48
] = xx [ 137 ] ; xx [ 49 ] = xx [ 159 ] ; xx [ 50 ] = xx [ 161 ] ;
pm_math_Vector3_cross_ra ( xx + 30 , xx + 538 , xx + 51 ) ; xx [ 28 ] = xx [
15 ] + xx [ 51 ] ; xx [ 29 ] = xx [ 3 ] + xx [ 52 ] ; xx [ 30 ] = xx [ 4 ] +
xx [ 53 ] ; pm_math_Quaternion_inverseXform_ra ( xx + 469 , xx + 28 , xx + 31
) ; xx [ 3 ] = xx [ 92 ] + pm_math_Vector3_dot_ra ( xx + 12 , xx + 45 ) +
pm_math_Vector3_dot_ra ( xx + 48 , xx + 31 ) ; xx [ 12 ] = xx [ 45 ] + xx [ 3
] * xx [ 38 ] - xx [ 11 ] ; xx [ 13 ] = xx [ 46 ] - xx [ 3 ] * xx [ 35 ] - xx
[ 40 ] ; xx [ 14 ] = xx [ 47 ] + xx [ 56 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 473 , xx + 12 , xx + 28 ) ;
pm_math_Vector3_cross_ra ( xx + 12 , xx + 109 , xx + 34 ) ; xx [ 11 ] = xx [
31 ] + xx [ 58 ] + xx [ 34 ] ; xx [ 12 ] = xx [ 32 ] + xx [ 87 ] + xx [ 35 ]
; xx [ 13 ] = xx [ 33 ] - xx [ 3 ] * xx [ 104 ] + xx [ 118 ] + xx [ 36 ] ;
pm_math_Quaternion_inverseXform_ra ( xx + 473 , xx + 11 , xx + 29 ) ; deriv [
0 ] = state [ 7 ] ; deriv [ 1 ] = state [ 8 ] ; deriv [ 2 ] = state [ 9 ] ;
deriv [ 3 ] = xx [ 7 ] ; deriv [ 4 ] = xx [ 8 ] ; deriv [ 5 ] = xx [ 9 ] ;
deriv [ 6 ] = xx [ 10 ] ; deriv [ 7 ] = xx [ 2 ] ; deriv [ 8 ] = xx [ 6 ] ;
deriv [ 9 ] = xx [ 18 ] ; deriv [ 10 ] = xx [ 20 ] ; deriv [ 11 ] = xx [ 21 ]
; deriv [ 12 ] = xx [ 26 ] ; deriv [ 13 ] = state [ 14 ] ; deriv [ 14 ] = -
xx [ 1 ] ; deriv [ 15 ] = state [ 16 ] ; deriv [ 16 ] = - xx [ 0 ] ; deriv [
17 ] = state [ 18 ] ; deriv [ 18 ] = - xx [ 5 ] ; deriv [ 19 ] = state [ 20 ]
; deriv [ 20 ] = xx [ 205 ] + xx [ 393 ] * xx [ 22 ] ; deriv [ 21 ] = state [
22 ] ; deriv [ 22 ] = - xx [ 16 ] ; deriv [ 23 ] = state [ 24 ] ; deriv [ 24
] = - ( xx [ 202 ] + xx [ 377 ] * xx [ 24 ] - xx [ 420 ] * xx [ 27 ] ) ;
deriv [ 25 ] = state [ 26 ] ; deriv [ 26 ] = - xx [ 17 ] ; deriv [ 27 ] =
state [ 28 ] ; deriv [ 28 ] = xx [ 117 ] + xx [ 304 ] * xx [ 41 ] ; deriv [
29 ] = state [ 30 ] ; deriv [ 30 ] = - xx [ 19 ] ; deriv [ 31 ] = state [ 32
] ; deriv [ 32 ] = - ( xx [ 158 ] + xx [ 198 ] * xx [ 43 ] - xx [ 239 ] * xx
[ 44 ] ) ; deriv [ 33 ] = state [ 34 ] ; deriv [ 34 ] = - xx [ 3 ] ; deriv [
35 ] = state [ 36 ] ; deriv [ 36 ] = - ( xx [ 546 ] + xx [ 136 ] * xx [ 28 ]
- xx [ 103 ] * xx [ 30 ] ) ; errorResult [ 0 ] = xx [ 463 ] ; return NULL ; }
PmfMessageId simple_robot_9eb3ef65_1_numJacPerturbLoBounds ( const
RuntimeDerivedValuesBundle * rtdv , const int * eqnEnableFlags , const double
* state , const int * modeVector , const double * input , const double *
inputDot , const double * inputDdot , const double * discreteState , double *
bounds , double * errorResult , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 2 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) eqnEnableFlags ; ( void ) state ; ( void ) modeVector ; ( void ) input ; (
void ) inputDot ; ( void ) inputDdot ; ( void ) discreteState ; ( void )
neDiagMgr ; xx [ 0 ] = 1.0e-9 ; xx [ 1 ] = 1.0e-8 ; bounds [ 0 ] = xx [ 0 ] ;
bounds [ 1 ] = xx [ 0 ] ; bounds [ 2 ] = xx [ 0 ] ; bounds [ 3 ] = xx [ 1 ] ;
bounds [ 4 ] = xx [ 1 ] ; bounds [ 5 ] = xx [ 1 ] ; bounds [ 6 ] = xx [ 1 ] ;
bounds [ 7 ] = xx [ 0 ] ; bounds [ 8 ] = xx [ 0 ] ; bounds [ 9 ] = xx [ 0 ] ;
bounds [ 10 ] = xx [ 1 ] ; bounds [ 11 ] = xx [ 1 ] ; bounds [ 12 ] = xx [ 1
] ; bounds [ 13 ] = xx [ 1 ] ; bounds [ 14 ] = xx [ 1 ] ; bounds [ 15 ] = xx
[ 1 ] ; bounds [ 16 ] = xx [ 1 ] ; bounds [ 17 ] = xx [ 1 ] ; bounds [ 18 ] =
xx [ 1 ] ; bounds [ 19 ] = xx [ 1 ] ; bounds [ 20 ] = xx [ 1 ] ; bounds [ 21
] = xx [ 1 ] ; bounds [ 22 ] = xx [ 1 ] ; bounds [ 23 ] = xx [ 1 ] ; bounds [
24 ] = xx [ 1 ] ; bounds [ 25 ] = xx [ 1 ] ; bounds [ 26 ] = xx [ 1 ] ;
bounds [ 27 ] = xx [ 1 ] ; bounds [ 28 ] = xx [ 1 ] ; bounds [ 29 ] = xx [ 1
] ; bounds [ 30 ] = xx [ 1 ] ; bounds [ 31 ] = xx [ 1 ] ; bounds [ 32 ] = xx
[ 1 ] ; bounds [ 33 ] = xx [ 1 ] ; bounds [ 34 ] = xx [ 1 ] ; bounds [ 35 ] =
xx [ 1 ] ; bounds [ 36 ] = xx [ 1 ] ; errorResult [ 0 ] = 0.0 ; return NULL ;
} PmfMessageId simple_robot_9eb3ef65_1_numJacPerturbHiBounds ( const
RuntimeDerivedValuesBundle * rtdv , const int * eqnEnableFlags , const double
* state , const int * modeVector , const double * input , const double *
inputDot , const double * inputDdot , const double * discreteState , double *
bounds , double * errorResult , NeuDiagnosticManager * neDiagMgr ) { const
double * rtdvd = rtdv -> mDoubles . mValues ; const int * rtdvi = rtdv ->
mInts . mValues ; double xx [ 3 ] ; ( void ) rtdvd ; ( void ) rtdvi ; ( void
) eqnEnableFlags ; ( void ) state ; ( void ) modeVector ; ( void ) input ; (
void ) inputDot ; ( void ) inputDdot ; ( void ) discreteState ; ( void )
neDiagMgr ; xx [ 0 ] = + pmf_get_inf ( ) ; xx [ 1 ] = 0.1 ; xx [ 2 ] = 1.0 ;
bounds [ 0 ] = xx [ 0 ] ; bounds [ 1 ] = xx [ 0 ] ; bounds [ 2 ] = xx [ 0 ] ;
bounds [ 3 ] = xx [ 1 ] ; bounds [ 4 ] = xx [ 1 ] ; bounds [ 5 ] = xx [ 1 ] ;
bounds [ 6 ] = xx [ 1 ] ; bounds [ 7 ] = xx [ 0 ] ; bounds [ 8 ] = xx [ 0 ] ;
bounds [ 9 ] = xx [ 0 ] ; bounds [ 10 ] = xx [ 0 ] ; bounds [ 11 ] = xx [ 0 ]
; bounds [ 12 ] = xx [ 0 ] ; bounds [ 13 ] = xx [ 2 ] ; bounds [ 14 ] = xx [
0 ] ; bounds [ 15 ] = xx [ 2 ] ; bounds [ 16 ] = xx [ 0 ] ; bounds [ 17 ] =
xx [ 2 ] ; bounds [ 18 ] = xx [ 0 ] ; bounds [ 19 ] = xx [ 2 ] ; bounds [ 20
] = xx [ 0 ] ; bounds [ 21 ] = xx [ 2 ] ; bounds [ 22 ] = xx [ 0 ] ; bounds [
23 ] = xx [ 2 ] ; bounds [ 24 ] = xx [ 0 ] ; bounds [ 25 ] = xx [ 2 ] ;
bounds [ 26 ] = xx [ 0 ] ; bounds [ 27 ] = xx [ 2 ] ; bounds [ 28 ] = xx [ 0
] ; bounds [ 29 ] = xx [ 2 ] ; bounds [ 30 ] = xx [ 0 ] ; bounds [ 31 ] = xx
[ 2 ] ; bounds [ 32 ] = xx [ 0 ] ; bounds [ 33 ] = xx [ 2 ] ; bounds [ 34 ] =
xx [ 0 ] ; bounds [ 35 ] = xx [ 2 ] ; bounds [ 36 ] = xx [ 0 ] ; errorResult
[ 0 ] = 0.0 ; return NULL ; }
