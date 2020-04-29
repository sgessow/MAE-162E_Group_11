#include "pm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "pm_default_allocator.h"
#include "sm_ssci_NeDaePrivateData.h"
#include "sm_CTarget.h"
PmfMessageId sm_ssci_recordRunTimeError ( const char * errorId , const char *
errorMsg , NeuDiagnosticManager * mgr ) ;
#define pm_allocator_alloc(_allocator, _m, _n) ((_allocator)->mCallocFcn((_allocator), (_m), (_n)))
#define PM_ALLOCATE_ARRAY(_name, _type, _size, _allocator)\
 _name = (_type *) pm_allocator_alloc(_allocator, sizeof(_type), _size)
#define pm_size_to_int(_size)          ((int32_T) (_size))
PmIntVector * pm_create_int_vector ( size_t , PmAllocator * ) ; int_T
pm_create_int_vector_fields ( PmIntVector * , size_t , PmAllocator * ) ;
int_T pm_create_real_vector_fields ( PmRealVector * , size_t , PmAllocator *
) ; int_T pm_create_char_vector_fields ( PmCharVector * , size_t ,
PmAllocator * ) ; int_T pm_create_bool_vector_fields ( PmBoolVector * ,
size_t , PmAllocator * ) ; void pm_rv_equals_rv ( const PmRealVector * ,
const PmRealVector * ) ; void sm_ssci_setupLoggerFcn_codeGen ( const NeDae *
dae , NeLoggerBuilder * neLoggerBuilder ) ; int32_T sm_ssci_logFcn_codeGen (
const NeDae * dae , const NeSystemInput * systemInput , PmRealVector * output
) ; extern const NeAssertData simple_robot_9eb3ef65_1_assertData [ ] ; extern
const NeZCData simple_robot_9eb3ef65_1_ZCData [ ] ; void
simple_robot_9eb3ef65_1_computeRuntimeParameters ( const double *
runtimeRootVariables , double * runtimeParameters ) ; void
simple_robot_9eb3ef65_1_validateRuntimeParameters ( const double *
runtimeParameters , int32_T * assertSatisfactionFlags ) ; void
simple_robot_9eb3ef65_1_computeAsmRuntimeDerivedValues ( const double *
runtimeParameters , RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle )
; void simple_robot_9eb3ef65_1_computeSimRuntimeDerivedValues ( const double
* runtimeParameters , RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle
) ; PmfMessageId simple_robot_9eb3ef65_1_compDerivs ( const
RuntimeDerivedValuesBundle * , const int * , const double * , const int * ,
const double * , const double * , const double * , const double * , double *
, double * , NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
simple_robot_9eb3ef65_1_numJacPerturbLoBounds ( const
RuntimeDerivedValuesBundle * , const int * , const double * , const int * ,
const double * , const double * , const double * , const double * , double *
, double * , NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
simple_robot_9eb3ef65_1_numJacPerturbHiBounds ( const
RuntimeDerivedValuesBundle * , const int * , const double * , const int * ,
const double * , const double * , const double * , const double * , double *
, double * , NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
simple_robot_9eb3ef65_1_checkDynamics ( const RuntimeDerivedValuesBundle * ,
const double * , const double * , const double * , const double * , const
double * , double * , NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
simple_robot_9eb3ef65_1_compOutputsDyn ( const RuntimeDerivedValuesBundle * ,
const int * , const double * , const int * , const double * , const double *
, const double * , const double * , double * , double * , int * , double * ,
NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
simple_robot_9eb3ef65_1_compOutputsKin ( const RuntimeDerivedValuesBundle * ,
const double * , const int * , const double * , const double * , const double
* , const double * , double * , NeuDiagnosticManager * neDiagMgr ) ;
PmfMessageId simple_robot_9eb3ef65_1_compOutputs ( const
RuntimeDerivedValuesBundle * , const double * , const int * , const double *
, const double * , const double * , const double * , double * ,
NeuDiagnosticManager * neDiagMgr ) ; PmfMessageId
simple_robot_9eb3ef65_1_computeAsmModeVector ( const double * , const double
* , const double * , int * , double * , NeuDiagnosticManager * neDiagMgr ) ;
PmfMessageId simple_robot_9eb3ef65_1_computeSimModeVector ( const double * ,
const double * , const double * , int * , double * , NeuDiagnosticManager *
neDiagMgr ) ; PmfMessageId simple_robot_9eb3ef65_1_computeZeroCrossings (
const RuntimeDerivedValuesBundle * , const double * , const double * , const
double * , const double * , const double * , double * , double * ,
NeuDiagnosticManager * neDiagMgr ) ; void simple_robot_9eb3ef65_1_setTargets
( const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , CTarget *
targets ) ; void simple_robot_9eb3ef65_1_resetAsmStateVector ( const void *
mech , double * stateVector ) ; void
simple_robot_9eb3ef65_1_resetSimStateVector ( const void * mech , double *
stateVector ) ; void simple_robot_9eb3ef65_1_initializeTrackedAngleState (
const void * mech , const RuntimeDerivedValuesBundle *
runtimeDerivedValuesBundle , const int * modeVector , const double *
motionData , double * stateVector , void * neDiagMgr ) ; void
simple_robot_9eb3ef65_1_computeDiscreteState ( const void * mech , const
RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , double *
stateVector ) ; void simple_robot_9eb3ef65_1_adjustPosition ( const void *
mech , const double * dofDeltas , double * stateVector ) ; void
simple_robot_9eb3ef65_1_perturbAsmJointPrimitiveState ( const void * mech ,
size_t stageIdx , size_t primitiveIdx , double magnitude , boolean_T
doPerturbVelocity , double * stateVector ) ; void
simple_robot_9eb3ef65_1_perturbSimJointPrimitiveState ( const void * mech ,
size_t stageIdx , size_t primitiveIdx , double magnitude , boolean_T
doPerturbVelocity , double * stateVector ) ; void
simple_robot_9eb3ef65_1_perturbFlexibleBodyState ( const void * mech , size_t
stageIdx , double magnitude , boolean_T doPerturbVelocity , double *
stateVector ) ; void simple_robot_9eb3ef65_1_computePosDofBlendMatrix ( const
void * mech , size_t stageIdx , size_t primitiveIdx , const double *
stateVector , int partialType , double * matrix ) ; void
simple_robot_9eb3ef65_1_computeVelDofBlendMatrix ( const void * mech , size_t
stageIdx , size_t primitiveIdx , const double * stateVector , int partialType
, double * matrix ) ; void
simple_robot_9eb3ef65_1_projectPartiallyTargetedPos ( const void * mech ,
size_t stageIdx , size_t primitiveIdx , const double * origStateVector , int
partialType , double * stateVector ) ; void
simple_robot_9eb3ef65_1_propagateMotion ( const void * mech , const
RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , const double *
stateVector , double * motionData ) ; size_t
simple_robot_9eb3ef65_1_computeAssemblyError ( const void * mech , const
RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , size_t
constraintIdx , const double * stateVector , const int * modeVector , const
double * motionData , double * error ) ; size_t
simple_robot_9eb3ef65_1_computeAssemblyJacobian ( const void * mech , const
RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , size_t
constraintIdx , boolean_T forVelocitySatisfaction , const double *
stateVector , const int * modeVector , const double * motionData , double * J
) ; size_t simple_robot_9eb3ef65_1_computeFullAssemblyJacobian ( const void *
mech , const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , const
double * stateVector , const int * modeVector , const double * motionData ,
double * J ) ; int simple_robot_9eb3ef65_1_isInKinematicSingularity ( const
void * mech , const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle ,
size_t constraintIdx , const int * modeVector , const double * motionData ) ;
PmfMessageId simple_robot_9eb3ef65_1_convertStateVector ( const void *
asmMech , const RuntimeDerivedValuesBundle * asmRuntimeDerivedValuesBundle ,
const void * simMech , const double * asmStateVector , const int *
asmModeVector , const int * simModeVector , double * simStateVector , void *
neDiagMgr ) ; void simple_robot_9eb3ef65_1_constructStateVector ( const void
* mech , const double * solverStateVector , const double * u , const double *
uDot , const double * discreteStateVector , double * fullStateVector ) ; void
simple_robot_9eb3ef65_1_extractSolverStateVector ( const void * mech , const
double * fullStateVector , double * solverStateVector ) ; int
simple_robot_9eb3ef65_1_isPositionViolation ( const void * mech , const
RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , const int *
constraintEqnEnableFlags , const double * stateVector , const int *
modeVector ) ; int simple_robot_9eb3ef65_1_isVelocityViolation ( const void *
mech , const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , const
int * constraintEqnEnableFlags , const double * stateVector , const int *
modeVector ) ; PmfMessageId simple_robot_9eb3ef65_1_projectStateSim ( const
void * mech , const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle ,
const int * constraintEqnEnableFlags , const int * modeVector , const double
* inputVector , double * stateVector , void * neDiagMgr ) ; void
simple_robot_9eb3ef65_1_computeConstraintError ( const void * mech , const
RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle , const double *
stateVector , const int * modeVector , double * error ) ; void
simple_robot_9eb3ef65_1_resetModeVector ( const void * mech , int *
modeVector ) ; boolean_T simple_robot_9eb3ef65_1_hasJointDisToNormModeChange
( const void * mech , const int * prevModeVector , const int * modeVector ) ;
PmfMessageId simple_robot_9eb3ef65_1_performJointDisToNormModeChange ( const
void * mech , const RuntimeDerivedValuesBundle * runtimeDerivedValuesBundle ,
const int * prevModeVector , const int * modeVector , const double *
inputVector , double * stateVector , void * neDiagMgr ) ; PmfMessageId
simple_robot_9eb3ef65_1_onModeChangedCutJoints ( const void * mech , const
int * prevModeVector , int * modeVector , double * stateVector , void *
neDiagMgr ) ; PmfMessageId simple_robot_9eb3ef65_1_assemble ( const double *
u , double * udot , double * x , NeuDiagnosticManager * neDiagMgr ) { ( void
) x ; ( void ) u ; ( void ) udot ; ( void ) neDiagMgr ; return NULL ; }
static void dae_cg_setParameters_function ( const NeDae * dae , const
NeParameterBundle * paramBundle ) { const NeDaePrivateData * smData = dae ->
mPrivateData ; const double * runtimeRootVariables = paramBundle ->
mRealParameters . mX ; if ( smData -> mRuntimeParameterScalars . mN == 0 )
return ; simple_robot_9eb3ef65_1_computeRuntimeParameters (
runtimeRootVariables , smData -> mRuntimeParameterScalars . mX ) ;
simple_robot_9eb3ef65_1_computeAsmRuntimeDerivedValues ( smData ->
mRuntimeParameterScalars . mX , & dae -> mPrivateData ->
mAsmRuntimeDerivedValuesBundle ) ;
simple_robot_9eb3ef65_1_computeSimRuntimeDerivedValues ( smData ->
mRuntimeParameterScalars . mX , & dae -> mPrivateData ->
mSimRuntimeDerivedValuesBundle ) ;
sm_core_computeRedundantConstraintEquations ( & dae -> mPrivateData ->
mSimulationDelegate , & smData -> mSimRuntimeDerivedValuesBundle ) ;
#if 0
{ size_t i ; const size_t n = smData -> mSimulationDelegate .
mRunTimeEnabledEquations . mSize ; pmf_printf (
"\nRuntime Enabled Equations (%lu)\n" , n ) ; for ( i = 0 ; i < n ; ++ i )
pmf_printf ( "  %2lu:  %d\n" , i , smData -> mSimulationDelegate .
mRunTimeEnabledEquations . mValues [ i ] ) ; }
#endif
} static PmfMessageId dae_cg_pAssert_method ( const NeDae * dae , const
NeSystemInput * systemInput , NeDaeMethodOutput * daeMethodOutput ,
NeuDiagnosticManager * neDiagMgr ) { const NeDaePrivateData * smData = dae ->
mPrivateData ; const double * runtimeParams = smData ->
mRuntimeParameterScalars . mX ; int32_T * assertSatisfactionFlags =
daeMethodOutput -> mPASSERT . mX ; ( void ) systemInput ; ( void ) neDiagMgr
; simple_robot_9eb3ef65_1_validateRuntimeParameters ( runtimeParams ,
assertSatisfactionFlags ) ; return NULL ; } static PmfMessageId
dae_cg_deriv_method ( const NeDae * dae , const NeSystemInput * systemInput ,
NeDaeMethodOutput * daeMethodOutput , NeuDiagnosticManager * neDiagMgr ) {
const NeDaePrivateData * smData = dae -> mPrivateData ; PmfMessageId errorId
= NULL ; double errorResult = 0.0 ; if ( smData ->
mCachedDerivativesAvailable ) memcpy ( daeMethodOutput -> mXP0 . mX , smData
-> mCachedDerivatives . mX , 37 * sizeof ( real_T ) ) ; else errorId =
simple_robot_9eb3ef65_1_compDerivs ( & smData ->
mSimRuntimeDerivedValuesBundle , smData -> mSimulationDelegate .
mRunTimeEnabledEquations . mValues , systemInput -> mX . mX , systemInput ->
mM . mX , systemInput -> mU . mX , systemInput -> mU . mX + 0 , systemInput
-> mV . mX + 0 , systemInput -> mD . mX , daeMethodOutput -> mXP0 . mX , &
errorResult , neDiagMgr ) ; return errorId ; } static PmfMessageId
dae_cg_numJacPerturbLoBounds_method ( const NeDae * dae , const NeSystemInput
* systemInput , NeDaeMethodOutput * daeMethodOutput , NeuDiagnosticManager *
neDiagMgr ) { const NeDaePrivateData * smData = dae -> mPrivateData ;
PmfMessageId errorId = NULL ; double errorResult = 0.0 ; errorId =
simple_robot_9eb3ef65_1_numJacPerturbLoBounds ( & smData ->
mSimRuntimeDerivedValuesBundle , smData -> mSimulationDelegate .
mRunTimeEnabledEquations . mValues , systemInput -> mX . mX , systemInput ->
mM . mX , systemInput -> mU . mX , systemInput -> mU . mX + 0 , systemInput
-> mV . mX + 0 , systemInput -> mD . mX , daeMethodOutput -> mNUMJAC_DX_LO .
mX , & errorResult , neDiagMgr ) ; return errorId ; } static PmfMessageId
dae_cg_numJacPerturbHiBounds_method ( const NeDae * dae , const NeSystemInput
* systemInput , NeDaeMethodOutput * daeMethodOutput , NeuDiagnosticManager *
neDiagMgr ) { const NeDaePrivateData * smData = dae -> mPrivateData ;
PmfMessageId errorId = NULL ; double errorResult = 0.0 ; errorId =
simple_robot_9eb3ef65_1_numJacPerturbHiBounds ( & smData ->
mSimRuntimeDerivedValuesBundle , smData -> mSimulationDelegate .
mRunTimeEnabledEquations . mValues , systemInput -> mX . mX , systemInput ->
mM . mX , systemInput -> mU . mX , systemInput -> mU . mX + 0 , systemInput
-> mV . mX + 0 , systemInput -> mD . mX , daeMethodOutput -> mNUMJAC_DX_HI .
mX , & errorResult , neDiagMgr ) ; return errorId ; } static PmfMessageId
dae_cg_compOutputs_method ( const NeDae * dae , const NeSystemInput *
systemInput , NeDaeMethodOutput * daeMethodOutput , NeuDiagnosticManager *
neDiagMgr ) { PmfMessageId errorId = NULL ; ( void ) dae ; ( void )
systemInput ; ( void ) daeMethodOutput ; ( void ) neDiagMgr ; return errorId
; } static PmfMessageId dae_cg_mode_method ( const NeDae * dae , const
NeSystemInput * systemInput , NeDaeMethodOutput * daeMethodOutput ,
NeuDiagnosticManager * neDiagMgr ) { const NeDaePrivateData * smData = dae ->
mPrivateData ; PmfMessageId errorId = NULL ; double errorResult = 0.0 ;
errorId = simple_robot_9eb3ef65_1_computeSimModeVector ( systemInput -> mU .
mX , systemInput -> mU . mX + 0 , systemInput -> mV . mX + 0 ,
daeMethodOutput -> mMODE . mX , & errorResult , neDiagMgr ) ; memcpy ( smData
-> mCachedModeVector . mX , daeMethodOutput -> mMODE . mX , 0 * sizeof (
int32_T ) ) ; return errorId ; } static PmfMessageId
dae_cg_zeroCrossing_method ( const NeDae * dae , const NeSystemInput *
systemInput , NeDaeMethodOutput * daeMethodOutput , NeuDiagnosticManager *
neDiagMgr ) { const NeDaePrivateData * smData = dae -> mPrivateData ; double
errorResult = 0.0 ; return simple_robot_9eb3ef65_1_computeZeroCrossings ( &
smData -> mSimRuntimeDerivedValuesBundle , systemInput -> mX . mX ,
systemInput -> mU . mX , systemInput -> mU . mX + 0 , systemInput -> mV . mX
+ 0 , systemInput -> mD . mX , daeMethodOutput -> mZC . mX , & errorResult ,
neDiagMgr ) ; } static PmfMessageId dae_cg_project_solve ( const NeDae * dae
, const NeSystemInput * systemInput , NeuDiagnosticManager * neDiagMgr ) {
NeDaePrivateData * smData = dae -> mPrivateData ; return sm_core_projectState
( false , & smData -> mSimulationDelegate , & smData ->
mSimRuntimeDerivedValuesBundle , systemInput -> mM . mX , systemInput -> mU .
mX , systemInput -> mU . mX + 0 , systemInput -> mD . mX , systemInput -> mX
. mX , neDiagMgr ) ; } static PmfMessageId dae_cg_check_solve ( const NeDae *
dae , const NeSystemInput * systemInput , NeuDiagnosticManager * neDiagMgr )
{ NeDaePrivateData * smData = dae -> mPrivateData ; PmfMessageId errorId =
NULL ; if ( smData -> mNumConstraintEqns > 0 ) errorId = sm_core_projectState
( false , & smData -> mSimulationDelegate , & smData ->
mSimRuntimeDerivedValuesBundle , systemInput -> mM . mX , systemInput -> mU .
mX , systemInput -> mU . mX + 0 , systemInput -> mD . mX , systemInput -> mX
. mX , neDiagMgr ) ; if ( errorId == NULL && smData -> mDoCheckDynamics ) {
double result = 0.0 ; errorId = simple_robot_9eb3ef65_1_checkDynamics ( &
smData -> mSimRuntimeDerivedValuesBundle , systemInput -> mX . mX ,
systemInput -> mU . mX , systemInput -> mU . mX + 0 , systemInput -> mV . mX
+ 0 , systemInput -> mD . mX , & result , neDiagMgr ) ; } return errorId ; }
static PmfMessageId dae_cg_CIC_MODE_solve ( const NeDae * dae , const
NeSystemInput * systemInput , NeuDiagnosticManager * neDiagMgr ) {
NeDaePrivateData * smData = dae -> mPrivateData ; PmfMessageId errorId = NULL
; double errorResult = 0.0 ; const size_t mvSize = smData -> mModeVectorSize
; boolean_T modeChanged = false ; if ( mvSize > 0 ) { errorId =
simple_robot_9eb3ef65_1_computeSimModeVector ( systemInput -> mU . mX ,
systemInput -> mU . mX + 0 , systemInput -> mV . mX + 0 , systemInput -> mM .
mX , & errorResult , neDiagMgr ) ; if ( errorId != NULL ) return errorId ; {
size_t i ; for ( i = 0 ; i < mvSize ; ++ i ) if ( systemInput -> mM . mX [ i
] != smData -> mCachedModeVector . mX [ i ] ) { modeChanged = true ; break ;
} } } if ( modeChanged ) { errorId = sm_core_onModeChanged ( & smData ->
mSimulationDelegate , & smData -> mSimRuntimeDerivedValuesBundle ,
systemInput -> mU . mX , systemInput -> mU . mX + 0 , systemInput -> mD . mX
, smData -> mCachedModeVector . mX , systemInput -> mM . mX , systemInput ->
mX . mX , neDiagMgr ) ; if ( errorId != NULL ) return errorId ; memcpy (
smData -> mCachedModeVector . mX , systemInput -> mM . mX , 0 * sizeof (
int32_T ) ) ; } errorId = sm_core_projectState ( true , & smData ->
mSimulationDelegate , & smData -> mSimRuntimeDerivedValuesBundle ,
systemInput -> mM . mX , systemInput -> mU . mX , systemInput -> mU . mX + 0
, systemInput -> mD . mX , systemInput -> mX . mX , neDiagMgr ) ; return
errorId ; } static PmfMessageId dae_cg_assemble_solve ( const NeDae * dae ,
const NeSystemInput * systemInput , NeuDiagnosticManager * neDiagMgr ) {
NeDaePrivateData * smData = dae -> mPrivateData ; const SmMechanismDelegate *
delegate = & smData -> mAssemblyDelegate ; const RuntimeDerivedValuesBundle *
runtimeDerivedValuesBundle = & smData -> mAsmRuntimeDerivedValuesBundle ;
PmfMessageId errorId = NULL ; size_t i ; double errorResult = 0.0 ; const
size_t numTargets = 56 ; unsigned int asmStatus = 0 ; double *
assemblyFullStateVector = smData -> mAssemblyFullStateVector . mX ; double *
simulationFullStateVector = smData -> mSimulationFullStateVector . mX ; ( *
delegate -> mSetTargets ) ( runtimeDerivedValuesBundle , smData -> mTargets )
; { const double * u = systemInput -> mU . mX ; const double * uDot = u +
smData -> mInputVectorSize ; CTarget * target = smData -> mTargets + smData
-> mNumInternalTargets ; for ( i = 0 ; i < smData ->
mNumInputMotionPrimitives ; ++ i ) { const size_t inputOffset = smData ->
mMotionInputOffsets . mX [ i ] ; ( target ++ ) -> mValue [ 0 ] = u [
inputOffset ] ; ( target ++ ) -> mValue [ 0 ] = uDot [ inputOffset ] ; } } if
( smData -> mAssemblyModeVector . mN > 0 ) { errorId =
simple_robot_9eb3ef65_1_computeAsmModeVector ( systemInput -> mU . mX ,
systemInput -> mU . mX + 0 , systemInput -> mV . mX + 0 , smData ->
mAssemblyModeVector . mX , & errorResult , neDiagMgr ) ; if ( errorId != NULL
) return errorId ; } errorId = sm_core_computeStateVector ( delegate ,
runtimeDerivedValuesBundle , smData -> mAssemblyModeVector . mX , numTargets
, smData -> mTargets , assemblyFullStateVector , neDiagMgr ) ; if ( errorId
!= NULL ) return errorId ; asmStatus = sm_core_checkAssembly ( delegate ,
runtimeDerivedValuesBundle , assemblyFullStateVector , smData ->
mAssemblyModeVector . mX , NULL , NULL , NULL ) ; if ( asmStatus != 1 ) {
return sm_ssci_recordRunTimeError (
"sm:compiler:messages:simulationErrors:AssemblyFailure" , asmStatus == 2 ?
 "Model not assembled due to a position violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
: ( asmStatus == 3 ?
 "Model not assembled due to a velocity violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
:
 "Model not assembled due to a singularity violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
) , neDiagMgr ) ; }
#if 0
simple_robot_9eb3ef65_1_checkTargets ( & smData ->
mSimRuntimeDerivedValuesBundle , assemblyFullStateVector ) ;
#endif
if ( smData -> mModeVectorSize > 0 ) { errorId =
simple_robot_9eb3ef65_1_computeSimModeVector ( systemInput -> mU . mX ,
systemInput -> mU . mX + 0 , systemInput -> mV . mX + 0 , systemInput -> mM .
mX , & errorResult , neDiagMgr ) ; if ( errorId != NULL ) return errorId ;
memcpy ( smData -> mCachedModeVector . mX , systemInput -> mM . mX , 0 *
sizeof ( int32_T ) ) ; } errorId = ( * delegate -> mConvertStateVector ) (
NULL , runtimeDerivedValuesBundle , NULL , assemblyFullStateVector , smData
-> mAssemblyModeVector . mX , systemInput -> mM . mX ,
simulationFullStateVector , neDiagMgr ) ; for ( i = 0 ; i < smData ->
mStateVectorSize ; ++ i ) systemInput -> mX . mX [ i ] =
simulationFullStateVector [ smData -> mStateVectorMap . mX [ i ] ] ; memcpy (
systemInput -> mD . mX , simulationFullStateVector + smData ->
mFullStateVectorSize - smData -> mDiscreteStateSize , smData ->
mDiscreteStateSize * sizeof ( double ) ) ; return errorId ; } typedef struct
{ size_t first ; size_t second ; } SizePair ; static void checkMemAllocStatus
( int_T status ) { ( void ) status ; } static PmCharVector
cStringToCharVector ( const char * src ) { const size_t n = strlen ( src ) ;
PmCharVector charVect ; const int_T status = pm_create_char_vector_fields ( &
charVect , n + 1 , pm_default_allocator ( ) ) ; checkMemAllocStatus ( status
) ; strcpy ( charVect . mX , src ) ; return charVect ; } static void
initBasicAttributes ( NeDaePrivateData * smData ) { size_t i ; smData ->
mStateVectorSize = 37 ; smData -> mFullStateVectorSize = 37 ; smData ->
mDiscreteStateSize = 0 ; smData -> mModeVectorSize = 0 ; smData ->
mNumZeroCrossings = 0 ; smData -> mInputVectorSize = 0 ; smData ->
mOutputVectorSize = 0 ; smData -> mNumConstraintEqns = 0 ; smData ->
mDoCheckDynamics = true ; for ( i = 0 ; i < 4 ; ++ i ) smData -> mChecksum [
i ] = 0 ; } static void initStateVector ( NeDaePrivateData * smData ) {
PmAllocator * alloc = pm_default_allocator ( ) ; const int32_T stateVectorMap
[ 37 ] = { 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 ,
15 , 16 , 17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 , 25 , 26 , 27 , 28 , 29 , 30
, 31 , 32 , 33 , 34 , 35 , 36 } ; const CTarget targets [ 56 ] = { { 0 , 80 ,
0 , false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true
, 1 , { + 1.534383690500375597e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 80 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 81 , 0
, false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true ,
1 , { + 1.614145581743506197e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 81 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 82 , 0
, false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true ,
1 , { - 2.585950027028089515e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 82 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 83 , 0
, false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true ,
1 , { - 2.506188135784959137e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 83 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 84 , 0
, false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true ,
1 , { - 1.753023235890028841e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 84 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 85 , 0
, false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true ,
1 , { - 1.570796326794896558e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 85 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 86 , 0
, false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true ,
1 , { + 1.563270049618219915e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 86 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 87 , 0
, false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true ,
1 , { + 1.572764997910456763e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 87 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 88 , 0
, false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true ,
1 , { + 4.006210876031968127e-01 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 88 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 89 , 0
, false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true ,
1 , { + 1.886711956648620570e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 89 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 90 , 0
, false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true ,
1 , { - 1.361307450056739921e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 90 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 91 , 0
, false , 0 , 1 , "deg" , false , true , + 1.000000000000000000e+00 , true ,
1 , { - 3.930948104265200027e-01 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 91 , 0 , false , 0 , 0 , "1" , true ,
true , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00 ,
+ 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 211 , 0
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 211 , 0 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 211 , 1
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 211 , 1 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 211 , 2
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 211 , 2 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 1 , 211 , 3
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 4
, { + 1.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 2 , 211 , 3 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 3 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 212 , 0
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 212 , 0 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 212 , 1
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 212 , 1 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 212 , 2
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 212 , 2 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 1 , 212 , 3
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 4
, { + 1.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 2 , 212 , 3 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 3 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 213 , 0
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 213 , 0 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 213 , 1
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 213 , 1 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 213 , 2
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 213 , 2 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 1 , 213 , 3
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 4
, { + 1.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 2 , 213 , 3 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 3 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 214 , 0
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 214 , 0 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 214 , 1
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 214 , 1 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 0 , 214 , 2
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 1
, { + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 0 , 214 , 2 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 1 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } , { 1 , 214 , 3
, false , 0 , 0 , "1" , false , false , + 1.000000000000000000e+00 , true , 4
, { + 1.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 , + 0.000000000000000000e+00 } , { +
0.000000000000000000e+00 } } , { 2 , 214 , 3 , false , 0 , 0 , "1" , true ,
false , + 1.000000000000000000e+00 , true , 3 , { + 0.000000000000000000e+00
, + 0.000000000000000000e+00 , + 0.000000000000000000e+00 , +
0.000000000000000000e+00 } , { + 0.000000000000000000e+00 } } } ; const
size_t numTargets = 56 ; int_T status ; size_t i ; status =
pm_create_real_vector_fields ( & smData -> mAssemblyFullStateVector , 37 ,
alloc ) ; checkMemAllocStatus ( status ) ; status =
pm_create_real_vector_fields ( & smData -> mSimulationFullStateVector , 37 ,
alloc ) ; checkMemAllocStatus ( status ) ; status =
pm_create_int_vector_fields ( & smData -> mStateVectorMap , smData ->
mStateVectorSize , alloc ) ; checkMemAllocStatus ( status ) ; memcpy ( smData
-> mStateVectorMap . mX , stateVectorMap , smData -> mStateVectorSize *
sizeof ( int32_T ) ) ; smData -> mNumInternalTargets = 56 ; smData ->
mNumInputMotionPrimitives = 0 ; PM_ALLOCATE_ARRAY ( smData -> mTargets ,
CTarget , numTargets , alloc ) ; for ( i = 0 ; i < numTargets ; ++ i )
sm_compiler_CTarget_copy ( targets + i , smData -> mTargets + i ) ; } static
void initAsserts ( NeDaePrivateData * smData ) { PmAllocator * alloc =
pm_default_allocator ( ) ; int_T status = 0 ; smData -> mNumParamAsserts = 0
; smData -> mParamAssertObjects = NULL ; smData -> mParamAssertPaths = NULL ;
smData -> mParamAssertDescriptors = NULL ; smData -> mParamAssertMessages =
NULL ; smData -> mParamAssertMessageIds = NULL ; status =
pm_create_bool_vector_fields ( & smData -> mParamAssertIsWarnings , smData ->
mNumParamAsserts , alloc ) ; checkMemAllocStatus ( status ) ; if ( smData ->
mNumParamAsserts > 0 ) { const NeAssertData * ad =
simple_robot_9eb3ef65_1_assertData ; size_t i ; PM_ALLOCATE_ARRAY ( smData ->
mParamAssertObjects , PmCharVector , 0 , alloc ) ; PM_ALLOCATE_ARRAY ( smData
-> mParamAssertPaths , PmCharVector , 0 , alloc ) ; PM_ALLOCATE_ARRAY (
smData -> mParamAssertDescriptors , PmCharVector , 0 , alloc ) ;
PM_ALLOCATE_ARRAY ( smData -> mParamAssertMessages , PmCharVector , 0 , alloc
) ; PM_ALLOCATE_ARRAY ( smData -> mParamAssertMessageIds , PmCharVector , 0 ,
alloc ) ; for ( i = 0 ; i < smData -> mNumParamAsserts ; ++ i , ++ ad ) {
smData -> mParamAssertObjects [ i ] = cStringToCharVector ( ad -> mObject ) ;
smData -> mParamAssertPaths [ i ] = cStringToCharVector ( ad -> mPath ) ;
smData -> mParamAssertDescriptors [ i ] = cStringToCharVector ( ad ->
mDescriptor ) ; smData -> mParamAssertMessages [ i ] = cStringToCharVector (
ad -> mMessage ) ; smData -> mParamAssertMessageIds [ i ] =
cStringToCharVector ( ad -> mMessageID ) ; smData -> mParamAssertIsWarnings .
mX [ i ] = ad -> mIsWarn ; } } } static void initModeVector (
NeDaePrivateData * smData ) { { size_t i ; const int_T status =
pm_create_int_vector_fields ( & smData -> mAssemblyModeVector , 0 ,
pm_default_allocator ( ) ) ; checkMemAllocStatus ( status ) ; for ( i = 0 ; i
< smData -> mAssemblyModeVector . mN ; ++ i ) smData -> mAssemblyModeVector .
mX [ i ] = 0 ; } { size_t i ; const int_T status =
pm_create_int_vector_fields ( & smData -> mCachedModeVector , 0 ,
pm_default_allocator ( ) ) ; checkMemAllocStatus ( status ) ; for ( i = 0 ; i
< smData -> mModeVectorSize ; ++ i ) smData -> mCachedModeVector . mX [ i ] =
0 ; } } static void initZeroCrossings ( NeDaePrivateData * smData ) {
PmAllocator * alloc = pm_default_allocator ( ) ; int_T status = 0 ; smData ->
mZeroCrossingObjects = NULL ; smData -> mZeroCrossingPaths = NULL ; smData ->
mZeroCrossingDescriptors = NULL ; status = pm_create_int_vector_fields ( &
smData -> mZeroCrossingTypes , 0 , alloc ) ; checkMemAllocStatus ( status ) ;
if ( smData -> mNumZeroCrossings > 0 ) { const NeZCData * zcd =
simple_robot_9eb3ef65_1_ZCData ; size_t i ; PM_ALLOCATE_ARRAY ( smData ->
mZeroCrossingObjects , PmCharVector , 0 , alloc ) ; PM_ALLOCATE_ARRAY (
smData -> mZeroCrossingPaths , PmCharVector , 0 , alloc ) ; PM_ALLOCATE_ARRAY
( smData -> mZeroCrossingDescriptors , PmCharVector , 0 , alloc ) ; for ( i =
0 ; i < smData -> mNumZeroCrossings ; ++ i , ++ zcd ) { smData ->
mZeroCrossingObjects [ i ] = cStringToCharVector ( zcd -> mObject ) ; smData
-> mZeroCrossingPaths [ i ] = cStringToCharVector ( zcd -> mPath ) ; smData
-> mZeroCrossingDescriptors [ i ] = cStringToCharVector ( zcd -> mDescriptor
) ; smData -> mZeroCrossingTypes . mX [ i ] = zcd -> mType ; } } } static
void initVariables ( NeDaePrivateData * smData ) { const char * varFullPaths
[ 37 ] = { "x6_DOF_Joint.Px.p" , "x6_DOF_Joint.Py.p" , "x6_DOF_Joint.Pz.p" ,
"x6_DOF_Joint.S.Q" , "x6_DOF_Joint.S.Q" , "x6_DOF_Joint.S.Q" ,
"x6_DOF_Joint.S.Q" , "x6_DOF_Joint.Px.v" , "x6_DOF_Joint.Py.v" ,
"x6_DOF_Joint.Pz.v" , "x6_DOF_Joint.S.w" , "x6_DOF_Joint.S.w" ,
"x6_DOF_Joint.S.w" , "Revolute7.Rz.q" , "Revolute7.Rz.w" , "Revolute5.Rz.q" ,
"Revolute5.Rz.w" , "Revolute1.Rz.q" , "Revolute1.Rz.w" , "Revolute10.Rz.q" ,
"Revolute10.Rz.w" , "Revolute.Rz.q" , "Revolute.Rz.w" , "Revolute11.Rz.q" ,
"Revolute11.Rz.w" , "Revolute2.Rz.q" , "Revolute2.Rz.w" , "Revolute8.Rz.q" ,
"Revolute8.Rz.w" , "Revolute3.Rz.q" , "Revolute3.Rz.w" , "Revolute9.Rz.q" ,
"Revolute9.Rz.w" , "Revolute4.Rz.q" , "Revolute4.Rz.w" , "Revolute6.Rz.q" ,
"Revolute6.Rz.w" } ; const char * varObjects [ 37 ] = {
"simple_robot/6-DOF Joint" , "simple_robot/6-DOF Joint" ,
"simple_robot/6-DOF Joint" , "simple_robot/6-DOF Joint" ,
"simple_robot/6-DOF Joint" , "simple_robot/6-DOF Joint" ,
"simple_robot/6-DOF Joint" , "simple_robot/6-DOF Joint" ,
"simple_robot/6-DOF Joint" , "simple_robot/6-DOF Joint" ,
"simple_robot/6-DOF Joint" , "simple_robot/6-DOF Joint" ,
"simple_robot/6-DOF Joint" , "simple_robot/Revolute7" ,
"simple_robot/Revolute7" , "simple_robot/Revolute5" ,
"simple_robot/Revolute5" , "simple_robot/Revolute1" ,
"simple_robot/Revolute1" , "simple_robot/Revolute10" ,
"simple_robot/Revolute10" , "simple_robot/Revolute" , "simple_robot/Revolute"
, "simple_robot/Revolute11" , "simple_robot/Revolute11" ,
"simple_robot/Revolute2" , "simple_robot/Revolute2" ,
"simple_robot/Revolute8" , "simple_robot/Revolute8" ,
"simple_robot/Revolute3" , "simple_robot/Revolute3" ,
"simple_robot/Revolute9" , "simple_robot/Revolute9" ,
"simple_robot/Revolute4" , "simple_robot/Revolute4" ,
"simple_robot/Revolute6" , "simple_robot/Revolute6" } ; smData ->
mNumVarScalars = 37 ; smData -> mVarFullPaths = NULL ; smData -> mVarObjects
= NULL ; if ( smData -> mNumVarScalars > 0 ) { size_t s ; PmAllocator * alloc
= pm_default_allocator ( ) ; PM_ALLOCATE_ARRAY ( smData -> mVarFullPaths ,
PmCharVector , 37 , alloc ) ; PM_ALLOCATE_ARRAY ( smData -> mVarObjects ,
PmCharVector , 37 , alloc ) ; for ( s = 0 ; s < smData -> mNumVarScalars ; ++
s ) { smData -> mVarFullPaths [ s ] = cStringToCharVector ( varFullPaths [ s
] ) ; smData -> mVarObjects [ s ] = cStringToCharVector ( varObjects [ s ] )
; } } } static void initRuntimeParameters ( NeDaePrivateData * smData ) {
PmAllocator * alloc = pm_default_allocator ( ) ; int_T status = 0 ; size_t i
= 0 ; const int32_T * rtpRootVarRows = NULL ; const int32_T * rtpRootVarCols
= NULL ; const char * * rtpFullPaths = NULL ; smData -> mNumRtpRootVars = 0 ;
status = pm_create_int_vector_fields ( & smData -> mRtpRootVarRows , smData
-> mNumRtpRootVars , alloc ) ; checkMemAllocStatus ( status ) ; memcpy (
smData -> mRtpRootVarRows . mX , rtpRootVarRows , smData -> mNumRtpRootVars *
sizeof ( int32_T ) ) ; status = pm_create_int_vector_fields ( & smData ->
mRtpRootVarCols , smData -> mNumRtpRootVars , alloc ) ; checkMemAllocStatus (
status ) ; memcpy ( smData -> mRtpRootVarCols . mX , rtpRootVarCols , smData
-> mNumRtpRootVars * sizeof ( int32_T ) ) ; smData -> mRtpFullPaths = NULL ;
if ( smData -> mNumRtpRootVars > 0 ) { size_t v ; PM_ALLOCATE_ARRAY ( smData
-> mRtpFullPaths , PmCharVector , 0 , alloc ) ; for ( v = 0 ; v < smData ->
mNumRtpRootVars ; ++ v ) { smData -> mRtpFullPaths [ v ] =
cStringToCharVector ( rtpFullPaths [ v ] ) ; } } smData ->
mNumRuntimeRootVarScalars = 0 ; status = pm_create_real_vector_fields ( &
smData -> mRuntimeParameterScalars , 0 , alloc ) ; checkMemAllocStatus (
status ) ; for ( i = 0 ; i < smData -> mRuntimeParameterScalars . mN ; ++ i )
smData -> mRuntimeParameterScalars . mX [ i ] = 0.0 ;
sm_core_RuntimeDerivedValuesBundle_create ( & smData ->
mAsmRuntimeDerivedValuesBundle , 0 , 0 ) ;
sm_core_RuntimeDerivedValuesBundle_create ( & smData ->
mSimRuntimeDerivedValuesBundle , 0 , 0 ) ; } static void initIoInfoHelper (
size_t n , const char * portPathsSource [ ] , const char * unitsSource [ ] ,
const SizePair dimensions [ ] , boolean_T doInputs , NeDaePrivateData *
smData ) { PmCharVector * portPaths = NULL ; PmCharVector * units = NULL ;
NeDsIoInfo * infos = NULL ; if ( n > 0 ) { size_t s ; PmAllocator * alloc =
pm_default_allocator ( ) ; PM_ALLOCATE_ARRAY ( portPaths , PmCharVector , n ,
alloc ) ; PM_ALLOCATE_ARRAY ( units , PmCharVector , n , alloc ) ;
PM_ALLOCATE_ARRAY ( infos , NeDsIoInfo , n , alloc ) ; for ( s = 0 ; s < n ;
++ s ) { portPaths [ s ] = cStringToCharVector ( portPathsSource [ s ] ) ;
units [ s ] = cStringToCharVector ( unitsSource [ s ] ) ; { NeDsIoInfo * info
= infos + s ; info -> mName = info -> mIdentifier = portPaths [ s ] . mX ;
info -> mM = dimensions [ s ] . first ; info -> mN = dimensions [ s ] .
second ; info -> mUnit = units [ s ] . mX ; } } } if ( doInputs ) { smData ->
mNumInputs = n ; smData -> mInputPortPaths = portPaths ; smData ->
mInputUnits = units ; smData -> mInputInfos = infos ; } else { smData ->
mNumOutputs = n ; smData -> mOutputPortPaths = portPaths ; smData ->
mOutputUnits = units ; smData -> mOutputInfos = infos ; } } static void
initIoInfo ( NeDaePrivateData * smData ) { const char * * inputPortPaths =
NULL ; const char * * inputUnits = NULL ; const SizePair * inputDimensions =
NULL ; const char * * outputPortPaths = NULL ; const char * * outputUnits =
NULL ; const SizePair * outputDimensions = NULL ; initIoInfoHelper ( 0 ,
inputPortPaths , inputUnits , inputDimensions , true , smData ) ;
initIoInfoHelper ( 0 , outputPortPaths , outputUnits , outputDimensions ,
false , smData ) ; } static void initInputDerivs ( NeDaePrivateData * smData
) { const int32_T * numInputDerivs = NULL ; PmAllocator * alloc =
pm_default_allocator ( ) ; const int_T status = pm_create_int_vector_fields (
& smData -> mNumInputDerivs , smData -> mInputVectorSize , alloc ) ;
checkMemAllocStatus ( status ) ; memcpy ( smData -> mNumInputDerivs . mX ,
numInputDerivs , 0 * sizeof ( int32_T ) ) ; smData -> mInputOrder = 1 ; }
static void initDirectFeedthrough ( NeDaePrivateData * smData ) { const
boolean_T * directFeedthroughVector = NULL ; const boolean_T *
directFeedthroughMatrix = NULL ; PmAllocator * alloc = pm_default_allocator (
) ; { const int_T status = pm_create_bool_vector_fields ( & smData ->
mDirectFeedthroughVector , 0 , alloc ) ; checkMemAllocStatus ( status ) ;
memcpy ( smData -> mDirectFeedthroughVector . mX , directFeedthroughVector ,
0 * sizeof ( boolean_T ) ) ; } { const int_T status =
pm_create_bool_vector_fields ( & smData -> mDirectFeedthroughMatrix , 0 ,
alloc ) ; checkMemAllocStatus ( status ) ; memcpy ( smData ->
mDirectFeedthroughMatrix . mX , directFeedthroughMatrix , 0 * sizeof (
boolean_T ) ) ; } } static void initOutputDerivProc ( NeDaePrivateData *
smData ) { PmAllocator * alloc = pm_default_allocator ( ) ; const int32_T *
outputFunctionMap = NULL ; smData -> mOutputFunctionMap =
pm_create_int_vector ( 0 , alloc ) ; memcpy ( smData -> mOutputFunctionMap ->
mX , outputFunctionMap , 0 * sizeof ( int32_T ) ) ; smData ->
mNumOutputClasses = 0 ; smData -> mHasKinematicOutputs = false ; smData ->
mHasDynamicOutputs = false ; smData -> mIsOutputClass0Dynamic = false ;
smData -> mDoComputeDynamicOutputs = false ; smData ->
mCachedDerivativesAvailable = false ; { size_t i = 0 ; const int_T status =
pm_create_real_vector_fields ( & smData -> mCachedDerivatives , 0 ,
pm_default_allocator ( ) ) ; checkMemAllocStatus ( status ) ; for ( i = 0 ; i
< smData -> mCachedDerivatives . mN ; ++ i ) smData -> mCachedDerivatives .
mX [ i ] = 0.0 ; } }
#if 0
static void initializeSizePairVector ( const SmSizePair * data ,
SmSizePairVector * vector ) { const size_t n = sm_core_SmSizePairVector_size
( vector ) ; size_t i ; for ( i = 0 ; i < n ; ++ i , ++ data )
sm_core_SmSizePairVector_setValue ( vector , i , data ++ ) ; }
#endif
static void initAssemblyDelegate ( SmMechanismDelegate * delegate ) {
SmMechanismDelegateScratchpad * scratchpad = NULL ; const SmSizePair
jointToStageIdx [ 13 ] = { { 80 , 5 } , { 81 , 3 } , { 82 , 4 } , { 83 , 6 }
, { 84 , 7 } , { 85 , 9 } , { 86 , 11 } , { 87 , 2 } , { 88 , 12 } , { 89 , 1
} , { 90 , 8 } , { 91 , 10 } , { 211 , 0 } } ; const size_t primitiveIndices
[ 13 + 1 ] = { 0 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 }
; const SmSizePair stateOffsets [ 16 ] = { { 0 , 7 } , { 1 , 8 } , { 2 , 9 }
, { 3 , 10 } , { 13 , 14 } , { 15 , 16 } , { 17 , 18 } , { 19 , 20 } , { 21 ,
22 } , { 23 , 24 } , { 25 , 26 } , { 27 , 28 } , { 29 , 30 } , { 31 , 32 } ,
{ 33 , 34 } , { 35 , 36 } } ; const SmSizePair dofOffsets [ 16 ] = { { 0 , 1
} , { 1 , 2 } , { 2 , 3 } , { 3 , 6 } , { 6 , 7 } , { 7 , 8 } , { 8 , 9 } , {
9 , 10 } , { 10 , 11 } , { 11 , 12 } , { 12 , 13 } , { 13 , 14 } , { 14 , 15
} , { 15 , 16 } , { 16 , 17 } , { 17 , 18 } } ; const size_t * flexibleStages
= NULL ; const size_t remodIndices [ 12 ] = { 13 , 15 , 17 , 19 , 21 , 23 ,
25 , 27 , 29 , 31 , 33 , 35 } ; const size_t equationsPerConstraint [ 3 ] = {
0 , 0 , 0 } ; const size_t dofToVelSlot [ 18 ] = { 7 , 8 , 9 , 10 , 11 , 12 ,
14 , 16 , 18 , 20 , 22 , 24 , 26 , 28 , 30 , 32 , 34 , 36 } ; const size_t
constraintDofs [ 30 ] = { 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 14 , 15 , 0 , 1 , 2
, 3 , 4 , 5 , 6 , 7 , 12 , 13 , 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 16 , 17 } ;
const size_t constraintDofOffsets [ 3 + 1 ] = { 0 , 10 , 20 , 30 } ; const
size_t Jm = 0 ; const size_t Jn = 18 ; SmSizePair zeroSizePair ; zeroSizePair
. mFirst = zeroSizePair . mSecond = 0 ;
sm_core_MechanismDelegate_allocScratchpad ( delegate ) ; scratchpad =
delegate -> mScratchpad ; delegate -> mTargetStrengthFree = 0 ; delegate ->
mTargetStrengthSuggested = 1 ; delegate -> mTargetStrengthDesired = 2 ;
delegate -> mTargetStrengthRequired = 3 ; delegate -> mConsistencyTol = +
1.000000000000000062e-09 ; delegate -> mTreeJointDof = 18 ; delegate -> mDof
= 18 ; delegate -> mStateSize = 37 ; delegate -> mContinuousStateSize = 37 ;
delegate -> mModeVectorSize = 0 ; delegate -> mNumStages = 13 ; delegate ->
mNumConstraints = 3 ; delegate -> mNumAllConstraintEquations = 0 ;
sm_core_SmSizePairVector_create ( & delegate -> mJointToStageIdx , 13 , &
zeroSizePair ) ; memcpy ( sm_core_SmSizePairVector_nonConstValues ( &
delegate -> mJointToStageIdx ) , jointToStageIdx , 13 * sizeof ( SmSizePair )
) ; sm_core_SmSizeTVector_create ( & delegate -> mPrimitiveIndices , delegate
-> mNumStages + 1 , 0 ) ; memcpy ( sm_core_SmSizeTVector_nonConstValues ( &
delegate -> mPrimitiveIndices ) , primitiveIndices , ( delegate -> mNumStages
+ 1 ) * sizeof ( size_t ) ) ; sm_core_SmSizePairVector_create ( & delegate ->
mStateOffsets , 16 , & zeroSizePair ) ; memcpy (
sm_core_SmSizePairVector_nonConstValues ( & delegate -> mStateOffsets ) ,
stateOffsets , 16 * sizeof ( SmSizePair ) ) ; sm_core_SmSizePairVector_create
( & delegate -> mDofOffsets , 16 , & zeroSizePair ) ; memcpy (
sm_core_SmSizePairVector_nonConstValues ( & delegate -> mDofOffsets ) ,
dofOffsets , 16 * sizeof ( SmSizePair ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mFlexibleStages , 0 , 0 ) ; memcpy (
sm_core_SmSizeTVector_nonConstValues ( & delegate -> mFlexibleStages ) ,
flexibleStages , 0 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mRemodIndices , 12 , 0 ) ; memcpy (
sm_core_SmSizeTVector_nonConstValues ( & delegate -> mRemodIndices ) ,
remodIndices , 12 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mEquationsPerConstraint , delegate -> mNumConstraints , 0 ) ;
memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mEquationsPerConstraint ) , equationsPerConstraint , delegate ->
mNumConstraints * sizeof ( size_t ) ) ; sm_core_SmIntVector_create ( &
delegate -> mRunTimeEnabledEquations , delegate -> mNumAllConstraintEquations
, 1 ) ; sm_core_SmSizeTVector_create ( & delegate -> mDofToVelSlot , delegate
-> mDof , 0 ) ; memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mDofToVelSlot ) , dofToVelSlot , delegate -> mDof * sizeof ( size_t ) ) ;
sm_core_SmSizeTVector_create ( & delegate -> mConstraintDofs , 30 , 0 ) ;
memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate -> mConstraintDofs
) , constraintDofs , 30 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create
( & delegate -> mConstraintDofOffsets , delegate -> mNumConstraints + 1 , 0 )
; memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mConstraintDofOffsets ) , constraintDofOffsets , ( delegate ->
mNumConstraints + 1 ) * sizeof ( size_t ) ) ; sm_core_SmBoundedSet_create ( &
scratchpad -> mPosRequired , 18 ) ; sm_core_SmBoundedSet_create ( &
scratchpad -> mPosDesired , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad
-> mPosSuggested , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosFree , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosNonRequired , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosSuggAndFree , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelRequired , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelDesired , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelSuggested , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad -> mVelFree
, 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad -> mVelNonRequired , 18 )
; sm_core_SmBoundedSet_create ( & scratchpad -> mVelSuggAndFree , 18 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mConstraintFilter , 3 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveConstraints , 3 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveDofs , 18 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveDofs0 , 18 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mNewConstraints , 3 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mNewDofs , 18 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mUnsatisfiedConstraints , 3 ) ;
sm_core_SmSizeTVector_create ( & scratchpad -> mActiveConstraintsVect , 3 , 0
) ; sm_core_SmSizeTVector_create ( & scratchpad -> mActiveDofsVect , 18 , 0 )
; sm_core_SmSizeTVector_create ( & scratchpad -> mFullDofToActiveDof , 18 , 0
) ; sm_core_SmSizePairVector_create ( & scratchpad ->
mPartiallyPosTargetedPrims , 16 , & zeroSizePair ) ;
sm_core_SmSizePairVector_create ( & scratchpad -> mPartiallyVelTargetedPrims
, 16 , & zeroSizePair ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mPosPartialTypes , 16 , 0 ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mVelPartialTypes , 16 , 0 ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mPartiallyActivePrims , 16 , 0 ) ; sm_core_SmSizePairVector_create ( &
scratchpad -> mBaseFrameVelOffsets , 1 , & zeroSizePair ) ;
sm_core_SmSizePairVector_create ( & scratchpad -> mCvVelOffsets , 16 , &
zeroSizePair ) ; sm_core_SmRealVector_create ( & scratchpad ->
mCvAzimuthValues , 16 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mInitialState , 37 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mStartState , 37 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mTestState , 37 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mFullStateVector , 37 , 0.0 ) ; sm_core_SmIntVector_create ( & scratchpad ->
mModeVector , 0 , 0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mJacobianRowMaj , Jm * Jn , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mJacobian , Jm * Jn , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mJacobianPrimSubmatrix , Jm * 6 , 0.0 ) ;
sm_core_SmRealVector_create ( & scratchpad -> mConstraintNonhomoTerms , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mConstraintError , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mBestConstraintError ,
Jm , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mDeltas , Jn * (
Jm <= Jn ? Jm : Jn ) , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mSvdWork , 361 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mLineSearchScaledDeltaVect , 18 , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mLineSearchTestStateVect , 37 , 0.0 ) ;
sm_core_SmRealVector_create ( & scratchpad -> mLineSearchErrorVect , Jm , 0.0
) ; sm_core_SmRealVector_create ( & scratchpad -> mActiveDofVelsVect , 18 ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mVelSystemRhs , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mMotionData , 288 , 0.0
) ; delegate -> mSetTargets = simple_robot_9eb3ef65_1_setTargets ; delegate
-> mResetStateVector = simple_robot_9eb3ef65_1_resetAsmStateVector ; delegate
-> mInitializeTrackedAngleState =
simple_robot_9eb3ef65_1_initializeTrackedAngleState ; delegate ->
mComputeDiscreteState = simple_robot_9eb3ef65_1_computeDiscreteState ;
delegate -> mAdjustPosition = simple_robot_9eb3ef65_1_adjustPosition ;
delegate -> mPerturbJointPrimitiveState =
simple_robot_9eb3ef65_1_perturbAsmJointPrimitiveState ; delegate ->
mPerturbFlexibleBodyState = NULL ; delegate -> mComputePosDofBlendMatrix =
simple_robot_9eb3ef65_1_computePosDofBlendMatrix ; delegate ->
mComputeVelDofBlendMatrix = simple_robot_9eb3ef65_1_computeVelDofBlendMatrix
; delegate -> mProjectPartiallyTargetedPos =
simple_robot_9eb3ef65_1_projectPartiallyTargetedPos ; delegate ->
mPropagateMotion = simple_robot_9eb3ef65_1_propagateMotion ; delegate ->
mComputeAssemblyError = simple_robot_9eb3ef65_1_computeAssemblyError ;
delegate -> mComputeAssemblyJacobian =
simple_robot_9eb3ef65_1_computeAssemblyJacobian ; delegate ->
mComputeFullAssemblyJacobian =
simple_robot_9eb3ef65_1_computeFullAssemblyJacobian ; delegate ->
mIsInKinematicSingularity = simple_robot_9eb3ef65_1_isInKinematicSingularity
; delegate -> mConvertStateVector =
simple_robot_9eb3ef65_1_convertStateVector ; delegate ->
mConstructStateVector = NULL ; delegate -> mExtractSolverStateVector = NULL ;
delegate -> mIsPositionViolation = NULL ; delegate -> mIsVelocityViolation =
NULL ; delegate -> mProjectStateSim = NULL ; delegate ->
mComputeConstraintError = NULL ; delegate -> mResetModeVector = NULL ;
delegate -> mHasJointDisToNormModeChange = NULL ; delegate ->
mPerformJointDisToNormModeChange = NULL ; delegate -> mOnModeChangedCutJoints
= NULL ; delegate -> mMech = NULL ; } static void initSimulationDelegate (
SmMechanismDelegate * delegate ) { SmMechanismDelegateScratchpad * scratchpad
= NULL ; const SmSizePair jointToStageIdx [ 13 ] = { { 80 , 5 } , { 81 , 3 }
, { 82 , 4 } , { 83 , 6 } , { 84 , 7 } , { 85 , 9 } , { 86 , 11 } , { 87 , 2
} , { 88 , 12 } , { 89 , 1 } , { 90 , 8 } , { 91 , 10 } , { 211 , 0 } } ;
const size_t primitiveIndices [ 13 + 1 ] = { 0 , 4 , 5 , 6 , 7 , 8 , 9 , 10 ,
11 , 12 , 13 , 14 , 15 , 16 } ; const SmSizePair stateOffsets [ 16 ] = { { 0
, 7 } , { 1 , 8 } , { 2 , 9 } , { 3 , 10 } , { 13 , 14 } , { 15 , 16 } , { 17
, 18 } , { 19 , 20 } , { 21 , 22 } , { 23 , 24 } , { 25 , 26 } , { 27 , 28 }
, { 29 , 30 } , { 31 , 32 } , { 33 , 34 } , { 35 , 36 } } ; const SmSizePair
dofOffsets [ 16 ] = { { 0 , 1 } , { 1 , 2 } , { 2 , 3 } , { 3 , 6 } , { 6 , 7
} , { 7 , 8 } , { 8 , 9 } , { 9 , 10 } , { 10 , 11 } , { 11 , 12 } , { 12 ,
13 } , { 13 , 14 } , { 14 , 15 } , { 15 , 16 } , { 16 , 17 } , { 17 , 18 } }
; const size_t * flexibleStages = NULL ; const size_t remodIndices [ 12 ] = {
13 , 15 , 17 , 19 , 21 , 23 , 25 , 27 , 29 , 31 , 33 , 35 } ; const size_t
equationsPerConstraint [ 3 ] = { 0 , 0 , 0 } ; const size_t dofToVelSlot [ 18
] = { 7 , 8 , 9 , 10 , 11 , 12 , 14 , 16 , 18 , 20 , 22 , 24 , 26 , 28 , 30 ,
32 , 34 , 36 } ; const size_t constraintDofs [ 30 ] = { 0 , 1 , 2 , 3 , 4 , 5
, 6 , 7 , 14 , 15 , 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 12 , 13 , 0 , 1 , 2 , 3 ,
4 , 5 , 6 , 7 , 16 , 17 } ; const size_t constraintDofOffsets [ 3 + 1 ] = { 0
, 10 , 20 , 30 } ; const size_t Jm = 0 ; const size_t Jn = 18 ; SmSizePair
zeroSizePair ; zeroSizePair . mFirst = zeroSizePair . mSecond = 0 ;
sm_core_MechanismDelegate_allocScratchpad ( delegate ) ; scratchpad =
delegate -> mScratchpad ; delegate -> mTargetStrengthFree = 0 ; delegate ->
mTargetStrengthSuggested = 1 ; delegate -> mTargetStrengthDesired = 2 ;
delegate -> mTargetStrengthRequired = 3 ; delegate -> mConsistencyTol = +
1.000000000000000062e-09 ; delegate -> mTreeJointDof = 18 ; delegate -> mDof
= 18 ; delegate -> mStateSize = 37 ; delegate -> mContinuousStateSize = 37 ;
delegate -> mModeVectorSize = 0 ; delegate -> mNumStages = 13 ; delegate ->
mNumConstraints = 3 ; delegate -> mNumAllConstraintEquations = 0 ;
sm_core_SmSizePairVector_create ( & delegate -> mJointToStageIdx , 13 , &
zeroSizePair ) ; memcpy ( sm_core_SmSizePairVector_nonConstValues ( &
delegate -> mJointToStageIdx ) , jointToStageIdx , 13 * sizeof ( SmSizePair )
) ; sm_core_SmSizeTVector_create ( & delegate -> mPrimitiveIndices , delegate
-> mNumStages + 1 , 0 ) ; memcpy ( sm_core_SmSizeTVector_nonConstValues ( &
delegate -> mPrimitiveIndices ) , primitiveIndices , ( delegate -> mNumStages
+ 1 ) * sizeof ( size_t ) ) ; sm_core_SmSizePairVector_create ( & delegate ->
mStateOffsets , 16 , & zeroSizePair ) ; memcpy (
sm_core_SmSizePairVector_nonConstValues ( & delegate -> mStateOffsets ) ,
stateOffsets , 16 * sizeof ( SmSizePair ) ) ; sm_core_SmSizePairVector_create
( & delegate -> mDofOffsets , 16 , & zeroSizePair ) ; memcpy (
sm_core_SmSizePairVector_nonConstValues ( & delegate -> mDofOffsets ) ,
dofOffsets , 16 * sizeof ( SmSizePair ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mFlexibleStages , 0 , 0 ) ; memcpy (
sm_core_SmSizeTVector_nonConstValues ( & delegate -> mFlexibleStages ) ,
flexibleStages , 0 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mRemodIndices , 12 , 0 ) ; memcpy (
sm_core_SmSizeTVector_nonConstValues ( & delegate -> mRemodIndices ) ,
remodIndices , 12 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create ( &
delegate -> mEquationsPerConstraint , delegate -> mNumConstraints , 0 ) ;
memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mEquationsPerConstraint ) , equationsPerConstraint , delegate ->
mNumConstraints * sizeof ( size_t ) ) ; sm_core_SmIntVector_create ( &
delegate -> mRunTimeEnabledEquations , delegate -> mNumAllConstraintEquations
, 1 ) ; sm_core_SmSizeTVector_create ( & delegate -> mDofToVelSlot , delegate
-> mDof , 0 ) ; memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mDofToVelSlot ) , dofToVelSlot , delegate -> mDof * sizeof ( size_t ) ) ;
sm_core_SmSizeTVector_create ( & delegate -> mConstraintDofs , 30 , 0 ) ;
memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate -> mConstraintDofs
) , constraintDofs , 30 * sizeof ( size_t ) ) ; sm_core_SmSizeTVector_create
( & delegate -> mConstraintDofOffsets , delegate -> mNumConstraints + 1 , 0 )
; memcpy ( sm_core_SmSizeTVector_nonConstValues ( & delegate ->
mConstraintDofOffsets ) , constraintDofOffsets , ( delegate ->
mNumConstraints + 1 ) * sizeof ( size_t ) ) ; sm_core_SmBoundedSet_create ( &
scratchpad -> mPosRequired , 18 ) ; sm_core_SmBoundedSet_create ( &
scratchpad -> mPosDesired , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad
-> mPosSuggested , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosFree , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosNonRequired , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mPosSuggAndFree , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelRequired , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelDesired , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad ->
mVelSuggested , 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad -> mVelFree
, 18 ) ; sm_core_SmBoundedSet_create ( & scratchpad -> mVelNonRequired , 18 )
; sm_core_SmBoundedSet_create ( & scratchpad -> mVelSuggAndFree , 18 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mConstraintFilter , 3 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveConstraints , 3 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveDofs , 18 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mActiveDofs0 , 18 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mNewConstraints , 3 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mNewDofs , 18 ) ;
sm_core_SmBoundedSet_create ( & scratchpad -> mUnsatisfiedConstraints , 3 ) ;
sm_core_SmSizeTVector_create ( & scratchpad -> mActiveConstraintsVect , 3 , 0
) ; sm_core_SmSizeTVector_create ( & scratchpad -> mActiveDofsVect , 18 , 0 )
; sm_core_SmSizeTVector_create ( & scratchpad -> mFullDofToActiveDof , 18 , 0
) ; sm_core_SmSizePairVector_create ( & scratchpad ->
mPartiallyPosTargetedPrims , 16 , & zeroSizePair ) ;
sm_core_SmSizePairVector_create ( & scratchpad -> mPartiallyVelTargetedPrims
, 16 , & zeroSizePair ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mPosPartialTypes , 16 , 0 ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mVelPartialTypes , 16 , 0 ) ; sm_core_SmSizeTVector_create ( & scratchpad ->
mPartiallyActivePrims , 16 , 0 ) ; sm_core_SmSizePairVector_create ( &
scratchpad -> mBaseFrameVelOffsets , 1 , & zeroSizePair ) ;
sm_core_SmSizePairVector_create ( & scratchpad -> mCvVelOffsets , 16 , &
zeroSizePair ) ; sm_core_SmRealVector_create ( & scratchpad ->
mCvAzimuthValues , 16 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mInitialState , 37 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mStartState , 37 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mTestState , 37 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mFullStateVector , 37 , 0.0 ) ; sm_core_SmIntVector_create ( & scratchpad ->
mModeVector , 0 , 0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mJacobianRowMaj , Jm * Jn , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mJacobian , Jm * Jn , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mJacobianPrimSubmatrix , Jm * 6 , 0.0 ) ;
sm_core_SmRealVector_create ( & scratchpad -> mConstraintNonhomoTerms , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mConstraintError , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mBestConstraintError ,
Jm , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mDeltas , Jn * (
Jm <= Jn ? Jm : Jn ) , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mSvdWork , 361 , 0.0 ) ; sm_core_SmRealVector_create ( & scratchpad ->
mLineSearchScaledDeltaVect , 18 , 0.0 ) ; sm_core_SmRealVector_create ( &
scratchpad -> mLineSearchTestStateVect , 37 , 0.0 ) ;
sm_core_SmRealVector_create ( & scratchpad -> mLineSearchErrorVect , Jm , 0.0
) ; sm_core_SmRealVector_create ( & scratchpad -> mActiveDofVelsVect , 18 ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mVelSystemRhs , Jm ,
0.0 ) ; sm_core_SmRealVector_create ( & scratchpad -> mMotionData , 288 , 0.0
) ; delegate -> mSetTargets = NULL ; delegate -> mResetStateVector =
simple_robot_9eb3ef65_1_resetSimStateVector ; delegate ->
mInitializeTrackedAngleState = NULL ; delegate -> mComputeDiscreteState =
NULL ; delegate -> mAdjustPosition = NULL ; delegate ->
mPerturbJointPrimitiveState =
simple_robot_9eb3ef65_1_perturbSimJointPrimitiveState ; delegate ->
mPerturbFlexibleBodyState = simple_robot_9eb3ef65_1_perturbFlexibleBodyState
; delegate -> mComputePosDofBlendMatrix = NULL ; delegate ->
mComputeVelDofBlendMatrix = NULL ; delegate -> mProjectPartiallyTargetedPos =
NULL ; delegate -> mPropagateMotion = NULL ; delegate ->
mComputeAssemblyError = NULL ; delegate -> mComputeAssemblyJacobian = NULL ;
delegate -> mComputeFullAssemblyJacobian = NULL ; delegate ->
mIsInKinematicSingularity = NULL ; delegate -> mConvertStateVector = NULL ;
delegate -> mConstructStateVector =
simple_robot_9eb3ef65_1_constructStateVector ; delegate ->
mExtractSolverStateVector = simple_robot_9eb3ef65_1_extractSolverStateVector
; delegate -> mIsPositionViolation =
simple_robot_9eb3ef65_1_isPositionViolation ; delegate ->
mIsVelocityViolation = simple_robot_9eb3ef65_1_isVelocityViolation ; delegate
-> mProjectStateSim = simple_robot_9eb3ef65_1_projectStateSim ; delegate ->
mComputeConstraintError = simple_robot_9eb3ef65_1_computeConstraintError ;
delegate -> mResetModeVector = simple_robot_9eb3ef65_1_resetModeVector ;
delegate -> mHasJointDisToNormModeChange =
simple_robot_9eb3ef65_1_hasJointDisToNormModeChange ; delegate ->
mPerformJointDisToNormModeChange =
simple_robot_9eb3ef65_1_performJointDisToNormModeChange ; delegate ->
mOnModeChangedCutJoints = simple_robot_9eb3ef65_1_onModeChangedCutJoints ;
delegate -> mMech = NULL ; } static void initMechanismDelegates (
NeDaePrivateData * smData ) { PmAllocator * alloc = pm_default_allocator ( )
; const int32_T * motionInputOffsets = NULL ; int_T status = 0 ;
initAssemblyDelegate ( & smData -> mAssemblyDelegate ) ;
initSimulationDelegate ( & smData -> mSimulationDelegate ) ; status =
pm_create_int_vector_fields ( & smData -> mMotionInputOffsets , smData ->
mNumInputMotionPrimitives , alloc ) ; checkMemAllocStatus ( status ) ; memcpy
( smData -> mMotionInputOffsets . mX , motionInputOffsets , 0 * sizeof (
int32_T ) ) ; } static void initComputationFcnPtrs ( NeDaePrivateData *
smData ) { smData -> mSetParametersFcn = dae_cg_setParameters_function ;
smData -> mPAssertFcn = dae_cg_pAssert_method ; smData -> mDerivativeFcn =
dae_cg_deriv_method ; smData -> mNumJacPerturbLoBoundsFcn =
dae_cg_numJacPerturbLoBounds_method ; smData -> mNumJacPerturbHiBoundsFcn =
dae_cg_numJacPerturbHiBounds_method ; smData -> mOutputFcn =
dae_cg_compOutputs_method ; smData -> mModeFcn = dae_cg_mode_method ; smData
-> mZeroCrossingFcn = dae_cg_zeroCrossing_method ; smData -> mProjectionFcn =
dae_cg_project_solve ; smData -> mCIC_MODE_Fcn = dae_cg_CIC_MODE_solve ;
smData -> mCheckFcn = ( smData -> mStateVectorSize == 0 ) ?
dae_cg_check_solve : NULL ; smData -> mAssemblyFcn = dae_cg_assemble_solve ;
smData -> mSetupLoggerFcn = sm_ssci_setupLoggerFcn_codeGen ; smData ->
mLogFcn = sm_ssci_logFcn_codeGen ; smData -> mResidualsFcn = NULL ; smData ->
mLinearizeFcn = NULL ; smData -> mGenerateFcn = NULL ; } static void
initLiveLinkToSm ( NeDaePrivateData * smData ) { smData -> mLiveSmLink = NULL
; smData -> mLiveSmLink_destroy = NULL ; smData -> mLiveSmLink_copy = NULL ;
} void simple_robot_9eb3ef65_1_NeDaePrivateData_create ( NeDaePrivateData *
smData ) { initBasicAttributes ( smData ) ; initStateVector ( smData ) ;
initAsserts ( smData ) ; initModeVector ( smData ) ; initZeroCrossings (
smData ) ; initVariables ( smData ) ; initRuntimeParameters ( smData ) ;
initIoInfo ( smData ) ; initInputDerivs ( smData ) ; initDirectFeedthrough (
smData ) ; initOutputDerivProc ( smData ) ; initMechanismDelegates ( smData )
; initComputationFcnPtrs ( smData ) ; initLiveLinkToSm ( smData ) ; }
