#ifndef RTW_HEADER_simple_robot_cap_host_h_
#define RTW_HEADER_simple_robot_cap_host_h_
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"
typedef struct { rtwCAPI_ModelMappingInfo mmi ; }
simple_robot_host_DataMapInfo_T ;
#ifdef __cplusplus
extern "C" {
#endif
void simple_robot_host_InitializeDataMapInfo (
simple_robot_host_DataMapInfo_T * dataMap , const char * path ) ;
#ifdef __cplusplus
}
#endif
#endif
#endif
