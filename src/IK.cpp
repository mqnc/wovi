
// Matrix element order:
// 0 4 8 C
// 1 5 9 D
// 2 6 A E
// 3 7 B F

// joint encoding:
// 2 bytes
// -32400 corresponds to -180deg
//      0 corresponds to    0deg
//  32400 corresponds to  180deg (so resolution is 180 steps per degree)

// interval encoding:
// 2 bytes per border (short), lower border first
// -32400 corresponds to -180deg
//      0 corresponds to    0deg
//  32400 corresponds to  180deg (so resolution is 180 steps per degree)
//  32767 means the interval is not used

#include "IK.h"

#include <Eigen/Geometry>
#include "LwrLibrary.hpp"

#include "ik_kernel.cl"

#define GPU_LOCAL_SIZE 64

IK::IK(void){
    int found;

    found=0;
    hardware = sclGetAllHardware(&found);

    softwareSingle = sclGetCLSoftwareFromString(const_cast<char*>(ikKernelSrc), (char*)"ik", hardware[0]);

    const char* dpflag = "#define DOUBLE_PRECISION\n";
    char* ikKernelSrc_double = new char[strlen(dpflag) + strlen(ikKernelSrc) + 1];
    strcpy(ikKernelSrc_double, dpflag);
    strcat(ikKernelSrc_double, ikKernelSrc);
    softwareDouble = sclGetCLSoftwareFromString(ikKernelSrc_double, (char*)"ik", hardware[0]);
}

void IK::compute(
        int8_t* error,
        int16_t* joints,
        void* target,
        void* nsparam,
        int8_t* config,
        int32_t n,
        bool doublePrecision){

    size_t global_size[2];
    size_t local_size[2];

    if(n%GPU_LOCAL_SIZE != 0){
        printf("Error while trying to compute IK on GPU: Number of targets has to be a multiple of GPU_LOCAL_SIZE (=%d) (just append some meaningless elements)", GPU_LOCAL_SIZE);
        return;
    }

    global_size[0]=n; global_size[1]=1;
    local_size[0]=GPU_LOCAL_SIZE; local_size[1]=1;

    if(!doublePrecision){
        sclManageArgsLaunchKernel( hardware[0], softwareSingle,
                                   global_size, local_size,
                                   "%w %w %r %r %r",
                                   sizeof(cl_uchar)*n, (void*)error,
                                   sizeof(cl_short8)*n, (void*)joints,
                                   sizeof(cl_float16)*n, (void*)target,
                                   sizeof(cl_float)*n, (void*)nsparam,
                                   sizeof(cl_uchar)*n, (void*)config);
    }
    else{
        sclManageArgsLaunchKernel( hardware[0], softwareDouble,
                                   global_size, local_size,
                                   "%w %w %r %r %r",
                                   sizeof(cl_uchar)*n, (void*)error,
                                   sizeof(cl_short8)*n, (void*)joints,
                                   sizeof(cl_double16)*n, (void*)target,
                                   sizeof(cl_double)*n, (void*)nsparam,
                                   sizeof(cl_uchar)*n, (void*)config);
    }

}


