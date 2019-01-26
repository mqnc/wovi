#include "IKRay.h"

#include "ik_ray_kernel.cl"
#include "ik_ray_kernel_auto_elbow.cl"

////////////////////////////// constructor

IKRay::IKRay(void){

    int found;

    found=0;
    hardware = sclGetAllHardware(&found);

    // I have taken most of the stuff out so it compiles faster for GPU

    const char* dpflag = "#define DOUBLE_PRECISION\n";

    /*
    softwareSingle = sclGetCLSoftwareFromString(const_cast<char*>(ikRayKernelSrc), (char*)"ikray", hardware[0]);
    char* ikRayKernelSrc_double = new char[strlen(dpflag) + strlen(ikRayKernelSrc) + 1];
    strcpy(ikRayKernelSrc_double, dpflag);
    strcat(ikRayKernelSrc_double, ikRayKernelSrc);
    softwareDouble = sclGetCLSoftwareFromString(ikRayKernelSrc_double, (char*)"ikray", hardware[0]);
    delete[] ikRayKernelSrc_double;
    */

#ifndef DOUBLE_PRECISION
    software2Single = sclGetCLSoftwareFromString(const_cast<char*>(ikRayKernelAutoElbowSrc), (char*)"ikray", hardware[0]);
#else
    char* ikRayKernelAutoElbowSrc_double = new char[strlen(dpflag) + strlen(ikRayKernelAutoElbowSrc) + 1];
    strcpy(ikRayKernelAutoElbowSrc_double, dpflag);
    strcat(ikRayKernelAutoElbowSrc_double, ikRayKernelAutoElbowSrc);
    software2Double = sclGetCLSoftwareFromString(ikRayKernelAutoElbowSrc_double, (char*)"ikray", hardware[0]);
    delete[] ikRayKernelAutoElbowSrc_double;
#endif
}


////////////////////////////// method for actual computation

void IKRay::compute(void* endParameters,
        void* rayEnds,
        void* rayColors,
        void* startPos,
        void* startOri,
        void* invTool,
        void* nsparam, // at start pose
        int8_t config,
        void* targets,
        void* targetOris,
        uint32_t nrays,
        uint32_t nsteps,
        bool autoElbow,
        bool doublePrecision,
        void* debugOutput,
        uint32_t debugIndex){

    size_t global_size[2];
    size_t local_size[2];

    if(nrays%GPU_LOCAL_SIZE != 0){
        printf("Error while trying to compute IK on GPU: Number of IK rays has to be a multiple of GPU_LOCAL_SIZE (=%d) (just append some meaningless elements)", GPU_LOCAL_SIZE);
        return;
    }

    global_size[0]=nrays; global_size[1]=1;
    local_size[0]=GPU_LOCAL_SIZE; local_size[1]=1;

    sclSoft sw;

    if(!doublePrecision){
        if(!autoElbow){sw = softwareSingle;}
        else{sw = software2Single;}

        sclManageArgsLaunchKernel(
                    hardware[0], sw,
                    global_size, local_size,
                    "%w %w %w %w %r %r %r %r %r %r %r %r %r",

                    sizeof(cl_float)*nrays, (void*)endParameters,
                    sizeof(cl_float3)*nrays, (void*)rayEnds,
                    sizeof(cl_float4)*nrays, (void*)rayColors,
                    sizeof(cl_float)*nrays*nsteps*7, (void*)debugOutput,

                    sizeof(cl_uint), (void*)&debugIndex,
                    sizeof(cl_float3), (void*)startPos,
                    sizeof(cl_float4), (void*)startOri,
                    sizeof(cl_float16), (void*)invTool,
                    sizeof(cl_float), (void*)nsparam,
                    sizeof(cl_uchar), (void*)&config,
                    sizeof(cl_float3)*nrays, (void*)targets,
                    sizeof(cl_float4)*nrays, (void*)targetOris,
                    sizeof(cl_uint), (void*)&nsteps);
    }
    else{
        if(!autoElbow){sw = softwareDouble;}
        else{sw = software2Double;}

        sclManageArgsLaunchKernel(
                    hardware[0], sw,
                    global_size, local_size,
                    "%w %w %w %w %r %r %r %r %r %r %r %r %r",

                    sizeof(cl_double)*nrays, (void*)endParameters,
                    sizeof(cl_double3)*nrays, (void*)rayEnds,
                    sizeof(cl_double4)*nrays, (void*)rayColors,
                    sizeof(cl_double)*nrays*nsteps*7, (void*)debugOutput,

                    sizeof(cl_uint), (void*)&debugIndex,
                    sizeof(cl_double3), (void*)startPos,
                    sizeof(cl_double4), (void*)startOri,
                    sizeof(cl_double16), (void*)invTool,
                    sizeof(cl_double), (void*)nsparam,
                    sizeof(cl_uchar), (void*)&config,
                    sizeof(cl_double3)*nrays, (void*)targets,
                    sizeof(cl_double4)*nrays, (void*)targetOris,
                    sizeof(cl_uint), (void*)&nsteps);
    }
}
