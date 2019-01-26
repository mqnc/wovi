
#include "AccMap.h"

//#include "intervals_kernel_numerics.cl"
#include "intervals_kernel.cl"

////////////////////////////// constructor

AccMap::AccMap(void){

    int found;

    found=0;
    hardware = sclGetAllHardware(&found);

    const char* dpflag = "#define DOUBLE_PRECISION\n";

#ifndef DOUBLE_PRECISION
    softwareSingle = sclGetCLSoftwareFromString(const_cast<char*>(intervalKernelSrc), (char*)"intervals", hardware[0]);
#else
    char* intervalKernelSrc_double = new char[strlen(dpflag) + strlen(intervalKernelSrc) + 1];
    strcpy(intervalKernelSrc_double, dpflag);
    strcat(intervalKernelSrc_double, intervalKernelSrc);
    softwareDouble = sclGetCLSoftwareFromString(intervalKernelSrc_double, (char*)"intervals", hardware[0]);
    delete[] intervalKernelSrc_double;
#endif
}


////////////////////////////// method for actual computation

void AccMap::compute(
             int16_t error[],
             int16_t intervals[],
             float cost[],
             void* targets,
             uint8_t configs[],
             void* invTool,
             uint32_t nSamples,
             bool doublePrecision){

    size_t global_size[2];
    size_t local_size[2];


    if(nSamples%GPU_LOCAL_SIZE != 0){
        printf("Error while trying to compute IK on GPU: Number of samples (=%d) has to be a multiple of GPU_LOCAL_SIZE (=%d) (just append some meaningless elements)", nSamples, GPU_LOCAL_SIZE);
        return;
    }

    global_size[0]=nSamples; global_size[1]=1;
    local_size[0]=GPU_LOCAL_SIZE; local_size[1]=1;

    if(!doublePrecision){

        sclManageArgsLaunchKernel(
                    hardware[0], softwareSingle,
                    global_size, local_size,
                    "%w %w %w %r %r %r",

                    sizeof(cl_short)*nSamples, (void*)error,
                    sizeof(cl_short)*64*(nSamples*0+1), (void*)intervals,
                    sizeof(cl_float)*nSamples, (void*)cost,
                    sizeof(cl_float16)*nSamples, (void*)targets,
                    sizeof(cl_uchar)*nSamples,  (void*)configs,
                    sizeof(cl_float16), (void*)invTool);
    }
    else{

        sclManageArgsLaunchKernel(
                    hardware[0], softwareDouble,
                    global_size, local_size,
                    "%w %w %w %r %r %r",

                    sizeof(cl_short)*nSamples, (void*)error,
                    sizeof(cl_short)*64*(nSamples*0+1), (void*)intervals,
                    sizeof(cl_float)*nSamples, (void*)cost,
                    sizeof(cl_double16)*nSamples, (void*)targets,
                    sizeof(cl_uchar)*nSamples,  (void*)configs,
                    sizeof(cl_double16), (void*)invTool);
    }

}
