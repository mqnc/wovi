
#define DOUBLE_PRECISION
#define GPU_LOCAL_SIZE 64

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

#ifndef DOUBLE_PRECISION
    #define FLOAT float
    #define CLFLOAT cl_float
    #define CLFLOAT2 cl_float2
    #define CLFLOAT3 cl_float3
    #define CLFLOAT4 cl_float4
    #define CLFLOAT8 cl_float8
    #define CLFLOAT16 cl_float16
#else
    #define FLOAT double
    #define CLFLOAT cl_double
    #define CLFLOAT2 cl_double2
    #define CLFLOAT3 cl_double3
    #define CLFLOAT4 cl_double4
    #define CLFLOAT8 cl_double8
    #define CLFLOAT16 cl_double16
#endif

#include <iostream>
#include <iomanip>
#include "simpleCL.h"
#include <ctime>
#include <stack>
#include <Geometry>

#include "LwrLibrary.hpp"

#define RAND (((FLOAT)rand())/((FLOAT)RAND_MAX)*2.0-1.0) // Random value between -1 and 1


// tic toc like in MATLAB

std::stack<clock_t> tictoc_stack;
void tic() {tictoc_stack.push(clock());}
void toc() {
    std::cout << std::endl << "Time elapsed: "  <<
                 ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC << "s = " <<
                 ((double)(clock() - tictoc_stack.top())) * 1000.0 / CLOCKS_PER_SEC << "ms" <<std::endl;
    tictoc_stack.pop();
}


// main

int main( int argc, char *argv[] ){
        (void)argc; (void)argv; // suppress unused parameter warning

        int found;
        sclHard* hardware;
        sclSoft software_ik;
        sclSoft software_intervals;

        size_t global_size[2];
        size_t local_size[2];

        long int n = 64000;
        n = n + (GPU_LOCAL_SIZE - (n%GPU_LOCAL_SIZE)); // make n a multiple of GPU_LOCAL_SIZE

        cl_ushort *error;
        cl_short8 *joints;
        cl_short *intervals;
        CLFLOAT16 *target;
        CLFLOAT *nsparam;
        cl_uchar *config;

        error = (cl_ushort*)malloc(sizeof(cl_ushort)*n);
        joints = (cl_short8*)malloc(sizeof(cl_short8)*n);
        intervals = (cl_short*)malloc(sizeof(cl_uint)*64*n);
        target = (CLFLOAT16*)malloc(sizeof(CLFLOAT16)*n);
        nsparam = (CLFLOAT*)malloc(sizeof(CLFLOAT)*n);
        config = (cl_uchar*)malloc(sizeof(cl_uchar)*n);

        global_size[0]=n; global_size[1]=1;
        local_size[0]=GPU_LOCAL_SIZE; local_size[1]=1;

        for (int i=0; i<n; i++){

            Eigen::Quaterniond q(1,1,1,1);

            while(q.norm()>1){
                q = Eigen::Quaterniond(RAND,RAND,RAND,RAND);
            }
            q.normalize();

            Eigen::Matrix3d T = q.toRotationMatrix();

            target[i].s0 = T(0,0);
            target[i].s1 = T(1,0);
            target[i].s2 = T(2,0);
            target[i].s3 = 0;

            target[i].s4 = T(0,1);
            target[i].s5 = T(1,1);
            target[i].s6 = T(2,1);
            target[i].s7 = 0;

            target[i].s8 = T(0,2);
            target[i].s9 = T(1,2);
            target[i].sa = T(2,2);
            target[i].sb = 0;

            target[i].sC = RAND*0.9;
            target[i].sD = RAND*0.9;
            target[i].sE = RAND*0.9+0.31;
            target[i].sF = 1;

            nsparam[i] = RAND*M_PI;
            config[i] = rand() % 8;
        }

        found=0;
        hardware = sclGetAllHardware(&found);
        software_ik = sclGetCLSoftware((char*)"ik_kernel.cl",(char*)"ik",hardware[0]);
        software_intervals = sclGetCLSoftware((char*)"intervals_kernel_debug.cl",(char*)"intervals",hardware[0]);

        tic();

        /*sclManageArgsLaunchKernel( hardware[0], software_ik,
                                   global_size, local_size,
                                   "%w %w %r %r %r", // writeonly / readonly
                                   //"%w %w",
                                   sizeof(cl_ushort)*n, (void*)error,
                                   sizeof(cl_short8)*n, (void*)joints,
                                   sizeof(CLFLOAT16)*n, (void*)target,
                                   sizeof(CLFLOAT)*n, (void*)nsparam,
                                   sizeof(cl_uchar)*n, (void*)config);
        */
        CLFLOAT debugval[1000];

        sclManageArgsLaunchKernel( hardware[0], software_intervals,
                                   global_size, local_size,
                                   "%w %w %r %r", // writeonly / readonly
                                   sizeof(cl_ushort)*n, (void*)error,
                                   sizeof(cl_short)*64*n, (void*)intervals,
                                   sizeof(CLFLOAT16)*n, (void*)target,
                                   sizeof(cl_uchar)*n, (void*)config/*,
                                   sizeof(CLFLOAT)*1000, (void*)debugval*/);

        toc();


        LwrXCart CPUtarget;
        LwrJoints CPUjoints;
        short* CPUintervals = new short[64*n];
        LwrErrorMsg* CPUerror = new LwrErrorMsg[n];

        LwrElbowInterval currentMargin;
        LwrElbowInterval reachable[11];
        LwrElbowInterval blocked[11];
        LwrElbowInterval blockedPerJoint[7][3];

        tic();
        int iRes = 0;
        for (int i=0; i<n; i++){

            CPUtarget.pose.pos = Eigen::Vector3d(target[i].sC,target[i].sD,target[i].sE);
            CPUtarget.pose.ori << target[i].s0, target[i].s4, target[i].s8,
                                  target[i].s1, target[i].s5, target[i].s9,
                                  target[i].s2, target[i].s6, target[i].sA;

            CPUtarget.config = config[i];
            CPUtarget.nsparam = nsparam[i];

            //Lwr::inverseKinematics(CPUjoints, CPUtarget);
            CPUerror[i] = Lwr::elbowIntervals(currentMargin,reachable,blocked,blockedPerJoint,CPUtarget);

            for(int k=0;k<11;k++){
                if(reachable[k].valid){
                    CPUintervals[iRes++] = reachable[k].lower/M_PI*32400;
                    CPUintervals[iRes++] = reachable[k].upper/M_PI*32400;
                }
                else{
                    CPUintervals[iRes++] = 32767;
                    CPUintervals[iRes++] = 32767;
                }
            }

            for(int j=0;j<7;j++){
                for(int k=0;k<3;k++){
                    if(blockedPerJoint[j][k].valid){
                        CPUintervals[iRes++] = blockedPerJoint[j][k].lower/M_PI*32400;
                        CPUintervals[iRes++] = blockedPerJoint[j][k].upper/M_PI*32400;
                    }
                    else{
                        CPUintervals[iRes++] = 32767;
                        CPUintervals[iRes++] = 32767;
                    }
                }
            }
        }
        toc();

#define BR std::cout << std::endl
#define COUT std::cout << std::setprecision(9) <<

        //std::cout << debugval[i] << std::endl;
        printf("\ndone\n");
/*
        COUT "---------------- CPU ---------------";
        BR;
        std::cout << "per joint" << std::endl;
        for (int j=0; j<7; j++){
            for (int i=0; i<3; i++){
                std::cout << blockedPerJoint[j][i] << " ";
            }
            BR;
        }
        std::cout << "blocked" << std::endl;
        for (int i=0; i<11; i++){
            std::cout << blocked[i] << " ";
        }
        std::cout << std::endl << "reachable" << std::endl;
        for (int i=0; i<11; i++){
            std::cout << reachable[i] << " ";
        }
        BR;


        COUT "---------------- GPU ---------------";
        BR;

        int k=0;
        for(int j=0;j<6;j++){
            for(int i=0;i<4;i++){
                COUT debugval[k++] << ": ";
                COUT debugval[k++];
                COUT " .. " << debugval[k++] << "  |  ";
            }
            BR;
        }
        BR;
        for(int i=0;i<20;i++){
            COUT debugval[k++]; BR;
        }*/
/*
        COUT "per joint"; BR;
        int k=0;
        for(int j=0;j<7;j++){
            for(int i=0;i<3;i++){
                COUT debugval[k++];
                COUT " .. ";
                COUT debugval[k++];
                COUT " | ";
            }
            BR;
        }
        COUT "combined"; BR;
        for(int i=0;i<11;i++){
            COUT debugval[k++];
            COUT "..";
            COUT debugval[k++];
            COUT "|";
        }
        BR;
        COUT "reachable"; BR;
        for(int i=0;i<11;i++){
            COUT debugval[k++];
            COUT "..";
            COUT debugval[k++];
            COUT "|";
        }
        BR;
*/
/*
        for(int i=0;i<100;i++){
            COUT i << ") " << debugval[i]; BR;
        }*/

        //std::cout << joints[0].s0 << "  " << joints[0].s1 << "  " << joints[0].s2 << "  " << joints[0].s3 << "  " << joints[0].s4 << "  " << joints[0].s5 << "  " << joints[0].s6 << std::endl;




/*
        for(int k=0;k<n;k++){
            bool equal=true;
            for(int i=0; i<32; i++){
                if( abs(CPUintervals[k*64+i] - intervals[k*64+i])>7 ) equal=false;
            }

            if(!equal){
                COUT "trial nr. " << k; BR;
                COUT "Target:"; BR;
                COUT target[k].s0 << "  " << target[k].s4 << "  " << target[k].s8 << "  " << target[k].sc; BR;
                COUT target[k].s1 << "  " << target[k].s5 << "  " << target[k].s9 << "  " << target[k].sd; BR;
                COUT target[k].s2 << "  " << target[k].s6 << "  " << target[k].sa << "  " << target[k].se; BR;
                COUT "Config: " << (int)config[k]; BR;BR;

                COUT "CPU              GPU"; BR;
                COUT "Errormsg: " << CPUerror[k] << "  | Errormsg: " << error[k]; BR;

                for(int i=0; i<32; i++){
                    COUT CPUintervals[k*64 + i*2] << ".." << CPUintervals[k*64 + i*2+1] << "  |  ";
                    COUT intervals[k*64 + i*2] << ".." << intervals[k*64 + i*2+1]; BR;

                    if(i==10 || i==13 || i==16 || i==19 || i==22 || i==25 || i==28)BR;
                }
                BR; BR;
            }
        }
*/

        free(error);
        free(joints);
        free(intervals);
        free(target);
        free(nsparam);
        free(config);

        delete[] CPUintervals;
        delete[] CPUerror;

        return 0;
}
