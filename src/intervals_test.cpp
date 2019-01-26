
#include "defs.h"
#include <ros/ros.h>
#include <Eigen/Core>
#include "sphere_grid.hpp"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <keyboard/Key.h>
#include <view_controller_msgs/CameraPlacement.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "helpers.h"
#include "lwrlib/LwrLibrary.hpp"
#include "tictoc.h"
#include "sphere_grid.hpp"

#include "AccMap.h"
#include "IKRay.h"

using namespace std;
using namespace Eigen;

#define RAND (((FLOAT)rand())/((FLOAT)RAND_MAX)*2.0-1.0) // Random value between -1 and 1

// main

int main( int argc, char *argv[] ){
        (void)argc; (void)argv; // suppress unused parameter warning

    bool doublePrecision = false;
    #ifdef DOUBLE_PRECISION
        doublePrecision = true;
    #endif


    printf("\ncompiling accessability map stuff for GPU...\n");
    AccMap accmap;

    printf("\nfilling arrays...\n");

    int nSamples = 10000;
    int nSamples64 = ((nSamples)/64+1)*64; // increased so 64 devides it
    int n = nSamples64;

    int16_t* errors = new int16_t[nSamples64];
    int16_t* intervals = new int16_t[64*nSamples64];
    float* mcCost = new float[nSamples64]; // cost for marching cubes
    FLOAT* targets = new FLOAT[16*nSamples64];
    uint8_t* configs = new uint8_t[nSamples64];
    int found;

    for (int i=0; i<nSamples64; i++){

        Eigen::Quaterniond q(1,1,1,1);

        while(q.norm()>1){
            q = Eigen::Quaterniond(RAND,RAND,RAND,RAND);
        }
        q.normalize();

        Eigen::Matrix3d T = q.toRotationMatrix();

        targets[i*16   ] = T(0,0);
        targets[i*16+ 1] = T(1,0);
        targets[i*16+ 2] = T(2,0);
        targets[i*16+ 3] = 0;

        targets[i*16+ 4] = T(0,1);
        targets[i*16+ 5] = T(1,1);
        targets[i*16+ 6] = T(2,1);
        targets[i*16+ 7] = 0;

        targets[i*16+ 8] = T(0,2);
        targets[i*16+ 9]= T(1,2);
        targets[i*16+10] = T(2,2);
        targets[i*16+11] = 0;

        targets[i*16+12] = RAND*0.9;
        targets[i*16+13] = RAND*0.9;
        targets[i*16+14] = RAND*0.9+0.31;
        targets[i*16+15] = 1;

    }

    FLOAT invToolOcl[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

    tic();

    accmap.compute(errors,
                   intervals,
                   mcCost,
                   (void*) targets,
                   configs,
                   (void*) invToolOcl,
                   nSamples64,
                   doublePrecision);

    toc();

    found=0;


    LwrXCart CPUtargets;
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

        CPUtargets.pose.pos = Eigen::Vector3d(targets[i*16+12],targets[i*16+13],targets[i*16+14]);
        CPUtargets.pose.ori << targets[i*16   ], targets[i*16+ 4], targets[i*16+ 8],
                               targets[i*16+ 1], targets[i*16+ 5], targets[i*16+ 9],
                               targets[i*16+ 2], targets[i*16+ 6], targets[i*16+10];

        CPUtargets.config = configs[i];
        CPUtargets.nsparam = 0;

        //Lwr::inverseKinematics(CPUjoints, CPUtargets);
        CPUerror[i] = Lwr::elbowIntervals(currentMargin,reachable,blocked,blockedPerJoint,CPUtargets);

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





        for(int k=0;k<n;k++){
            bool equal=true;
            for(int i=0; i<32; i++){
                if( abs(CPUintervals[k*64+i] - intervals[k*64+i])>7 ) equal=false;
            }

            if(!equal){
                COUT "trial nr. " << k; BR;
                COUT "targets:"; BR;
                COUT targets[k*16+ 0] << "  " << targets[k*16+ 4] << "  " << targets[k*16+ 8] << "  " << targets[k*16+ 12]; BR;
                COUT targets[k*16+ 1] << "  " << targets[k*16+ 5] << "  " << targets[k*16+ 9] << "  " << targets[k*16+ 13]; BR;
                COUT targets[k*16+ 2] << "  " << targets[k*16+ 6] << "  " << targets[k*16+10] << "  " << targets[k*16+ 14]; BR;
                COUT "Config: " << (int)configs[k]; BR;BR;

                COUT "CPU              GPU"; BR;
                COUT "Errormsg: " << CPUerror[k] << "  | Errormsg: " << errors[k]; BR;

                for(int i=0; i<32; i++){
                    COUT CPUintervals[k*64 + i*2] << ".." << CPUintervals[k*64 + i*2+1] << "  |  ";
                    COUT intervals[k*64 + i*2] << ".." << intervals[k*64 + i*2+1]; BR;

                    if(i==10 || i==13 || i==16 || i==19 || i==22 || i==25 || i==28)BR;
                }
                BR; BR;
            }
        }


        delete[] errors;
        delete[] intervals;
        delete[] targets;
        delete[] mcCost;
        delete[] configs;

        delete[] CPUintervals;
        delete[] CPUerror;

        return 0;
}
