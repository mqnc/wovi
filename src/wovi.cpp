
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
#include <ctime>
#include <ecl/geometry.hpp>

#include "AccMap.h"
#include "IKRay.h"

#define TASK 3 // 0: Random pose, 1: Pallet, 2: Rubik, 3: Gripper, 4: Trajectory
#define TRAJECTORY 1 // 1: door 2: knee
#define RANDPOSE 0
#define SUPPORT 1
#define CONFIG 0
#define ISO 0.3 // 0.3
#define CONST_ORI_ALPHA 0.4
#define TOTAL_ORI_ALPHA 0.6
#define TOTAL_ORI_SPREAD 30.0
#define TOOL_LEN_TASK_3 0.12
#define TOOL_LEN_TRAJECTORY 0.15

using namespace std;
using namespace Eigen;

////////////////////////////// global variables and callback functions for gamepad and keyboard and spacemouse

int joyButtonsState[32];
int joyButtonsStateBefore[32];
bool joyButtonsPressed[32];
double joyAxesState[16];
bool keyState[1024];
bool keyPressed[1024];
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
    for(int i=0; i < joy->buttons.size(); i++){
        joyButtonsState[i] = joy->buttons[i];
        if(joyButtonsState[i] == 1 && joyButtonsStateBefore[i] == 0)
            {joyButtonsPressed[i] = true;}
    }
    for(int i=0; i < joy->axes.size(); i++){joyAxesState[i] = joy->axes[i];}
}
void keydownCallback(const keyboard::Key::ConstPtr& key){
    keyState[key->code] = true;
    keyPressed[key->code] = true;
}
void keyupCallback(const keyboard::Key::ConstPtr& key){keyState[key->code] = false;}

int spacenavButtonsState[32];
int spacenavButtonsStateBefore[32];
bool  spacenavButtonsPressed[32];
double spacenavAxesState[6];
int spacenavDominant;
void spacenavCallback(const sensor_msgs::Joy::ConstPtr& joy){
    for(int i=0; i < joy->buttons.size(); i++){
        spacenavButtonsState[i] = joy->buttons[i];
        if(spacenavButtonsState[i] == 1 && spacenavButtonsStateBefore[i] == 0)
            {spacenavButtonsPressed[i] = true;}
    }
    for(int i=0; i < joy->axes.size(); i++){
        spacenavAxesState[i] = joy->axes[i];
        if(fabs(spacenavAxesState[i]) < 0.1){
            spacenavAxesState[i] = 0; // deadzone
        }
    }

    if(spacenavButtonsPressed[0]){
        spacenavButtonsPressed[0] = false;
        spacenavDominant = (spacenavDominant+1) % 3;
        printf("SpaceNavMode: %d\n", spacenavDominant);
    }

    if(spacenavDominant == 1){
        if(pow(spacenavAxesState[0],2) + pow(spacenavAxesState[1],2) + pow(spacenavAxesState[2],2) >
                pow(spacenavAxesState[3],2) + pow(spacenavAxesState[4],2) + pow(spacenavAxesState[5],2)){
            spacenavAxesState[3] = spacenavAxesState[4] = spacenavAxesState[5] = 0;
        }
        else{
            spacenavAxesState[0] = spacenavAxesState[1] = spacenavAxesState[2] = 0;
        }
    }

    if(spacenavDominant == 2){
        int domi = 0;
        double domivalue = 0;
        for(int i=0; i<6; i++){
            if(fabs(spacenavAxesState[i]) > fabs(domivalue)){
                domivalue = spacenavAxesState[i];
                domi = i;
            }
        }
        for(int i=0; i<6; i++){
            spacenavAxesState[i] = 0;
        }
        spacenavAxesState[domi] = domivalue;
    }
}

////////////////////////////// log stuff

FILE * pLogFile;
ros::Time t0;

////////////////////////////// main

//void sonar();
//void accmap();
void wovi();

int main( int argc, char** argv )
{
    spacenavDominant = 0;
    for(int i=0; i<32; i++){
        joyButtonsState[i] = 0;
        joyButtonsStateBefore[i] = 0;
        joyButtonsPressed[i] = false;
        spacenavButtonsState[i] = 0;
        spacenavButtonsStateBefore[i] = 0;
        spacenavButtonsPressed[i] = false;
    }
    for(int i=0; i<16; i++){
        joyAxesState[i] = 0;
    }
    for(int i=0; i<1024; i++){
        keyPressed[i] = false;
    }
    joyAxesState[2] = joyAxesState[5] = 1.0;
    for(int i=0; i<1024; i++){keyState[i] = false;}

    ros::init(argc, argv, "workspace_visualization");

    //sonar();
    //accmap();

    // LOG:

    char fname[128];

    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    sprintf(fname, "/home/mirko/Diss/userstudy/log/%04d_%02d_%02d__%02d_%02d_%02d.txt",
           now->tm_year + 1900, now->tm_mon + 1, now->tm_mday,
           now->tm_hour, now->tm_min, now->tm_sec);

    pLogFile = fopen(fname, "w");

    fprintf(pLogFile, "Task\n%d\nPose number\n%d\nSupport\n%d\n", TASK, RANDPOSE, SUPPORT);

    wovi();

    fprintf(pLogFile, "\n\n- - - logfile properly closed\n");
    fclose(pLogFile);

    return 0;
}


void wovi(){

    ////////////////////////////// create publishers and subscribers

    ros::NodeHandle node;
    ros::Subscriber subjoy = node.subscribe("joy", 5, joyCallback);
    ros::Subscriber subspacenav = node.subscribe("spacenav/joy", 5, spacenavCallback);
    ros::Subscriber subkeydn = node.subscribe("keyboard/keydown", 5, keydownCallback);
    ros::Subscriber subkeyup = node.subscribe("keyboard/keyup", 5, keyupCallback);

    double dt = 1.0/10.0;//1.0/10.0;
    double t = 0;
    ros::Rate rate(1.0/dt);
    ros::Publisher publisherSonar = node.advertise <visualization_msgs::Marker> ("sonar", 1);
    ros::Publisher publisherOriSonar = node.advertise <visualization_msgs::Marker> ("orisonar", 5);
    ros::Publisher publisherRobot = node.advertise <visualization_msgs::Marker> ("robot", 1);
    ros::Publisher publisherTool = node.advertise <visualization_msgs::Marker> ("tool", 1);
    ros::Publisher publisherJointLabels = node.advertise <visualization_msgs::Marker> ("jointlabels", 7);
    ros::Publisher publisherElbowCost = node.advertise <visualization_msgs::Marker> ("elbowcost", 2);
    ros::Publisher publisherCam = node.advertise <view_controller_msgs::CameraPlacement>("rviz/camera_placement", 1);
    ros::Publisher publisherModel = node.advertise <sensor_msgs::JointState> ("joint_states", 1);
    ros::Publisher publisherAccMap = node.advertise <visualization_msgs::Marker> ("workspace", 1);
    ros::Publisher publisherIntersection = node.advertise <visualization_msgs::Marker> ("wsintersection", 1);
    ros::Publisher publisherTestOris = node.advertise <visualization_msgs::Marker> ("test_oris", 1);
    ros::Publisher publisherWorkpiece = node.advertise <visualization_msgs::Marker> ("workpiece", 1);
    ros::Publisher publisherWorkpiece2 = node.advertise <visualization_msgs::Marker> ("targetworkpieces", 5);
    ros::Publisher publisherSignalTable = node.advertise <visualization_msgs::Marker> ("signalTable", 1);
    ros::Publisher publisherTrajectory1 = node.advertise <visualization_msgs::Marker> ("trajectory1", 1);
    ros::Publisher publisherTrajectory2 = node.advertise <visualization_msgs::Marker> ("trajectory2", 1);
    ros::Publisher publisherTrajectory3 = node.advertise <visualization_msgs::Marker> ("trajectory3", 1);
    ros::Publisher publisherTrajectory4 = node.advertise <visualization_msgs::Marker> ("trajectory4", 1);

    bool autoElbow = true;
    bool transparent = true;

    ////////////////////////////// initialize test trajectory + marker

    ecl::Array<double> trajtnsp1(5);
    ecl::Array<double> trajnsp1(5);
    trajtnsp1 << 0,   0.33, 0.5, 0.78, 1;
    trajnsp1 <<  0.8, 0.47, 1.1, 0.38, 0.8;
    LwrFrame trajOffset1;
    trajOffset1.pos << 0.12, 0.5, 0.32;
    trajOffset1.ori << -0.95511, 0.25996, 0.14208, 0.075655, -0.24965, 0.96538, 0.28643, 0.93279, 0.21878;

    ecl::Array<double> trajtnsp2(5);
    ecl::Array<double> trajnsp2(5);
    trajtnsp2 << 0,    0.16, 0.34, 0.79, 1;
    trajnsp2 <<  0.61, 0.73, 0.36, 0.27, 0.61;
    LwrFrame trajOffset2;
    trajOffset2.pos << 0.46315, 0.14827, 0.27101;
    trajOffset2.ori << -0.53439, 0.11614, 0.83722, 0.63882, -0.59311, 0.49003, 0.55348, 0.7967, 0.24275;

/*    ecl::Array<double> trajtnsp3(2);
    ecl::Array<double> trajnsp3(2);
    trajtnsp3 << 0, 1;
    trajnsp3 <<  -1,-1;
    LwrFrame trajOffset3;

    trajOffset3.pos << 0.59594, -0.067714, 0.016276;
    trajOffset3.ori << -0.92392, -0.13685, 0.35728, 0.020105, -0.94992, -0.31185, 0.38206, -0.28095, 0.8804;
*/

    ecl::Array<double> trajtnsp3(2);
    ecl::Array<double> trajnsp3(2);
    trajtnsp3 << 0, 1;
    trajnsp3 <<  1.3,1.1;
    LwrFrame trajOffset3;

    trajOffset3.pos << 0.36079, 0.43761, 0.19578;
    trajOffset3.ori << -0.51815, -0.84739, 0.116, -0.82344, 0.45758, -0.33552, 0.23124, -0.26936, -0.93486;


    ecl::Array<double> trajtnsp4(2);
    ecl::Array<double> trajnsp4(2);
    trajtnsp4 << 0, 1;
    trajnsp4 <<  0.38, 0.38;
    LwrFrame trajOffset4;
    trajOffset4.pos << -0.32039, -0.4835, 0.15357;
    trajOffset4.ori << -0.8051, 0.49091, 0.3329, 0.5907, 0.71446, 0.375, -0.053752, 0.49856, -0.86519;

    ecl::Array<double> trajt12(9);
    ecl::Array<double> trajx12(9);
    ecl::Array<double> trajy12(9);
    ecl::Array<double> trajz12(9);
    ecl::Array<double> trajqx12(9);
    ecl::Array<double> trajqy12(9);
    ecl::Array<double> trajqz12(9);
    ecl::Array<double> trajqw12(9);

    trajt12 << 0,    0.1429,    0.1607,    0.3393,    0.4821,    0.6250,    0.6429,    0.7857,    1.0000;
    trajx12 << -0.3500,   -0.3500,   -0.3500,   -0.1000,    0.3500,    0.3500,    0.3500,    0.3500,   -0.3500;
    trajy12 << -0.4000,         0,         0,    0.4000,    0.4000,         0,         0,   -0.4000,   -0.4000;
    trajz12 << 0,         0,         0,    0.1500,    0.1500,         0,         0,         0,         0;
    trajqx12 << 0,         0,    0.1740,    0.1740,    0.1740,    0.1740,         0,         0,         0;
    trajqy12 << 0,0,0,0,0,0,0,0,0;
    trajqz12 << 0,0,0,0,0,0,0,0,0;
    trajqw12 << 1.0000,    1.0000,    0.9850,    0.9850,    0.9850,    0.9850,    1.0000,    1.0000,    1.0000;

    ecl::Array<double> trajt34(20);
    ecl::Array<double> trajx34(20);
    ecl::Array<double> trajy34(20);
    ecl::Array<double> trajz34(20);
    ecl::Array<double> trajqx34(20);
    ecl::Array<double> trajqy34(20);
    ecl::Array<double> trajqz34(20);
    ecl::Array<double> trajqw34(20);

    trajt34 << 0, 0.0787, 0.1024, 0.1811, 0.2047, 0.2835, 0.3071, 0.3858, 0.4094, 0.4882, 0.5118, 0.5906, 0.6142, 0.6929, 0.7165, 0.7953, 0.8189, 0.8976, 0.9213, 1;
    trajx34 << -0.064, 0, 0, -0.05, -0.05, 0, 0, -0.05, -0.05, 0, 0, -0.05, -0.05, 0, 0, -0.05, -0.05, 0, 0, -0.05;
    trajy34 << 0, 0, 0.0175, 0.0175, 0.0225, 0.0225, 0.0350, 0.0350, 0.0375, 0.0375, 0.0375, 0.0375, 0.0350, 0.0350, 0.0225, 0.0225, 0.0175, 0.0175, 0, 0;
    trajz34 << 0, 0, 0, 0, 0.0025, 0.0025, 0.0150, 0.0150, 0.0200, 0.0200, 0.0375, 0.0375, 0.0425, 0.0425, 0.0550, 0.0550, 0.0575, 0.0575, 0.0575, 0.0575;
    trajqx34 << 0, 0, 0, 0, 0.3827, 0.3827, 0.3827, 0.3827, 0.7071, 0.7071, 0.7071, 0.7071, 0.9239, 0.9239, 0.9239, 0.9239, 1, 1, 1, 1;
    trajqy34 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    trajqz34 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    trajqw34 << 1, 1, 1, 1, 0.9239, 0.9239, 0.9239, 0.9239, 0.7071, 0.7071, 0.7071, 0.7071, 0.3827, 0.3827, 0.3827, 0.3827, 0, 0, 0, 0;



    double curv = 20000;

    ecl::SmoothLinearSpline splnx12, splny12, splnz12, splnqx12, splnqy12, splnqz12, splnqw12, splnnsp1, splnnsp2;
    splnx12 = ecl::SmoothLinearSpline(trajt12, trajx12, curv);
    splny12 = ecl::SmoothLinearSpline(trajt12, trajy12, curv);
    splnz12 = ecl::SmoothLinearSpline(trajt12, trajz12, curv);
    splnqx12 = ecl::SmoothLinearSpline(trajt12, trajqx12, curv);
    splnqy12 = ecl::SmoothLinearSpline(trajt12, trajqy12, curv);
    splnqz12 = ecl::SmoothLinearSpline(trajt12, trajqz12, curv);
    splnqw12 = ecl::SmoothLinearSpline(trajt12, trajqw12, curv);
    splnnsp1 = ecl::SmoothLinearSpline(trajtnsp1, trajnsp1, curv);
    splnnsp2 = ecl::SmoothLinearSpline(trajtnsp2, trajnsp2, curv);

    ecl::SmoothLinearSpline splnx34, splny34, splnz34, splnqx34, splnqy34, splnqz34, splnqw34, splnnsp3, splnnsp4;
    splnx34 = ecl::SmoothLinearSpline(trajt34, trajx34, curv);
    splny34 = ecl::SmoothLinearSpline(trajt34, trajy34, curv);
    splnz34 = ecl::SmoothLinearSpline(trajt34, trajz34, curv);
    splnqx34 = ecl::SmoothLinearSpline(trajt34, trajqx34, curv);
    splnqy34 = ecl::SmoothLinearSpline(trajt34, trajqy34, curv);
    splnqz34 = ecl::SmoothLinearSpline(trajt34, trajqz34, curv);
    splnqw34 = ecl::SmoothLinearSpline(trajt34, trajqw34, curv);
    splnnsp3 = ecl::SmoothLinearSpline(trajtnsp3, trajnsp3, curv);
    splnnsp4 = ecl::SmoothLinearSpline(trajtnsp4, trajnsp4, curv);

    double splnParam = 0;
    bool onTrajectory = false;
    int currentTrajectory = TRAJECTORY;
    int trajcfg = 0;

    visualization_msgs::Marker markerTrajectory1 =
            initializeMarker(131, visualization_msgs::Marker::LINE_STRIP, 500);
    markerTrajectory1.scale.x = markerTrajectory1.scale.y = markerTrajectory1.scale.z = 0.01;

    visualization_msgs::Marker markerTrajectory2 =
            initializeMarker(132, visualization_msgs::Marker::LINE_STRIP, 500);
    markerTrajectory2.scale.x = markerTrajectory2.scale.y = markerTrajectory2.scale.z = 0.01;

    visualization_msgs::Marker markerTrajectory3 =
            initializeMarker(133, visualization_msgs::Marker::LINE_STRIP, 500);
    markerTrajectory3.scale.x = markerTrajectory3.scale.y = markerTrajectory3.scale.z = 0.003;

    visualization_msgs::Marker markerTrajectory4 =
            initializeMarker(134, visualization_msgs::Marker::LINE_STRIP, 500);
    markerTrajectory4.scale.x = markerTrajectory4.scale.y = markerTrajectory4.scale.z = 0.003;

    for(int s=0; s<500; s++){
        LwrFrame buffy;
        buffy.pos = Vector3d(splnx12(1.0/499.0*s), splny12(1.0/499.0*s), splnz12(1.0/499.0*s));
        buffy = trajOffset1 * buffy;

        markerTrajectory1.points[s].x = buffy.pos(0);
        markerTrajectory1.points[s].y = buffy.pos(1);
        markerTrajectory1.points[s].z = buffy.pos(2);
        markerTrajectory1.colors[s] = colorFromEigen(Vector4d(0.2,0.8,0.6,1.0));
    }

    for(int s=0; s<500; s++){
        LwrFrame buffy;
        buffy.pos = Vector3d(splnx12(1.0/499.0*s), splny12(1.0/499.0*s), splnz12(1.0/499.0*s));
        buffy = trajOffset2 * buffy;

        markerTrajectory2.points[s].x = buffy.pos(0);
        markerTrajectory2.points[s].y = buffy.pos(1);
        markerTrajectory2.points[s].z = buffy.pos(2);
        markerTrajectory2.colors[s] = colorFromEigen(Vector4d(0.2,0.6,0.9,1.0));
    }

    for(int s=0; s<500; s++){
        LwrFrame buffy;
        buffy.pos = Vector3d(splnx34(1.0/499.0*s), splny34(1.0/499.0*s), splnz34(1.0/499.0*s));
        buffy = trajOffset3 * buffy;

        markerTrajectory3.points[s].x = buffy.pos(0);
        markerTrajectory3.points[s].y = buffy.pos(1);
        markerTrajectory3.points[s].z = buffy.pos(2);
        markerTrajectory3.colors[s] = colorFromEigen(Vector4d(0.6,0.6,0.9,1.0));
    }

    for(int s=0; s<500; s++){
        LwrFrame buffy;
        buffy.pos = Vector3d(splnx34(1.0/499.0*s), splny34(1.0/499.0*s), splnz34(1.0/499.0*s));
        buffy = trajOffset4 * buffy;

        markerTrajectory4.points[s].x = buffy.pos(0);
        markerTrajectory4.points[s].y = buffy.pos(1);
        markerTrajectory4.points[s].z = buffy.pos(2);
        markerTrajectory4.colors[s] = colorFromEigen(Vector4d(0.6,0.8,0.6,1.0));
    }

    ////////////////////////////// initialize markers for sonar (ik probe rays)

    bool sonarUseTriangles = true;

    visualization_msgs::Marker markerSonar;

    int splitFactor = 10;
    int nSonarRays = sphere_icos_point_num(splitFactor);
    printf("SonarRays: %d\n", nSonarRays);

    // geodesic sphere

    printf("\n\ncreating geodesic sphere for probe directions using qconvex (qhull must be installed!)...\n");

    double *sphereGridPoints = new double[nSonarRays*3];
    sphereGridPoints = sphere_icos2_points(splitFactor, nSonarRays);

    // calculate faces using external program qhull (qconvex)

    stringstream ss;
    ss << "echo \"3 " << nSonarRays << "\n";

    for(int i=0; i<nSonarRays; i++){
        ss << sphereGridPoints[i*3  ] << " ";
        ss << sphereGridPoints[i*3+1] << " ";
        ss << sphereGridPoints[i*3+2] << "\n";
    }
    ss << "\" | qconvex i";

    //cout << "\n\n" << ss.str() << "\n\n";

    FILE *fp;

    fp = popen(ss.str().c_str(), "r");
    if (fp == NULL){printf("Failed to run command\n" ); exit(1);}

    int nFaces;
    fscanf(fp, "%d", &nFaces);
    vector<int> geoFaces(nFaces*3, -1);
    int i1, i2, i3;
    for(int i=0; i<nFaces; i++){
        fscanf(fp, "%d %d %d\n", &i1, &i2, &i3);
        geoFaces[i*3  ] = i1;
        geoFaces[i*3+1] = i2;
        geoFaces[i*3+2] = i3;
    }
    pclose(fp);

    if(!sonarUseTriangles){
        markerSonar = initializeMarker(0, visualization_msgs::Marker::POINTS, nSonarRays);
        markerSonar.scale.x = markerSonar.scale.y = markerSonar.scale.z = 0.01;
    }
    else{
        markerSonar = initializeMarker(0, visualization_msgs::Marker::TRIANGLE_LIST, nFaces*3);
    }

    ////////////////////////////// initialize line marker for robot and joints for publishing

    visualization_msgs::Marker markerRobot =
            initializeMarker(1, visualization_msgs::Marker::LINE_STRIP, 7);
    markerRobot.scale.x = markerRobot.scale.y = markerRobot.scale.z = 0.04;

    sensor_msgs::JointState jointsPublish;
    jointsPublish.position.resize(7);
    jointsPublish.name.resize(7);
    jointsPublish.name[0] = "lwr1_base_joint";
    jointsPublish.name[1] = "lwr1_shoulder_joint";
    jointsPublish.name[2] = "lwr1_upperarm_joint";
    jointsPublish.name[3] = "lwr1_elbow_joint";
    jointsPublish.name[4] = "lwr1_lowerarm_joint";
    jointsPublish.name[5] = "lwr1_wrist_joint";
    jointsPublish.name[6] = "lwr1_flange_joint";

    ////////////////////////////// initialize text markers for robot joints

    visualization_msgs::Marker markerJoints[7];
    for(int j=0; j<7; j++){
        markerJoints[j] = initializeMarker(10+j, visualization_msgs::Marker::TEXT_VIEW_FACING, 1);
        markerJoints[j].scale.x = markerJoints[j].scale.y = markerJoints[j].scale.z = 0.04;
        markerJoints[j].color = colorFromEigen(Vector4d(0.2,1,0,1));
    }

    ////////////////////////////// initialize triangle marker for elbow cost function

    int nElbowCostSteps = 256;
    double elbowCostR1 = 0.5;
    double elbowCostR2 = 0.6;

    visualization_msgs::Marker markerElbowCost =
            initializeMarker(2, visualization_msgs::Marker::LINE_STRIP, nElbowCostSteps+1);
    markerElbowCost.scale.x = markerElbowCost.scale.y = markerElbowCost.scale.z = 0.02;

    visualization_msgs::Marker markerElbowZero =
            initializeMarker(3, visualization_msgs::Marker::LINE_STRIP, nElbowCostSteps+1);
    markerElbowZero.scale.x = markerElbowZero.scale.y = markerElbowZero.scale.z = 0.004;

    for(int iv=0; iv<nElbowCostSteps+1; iv++){
        markerElbowZero.colors[iv].r = 0.8; markerElbowZero.colors[iv].g = 0.9; markerElbowZero.colors[iv].b = 1.0;
    }

    ////////////////////////////// initialize triangle marker for signal table

    visualization_msgs::Marker markerSignalTable =
            initializeMarker(77, visualization_msgs::Marker::TRIANGLE_LIST, 6);
    markerSignalTable.points[0] = positionFromEigen(Vector3d(-1,-1,0));
    markerSignalTable.points[1] = positionFromEigen(Vector3d(1,-1,0));
    markerSignalTable.points[2] = positionFromEigen(Vector3d(1,1,0));
    markerSignalTable.points[3] = positionFromEigen(Vector3d(-1,-1,0));
    markerSignalTable.points[4] = positionFromEigen(Vector3d(1,1,0));
    markerSignalTable.points[5] = positionFromEigen(Vector3d(-1,1,0));
    markerSignalTable.colors.resize(0);

    ////////////////////////////// initialize tool and workpiece

    LwrFrame tool;
    visualization_msgs::Marker markerTool = initializeMarker(37, visualization_msgs::Marker::MESH_RESOURCE, 0);
    visualization_msgs::Marker markerWorkpiece = initializeMarker(1000, visualization_msgs::Marker::MESH_RESOURCE, 0);

    double deg = M_PI/180.0;
    LwrJoints startJoints;
    startJoints.setJoints(-90*deg, 45*deg, 0, -90*deg, 0, 45*deg, 0);
    startJoints.setJoints(-45*deg, 90*deg, 90*deg, 90*deg, 90*deg, -90*deg, -45*deg);
    startJoints.setJoints(-90*deg, 45*deg, 0, -90*deg, 0, -45*deg, 0);
    startJoints.setJoints(-90*deg, -30*deg, 0, 120*deg, 0, 60*deg, 0);

    LwrFrame workpiecePose;
    workpiecePose.setPos(-0.6,0.5,0.3);

    if(TASK==0){

        tool.setPos(0,0,0.10);
        tool.setQuat(1,0,0,0);

        for(int i=0; i<(RANDPOSE%20)*10; i++){RAND;}
        startJoints.setJoints(170*RANDDEG,
                              120*RANDDEGPOS,
                              170*RANDDEG,
                              120*RANDDEGPOS,
                              170*RANDDEG,
                              120*RANDDEGPOS,
                              170*RANDDEG);
        LwrJoints jtarget(170*RANDDEG,
                          90*RANDDEGPOS+30*DEG,
                          170*RANDDEG,
                          70*RANDDEGPOS+50*DEG,
                          170*RANDDEG,
                          90*RANDDEGPOS+30*DEG,
                          170*RANDDEG);

        if(SUPPORT){ // haha, SUPPORT actually means that we flip the situation
            for(int j=0; j<7; j+=2){
                startJoints[j] *= -1;
                jtarget[j] *= -1;
            }
        }

        cout << "startjoints: " << startJoints << endl << "targetjoints: " << jtarget << endl;

        LwrXCart posetarget;
        Lwr::forwardKinematics(posetarget, jtarget);
        //if(posetarget.config != CONFIG){printf("WARNING! Config does not match, target may be unreachable");}

        workpiecePose = posetarget.pose * tool;

        markerTool.mesh_resource = "";
        markerTool.mesh_use_embedded_materials = true;
        markerTool.color = colorFromEigen(Vector4d(0.4,0.3,0.2,0.6));
        markerTool.scale.x = markerTool.scale.y = markerTool.scale.z = 0.9;

        markerWorkpiece.mesh_resource = "";
        markerWorkpiece.mesh_use_embedded_materials = true;
        markerWorkpiece.color = colorFromEigen(Vector4d(0.6,0.5,0.4,1));
        markerWorkpiece.scale.x = markerWorkpiece.scale.y = markerWorkpiece.scale.z = 0.9;
    }
    if(TASK==1){

        tool.setPos(0,0,0.18);
        tool.setQuat(0,0,1,0);

        markerTool.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/box.dae";
        markerTool.mesh_use_embedded_materials = true;
        markerTool.color = colorFromEigen(Vector4d(0.4,0.3,0.2,0.6));

        markerWorkpiece.mesh_use_embedded_materials = true;
        markerWorkpiece.color = colorFromEigen(Vector4d(0.6,0.5,0.4,0.8));

        if(SUPPORT == 1){
            markerWorkpiece.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/pallet9.dae";
            markerTool.scale.x = 0.6; markerTool.scale.y = 0.5; markerTool.scale.z = 0.6;
            markerWorkpiece.scale.x = 0.6; markerWorkpiece.scale.y = 0.5; markerWorkpiece.scale.z = 0.6;
            //markerTool.scale.x = markerTool.scale.y = markerTool.scale.z = 0.9;
            //markerWorkpiece.scale.x = markerWorkpiece.scale.y = markerWorkpiece.scale.z = 0.9;
        }
        else{
            markerWorkpiece.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/pallet8.dae";
            markerTool.scale.x = markerTool.scale.y = 1.5;
        }

    }
    if(TASK==2){

        if(SUPPORT == 0){
            workpiecePose.setQuat(-0.8205,-0.4247,0.1759,-0.3399);
        }

        tool.setPos(0,0,0.08);
        tool.setQuat(1,0,0,0);

        markerTool.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/rubiktool.dae";
        markerTool.mesh_use_embedded_materials = true;
        markerTool.color = colorFromEigen(Vector4d(0.1,0.3,1,1));

        markerWorkpiece.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/rubiktool.dae";
        markerWorkpiece.mesh_use_embedded_materials = true;
        markerWorkpiece.color = colorFromEigen(Vector4d(0.1,0.3,1,1));
        //markerWorkpiece.scale.x = markerWorkpiece.scale.y = markerWorkpiece.scale.z = 0.1;
    }
    if(TASK==3){

        tool.setPos(TOOL_LEN_TASK_3,0,0.02);
        tool.setQuat(0.5*sqrt(2),0,0.5*sqrt(2),0);
        //tool.setQuat(1,0,0,0);

        markerTool.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/gripper.dae";
        markerTool.mesh_use_embedded_materials = true;
        markerTool.color = colorFromEigen(Vector4d(0.75,0.75,0.75,1));
        markerTool.scale.x = markerTool.scale.y = 0.33333;
        markerTool.scale.z = TOOL_LEN_TASK_3;

        markerWorkpiece.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/rubik.dae";
        markerWorkpiece.mesh_use_embedded_materials = true;
        markerWorkpiece.color = colorFromEigen(Vector4d(0.1,0.3,1,1));
        markerWorkpiece.scale.x = markerWorkpiece.scale.y = markerWorkpiece.scale.z = 0.1;
    }
    if(TASK==4){

        tool.setPos(TOOL_LEN_TRAJECTORY,0,0.02);
        tool.setQuat(0.5*sqrt(2),0,0.5*sqrt(2),0);
        //tool.setQuat(1,0,0,0);ffff

        markerTool.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/gripper.dae";
        markerTool.mesh_use_embedded_materials = true;
        markerTool.color = colorFromEigen(Vector4d(0.75,0.75,0.75,1));
        markerTool.scale.x = markerTool.scale.y = 0.33333;
        markerTool.scale.z = TOOL_LEN_TRAJECTORY;

        markerWorkpiece.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/gripper.dae";
        markerWorkpiece.mesh_use_embedded_materials = true;
        markerWorkpiece.color = colorFromEigen(Vector4d(0.75,0.75,0.75,1));
        markerWorkpiece.scale.x = markerWorkpiece.scale.y = 0.33333;
        markerWorkpiece.scale.z = TOOL_LEN_TRAJECTORY;
    }

    fprintf(pLogFile, "Tool\n[%f, %f, %f, %f; %f, %f, %f, %f; %f, %f, %f, %f]\n",
            tool.ori(0,0), tool.ori(0,1), tool.ori(0,2), tool.pos(0),
            tool.ori(1,0), tool.ori(1,1), tool.ori(1,2), tool.pos(1),
            tool.ori(2,0), tool.ori(2,1), tool.ori(2,2), tool.pos(2));


    ////////////////////////////// initialize orientation targets and line marker for orientation checks

    int nOriSonarRays = 128+2; // tilting about 32 axes and rolling to the left and to the right
    Quaterniond *targetOrisRel = new Quaterniond[nOriSonarRays];

    double oriPhiMax = 0.5*M_PI;

    for(int i=0; i<nOriSonarRays-2; i++){
        double yaw = (double)i / ((double)nOriSonarRays-2) *2.0*M_PI;
        targetOrisRel[i] = AngleAxisd(oriPhiMax, Vector3d(cos(yaw), sin(yaw), 0) );
    }

    targetOrisRel[nOriSonarRays-2] = AngleAxisd( -oriPhiMax, Vector3d(0,0,1) );
    targetOrisRel[nOriSonarRays-1] = AngleAxisd( +oriPhiMax, Vector3d(0,0,1) );

    visualization_msgs::Marker markerOriSonar =
            initializeMarker(4, visualization_msgs::Marker::LINE_STRIP, nOriSonarRays-2+1);
    markerOriSonar.scale.x = markerOriSonar.scale.y = markerOriSonar.scale.z = 0.01;
    markerOriSonar.color.a = 0.99; // so per vertex alpha is active

    visualization_msgs::Marker markerRollSonar =
            initializeMarker(5, visualization_msgs::Marker::LINE_LIST, 8);
    markerRollSonar.scale.x = markerRollSonar.scale.y = markerRollSonar.scale.z = 0.01;
    markerRollSonar.color.a = 0.99; // so per vertex alpha is active

    visualization_msgs::Marker markerOriSonarSphere =
            initializeMarker(6, visualization_msgs::Marker::SPHERE, 1);
    markerOriSonarSphere.scale.x = markerOriSonarSphere.scale.y = markerOriSonarSphere.scale.z = 0.5;

    markerOriSonarSphere.pose.position.z = 0.31;
    markerOriSonarSphere.color = colorFromEigen(Vector4d(1,1,1,0.5));
    markerOriSonarSphere.pose.orientation = orientationFromEigen(Vector4d(0.5,0.5,0.5,0.5));

    ////////////////////////////// create test orientations

    Vector3d ex=Vector3d::UnitX(); Vector3d ey=Vector3d::UnitY(); Vector3d ez=Vector3d::UnitZ();

    vector<Quaterniond> testOris(0);
    visualization_msgs::Marker markerTestOris = initializeMarker(30, visualization_msgs::Marker::TRIANGLE_LIST, 0);
    Matrix3d testOriParent = Matrix3d::Identity();
    testOriParent = AngleAxisd(-M_PI/2, ez)*AngleAxisd(-M_PI/2, ey);

/*  testOris.push_back(Quaterniond(1,0,0,0));
    testOris.push_back(Quaterniond(0,1,0,0));
    testOris.push_back(Quaterniond(0,0,1,0));
    testOris.push_back(Quaterniond(0,0,0,1));*/


    if(TOTAL_ORI_SPREAD > 0 && TASK == 3){

        double alpha = TOTAL_ORI_SPREAD * M_PI/180.0;
        double beta = 0;

        Matrix3d m;
        Quaterniond q;

        for(int i=-1; i<=1; i++){
            beta = i*alpha;

            m=AngleAxisd(beta, ez); q=m; testOris.push_back(q);
            m=AngleAxisd(alpha, ( ex   ).normalized())*AngleAxisd(beta, ez); q=m; testOris.push_back(q);
            m=AngleAxisd(alpha, ( ex+ey).normalized())*AngleAxisd(beta, ez); q=m; testOris.push_back(q);
            m=AngleAxisd(alpha, (    ey).normalized())*AngleAxisd(beta, ez); q=m; testOris.push_back(q);
            m=AngleAxisd(alpha, (-ex+ey).normalized())*AngleAxisd(beta, ez); q=m; testOris.push_back(q);
            m=AngleAxisd(alpha, (-ex   ).normalized())*AngleAxisd(beta, ez); q=m; testOris.push_back(q);
            m=AngleAxisd(alpha, (-ex-ey).normalized())*AngleAxisd(beta, ez); q=m; testOris.push_back(q);
            m=AngleAxisd(alpha, (   -ey).normalized())*AngleAxisd(beta, ez); q=m; testOris.push_back(q);
            m=AngleAxisd(alpha, ( ex-ey).normalized())*AngleAxisd(beta, ez); q=m; testOris.push_back(q);
        }
    }

    if(TASK == 2)
    {
        Quaterniond q;
        /*q = Quaterniond(1, 0, 0, 0).normalized(); testOris.push_back(q);
        q = Quaterniond(0, 1, 0, 0).normalized(); testOris.push_back(q);
        q = Quaterniond(0, 0, 1, 0).normalized(); testOris.push_back(q);
        q = Quaterniond(0, 0, 0, 1).normalized(); testOris.push_back(q);
        q = Quaterniond(0, 0, -0.7071, 0.7071).normalized(); testOris.push_back(q);
        q = Quaterniond(0, 0, 0.7071, 0.7071).normalized(); testOris.push_back(q);
        q = Quaterniond(0, -0.7071, 0, 0.7071).normalized(); testOris.push_back(q);
        q = Quaterniond(0.7071, 0, 0.7071, 0).normalized(); testOris.push_back(q);
        q = Quaterniond(0.7071, 0.7071, 0, 0).normalized(); testOris.push_back(q);
        q = Quaterniond(-0.7071, 0.7071, 0, 0).normalized(); testOris.push_back(q);
        q = Quaterniond(0, 0.7071, 0, 0.7071).normalized(); testOris.push_back(q);
        q = Quaterniond(-0.7071, 0, 0.7071, 0).normalized(); testOris.push_back(q);
        q = Quaterniond(-0.7071, 0, 0, 0.7071).normalized(); testOris.push_back(q);
        q = Quaterniond(0, -0.7071, 0.7071, 0).normalized(); testOris.push_back(q);
        q = Quaterniond(0.7071, 0, 0, 0.7071).normalized(); testOris.push_back(q);
        q = Quaterniond(0, 0.7071, 0.7071, 0).normalized(); testOris.push_back(q);
        q = Quaterniond(-0.5, 0.5, 0.5, -0.5).normalized(); testOris.push_back(q);
        q = Quaterniond(-0.5, 0.5, 0.5, 0.5).normalized(); testOris.push_back(q);
        q = Quaterniond(0.5, 0.5, 0.5, 0.5).normalized(); testOris.push_back(q);
        q = Quaterniond(0.5, -0.5, 0.5, -0.5).normalized(); testOris.push_back(q);
        q = Quaterniond(-0.5, -0.5, 0.5, 0.5).normalized(); testOris.push_back(q);
        q = Quaterniond(-0.5, -0.5, 0.5, -0.5).normalized(); testOris.push_back(q);
        q = Quaterniond(0.5, 0.5, 0.5, -0.5).normalized(); testOris.push_back(q);
        q = Quaterniond(0.5, -0.5, 0.5, 0.5).normalized(); testOris.push_back(q);*/

        q = Quaterniond(0.5,0.5,0.5,0.5);testOris.push_back(q);
        q = Quaterniond(-0.5,-0.5,0.5,0.5);testOris.push_back(q);
        q = Quaterniond(-0.5,-0.5,0.5,-0.5);testOris.push_back(q);
        q = Quaterniond(0.5,-0.5,0.5,0.5);testOris.push_back(q);
        q = Quaterniond(0,1,0,0);testOris.push_back(q);
        q = Quaterniond(0,0,0,1);testOris.push_back(q);
    }

    vector<Vector3d> arrowMesh(0);
    arrowMesh.push_back(Vector3d(0,0,0));
    arrowMesh.push_back(Vector3d(0,0,-1));
    arrowMesh.push_back(Vector3d(0.1,0,-1));
    arrowMesh.push_back(Vector3d(0,0,0));
    arrowMesh.push_back(Vector3d(0.1,0,-1));
    arrowMesh.push_back(Vector3d(0.1,0,-0.8));
    arrowMesh.push_back(Vector3d(0,0,0));
    arrowMesh.push_back(Vector3d(0.1,0,-0.8));
    arrowMesh.push_back(Vector3d(0.2,0,-0.83));

    ////////////////////////////// configure camera position

    view_controller_msgs::CameraPlacement cam;
    double camyaw = 0;
    double campitch = 0;
    double camdst = 3;
    cam.interpolation_mode = 1;
    cam.target_frame = "";
    cam.time_from_start.sec = 0;
    cam.time_from_start.nsec = 1;//20*1e6;
    cam.eye.point.x = 3; cam.eye.point.y = 2; cam.eye.point.z = 1;
    cam.focus.point.x = 0; cam.focus.point.y = 0; cam.focus.point.z = 0.31;
    cam.up.vector.x = 0; cam.up.vector.y = 0; cam.up.vector.z = 1;
    cam.mouse_interaction_mode = 0;
    cam.interaction_disabled = false;
    cam.allow_free_yaw_axis = false;

    vector<Quaterniond> snapOriList = generateOris();

    ////////////////////////////// prepare sonar GPU stuff

    printf("compiling sonar stuff for GPU...\n");
    IKRay ikray;

    printf("\nfilling arrays...\n");
    int nSonarRays64 = ((nSonarRays+nOriSonarRays)/64+1)*64; // increased so 64 devides it

    FLOAT* endParams = new FLOAT[nSonarRays64];
    FLOAT* rayEnds = new FLOAT[nSonarRays64*4]; // cl_double3 is the same as cl_double4
    FLOAT* rayColors = new FLOAT[nSonarRays64*4];

    FLOAT* tcpPosStart = new FLOAT[4];
    FLOAT* tcpOriStart = new FLOAT[4];

    LwrFrame invTool = tool.inverse();

    cout << "Tool:" << endl << tool << endl << endl;

    FLOAT* invToolOcl = new FLOAT[16];

    oclFromFrame(invToolOcl, &invTool);

    LwrFrame currentTcp;
    LwrXCart currentPose;

    //startJoints.setJoints(36.0*deg,33.0*deg,0.0,-78.0*deg,-4.0*deg,29.0*deg,109.0*deg); debugIndex -> 647

    uint32_t debugIndex = 382;

    Lwr::forwardKinematics(currentPose, startJoints);
    currentTcp = currentPose.pose * tool;

    //currentTcp.setQuat(0.0, 0.0, 1.0, 0.0);
    //currentTcp.setPos(-0.6,0.0,0.3);

    currentPose.pose = currentTcp*invTool;
    tcpPosStart[0]=currentTcp.pos.x(); tcpPosStart[1]=currentTcp.pos.y(); tcpPosStart[2]=currentTcp.pos.z();
    double buffer4[4];
    currentTcp.getQuat(buffer4); // w x y z
    tcpOriStart[0]=buffer4[0]; tcpOriStart[1]=buffer4[1]; tcpOriStart[2]=buffer4[2]; tcpOriStart[3]=buffer4[3];

    FLOAT nsparam = 0;
    int8_t config = CONFIG;
    FLOAT* targetsSonar = new FLOAT[nSonarRays64*4];
    FLOAT* targetOris = new FLOAT[nSonarRays64*4];
    uint32_t nrays = nSonarRays64;
    uint32_t nsteps = 25;
    FLOAT probeRadius = 0.7;

    FLOAT* debugOutput = new FLOAT[nrays*nsteps*7];

    bool doublePrecision = false;
    #ifdef DOUBLE_PRECISION
        doublePrecision = true;
    #endif

    for(int iv=0; iv<nSonarRays; iv++){
        targetsSonar[iv*4  ] = tcpPosStart[0] + sphereGridPoints[iv*3  ]*probeRadius;
        targetsSonar[iv*4+1] = tcpPosStart[1] + sphereGridPoints[iv*3+1]*probeRadius;
        targetsSonar[iv*4+2] = tcpPosStart[2] + sphereGridPoints[iv*3+2]*probeRadius;

        targetOris[iv*4  ] = tcpOriStart[0];
        targetOris[iv*4+1] = tcpOriStart[1];
        targetOris[iv*4+2] = tcpOriStart[2];
        targetOris[iv*4+3] = tcpOriStart[3];
    }

    for(int iv=0; iv<nOriSonarRays; iv++){
        targetsSonar[(iv+nSonarRays)*4  ] = tcpPosStart[0];
        targetsSonar[(iv+nSonarRays)*4+1] = tcpPosStart[1];
        targetsSonar[(iv+nSonarRays)*4+2] = tcpPosStart[2];

        Quaterniond bufq(tcpOriStart[0], tcpOriStart[1], tcpOriStart[2], tcpOriStart[3]);
        bufq = bufq * targetOrisRel[iv];

        targetOris[(iv+nSonarRays)*4  ] = bufq.w();
        targetOris[(iv+nSonarRays)*4+1] = bufq.x();
        targetOris[(iv+nSonarRays)*4+2] = bufq.y();
        targetOris[(iv+nSonarRays)*4+3] = bufq.z();
    }

    printf("GPU testrun...\n");
    tic();
    ikray.compute(
                endParams,
                rayEnds,
                rayColors,
                tcpPosStart,
                tcpOriStart,
                invToolOcl,
                &nsparam,
                config,
                targetsSonar,
                targetOris,
                nrays,
                nsteps,
                true,
                doublePrecision,
                    debugOutput,
                    debugIndex);
    toc();
    printf("done.\n");

    ////////////////////////////// prepare accmap GPU stuff

    printf("\ncompiling accessability map stuff for GPU...\n");
    AccMap accmap;

    printf("\nfilling arrays...\n");

    const uint32_t nTriangles = 100000;
    //const uint32_t nTriangles = 30;

    visualization_msgs::Marker markeraccmap;
    markeraccmap = initializeMarker(0, visualization_msgs::Marker::TRIANGLE_LIST, nTriangles*3);
    visualization_msgs::Marker markeraccmapintersect;
    markeraccmapintersect = initializeMarker(10, visualization_msgs::Marker::TRIANGLE_LIST, nTriangles*3);


//    uint16_t nx = 74, ny = 74, nz = 68;
//    double x0 =-0.9, y0 =x0, z0 = -0.40;
//    double x1 = -x0, y1 = x1, z1 = 1.25;

    uint16_t nx = 54, ny = 54, nz = 47;
    double x0 =-0.8, y0 =-0.8, z0 = -0.3;
    double x1 = 0.8, y1 = 0.8, z1 = 1.11;
    double iso = ISO; // necessary elbow interval size

    //double x0=-1.4, y0=-2, z0=-1.7,    x1=2.6, y1=2, z1=2.3;

    printf("Grid cell size: %f x %f x %f cm\n", (x1-x0)/(nx-1)*100, (y1-y0)/(ny-1)*100, (z1-z0)/(nz-1)*100);

    float* voxelData = new float[nx*ny*nz];
    double dx = (x1-x0)/(double)(nx-1);
    double dy = (y1-y0)/(double)(ny-1);
    double dz = (z1-z0)/(double)(nz-1);
    double x = x0;
    double y = y0;
    double z = z0;

    int nSamples = nx*ny*nz;
    int nSamples64 = ((nSamples)/64+1)*64; // increased so 64 devides it

    int16_t* errors = new int16_t[nSamples64];
    //printf("\nerrors from 0x%X to 0x%X, %d bytes\n", errors, &(errors[nSamples64-1]) + 15, nSamples64*sizeof(int16_t));
    int16_t* intervals = new int16_t[64*nSamples64];
    //printf("\nintervals from 0x%X to 0x%X, %d bytes\n", intervals, &(intervals[64*nSamples64-1]) + 15, 64*nSamples64*sizeof(int16_t));
    float* mcCost = new float[nSamples64]; // cost for marching cubes

    FLOAT* targetsAccmap = new FLOAT[16*nSamples64];
    //printf("\ntargetsAccmap from 0x%X to 0x%X+.., %d bytes\n", targetsAccmap, &(targetsAccmap[16*nSamples64-1]), 16*nSamples64*sizeof(FLOAT));
    uint8_t* configs = new uint8_t[nSamples64];
    //printf("\nconfigs from 0x%X to 0x%X, %d bytes\n", configs, &(configs[nSamples64-1]) + 7, nSamples64*sizeof(uint8_t));

    LwrFrame bufferFrame;

    int iV=0;
    x=x0; for(int ix=0; ix<nx; ix++){y=y0; for(int iy=0; iy<ny; iy++){z=z0; for(int iz=0; iz<nz; iz++){
                bufferFrame.setPos(x, y, z);
                oclFromFrame(&(targetsAccmap[iV*16]), &bufferFrame);
                configs[iV] = config;
                iV++;
    z+=dz;} y+=dy;} x+=dx;}

    printf("testrun...\n");
    tic();

    accmap.compute(errors, intervals,
                   mcCost,
                   (void*) targetsAccmap, configs, (void*) invToolOcl, nSamples64, doublePrecision);

    /*for(int i=0; i<nSamples; i+=10){
        if(mcCost[i] != 5){printf("%f  ", mcCost[i]);}
    }*/

    toc();
    printf("done.\n\n");

    printf("Alright, have fun!\n");

    ////////////////////////////////////////
    ////////////////////////////////////////
    ////////////////////////////// game loop
    ////////////////////////////////////////
    ////////////////////////////////////////

    t0 = ros::Time::now();

    char controlObject = 'B';
    bool unreachable = false;
    LwrJoints currentJoints;

    bool firstRun = true;
    while (ros::ok())
    {

        ////////////////////////////// gamepad control

        bool alternative = joyButtonsState[5];
        bool gridsnap = joyButtonsState[4];

        Matrix3d camcs = camcsys(camyaw, campitch, false); // camera coordinates
        Matrix3d camcssnap = camcsys(camyaw, campitch, true); // camera coordinates, snapped

        Matrix3d csys = camcs;
        if(gridsnap){csys = camcssnap;}

        if(keyPressed[97]){
            keyPressed[97]=false;
            transparent = !transparent;
        }

        if(keyPressed[118]){keyPressed[118]=false; currentTrajectory = 1;} // v
        if(keyPressed[98]){keyPressed[98]=false; currentTrajectory = 2;} // b
        if(keyPressed[110]){keyPressed[110]=false; currentTrajectory = 3;} // n
        if(keyPressed[109]){keyPressed[109]=false; currentTrajectory = 4;} // m

        bool repositioned = false;

        if(TASK == 1){
            if(keyPressed[121]){ // y
                keyPressed[121] = false;
                currentJoints = LwrJoints(-44*DEG,87*DEG,23*DEG,-98*DEG,-40*DEG,-78*DEG,-65*DEG);
                repositioned = true;
            }
            if(keyPressed[120]){ // x
                keyPressed[120] = false;
                currentJoints = LwrJoints(-63*DEG,58*DEG,-42*DEG,-87*DEG,-135*DEG,-53*DEG,60*DEG);
                repositioned = true;
            }
            if(keyPressed[99]){ // c
                keyPressed[99] = false;
                currentJoints = LwrJoints(102*DEG,37*DEG,-52*DEG,-120*DEG,-30*DEG,-74*DEG,132*DEG);
                repositioned = true;
            }
        }

        if(TASK == 2){
            if(keyPressed[121]){ // y
                keyPressed[121] = false;
                currentJoints = LwrJoints(-130*DEG,31*DEG,90*DEG,-108*DEG,-99*DEG,-34*DEG,-28*DEG);
                repositioned = true;
            }
            if(keyPressed[120]){ // x
                keyPressed[120] = false;
                currentJoints = LwrJoints(124*DEG,45*DEG,27*DEG,98*DEG,62*DEG,-40*DEG,57*DEG);
                repositioned = true;
            }
            if(keyPressed[99]){ // c
                keyPressed[99] = false;
                currentJoints = LwrJoints(46*DEG,32*DEG,120*DEG,106*DEG,32*DEG,-60*DEG,63*DEG);
                repositioned = true;
            }
        }

        if(repositioned){
            Lwr::forwardKinematics(currentPose,currentJoints);
            currentTcp = currentPose.pose * tool;
            config = currentPose.config;
            nsparam = currentPose.nsparam;
        }

        //if(joyButtonsPressed[0]){controlObject = 'A'; joyButtonsPressed[0] = false;}
        if(joyButtonsPressed[0] && TASK == 4){onTrajectory = !onTrajectory; joyButtonsPressed[0] = false;}
        if(joyButtonsPressed[1]){autoElbow = true; controlObject = 'B'; joyButtonsPressed[1] = false;}
        if(joyButtonsPressed[2]){controlObject = 'X'; joyButtonsPressed[2] = false;}
        if(joyButtonsPressed[3]){controlObject = 'Y'; joyButtonsPressed[3] = false;}


        if(joyButtonsPressed[7]){
            Quaterniond qbuffer;
            currentTcp.getQuat(qbuffer.w(), qbuffer.x(), qbuffer.y(), qbuffer.z());
            //workpiecePose.getQuat(qbuffer.w(), qbuffer.x(), qbuffer.y(), qbuffer.z());
            qbuffer = testOriParent.inverse() * qbuffer;
            testOris.push_back(qbuffer);
            joyButtonsPressed[7] = false;
        }
        if(joyButtonsPressed[6]){
            joyButtonsPressed[6] = false;
            if(testOris.size()>0){testOris.pop_back();}
        }

        LwrFrame controlFrame;

        switch (controlObject){
        case 'B': // roBot
            controlFrame = currentTcp;
            break;
        case 'X': // workpiece
            controlFrame = workpiecePose; break;
            //controlFrame = trajOffset;
            //onTrajectory = false;
            break;
        case 'Y': // test orientations
            controlFrame.ori = testOriParent;
            controlFrame.setPos(0,0,1.3);
            break;
        }

        if(!onTrajectory){

            Quaterniond normq;
            normq = controlFrame.ori;
            normq.normalize();
            controlFrame.ori = normq;

            controlFrame.pos = controlFrame.pos + 0.03*csys*
                    Vector3d(-spacenavAxesState[1], spacenavAxesState[2], -spacenavAxesState[0]);

            controlFrame.ori = AngleAxis<double>(-0.1 * spacenavAxesState[4], csys.col(0)) * controlFrame.ori;
            controlFrame.ori = AngleAxis<double>(0.1 * spacenavAxesState[5], csys.col(1)) * controlFrame.ori;
            controlFrame.ori = AngleAxis<double>(-0.1 * spacenavAxesState[3], csys.col(2)) * controlFrame.ori;

            controlFrame.pos = controlFrame.pos + csys.col(0) * (-0.02*joyAxesState[0]);
            controlFrame.ori = AngleAxis<double>(-0.1 * joyAxesState[4], csys.col(0)) * controlFrame.ori;

            if(!alternative){
                controlFrame.ori = AngleAxis<double>(-0.1 * joyAxesState[3], csys.col(1)) * controlFrame.ori;
                controlFrame.pos = controlFrame.pos + csys.col(1) * (+0.02*joyAxesState[1]);
            }
            else{
                controlFrame.ori = AngleAxis<double>( 0.1 * joyAxesState[3], csys.col(2)) * controlFrame.ori;
                controlFrame.pos = controlFrame.pos
                                  + csys.col(2) * (-0.02*joyAxesState[1]);
            }

            controlFrame.ori = AngleAxis<double>(-0.1 * (joyAxesState[5]-joyAxesState[2]), controlFrame.ori.col(2)) * controlFrame.ori;

            if(joyButtonsPressed[9]){
                joyButtonsPressed[9] = false;
                controlFrame.pos.x() = roundf((controlFrame.pos.x()-0.001)/0.05)*0.05+0.001; // singularity cheat
                controlFrame.pos.y() = roundf(controlFrame.pos.y()/0.05)*0.05;
                controlFrame.pos.z() = roundf(controlFrame.pos.z()/0.05)*0.05;
            }

            if(joyButtonsPressed[10]){
                joyButtonsPressed[10] = false;
                controlFrame.ori = snapOri(controlFrame.ori, snapOriList);
            }

        }
        else{ // on Trajectory
            splnParam += 0.004*(joyAxesState[2]-joyAxesState[5]);
            if(splnParam<0){splnParam = 0;}
            if(splnParam>1){splnParam = 1;}

            if(currentTrajectory == 1 || currentTrajectory == 2){
                controlFrame.setPos(splnx12(splnParam), splny12(splnParam), splnz12(splnParam));
                Vector4d splnquat(splnqw12(splnParam), splnqx12(splnParam), splnqy12(splnParam), splnqz12(splnParam));
                splnquat.normalize();
                controlFrame.setQuat(splnquat(0), splnquat(1), splnquat(2), splnquat(3));
            }
            if(currentTrajectory == 3 || currentTrajectory == 4){
                controlFrame.setPos(splnx34(splnParam), splny34(splnParam), splnz34(splnParam));
                Vector4d splnquat(splnqw34(splnParam), splnqx34(splnParam), splnqy34(splnParam), splnqz34(splnParam));
                splnquat.normalize();
                controlFrame.setQuat(splnquat(0), splnquat(1), splnquat(2), splnquat(3));
            }
            if(currentTrajectory == 1){controlFrame = trajOffset1 * controlFrame;}
            if(currentTrajectory == 2){controlFrame = trajOffset2 * controlFrame;}
            if(currentTrajectory == 3){controlFrame = trajOffset3 * controlFrame;}
            if(currentTrajectory == 4){controlFrame = trajOffset4 * controlFrame;}
        }

        if(!alternative){
            campitch += 0.2*joyAxesState[7];
            if(campitch > 1.57){campitch = 1.57;}
            if(campitch < -1.57){campitch = -1.57;}
        }
        else{
            camdst *= 1 - 0.1*joyAxesState[7];
        }

        camyaw += -0.2*joyAxesState[6];

        cam.focus.point.x = controlFrame.pos.x();
        cam.focus.point.y = controlFrame.pos.y();
        cam.focus.point.z = controlFrame.pos.z();

        cam.eye.point.x = cos(camyaw)*cos(campitch)*camdst + cam.focus.point.x;
        cam.eye.point.y = sin(camyaw)*cos(campitch)*camdst + cam.focus.point.y;
        cam.eye.point.z = sin(campitch)*camdst + cam.focus.point.z;


        { // so test variables are deleted after case thingy
        LwrJoints testJoints; // test if the robot can go there
        LwrXCart testPose = currentPose;
        testPose.pose = controlFrame * invTool;
        double costs;

        switch (controlObject){
        case 'B': // roBot

            if(onTrajectory){
                if(currentTrajectory == 1){testPose.nsparam = splnnsp1(splnParam);}
                if(currentTrajectory == 2){testPose.nsparam = splnnsp2(splnParam);}
                if(currentTrajectory == 3){testPose.nsparam = splnnsp3(splnParam);}
                if(currentTrajectory == 4){testPose.nsparam = splnnsp4(splnParam);}
                testPose.config = trajcfg;
                Lwr::inverseKinematics(testJoints, testPose);
                costs = elbowCost2(testJoints, testJoints);
            }
            else{
                Lwr::inverseKinematics(testJoints, testPose);
                costs = elbowCost2(testJoints, currentJoints);
            }

            if(costs < 0){
                currentTcp = controlFrame;
                if(onTrajectory){
                    currentPose.nsparam = testPose.nsparam;
                    currentPose.config = trajcfg;
                }
            }
            else{
                unreachable = true;
            }
            break;
        case 'X': // workpiece
            workpiecePose = controlFrame; break;
            //trajOffset = controlFrame;
            //cout << trajOffset;
            break;
        case 'Y': // test orientations
            testOriParent = controlFrame.ori; break;
        }
        }


        currentPose.pose = currentTcp*invTool;

        ////////////////////////////// single joint control

        Lwr::inverseKinematics(currentJoints, currentPose);

        double keyv = 0.07;
        if(keyState[49]){currentJoints[0] += keyv; autoElbow = false;}
        if(keyState[50]){currentJoints[1] += keyv; autoElbow = false;}
        if(keyState[51]){currentJoints[2] += keyv; autoElbow = false;}
        if(keyState[52]){currentJoints[3] += keyv; autoElbow = false;}
        if(keyState[53]){currentJoints[4] += keyv; autoElbow = false;}
        if(keyState[54]){currentJoints[5] += keyv; autoElbow = false;}
        if(keyState[55]){currentJoints[6] += keyv; autoElbow = false;}
        if(keyState[113]){currentJoints[0] -= keyv; autoElbow = false;}
        if(keyState[119]){currentJoints[1] -= keyv; autoElbow = false;}
        if(keyState[101]){currentJoints[2] -= keyv; autoElbow = false;}
        if(keyState[114]){currentJoints[3] -= keyv; autoElbow = false;}
        if(keyState[116]){currentJoints[4] -= keyv; autoElbow = false;}
        if(keyState[122]){currentJoints[5] -= keyv; autoElbow = false;}
        if(keyState[117]){currentJoints[6] -= keyv; autoElbow = false;}

        for(int j=0; j<7; j+=2){
            if(currentJoints[j] > 169.9/180.0*M_PI){currentJoints[j] = 169.9/180.0*M_PI;}
            if(currentJoints[j] < -169.9/180.0*M_PI){currentJoints[j] = -169.9/180.0*M_PI;}
        }
        for(int j=1; j<7; j+=2){
            if(currentJoints[j] > 119.9/180.0*M_PI){currentJoints[j] = 119.9/180.0*M_PI;}
            if(currentJoints[j] < -119.9/180.0*M_PI){currentJoints[j] = -119.9/180.0*M_PI;}
        }

        Lwr::forwardKinematics(currentPose, currentJoints);
        currentTcp = currentPose.pose * tool;
        config = currentPose.config;
        nsparam = currentPose.nsparam;

        ////////////////////////////// elbow optimization

        LwrXCart testPose = currentPose;
        LwrJoints testJoints;

        int nTrials = 15; // how many samples
        //double delta = 0.35; // up to which angle (abt. 20deg)
        double delta = 0.01; // up to which angle
        double nsp = nsparam;

        currentPose.pose = currentTcp*invTool;

        testPose.nsparam = nsparam;
        Lwr::inverseKinematics(currentJoints, testPose);
        double costBefore = elbowCost2(currentJoints, currentJoints);

        //if(autoElbow && !(onTrajectory && controlObject == 'B')){
        if(autoElbow){

            testPose.nsparam = nsparam+1e-5;
            Lwr::inverseKinematics(testJoints, testPose);
            double costNext = elbowCost2(testJoints, currentJoints);

            double dir = +1;
            if(costBefore < costNext){dir = -1;}

            // sample cost function in descending direction and stop if it starts rising again

            for(int i=1; i<nTrials+1; i++){
                double q = (double)i/(double)nTrials;
                q = pow(q, 1.5);
                nsp = nsparam + q*dir*delta;

                testPose.nsparam = nsp;
                Lwr::inverseKinematics(testJoints, testPose);
                costNext = elbowCost2(testJoints, currentJoints);

                if(costNext < costBefore){
                    costBefore = costNext;
                    nsparam = nsp;
                }
                else{
                    break;
                }
            }
        }

        //if(autoElbow){nsparam = 0;}



        ////////////////////////////// draw robot

        currentPose.nsparam = nsparam;
        currentPose.config = config;
        LwrJoints jnts;
        LwrErrorMsg ikreturn = Lwr::inverseKinematics(jnts, currentPose);
        LwrFrame dhPositions[8];
        fk(dhPositions, jnts);
        Vector3d jointPositions[7];

        jointPositions[0] = Vector3d(0,0,0);
        jointPositions[1] = Vector3d(0,0,0.31);
        jointPositions[2] = 0.5*(Vector3d(0,0,0.31) + dhPositions[3].pos);
        jointPositions[3] = dhPositions[3].pos;
        jointPositions[4] = 0.5*(dhPositions[3].pos + dhPositions[5].pos);
        jointPositions[5] = dhPositions[5].pos;
        jointPositions[6] = dhPositions[7].pos;

        for(int j=0; j<7; j++){
            markerRobot.points[j] = positionFromEigen(jointPositions[j]);
            markerRobot.colors[j] = jointColor(j, jnts[j], Vector4d(1.0, 0.8, 0.6, 1.0));
            markerJoints[j].pose.position =
                    positionFromEigen(jointPositions[j]*0.8 + 0.2*Vector3d(cam.eye.point.x, cam.eye.point.y, cam.eye.point.z));
            markerJoints[j].text = int2str(floor(jnts[j]/M_PI*180+0.5));
            if(j%2 == 0)
                if(fabs(jnts[j]) > 160.0*M_PI/180.0){
                    markerJoints[j].color = colorFromEigen(Vector4d(0.8,0,0,1));}
                else{
                    markerJoints[j].color = colorFromEigen(Vector4d(0,0.4,1.0,1));}
            else{
                if(fabs(jnts[j]) > 110.0*M_PI/180.0){
                    markerJoints[j].color = colorFromEigen(Vector4d(0.8,0,0,1));}
                else{
                    markerJoints[j].color = colorFromEigen(Vector4d(0,0.4,1.0,1));}
            }
            markerJoints[j].color = colorFromEigen(Vector4d(0.2,1,0,1));
        }

        markerSignalTable.color = colorFromEigen(Vector4d(0,0.7,0.3,1));
        if(ikreturn & LWR_JOINTLIMIT){
            markerSignalTable.color = colorFromEigen(Vector4d(1,0.67,0,1));
        }
        if(ikreturn & LWR_ERROR){
            markerSignalTable.color = colorFromEigen(Vector4d(1,0,0,1));
        }

        ////////////////////////////// draw elbow ring

        Vector3d elbowPivot = 0.5*(Vector3d(0,0,0.31) + dhPositions[5].pos);
        Vector3d elbowCircleNormal = dhPositions[5].pos - Vector3d(0,0,0.31);
        elbowCircleNormal.normalize();

        Vector3d sinVec = -elbowCircleNormal.cross(Vector3d(0,0,1));
        sinVec.normalize();
        Vector3d cosVec = -sinVec.cross(elbowCircleNormal);

        for(int i=0; i<nElbowCostSteps+1; i++){
            double phi = (double)i / (double)nElbowCostSteps * 2 * M_PI;

            testPose.nsparam = phi;
            Lwr::inverseKinematics(testJoints, testPose);
            double q = elbowCost2(testJoints, currentJoints);
            double qc = 0;
            double qr = 0;
            //double q = elbowCost(testJoints, currentJoints);

            if(q > 0){
                qc = q/3;
                qr = q/3;
                if(qc>1){qc=1;}
                markerElbowCost.colors[i] = colorFromEigen((1-qc)*Vector4d(1,0,0,1) + qc*Vector4d(1,1,1,1));
            }
            else{
                //q = pow(1-exp(-q),20); // function like this:  __/"" from 0 to 1
                qc = -q*10;
                qr = q*10;
                if(qc>1){qc=1;}
                markerElbowCost.colors[i] = colorFromEigen((1-qc)*Vector4d(1,0.73,0,1) + qc*Vector4d(0,0.73,1,1));
            }
            qr = q;

            markerElbowCost.points[i] =
                    positionFromEigen(elbowPivot +
                              0.2*qr*elbowCircleNormal +
                              (sinVec*sin(phi) + cosVec*cos(phi))*(elbowCostR1));
            markerElbowZero.points[i] =
                    positionFromEigen(elbowPivot + (sinVec*sin(phi) + cosVec*cos(phi))*elbowCostR1);

        }

        ////////////////////////////// compute and publish GPU probe rays if there is a subscriber

        tcpPosStart[0]=currentTcp.pos.x(); tcpPosStart[1]=currentTcp.pos.y(); tcpPosStart[2]=currentTcp.pos.z();
        currentTcp.getQuat(buffer4);
        tcpOriStart[0]=buffer4[0]; tcpOriStart[1]=buffer4[1]; tcpOriStart[2]=buffer4[2]; tcpOriStart[3]=buffer4[3];

        if(publisherSonar.getNumSubscribers()>0 || publisherOriSonar.getNumSubscribers()>0){

            for(int iv=0; iv<nSonarRays; iv++){
                targetsSonar[iv*4  ] = tcpPosStart[0] + sphereGridPoints[iv*3  ]*probeRadius;
                targetsSonar[iv*4+1] = tcpPosStart[1] + sphereGridPoints[iv*3+1]*probeRadius;
                targetsSonar[iv*4+2] = tcpPosStart[2] + sphereGridPoints[iv*3+2]*probeRadius;

                targetOris[iv*4  ] = tcpOriStart[0];
                targetOris[iv*4+1] = tcpOriStart[1];
                targetOris[iv*4+2] = tcpOriStart[2];
                targetOris[iv*4+3] = tcpOriStart[3];
            }

            for(int iv=0; iv<nOriSonarRays; iv++){
                targetsSonar[(iv+nSonarRays)*4  ] = tcpPosStart[0];
                targetsSonar[(iv+nSonarRays)*4+1] = tcpPosStart[1];
                targetsSonar[(iv+nSonarRays)*4+2] = tcpPosStart[2];

                Quaterniond bufq(tcpOriStart[0], tcpOriStart[1], tcpOriStart[2], tcpOriStart[3]);
                bufq = bufq * targetOrisRel[iv];

                targetOris[(iv+nSonarRays)*4  ] = bufq.w();
                targetOris[(iv+nSonarRays)*4+1] = bufq.x();
                targetOris[(iv+nSonarRays)*4+2] = bufq.y();
                targetOris[(iv+nSonarRays)*4+3] = bufq.z();
            }

            //printf("sonar: ");
            //tic();
            ikray.compute(
                        endParams,
                        rayEnds,
                        rayColors,
                        tcpPosStart,
                        tcpOriStart,
                        invToolOcl,
                        &nsparam,
                        config,
                        targetsSonar,
                        targetOris,
                        nrays,
                        nsteps,
                        true,
                        doublePrecision,
                            debugOutput,
                            debugIndex);

            double minParam;

            minParam = 10000;
            for(int i=0; i<nSonarRays; i++){
                if(endParams[i] < minParam){
                    minParam = endParams[i];
                }
            }
            printf("minSonarDist: %f cm\n", minParam * probeRadius*100);
            minParam = 10000;
            for(int i=nSonarRays; i<nSonarRays+nOriSonarRays; i++){
                if(endParams[i] < minParam){
                    minParam = endParams[i];
                }
            }
            printf("minOriSonarDist: %f deg\n", minParam * oriPhiMax/M_PI*180.0);

            //toc();
            //printf("\n");

            /*if(joyButtonsPressed[0]){ // debug output if (A) is pressed
                joyButtonsPressed[0] = false;
                printf("TCP = [%f, %f, %f];\n", tcpPosStart[0], tcpPosStart[1], tcpPosStart[2]);
                for(int iv=0; iv<nSonarRays; iv++){
                    printf("p(%d, 1:3) = [%f, %f, %f]; ", iv+1, rayEnds[iv*4], rayEnds[iv*4+1], rayEnds[iv*4+2]);
                    if(iv%20==0){printf("\n");}
                }
                printf("\n\n");
                cout << "TCPstart = [" << endl << currentTcp << "];" << endl << endl;
                cout << "raytarget = [" << targetsSonar[debugIndex*4] << "; "
                        << targetsSonar[debugIndex*4+1] << "; "
                        << targetsSonar[debugIndex*4+2] << "]; % "
                        << "debugIndex = " << debugIndex << " (0-based)";

                printf("\n\n");
                for(int iv=1; iv<=nsteps+3; iv++){
                    if(debugOutput[iv*20] == -1){break;}
                    printf("q(%d) = %f; "  , iv, debugOutput[iv*20+1]);
                    printf("c(%d) = %f; "  , iv, debugOutput[iv*20+2]);
                    printf("j(%d,1) = %f; ", iv, debugOutput[iv*20+3]);
                    printf("j(%d,2) = %f; ", iv, debugOutput[iv*20+4]);
                    printf("j(%d,3) = %f; ", iv, debugOutput[iv*20+5]);
                    printf("j(%d,4) = %f; ", iv, debugOutput[iv*20+6]);
                    printf("j(%d,5) = %f; ", iv, debugOutput[iv*20+7]);
                    printf("j(%d,6) = %f; ", iv, debugOutput[iv*20+8]);
                    printf("j(%d,7) = %f; ", iv, debugOutput[iv*20+9]);
                    printf("\n");
                }
                printf("\n\n");
            }*/

            if(keyPressed[100]){ // debug output if [D] is pressed
                keyPressed[100] = false;
                FLOAT startJoints[7];
                for(int ij=0; ij<7; ij++){
                    startJoints[ij] = jnts.j[ij];
                }

                FILE* pFile;
                pFile = fopen("/home/mirko/catkin_ws/src/wovi/src/debug.bin", "wb");
                fwrite(startJoints, sizeof(FLOAT), 7, pFile);
                fwrite(debugOutput, sizeof(FLOAT), nrays*nsteps*7, pFile);
                fclose(pFile);
                printf("Written debug file\n");
            }

            if(!sonarUseTriangles){
                for(int iv=0; iv<nSonarRays; iv++){
                    markerSonar.points[iv].x = rayEnds[iv*4  ];
                    markerSonar.points[iv].y = rayEnds[iv*4+1];
                    markerSonar.points[iv].z = rayEnds[iv*4+2];

                    markerSonar.colors[iv].r = rayColors[iv*4  ];
                    markerSonar.colors[iv].g = rayColors[iv*4+1];
                    markerSonar.colors[iv].b = rayColors[iv*4+2];
                    markerSonar.colors[iv].a = rayColors[iv*4+3];
                }
            }
            else{
                for(int iv=0; iv<3*nFaces; iv++){
                    markerSonar.points[iv].x = rayEnds[geoFaces[iv]*4  ];
                    markerSonar.points[iv].y = rayEnds[geoFaces[iv]*4+1];
                    markerSonar.points[iv].z = rayEnds[geoFaces[iv]*4+2];

                    markerSonar.colors[iv].r = rayColors[geoFaces[iv]*4  ];
                    markerSonar.colors[iv].g = rayColors[geoFaces[iv]*4+1];
                    markerSonar.colors[iv].b = rayColors[geoFaces[iv]*4+2];
                    markerSonar.colors[iv].a = rayColors[geoFaces[iv]*4+3];
                }
            }

            Matrix3d twist;
            double w;

            for(int iv=0; iv<nOriSonarRays-2; iv++){

                double yaw = (double)iv / ((double)nOriSonarRays-2) *2.0*M_PI;
                w = endParams[iv+nSonarRays];

                twist = AngleAxisd(w*oriPhiMax, Vector3d(cos(yaw), sin(yaw), 0) );

                //markerOriSonar.points[iv] = positionFromEigen(Vector3d(cos(phi)*r, sin(phi)*r, 0));

                markerOriSonar.points[iv] = positionFromEigen(
                           currentTcp.ori*twist*Vector3d(0,0,-1)*0.26 + currentTcp.pos);

                markerOriSonar.colors[iv] = colorFromEigen(Vector4d(1,0,1,1)*(1-w) + Vector4d(0.59,0.59,0.59,0)*w);
            }

            markerOriSonar.points[nOriSonarRays-2] = markerOriSonar.points[0];
            markerOriSonar.colors[nOriSonarRays-2] = markerOriSonar.colors[0];


            markerRollSonar.points[0] =
            markerRollSonar.points[2] =
            markerRollSonar.points[4] =
            markerRollSonar.points[6] = positionFromEigen(
                        currentTcp.ori*Vector3d(0,0,-1)*0.26 + currentTcp.pos);

            markerRollSonar.points[1] = positionFromEigen(
                        currentTcp.ori*Vector3d(0.3,0,-1)*0.26 + currentTcp.pos);
            markerRollSonar.colors[0] = markerRollSonar.colors[1] = colorFromEigen(Vector4d(0.4,0,0.4,0.7));

            w = endParams[nSonarRays+nOriSonarRays-2];
            twist = AngleAxisd(-w*oriPhiMax, Vector3d(0,0,1) );
            markerRollSonar.points[3] = positionFromEigen(
                        currentTcp.ori*twist*Vector3d(0.3,0,-1)*0.26 + currentTcp.pos);
            markerRollSonar.colors[2] = markerRollSonar.colors[3] =
                    colorFromEigen(Vector4d(1,0,1,1)*(1-w) + Vector4d(0.59,0.59,0.59,0)*w);

            w = endParams[nSonarRays+nOriSonarRays-1];
            twist = AngleAxisd(w*oriPhiMax, Vector3d(0,0,1) );
            markerRollSonar.points[5] = positionFromEigen(
                        currentTcp.ori*twist*Vector3d(0.3,0,-1)*0.26 + currentTcp.pos);
            markerRollSonar.colors[4] = markerRollSonar.colors[5] =
                    colorFromEigen(Vector4d(1,0,1,1)*(1-w) + Vector4d(0.59,0.59,0.59,0)*w);

            markerRollSonar.points[7] = positionFromEigen(currentTcp.pos);
            markerRollSonar.colors[6] = markerRollSonar.colors[7] = colorFromEigen(Vector4d(0.4,0,0.4,0.7));

            markerOriSonarSphere.pose.position = positionFromEigen(currentTcp.pos);


            publisherSonar.publish(markerSonar);
            publisherOriSonar.publish(markerOriSonar);
            publisherOriSonar.publish(markerRollSonar);
            publisherOriSonar.publish(markerOriSonarSphere);

        }


        ////////////////////////////// compute and publish IK samples on GPU if there is a subscriber

        if(publisherAccMap.getNumSubscribers()>0){

            bufferFrame.ori = currentTcp.ori;

            Vector3d wristOffset;
            wristOffset = currentPose.pose.ori * (Vector3d(0,0,0.078) + tool.pos);

            iV = 0;
            x=x0+wristOffset(0); for(int ix=0; ix<nx; ix++){
                y=y0+wristOffset(1); for(int iy=0; iy<ny; iy++){
                    z=z0+wristOffset(2); for(int iz=0; iz<nz; iz++){
                        bufferFrame.setPos(x,y,z);
                        oclFromFrame(&(targetsAccmap[iV*16]), &bufferFrame);
                        configs[iV] = config;
                        iV++;
            z+=dz;} y+=dy;} x+=dx;}

            //printf("constant ori ws: ");

            //tic();
            accmap.compute(errors, intervals,
                           mcCost,
                           (void*) targetsAccmap, configs, (void*) invToolOcl, nSamples64, doublePrecision);
            //toc();

            iV = 0;
            x=x0+wristOffset(0); for(int ix=0; ix<nx; ix++){
                y=y0+wristOffset(1); for(int iy=0; iy<ny; iy++){
                    z=z0+wristOffset(2); for(int iz=0; iz<nz; iz++){
                        //voxelData[ix*ny*nz + iy*nz + iz] = (errors[iV] & (LWR_NO_SOLUTION_FOR_ELBOW | LWR_ERROR))>0 ? 10 : -10;
                        voxelData[ix*ny*nz + iy*nz + iz] = mcCost[iV];
                        iV++;
            z+=dz;} y+=dy;} x+=dx;}

            if(transparent){
                marchCubes(&markeraccmap, nTriangles, voxelData,
                           x0+wristOffset(0), y0+wristOffset(1), z0+wristOffset(2), dx, dy, dz, nx, ny, nz, iso, CONST_ORI_ALPHA);
            }
            else{
                marchCubes(&markeraccmap, nTriangles, voxelData,
                           x0+wristOffset(0), y0+wristOffset(1), z0+wristOffset(2), dx, dy, dz, nx, ny, nz, iso, 1);
            }

            publisherAccMap.publish(markeraccmap);
        }

// first version before smoothin:
        // display intersection of all tested orientations (could take some time)
/*        if(joyButtonsPressed[8]){
            joyButtonsPressed[8] = false;

            // set all points to reachable
            iV = 0;
            x=x0; for(int ix=0; ix<nx; ix++){y=y0; for(int iy=0; iy<ny; iy++){z=z0; for(int iz=0; iz<nz; iz++){
                        voxelData[ix*ny*nz + iy*nz + iz] = 10;
                        iV++;
            z+=dz;} y+=dy;} x+=dx;}

            for(int iOri=0; iOri<testOris.size(); iOri++){

                bufferFrame.ori = testOriParent * testOris[iOri];

                // fill target array
                iV = 0;
                x=x0; for(int ix=0; ix<nx; ix++){y=y0; for(int iy=0; iy<ny; iy++){z=z0; for(int iz=0; iz<nz; iz++){
                            bufferFrame.setPos(x,y,z);
                            oclFromFrame(&(targetsAccmap[iV*16]), &bufferFrame);
                            configs[iV] = config;
                            iV++;
                z+=dz;} y+=dy;} x+=dx;}

                // GPU
                accmap.compute(errors, intervals,
                               mcCost,
                               (void*) targetsAccmap, configs, (void*) invToolOcl, nSamples64, doublePrecision);

                // subtract unreachable regions
                iV = 0;
                x=x0; for(int ix=0; ix<nx; ix++){y=y0; for(int iy=0; iy<ny; iy++){z=z0; for(int iz=0; iz<nz; iz++){
                            if((errors[iV] & (LWR_NO_SOLUTION_FOR_ELBOW | LWR_ERROR))){
                                voxelData[ix*ny*nz + iy*nz + iz] = -10;
                            }
                            iV++;
                z+=dz;} y+=dy;} x+=dx;}

            }

            marchCubes(&markeraccmapintersect, nTriangles, voxelData, x0, y0, z0, dx, dy, dz, nx, ny, nz, iso, 0.6);

            publisherIntersection.publish(markeraccmapintersect);
        }
*/

       if(joyButtonsPressed[8]){ // display intersection of all tested orientations (could take some time)
            joyButtonsPressed[8] = false;

            fprintf(pLogFile, "TOTALOWS, %d oris: ", (int)testOris.size());

            // set all points to reachable
            iV = 0;
            x=x0; for(int ix=0; ix<nx; ix++){y=y0; for(int iy=0; iy<ny; iy++){z=z0; for(int iz=0; iz<nz; iz++){
                        voxelData[ix*ny*nz + iy*nz + iz] = 10;
                        iV++;
            z+=dz;} y+=dy;} x+=dx;}

            double xmin = -1000, ymin = -1000, zmin = -1000;
            double xmax = 1000, ymax = 1000, zmax = 1000;
            double dx_, dy_, dz_;

            // find bounding box
            for(int iOri=0; iOri<testOris.size(); iOri++){
                Vector3d wristOffset;
                wristOffset = testOriParent * testOris[iOri] * invTool.ori * (Vector3d(0,0,0.078) + tool.pos);

                if(x0 + wristOffset.x() > xmin){xmin = x0 + wristOffset.x();}
                if(y0 + wristOffset.y() > ymin){ymin = y0 + wristOffset.y();}
                if(z0 + wristOffset.z() > zmin){zmin = z0 + wristOffset.z();}
                if(x1 + wristOffset.x() < xmax){xmax = x1 + wristOffset.x();}
                if(y1 + wristOffset.y() < ymax){ymax = y1 + wristOffset.y();}
                if(z1 + wristOffset.z() < zmax){zmax = z1 + wristOffset.z();}
            }

            dx_ = (xmax-xmin)/(double)(nx-1);
            dy_ = (ymax-ymin)/(double)(ny-1);
            dz_ = (zmax-zmin)/(double)(nz-1);

            for(int iOri=0; iOri<testOris.size(); iOri++){

                bufferFrame.ori = testOriParent * testOris[iOri];

                fprintf(pLogFile, "[%f, %f, %f; %f, %f, %f; %f, %f, %f]; ",
                        bufferFrame.ori(0,0), bufferFrame.ori(0,1), bufferFrame.ori(0,2),
                        bufferFrame.ori(1,0), bufferFrame.ori(1,1), bufferFrame.ori(1,2),
                        bufferFrame.ori(2,0), bufferFrame.ori(2,1), bufferFrame.ori(2,2));

                // fill target array
                iV = 0;
                x=xmin; for(int ix=0; ix<nx; ix++){y=ymin; for(int iy=0; iy<ny; iy++){z=zmin; for(int iz=0; iz<nz; iz++){
                            bufferFrame.setPos(x,y,z);
                            oclFromFrame(&(targetsAccmap[iV*16]), &bufferFrame);
                            configs[iV] = config;
                            iV++;
                z+=dz_;} y+=dy_;} x+=dx_;}

                // GPU
                accmap.compute(errors, intervals,
                               mcCost,
                               (void*) targetsAccmap, configs, (void*) invToolOcl, nSamples64, doublePrecision);

                // subtract unreachable regions
                iV = 0;
                x=xmin; for(int ix=0; ix<nx; ix++){y=ymin; for(int iy=0; iy<ny; iy++){z=zmin; for(int iz=0; iz<nz; iz++){
                            if(mcCost[iV] < voxelData[ix*ny*nz + iy*nz + iz]){
                                voxelData[ix*ny*nz + iy*nz + iz] = mcCost[iV];
                            }
                            iV++;
                z+=dz_;} y+=dy_;} x+=dx_;}

            }

            if(transparent){
                marchCubes(&markeraccmapintersect, nTriangles, voxelData, xmin, ymin, zmin, dx_, dy_, dz_, nx, ny, nz, iso, TOTAL_ORI_ALPHA);
            }
            else{
                marchCubes(&markeraccmapintersect, nTriangles, voxelData, xmin, ymin, zmin, dx_, dy_, dz_, nx, ny, nz, iso, 1);
            }


            publisherIntersection.publish(markeraccmapintersect);
            fprintf(pLogFile, "\n");
        }

        ////////////////////////////// publish rest stuff

        publisherRobot.publish(markerRobot);
        publisherElbowCost.publish(markerElbowCost);
        publisherElbowCost.publish(markerElbowZero);
        publisherCam.publish(cam);
        for(int j=0; j<7; j++){
            publisherJointLabels.publish(markerJoints[j]);
        }
        for(int j=0; j<7; j++){
            jointsPublish.position[j] = jnts[j];
        }
        jointsPublish.header.stamp = ros::Time::now();
        publisherModel.publish(jointsPublish);

        markerTool.pose.position = positionFromEigen(currentTcp.pos);
        currentTcp.getQuat(markerTool.pose.orientation.w, markerTool.pose.orientation.x,
                           markerTool.pose.orientation.y, markerTool.pose.orientation.z);
        publisherTool.publish(markerTool);

        publisherTrajectory1.publish(markerTrajectory1);
        publisherTrajectory2.publish(markerTrajectory2);
        publisherTrajectory3.publish(markerTrajectory3);
        publisherTrajectory4.publish(markerTrajectory4);

        //pubcsys(Vector3d(cam.focus.point.x, cam.focus.point.y, cam.focus.point.z), ccs, "Focus");
        pubcsys(currentTcp.pos, currentTcp.ori, "TCP");
        pubcsys(workpiecePose.pos, workpiecePose.ori, "Workpiece");
        //pubcsys(Vector3d(0,0,0), Matrix3d::Identity(), "Base");


        Matrix3d R;
        markerTestOris.points.resize(0);
        markerTestOris.colors.resize(0);
        Vector3d testOriPos(0,0,1.3);

        /*if(controlObject == 'X'){
            for(int ip=0; ip<arrowMesh.size(); ip++){
                markerTestOris.points.push_back(positionFromEigen(0.5 * workpiecePose.ori * arrowMesh[ip] + testOriPos));
                markerTestOris.colors.push_back(colorFromEigen(Vector4d (0.9,0.9,0.9,1)));
            }
        }*/
        if(controlObject == 'B'){
            for(int ip=0; ip<arrowMesh.size(); ip++){
                markerTestOris.points.push_back(positionFromEigen(0.5 * currentTcp.ori * arrowMesh[ip] + testOriPos));
                markerTestOris.colors.push_back(colorFromEigen(Vector4d (0.9,0.9,0.9,1)));
            }
        }
        for(int i=0; i<testOris.size(); i++){
            R = testOriParent * testOris[i];
            for(int ip=0; ip<arrowMesh.size(); ip++){
                markerTestOris.points.push_back(positionFromEigen(0.3 * R * arrowMesh[ip] + testOriPos));
                markerTestOris.colors.push_back(colorFromEigen(Vector4d(testOris[i].x()*0.5+0.5, testOris[i].y()*0.5+0.5, testOris[i].z()*0.5+0.5, 1.0)));
            }
        }

        for(int i=0; i<snapOriList.size(); i++){
            Matrix3d schmi;
            schmi = snapOriList[i];
            string schmu = int2str(i);
            pubcsys(Vector3d(0,0,1.5), schmi, schmu.c_str());
        }

        publisherTestOris.publish(markerTestOris);

        markerWorkpiece.pose.position = positionFromEigen(workpiecePose.pos);
        workpiecePose.getQuat(markerWorkpiece.pose.orientation.w, markerWorkpiece.pose.orientation.x,
                              markerWorkpiece.pose.orientation.y, markerWorkpiece.pose.orientation.z);
        publisherWorkpiece.publish(markerWorkpiece);

        if(TASK == 1){
            int idbefore = markerWorkpiece.id;

            markerWorkpiece.id++;
            markerWorkpiece.pose.position = positionFromEigen(Vector3d(0,0.45,-0.25));
            markerWorkpiece.pose.orientation.x = 0; markerWorkpiece.pose.orientation.y = 0;
            markerWorkpiece.pose.orientation.z = 0; markerWorkpiece.pose.orientation.w = 1;
            markerWorkpiece.color = colorFromEigen(Vector4d(0.6,0.7,0.4,0.8));
            publisherWorkpiece2.publish(markerWorkpiece);
            pubcsys(Vector3d(0,0.45,-0.25), Vector4d(0,0,0,1), "wp1");

            markerWorkpiece.id++;
            markerWorkpiece.pose.position = positionFromEigen(Vector3d(-0.75,-0.05,0.05));
            markerWorkpiece.pose.orientation.x = 0.2706; markerWorkpiece.pose.orientation.y = 0.6533;
            markerWorkpiece.pose.orientation.z = 0.2706; markerWorkpiece.pose.orientation.w = 0.6533;
            markerWorkpiece.color = colorFromEigen(Vector4d(0.6,0.5,0.6,0.8));
            publisherWorkpiece2.publish(markerWorkpiece);
            pubcsys(Vector3d(-0.75,-0.05,0.05), Vector4d(0.2706,0.6533,0.2706,0.6533), "wp2");

            markerWorkpiece.id++;
            markerWorkpiece.pose.position = positionFromEigen(Vector3d(-0.2,-0.75,0.3));
            markerWorkpiece.pose.orientation.x = -0.707; markerWorkpiece.pose.orientation.y = 0;
            markerWorkpiece.pose.orientation.z = 0; markerWorkpiece.pose.orientation.w = 0.707;
            markerWorkpiece.color = colorFromEigen(Vector4d(0.75,0.65,0.4,0.8));
            publisherWorkpiece2.publish(markerWorkpiece);
            pubcsys(Vector3d(-0.2,-0.75,0.3), Vector4d(-0.707,0,0,0.707), "wp3");

            markerWorkpiece.color = colorFromEigen(Vector4d(0.6,0.5,0.4,0.8));
            markerWorkpiece.id = idbefore;
        }

        if(TASK == 2){
            int idbefore = markerWorkpiece.id;
            markerWorkpiece.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/rubik.dae";
            markerWorkpiece.scale.x = markerWorkpiece.scale.y = markerWorkpiece.scale.z = 0.1;

            markerWorkpiece.pose.orientation.x = 0; markerWorkpiece.pose.orientation.y = 0;
            markerWorkpiece.pose.orientation.z = 0; markerWorkpiece.pose.orientation.w = 1;

            markerWorkpiece.id++;
            markerWorkpiece.pose.position = positionFromEigen(Vector3d(-0.35,0.35,0.55));
            markerWorkpiece.color = colorFromEigen(Vector4d(1,0.5,0.1,1));
            publisherWorkpiece2.publish(markerWorkpiece);

            markerWorkpiece.id++;
            markerWorkpiece.pose.position = positionFromEigen(Vector3d(-0.45,0,0.3));
            markerWorkpiece.color = colorFromEigen(Vector4d(0.5,0.1,1,1));
            publisherWorkpiece2.publish(markerWorkpiece);

            markerWorkpiece.id++;
            markerWorkpiece.pose.position = positionFromEigen(Vector3d(-0.3,-0.1,0.8));
            markerWorkpiece.color = colorFromEigen(Vector4d(0.1,1,0.24,1));
            publisherWorkpiece2.publish(markerWorkpiece);

            markerWorkpiece.scale.x = markerWorkpiece.scale.y = markerWorkpiece.scale.z = 1;
            markerWorkpiece.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/rubiktool.dae";
            markerWorkpiece.color = colorFromEigen(Vector4d(0,0,1,1));
            markerWorkpiece.id = idbefore;
        }


        publisherSignalTable.publish(markerSignalTable);

        // log
        ros::Duration tnow = ros::Time::now() - t0;
        fprintf(pLogFile, "t=%f; ", tnow.toSec());
        fprintf(pLogFile, "wall=%d; ", (int) unreachable);
        fprintf(pLogFile, "j=[%f, %f, %f, %f, %f, %f, %f]; ", currentJoints[0], currentJoints[1], currentJoints[2], currentJoints[3], currentJoints[4], currentJoints[5], currentJoints[6]);
        fprintf(pLogFile, "workpiece=[%f, %f, %f, %f; %f, %f, %f, %f; %f, %f, %f, %f]; ",
                workpiecePose.ori(0,0), workpiecePose.ori(0,1), workpiecePose.ori(0,2), workpiecePose.pos(0),
                workpiecePose.ori(1,0), workpiecePose.ori(1,1), workpiecePose.ori(1,2), workpiecePose.pos(1),
                workpiecePose.ori(2,0), workpiecePose.ori(2,1), workpiecePose.ori(2,2), workpiecePose.pos(2));
        fprintf(pLogFile, "sonar=%d; tilts=%d; constows=%d; totalows=%d; ", publisherSonar.getNumSubscribers(),
                publisherOriSonar.getNumSubscribers(), publisherAccMap.getNumSubscribers(), publisherIntersection.getNumSubscribers());


        fprintf(pLogFile, "\n");
        rate.sleep();
        ros::spinOnce();

        t += dt;
        firstRun = false;
    }/**/

    delete[] sphereGridPoints;
    delete[] endParams;
    delete[] rayEnds;
    delete[] rayColors;
    delete[] tcpPosStart;
    delete[] tcpOriStart;
    delete[] invToolOcl;
    delete[] targetsSonar;
    delete[] targetOris;
    delete[] targetOrisRel;

    delete[] errors;
    delete[] intervals;
    delete[] targetsAccmap;
    delete[] configs;
    delete[] voxelData;

}

