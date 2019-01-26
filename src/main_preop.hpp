
#include "defs.h"
#include "sphere_grid.hpp"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <keyboard/Key.h>
#include <view_controller_msgs/CameraPlacement.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "helpers.h"

#include "lwrlib/LwrLibrary.hpp"

#include "AccMap.h"


  ///////////////////
 // main function //
///////////////////

void accmap(){

    ////////////////////////////// create publishers and subscribers

    ros::NodeHandle node;
    ros::Subscriber subjoy = node.subscribe("joy", 5, joyCallback);
    ros::Subscriber subkeydn = node.subscribe("keyboard/keydown", 5, keydownCallback);
    ros::Subscriber subkeyup = node.subscribe("keyboard/keyup", 5, keyupCallback);
    for(int i=0; i<1024; i++){keyState[i] = false;}

    double dt = 1.0/10.0;//1.0/10.0;
    double t = 0;
    ros::Rate rate(1.0/dt);
    ros::Publisher publisherAccMap = node.advertise <visualization_msgs::Marker> ("workspace", 1);
    ros::Publisher publisherIntersection = node.advertise <visualization_msgs::Marker> ("wsintersection", 1);
    //ros::Publisher publisherAccMap = node.advertise <visualization_msgs::Marker> ("MarchingCubes", 1);
    ros::Publisher publisherCam = node.advertise <view_controller_msgs::CameraPlacement>("rviz/camera_placement", 1);
    ros::Publisher publisherModel = node.advertise <sensor_msgs::JointState> ("joint_states", 1);
    ros::Publisher publisherTestOris = node.advertise <visualization_msgs::Marker> ("test_oris", 1);
    ros::Publisher publisherWorkpiece = node.advertise <visualization_msgs::Marker> ("workpiece", 1);

    ////////////////////////////// initialize joints for publishing

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
        markerJoints[j].color = colorFromEigen(Vector4d(0,0,0.8,1));
    }

    ////////////////////////////// initialize workpiece

    visualization_msgs::Marker markerWorkpiece = initializeMarker(87, visualization_msgs::Marker::MESH_RESOURCE, 0);
    markerWorkpiece.mesh_resource = "file:///home/mirko/catkin_ws/src/wovi/misc/rubik.dae";
    markerWorkpiece.mesh_use_embedded_materials = true;
    markerWorkpiece.color = colorFromEigen(Vector4d(0.1,0.3,1,1));
    markerWorkpiece.scale.x = markerWorkpiece.scale.y = markerWorkpiece.scale.z = 0.1;
    LwrFrame workpiecePose;

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

    ////////////////////////////// create test orientations

    vector<Quaterniond> testOris(0);
    visualization_msgs::Marker markerTestOris = initializeMarker(30, visualization_msgs::Marker::TRIANGLE_LIST, 0);

/*  testOris.push_back(Quaterniond(1,0,0,0));
    testOris.push_back(Quaterniond(0,1,0,0));
    testOris.push_back(Quaterniond(0,0,1,0));
    testOris.push_back(Quaterniond(0,0,0,1));*/

    vector<Vector3d> arrowMesh(0);
    arrowMesh.push_back(Vector3d(0,0,0));
    arrowMesh.push_back(Vector3d(0,0,1));
    arrowMesh.push_back(Vector3d(0.1,0,1));
    arrowMesh.push_back(Vector3d(0,0,0));
    arrowMesh.push_back(Vector3d(0.1,0,1));
    arrowMesh.push_back(Vector3d(0.1,0,0.8));
    arrowMesh.push_back(Vector3d(0,0,0));
    arrowMesh.push_back(Vector3d(0.1,0,0.8));
    arrowMesh.push_back(Vector3d(0.2,0,0.83));

    ////////////////////////////// prepare GPU stuff

    printf("compiling for GPU...\n");
    AccMap accmap;

    printf("\n\nfilling arrays...\n");

    const uint32_t nTriangles = 100000;
    //const uint32_t nTriangles = 30;

    visualization_msgs::Marker markeraccmap;
    markeraccmap = initializeMarker(0, visualization_msgs::Marker::TRIANGLE_LIST, nTriangles*3);
    visualization_msgs::Marker markeraccmapintersect;
    markeraccmapintersect = initializeMarker(10, visualization_msgs::Marker::TRIANGLE_LIST, nTriangles*3);

    //uint16_t nx = 175, ny = 175, nz = 155;
    uint16_t nx = 47, ny = 47, nz = 42;

    //uint16_t nx = 47*2-1, ny = 47*2-1, nz = 42*2-1;

    //int nx = 36, ny = 36, nz = 26;
//    double x0 =-0.87, y0 =-0.87, z0 =-0.36;
//    double x1 = 0.87, y1 = 0.87, z1 = 1.18;
    double x0 =-0.92, y0 =-0.92, z0 = -0.40;
    double x1 = 0.92, y1 = 0.92, z1 = 1.24;
    double iso = 0.9;

    //double x0=-1.4, y0=-2, z0=-1.7,    x1=2.6, y1=2, z1=2.3;

    printf("Grid cell size: %f x %f x %f cm\n", (x1-x0)/(nx-1)*100, (y1-y0)/(ny-1)*100, (z1-z0)/(nz-1)*100);

    int8_t* voxelData = new int8_t[nx*ny*nz];
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
    FLOAT* targets = new FLOAT[16*nSamples64];
    //printf("\ntargets from 0x%X to 0x%X+.., %d bytes\n", targets, &(targets[16*nSamples64-1]), 16*nSamples64*sizeof(FLOAT));
    uint8_t* configs = new uint8_t[nSamples64];
    //printf("\nconfigs from 0x%X to 0x%X, %d bytes\n", configs, &(configs[nSamples64-1]) + 7, nSamples64*sizeof(uint8_t));

    bool doublePrecision = false;
    #ifdef DOUBLE_PRECISION
        doublePrecision = true;
    #endif

    LwrFrame bufferFrame;

    int iV=0;
    x=x0; for(int ix=0; ix<nx; ix++){y=y0; for(int iy=0; iy<ny; iy++){z=z0; for(int iz=0; iz<nz; iz++){
                bufferFrame.setPos(x, y, z);
                oclFromFrame(&(targets[iV*16]), &bufferFrame);
                configs[iV] = 2;
                iV++;
    z+=dz;} y+=dy;} x+=dx;}


    LwrFrame tool;
    tool.setPos(0,0,0.1);
    tool.setQuat(1,0,0,0);
    LwrFrame invTool = tool.inverse();

    LwrFrame currentTcp;
    LwrXCart currentPose;

    currentTcp.setQuat(1.0, 0.0, 0.0, 0.0);
    currentTcp.setPos(0.0,0.0,1.2);

    currentPose.pose = currentTcp*invTool;

    FLOAT* invToolOcl = new FLOAT[16];

    oclFromFrame(invToolOcl, &invTool);

    printf("\n\ntestrun...\n");
    tic();

    accmap.compute(errors, intervals, (void*) targets, configs, (void*) invToolOcl, nSamples64, doublePrecision);

    toc();
    printf("\n\ndone...\n");


    ////////////////////////////// game loop

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

        cam.focus.point.x = 0;
        cam.focus.point.y = 0;
        cam.focus.point.z = 0.31;

        cam.eye.point.x = cos(camyaw)*cos(campitch)*camdst + cam.focus.point.x;
        cam.eye.point.y = sin(camyaw)*cos(campitch)*camdst + cam.focus.point.y;
        cam.eye.point.z = sin(campitch)*camdst + cam.focus.point.z;



        if(!alternative){
            workpiecePose.ori = AngleAxis<double>(-0.1 * joyAxesState[3], csys.col(1)) * workpiecePose.ori;
            workpiecePose.ori = AngleAxis<double>(-0.1 * joyAxesState[4], csys.col(0)) * workpiecePose.ori;

            workpiecePose.pos = workpiecePose.pos
                              + csys.col(0) * (-0.02*joyAxesState[0])
                              + csys.col(1) * (+0.02*joyAxesState[1]);

            camyaw += -0.2*joyAxesState[6];
            campitch += 0.2*joyAxesState[7];
            if(campitch > 1.57){campitch = 1.57;}
            if(campitch < -1.57){campitch = -1.57;}
        }
        else{
            workpiecePose.ori = AngleAxis<double>( 0.1 * joyAxesState[3], csys.col(2)) * workpiecePose.ori;

            workpiecePose.pos = workpiecePose.pos
                              + csys.col(2) * (-0.02*joyAxesState[1]);

            camdst *= 1 - 0.1*joyAxesState[7];
        }



















        ////////////////////////////// compute IK samples on GPU



        bufferFrame.ori = currentTcp.ori;

        iV = 0;
        x=x0; for(int ix=0; ix<nx; ix++){y=y0; for(int iy=0; iy<ny; iy++){z=z0; for(int iz=0; iz<nz; iz++){
                    bufferFrame.setPos(x,y,z);
                    oclFromFrame(&(targets[iV*16]), &bufferFrame);
                    configs[iV] = 2;
                    iV++;
        z+=dz;} y+=dy;} x+=dx;}

        accmap.compute(errors, intervals, (void*) targets, configs, (void*) invToolOcl, nSamples64, doublePrecision);

        iV = 0;
        x=x0; for(int ix=0; ix<nx; ix++){y=y0; for(int iy=0; iy<ny; iy++){z=z0; for(int iz=0; iz<nz; iz++){
                    voxelData[ix*ny*nz + iy*nz + iz] = (errors[iV] & (LWR_NO_SOLUTION_FOR_ELBOW | LWR_ERROR))>0 ? 10 : -10;
                    iV++;
        z+=dz;} y+=dy;} x+=dx;}

        marchCubes(&markeraccmap, nTriangles, voxelData, x0, y0, z0, dx, dy, dz, nx, ny, nz, iso, 0.3);


        // display intersection of all tested orientations (could take some time)
        if(joyButtonsPressed[8]){
            joyButtonsPressed[8] = false;

            // set all points to reachable
            iV = 0;
            x=x0; for(int ix=0; ix<nx; ix++){y=y0; for(int iy=0; iy<ny; iy++){z=z0; for(int iz=0; iz<nz; iz++){
                        voxelData[ix*ny*nz + iy*nz + iz] = 10;
                        iV++;
            z+=dz;} y+=dy;} x+=dx;}

            for(int iOri=0; iOri<testOris.size(); iOri++){

                bufferFrame.ori = testOris[iOri];

                // fill target array
                iV = 0;
                x=x0; for(int ix=0; ix<nx; ix++){y=y0; for(int iy=0; iy<ny; iy++){z=z0; for(int iz=0; iz<nz; iz++){
                            bufferFrame.setPos(x,y,z);
                            oclFromFrame(&(targets[iV*16]), &bufferFrame);
                            configs[iV] = 2;
                            iV++;
                z+=dz;} y+=dy;} x+=dx;}

                // GPU
                accmap.compute(errors, intervals, (void*) targets, configs, (void*) invToolOcl, nSamples64, doublePrecision);

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



        ////////////////////////////// publish all the stuff

        publisherAccMap.publish(markeraccmap);
        for(int j=0; j<7; j++){
            //jointsPublish.position[j] = jnts[j];
        }
        jointsPublish.header.stamp = ros::Time::now();
        publisherModel.publish(jointsPublish);
        publisherCam.publish(cam);
        pubcsys(currentTcp.pos, currentTcp.ori, "TCP");

        Matrix3d R;
        markerTestOris.points.resize(0);
        markerTestOris.colors.resize(0);

        for(int ip=0; ip<arrowMesh.size(); ip++){
            markerTestOris.points.push_back(positionFromEigen(0.5 * currentTcp.ori * arrowMesh[ip] + currentTcp.pos));
            markerTestOris.colors.push_back(colorFromEigen(Vector4d (0.9,0.9,0.9,1)));
        }
        for(int i=0; i<testOris.size(); i++){
            R = testOris[i];
            for(int ip=0; ip<arrowMesh.size(); ip++){
                markerTestOris.points.push_back(positionFromEigen(0.3 * R * arrowMesh[ip] + currentTcp.pos));
                markerTestOris.colors.push_back(colorFromEigen(Vector4d(testOris[i].x()*0.5+0.5, testOris[i].y()*0.5+0.5, testOris[i].z()*0.5+0.5, 1.0)));
            }
        }

        publisherTestOris.publish(markerTestOris);

        markerWorkpiece.pose.position = positionFromEigen(workpiecePose.pos);
        workpiecePose.getQuat(markerWorkpiece.pose.orientation.w, markerWorkpiece.pose.orientation.x,
                              markerWorkpiece.pose.orientation.y, markerWorkpiece.pose.orientation.z);
        publisherWorkpiece.publish(markerWorkpiece);

        rate.sleep();
        ros::spinOnce();

        t += dt;
        firstRun = false;
        //break;
        // if(t>1){break;}
    }

    delete[] errors;
    delete[] intervals;
    delete[] targets;
    delete[] configs;
    delete[] invToolOcl;
    delete[] voxelData;

}







/* sorted out:


gamepad control first version:


        camyaw += -0.2*accmapJoyAxesState[3];
        campitch += 0.2*accmapJoyAxesState[4];
        if(campitch > 1.57){campitch = 1.57;}
        if(campitch < -1.57){campitch = -1.57;}
        //camdst *= 1 + 0.1*(accmapJoyButtonsState[4]-accmapJoyButtonsState[5]);

        if(accmapJoyButtonsState[9]){autoElbow = true;}

        if(accmapJoyButtonsState[4] || accmapJoyButtonsState[5]){
            autoElbow = false;
            nsparam += (accmapJoyButtonsState[4]-accmapJoyButtonsState[5])*0.04;
        }

        camdst *= 1 + 0.1*(accmapJoyButtonsState[0]-accmapJoyButtonsState[3]);

        cam.eye.point.x = cos(camyaw)*cos(campitch)*camdst + cam.focus.point.x;
        cam.eye.point.y = sin(camyaw)*cos(campitch)*camdst + cam.focus.point.y;
        cam.eye.point.z = sin(campitch)*camdst + cam.focus.point.z;

        Matrix3d ccs = camcsys(camyaw, campitch, true);

        currentTcp.pos = currentTcp.pos
                + ccs.col(0) * (-0.02*accmapJoyAxesState[0])
                + ccs.col(1) * (+0.02*accmapJoyAxesState[1]);

        currentTcp.ori = AngleAxis<double>(0.1 * (accmapJoyAxesState[5]-accmapJoyAxesState[2]),
                ccs.col(2)) * currentTcp.ori;

        Vector3d camfocus(cam.focus.point.x, cam.focus.point.y, cam.focus.point.z);
        camfocus = camfocus + ccs.col(0) * (-0.02*accmapJoyAxesState[6])
                            + ccs.col(1) * (+0.02*accmapJoyAxesState[7]);

        cam.focus.point.x = camfocus.x(); cam.focus.point.y = camfocus.y(); cam.focus.point.z = camfocus.z();










        camyaw += -0.2*joyAxesState[6];
        campitch += 0.2*joyAxesState[7];
        if(campitch > 1.57){campitch = 1.57;}
        if(campitch < -1.57){campitch = -1.57;}
        camdst *= 1 + 0.1*(joyButtonsState[0]-joyButtonsState[3]);

        Matrix3d cc = camcsys(camyaw, campitch, false); // camera coordinates
        Matrix3d ccs = camcsys(camyaw, campitch, true); // camera coordinates, snapped

        cam.focus.point.x = 0;
        cam.focus.point.y = 0;
        cam.focus.point.z = 0.31;

        cam.eye.point.x = cos(camyaw)*cos(campitch)*camdst + cam.focus.point.x;
        cam.eye.point.y = sin(camyaw)*cos(campitch)*camdst + cam.focus.point.y;
        cam.eye.point.z = sin(campitch)*camdst + cam.focus.point.z;

        currentTcp.ori = AngleAxis<double>(-0.1 * joyAxesState[3], cc.col(1)) * currentTcp.ori;
        currentTcp.ori = AngleAxis<double>(-0.1 * joyAxesState[4], cc.col(0)) * currentTcp.ori;
        currentTcp.ori = AngleAxis<double>( 0.1 * (joyAxesState[5]-joyAxesState[2]), cc.col(2)) * currentTcp.ori;
        //currentTcp.ori = AngleAxis<double>(-0.1 *
        //     (accmapJoyAxesState[5]-accmapJoyAxesState[2]), currentTcp.ori.col(2)) * currentTcp.ori;

//        currentTcp.pos = currentTcp.pos
  //              + cc.col(0) * (-0.02*accmapJoyAxesState[0])
    //            + cc.col(1) * (+0.02*accmapJoyAxesState[1]);


        if(joyButtonsPressed[2]){
            joyButtonsPressed[2] = false;
            testOris.push_back(Quaterniond(currentTcp.ori));
        }
        if(joyButtonsPressed[1]){
            joyButtonsPressed[1] = false;
            testOris.resize(0);
        }









  elbow optimization using bisection was sucky

        testPose.nsparam = nsparam;
        Lwr::inverseKinematics(currentJoints, testPose);
        double costm = elbowCost2(currentJoints, currentJoints);

        testPose.nsparam = nspl;
        Lwr::inverseKinematics(testJoints, testPose);
        double costl = elbowCost2(testJoints, currentJoints);

        testPose.nsparam = nspr;
        Lwr::inverseKinematics(testJoints, testPose);
        double costr = elbowCost2(testJoints, currentJoints);

        for(int iTrial=0; iTrial<nTrials; iTrial++){ // bisection

            if(costl < costr){
                nspr = nspm;
                costr = costm;
            }
            else{
                nspl = nspm;
                costl = costm;
            }

            nspm = 0.5*(nspr+nspl);

            testPose.nsparam = nspm;
            Lwr::inverseKinematics(testJoints, testPose);
            costm = elbowCost2(testJoints, currentJoints);
        }






*/
