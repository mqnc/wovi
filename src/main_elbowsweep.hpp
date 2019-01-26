
#include "scl_custom.h"



void elbowSweep(){

    ros::NodeHandle node;
    double dt = 1.0/20.0;
    ros::Rate rate(1.0/dt);
    ros::Publisher publisher = node.advertise <visualization_msgs::Marker> ("workspace", 1);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "lwr1_world_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "wovi";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST; // ::POINTS
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0;
    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = marker.scale.y = marker.scale.z = 1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    const uint32_t nTriangles = 100000;

    marker.points = std::vector<geometry_msgs::Point>(nTriangles*3);
    marker.colors = std::vector<std_msgs::ColorRGBA> (nTriangles*3);

    //uint16_t nx = 175, ny = 175, nz = 155;
    uint16_t nx = 71, ny = 71, nz = 51;
    double x0 =-0.87, y0 =-0.87, z0 =-0.36;
    double x1 = 0.87, y1 = 0.87, z1 = 1.18;
    double iso = 0.9;

    int8_t* voxelData = new int8_t[nx*ny*nz];
    double dx = (x1-x0)/(double)(nx-1);
    double dy = (y1-y0)/(double)(ny-1);
    double dz = (z1-z0)/(double)(nz-1);
    double x = x0;
    double y = y0;
    double z = z0;

    int ix, iy, iz;

    double t = 0;

    printf("compiling for GPU...\n");
    IK ik;
    printf("filling arrays...\n");
    int nVerts = nx*ny*nz; // nodes around the robot
    nVerts = (nVerts/64+1)*64; // increased so 64 devides it

    int8_t* error = new int8_t[nVerts];
    int16_t* joints = new int16_t[nVerts*8];
    int16_t jointLimits[] = {30600, 21600, 30600, 21600, 30600, 21600, 30600};
    FLOAT* target = new FLOAT[nVerts*16];
    FLOAT* nsparam = new FLOAT[nVerts];
    int8_t* config = new int8_t[nVerts];

    bool doublePrecision = false;
    #ifdef DOUBLE_PRECISION
        doublePrecision = true;
    #endif

    int iV=0;

    x=x0; for(ix=0; ix<nx; ix++){y=y0; for(iy=0; iy<ny; iy++){z=z0; for(iz=0; iz<nz; iz++){

                for(int iCell=0; iCell<16; iCell++){
                    target[iV*16+iCell] = 0;
                }
                target[iV*16+0] = target[iV*16+5] = target[iV*16+10] = target[iV*16+15] = 1;
                target[iV*16+12] = x;
                target[iV*16+13] = y;
                target[iV*16+14] = z;
                nsparam[iV] = 1;
                config[iV] = 2;

                iV++;

    z+=dz;} y+=dy;} x+=dx;}


    printf("GPUing...\n");
    tic();
    ik.compute(error, joints, target, nsparam, config, nVerts, doublePrecision);
    toc();
    printf("done.\n");

    iV = 0;
    x=x0; for(ix=0; ix<nx; ix++){y=y0; for(iy=0; iy<ny; iy++){z=z0; for(iz=0; iz<nz; iz++){

                /*float badness = 0;
                float maxbadness = 0;
                for(int j=0; j<7; j++){
                    if(j%2 == 0){ // torsion joint
                        badness = (float)(fabs(joints[iV*8 + j]) - jointLimits[j]) / (float)jointLimits[j];*/

                voxelData[ix*ny*nz + iy*nz + iz] = (error[iV] & (LWR_JOINTLIMIT | LWR_ERROR))>0 ? 10 : -10;

                iV++;
    z+=dz;} y+=dy;} x+=dx;}


    while (ros::ok())
    {

        for(iV=0; iV<nVerts; iV++){
            nsparam[iV] = t*0.3;
        }

        ik.compute(error, joints, target, nsparam, config, nVerts, doublePrecision);

        iV = 0;
        x=x0; for(ix=0; ix<nx; ix++){y=y0; for(iy=0; iy<ny; iy++){z=z0; for(iz=0; iz<nz; iz++){

                    voxelData[ix*ny*nz + iy*nz + iz] = (error[iV] & (LWR_JOINTLIMIT | LWR_ERROR))>0 ? 10 : -10;

                    iV++;
        z+=dz;} y+=dy;} x+=dx;}


        Vector4d gridCell[8];
        uint32_t iTri = 0;

        x=x0; for(ix=0; ix<nx-1; ix++){y=y0; for(iy=0; iy<ny-1; iy++){z=z0; for(iz=0; iz<nz-1; iz++){

            if(iTri > nTriangles-5){
                goto breakloops;
            }

            gridCell[0][0] = x   ; gridCell[0][1] = y   ; gridCell[0][2] = z   ; gridCell[0][3] = voxelData[(ix  )*ny*nz + (iy  )*nz + iz  ];
            gridCell[1][0] = x+dx; gridCell[1][1] = y   ; gridCell[1][2] = z   ; gridCell[1][3] = voxelData[(ix+1)*ny*nz + (iy  )*nz + iz  ];
            gridCell[2][0] = x+dx; gridCell[2][1] = y+dy; gridCell[2][2] = z   ; gridCell[2][3] = voxelData[(ix+1)*ny*nz + (iy+1)*nz + iz  ];
            gridCell[3][0] = x   ; gridCell[3][1] = y+dy; gridCell[3][2] = z   ; gridCell[3][3] = voxelData[(ix  )*ny*nz + (iy+1)*nz + iz  ];
            gridCell[4][0] = x   ; gridCell[4][1] = y   ; gridCell[4][2] = z+dz; gridCell[4][3] = voxelData[(ix  )*ny*nz + (iy  )*nz + iz+1];
            gridCell[5][0] = x+dx; gridCell[5][1] = y   ; gridCell[5][2] = z+dz; gridCell[5][3] = voxelData[(ix+1)*ny*nz + (iy  )*nz + iz+1];
            gridCell[6][0] = x+dx; gridCell[6][1] = y+dy; gridCell[6][2] = z+dz; gridCell[6][3] = voxelData[(ix+1)*ny*nz + (iy+1)*nz + iz+1];
            gridCell[7][0] = x   ; gridCell[7][1] = y+dy; gridCell[7][2] = z+dz; gridCell[7][3] = voxelData[(ix  )*ny*nz + (iy+1)*nz + iz+1];

            int num = marchingCube(gridCell, iso, &(marker.points[3*iTri]));

            for(int it=0; it<num; it++){

                Vector3f v1(marker.points[3*(iTri+it)+0].x, marker.points[3*(iTri+it)+0].y, marker.points[3*(iTri+it)+0].z);
                Vector3f v2(marker.points[3*(iTri+it)+1].x, marker.points[3*(iTri+it)+1].y, marker.points[3*(iTri+it)+1].z);
                Vector3f v3(marker.points[3*(iTri+it)+2].x, marker.points[3*(iTri+it)+2].y, marker.points[3*(iTri+it)+2].z);

                Vector3f normal = (v2-v1).cross(v3-v1);
                normal.normalize();
                normal[0] = normal[0]*0.5 + 0.5;
                normal[1] = normal[1]*0.5 + 0.5;
                normal[2] = normal[2]*0.5 + 0.5;

                for(int ic=0; ic<3; ic++){
                    marker.colors[3*(iTri+it)+ic].r = normal[0];
                    marker.colors[3*(iTri+it)+ic].g = normal[1];
                    marker.colors[3*(iTri+it)+ic].b = normal[2];
                    marker.colors[3*(iTri+it)+ic].a = 1.0;
                }
            }

            iTri += num;

        z += dz;} y += dy;} x += dx;}
        breakloops:

        marker.points.resize(iTri*3);
        marker.colors.resize(iTri*3);

        // Publish the marker
        while (publisher.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        publisher.publish(marker);

        marker.points.resize(nTriangles*3);
        marker.colors.resize(nTriangles*3);

        rate.sleep();


        t += dt;
    }

    delete[] error;
    delete[] joints;
    delete[] target;
    delete[] nsparam;
    delete[] config;
    delete[] voxelData;

}
