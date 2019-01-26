
#include "helpers.h"

////////////////////////////// forward kinematics

LwrFrame dh(double d, double theta, double a, double alpha){
    LwrFrame result;
    double ca=cos(alpha); double sa=sin(alpha);	double ct=cos(theta); double st=sin(theta);
    result.ori << ct, -st*ca,  st*sa,
                  st,  ct*ca, -ct*sa,
                   0,     sa,     ca;
    result.pos << a*ct, a*st, d;
    return result;
}

void fk(LwrFrame jointPositions[], LwrJoints& jointAngles){

    LwrFrame T[7];
    T[0]=dh( 0.31, jointAngles[0],0 ,  0.5*M_PI);
    T[1]=dh(    0, jointAngles[1],0 , -0.5*M_PI);
    T[2]=dh( 0.40, jointAngles[2],0 , -0.5*M_PI);
    T[3]=dh(    0, jointAngles[3],0 ,  0.5*M_PI);
    T[4]=dh( 0.39, jointAngles[4],0 ,  0.5*M_PI);
    T[5]=dh(    0, jointAngles[5],0 , -0.5*M_PI);
    T[6]=dh(0.078, jointAngles[6],0 ,  0);

    jointPositions[0] = LwrFrame();
    for(int j=1; j<=7; j++){
        jointPositions[j] = jointPositions[j-1] * T[j-1];
    }
}


////////////////////////////// compute csys for control depending on viewpoint

Matrix3d camcsys(double camyaw, double campitch, bool snap){
    // Props to Simon Notheis

    double cp = campitch;
    double cy = camyaw;

    if(snap){
        cp = roundf(cp*2/M_PI)*M_PI/2;
        cy = roundf(cy*2/M_PI)*M_PI/2;
    }

    Vector3d z(cos(cy)*cos(cp), sin(cy)*cos(cp), sin(cp));
    Vector3d x(cos(cy+M_PI/2), sin(cy+M_PI/2), 0);
    Vector3d y = z.cross(x);

    Matrix3d res;
    res << x, y, z;
    return res;
}

////////////////////////////// snap orientation to grid

//Todo: get Oktaederwinkel straight

vector<Quaterniond> generateOris(){

    double dyaw=M_PI/4;
    double droll,dpitch;
    double yaw,pitch,roll;

    vector<Quaterniond> result;

    Quaterniond q;

    for(int ipitch=-2; ipitch<=2; ipitch++){
        int iyawmax=7;
        if(ipitch==2 || ipitch==-2){iyawmax=0;}
        for(int iyaw=0; iyaw<=iyawmax; iyaw++){
            int irollmax=7;
            droll=M_PI/4;
            dpitch=M_PI/4;
            if((iyaw+100)%2==1 && (ipitch+100)%2==1){
                irollmax=11;
                droll=M_PI/6;
                dpitch=atan(1/sqrt(2));
            }

            for(int iroll=0; iroll<=irollmax; iroll++){

                yaw = iyaw*dyaw;
                pitch = ipitch*dpitch;
                roll = iroll*droll;
                //printf("yaw=%4.2fpi, pitch=%4.2fpi, roll=%4.2fpi \n", yaw/M_PI, pitch/M_PI, roll/M_PI);
                q = AngleAxisd(yaw, Vector3d::UnitX()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(roll, Vector3d::UnitZ());
                //printf("<%f, %f, %f, %f>\n", q.w(), q.x(), q.y(), q.z());
                result.push_back(q);
            }
            //printf("\n");
        }
    }
    //printf(">%d<", d);
    return result;
}

Matrix3d snapOri(Matrix3d src, vector<Quaterniond> &list){

    int iclosest = -1;
    double dclosest = -2;

    Quaterniond qsrc;
    qsrc = src;

    for(int i=0; i<list.size(); i++){
        double d = fabs(qsrc.w()*list[i].w() + qsrc.x()*list[i].x() + qsrc.y()*list[i].y() + qsrc.z()*list[i].z());

        if(d>dclosest){
            dclosest = d;
            iclosest = i;
        }
    }

    Matrix3d result;
    result = list[iclosest];
    return result;
}


////////////////////////////// integer to string

string int2str (int a)
{
    ostringstream temp;
    temp<<a;
    return temp.str();
}

////////////////////////////// publish a csys for display in rviz

void pubcsys(Vector3d pos, Matrix3d ori, const char* name){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pos(0), pos(1), pos(2)) );
    Quaterniond qeigen(ori);
    tf::Quaternion qros(qeigen.x(), qeigen.y(), qeigen.z(), qeigen.w());
    transform.setRotation(qros);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "lwr1_world_link", name));
}

void pubcsys(Vector3d pos, Vector4d xyzw, const char* name){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pos(0), pos(1), pos(2)) );
    tf::Quaternion qros(xyzw(0), xyzw(1), xyzw(2), xyzw(3));
    transform.setRotation(qros);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "lwr1_world_link", name));
}

////////////////////////////// initialize markers

visualization_msgs::Marker initializeMarker(int id, int type, int nPoints){

    visualization_msgs::Marker result;
    result.header.frame_id = "/lwr1_world_link";
    //result.header.frame_id = "/map";
    result.header.stamp = ros::Time();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    result.ns = "wovi";
    result.id = id;
    result.type = type;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    result.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    result.pose.position.x = result.pose.position.y = result.pose.position.z = 0.0;
    result.pose.orientation.x = result.pose.orientation.y = result.pose.orientation.z = 0.0;
    result.pose.orientation.w = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    result.color.r = 0.0; result.color.g = 1.0; result.color.b = 0.0; result.color.a = 1.0;
    result.lifetime = ros::Duration();

    result.scale.x = result.scale.y = result.scale.z = 1;
    result.points = std::vector<geometry_msgs::Point>(nPoints);
    result.colors = std::vector<std_msgs::ColorRGBA> (nPoints);

    for(int iv=0; iv<nPoints; iv++){
        result.points[iv].x = 0.0;
        result.points[iv].y = 0.0;
        result.points[iv].z = 0.0;
        result.colors[iv].r = 1.0;
        result.colors[iv].g = 0.0;
        result.colors[iv].b = 0.0;
        result.colors[iv].a = 0.5;
    }

    return result;
}

////////////////////////////// convert Eigen vector to ros message

geometry_msgs::Point positionFromEigen(Vector3d v){
    geometry_msgs::Point res;
    res.x = v.x();
    res.y = v.y();
    res.z = v.z();
    return res;
}

geometry_msgs::Quaternion orientationFromEigen(Vector4d v){
    geometry_msgs::Quaternion res;
    res.w = v[0];
    res.x = v[1];
    res.y = v[2];
    res.z = v[3];
    return res;
}

std_msgs::ColorRGBA colorFromEigen(Vector4d v){
    std_msgs::ColorRGBA res;
    res.r = v.x();
    res.g = v.y();
    res.b = v.z();
    res.a = v.w();
    return res;
}


////////////////////////////// color the joints according to the angles
////////////////////////////// indicating limits and singularities

std_msgs::ColorRGBA jointColor(int joint, double value, Vector4d deflt){
    Vector4d resEigen;
    std_msgs::ColorRGBA resMsg;

    double cost;
    const double rad120 = 120.0*M_PI/180.0;
    const double rad170 = 170.0*M_PI/180.0;

    if(joint % 2 == 0){ // torsion joint
        if(fabs(value) > rad170){
            resEigen = Vector4d(1.0, 0.0, 0.0, 1.0);
        }
        else{
            cost = pow(value/rad170,4.0)*0.7;
            resEigen = cost*Vector4d(1.0, 0.0, 0.0, 1.0) + (1.0-cost)*deflt;
        }
    }
    else{ // hinge joint
        if(fabs(value) > rad120){
            resEigen = Vector4d(1.0, 0.0, 0.0, 1.0);
        }
        else{
            cost = pow(fabs(2.0*(value/rad120))-1, 4.0);
            if(fabs(value) < rad120/2){ // singularity
                resEigen = cost*Vector4d(0.0, 0.0, 0.2, 1.0) + (1.0-cost)*deflt;
            }
            else{ // jointlimit
                cost=cost*0.7;
                resEigen = cost*Vector4d(1.0, 0.0, 0.0, 1.0) + (1.0-cost)*deflt;
            }
        }
    }

    resMsg.r = resEigen.x();
    resMsg.g = resEigen.y();
    resMsg.b = resEigen.z();
    resMsg.a = deflt[3];

    return resMsg;
}

////////////////////////////// elbow position cost function
////////////////////////////// relative to current position: use joints before
////////////////////////////// absolute: same value for joints and joints before

double elbowCost(const LwrJoints &joints, const LwrJoints &jointsBefore){

    double cost=0;

    const double RAD170 = M_PI*170.0/180.0;
    const double RAD120 = M_PI*120.0/180.0;

    // if the jump in the joint angles is too big, dont accept the solution
    for (int j=0; j<7; j++){
        if(fabs(joints(j)-jointsBefore(j)) > 1.57){
            cost = 1e10;
            return cost;
        }
    }

    // keep joints away from limits and singularities
    for(int j=0; j<7; j++){
            if(j%2==0){ // torsion joint
                    if(fabs(joints(j))>RAD170){
                            cost = 1e10;
                    }
                    else{
                            cost += pow(atanh(joints(j)/RAD170),2.0);
                    }
            }
            else{ // hinge joints
                    if(fabs(joints(j))>RAD120){
                            cost = 1e10;
                    }
                    else{
                            cost += pow(atanh(fabs(2.0*(joints(j)/RAD120))-1.0),2.0);
                    }
            }
    }

    return cost;
}

////////////////////////////// different cost function
////////////////////////////// <0 if all limits are ok, >0 if at least one joint out of limit

double elbowCost2(const LwrJoints &joints, const LwrJoints &jointsBefore){

    const double k170 = 170.0/180.0;
    const double b170 = M_PI*k170/(1.0-2.0*k170);
    const double a170 = (k170-1.0)/k170 * b170*b170;
    const double c170 = -a170/b170;

    const double k120 = 120.0/180.0;
    const double b120 = M_PI*k120/(1.0-2.0*k120);
    const double a120 = (k120-1.0)/k120 * b120*b120;
    const double c120 = -a120/b120;

    const double k5 = 5.0/180.0;
    const double b5 = M_PI*k5/(1.0-2.0*k5);
    const double a5 = (k5-1.0)/k5 * b5*b5;
    const double c5 = -a5/b5;

    double fitpart[14];

    // fitness function designed so that
    // f(0)=1, f'(0)=0, f(180)=-1, f'(180)=0, f(120 or 170)=0

    // torsion joints:
    for(int j=0; j<7; j+=2){
        fitpart[j] = cos( a170/(fabs(joints(j))+b170) + c170);
    }
    // hinge joints:
    for(int j=1; j<7; j+=2){
        fitpart[j] = cos( a120/(fabs(joints(j))+b120) + c120);
    }
    // singularities (defined at +-5deg)
    fitpart[7] = -cos( a5/(fabs(joints(1))+b5) + c5);
    fitpart[8] = -cos( a5/(fabs(joints(3))+b5) + c5);
    fitpart[9] = -cos( a5/(fabs(joints(5))+b5) + c5);

    // distance to joints before (has to be smaller than 180deg)
    // only torsion joints relevant
    fitpart[10] = cos( 0.5*(joints(0)-jointsBefore(0)) );
    fitpart[11] = cos( 0.5*(joints(2)-jointsBefore(2)) );
    fitpart[12] = cos( 0.5*(joints(4)-jointsBefore(4)) );
    fitpart[13] = cos( 0.5*(joints(6)-jointsBefore(6)) );

    bool allGreater0 = true;

    for(int i=0; i<14; i++){
        if(fitpart[i]<=0){
            allGreater0 = false;
        }
    }

    if(allGreater0){
        double denom = 0;
        for(int i=0; i<14; i++){
            denom += 1.0/fitpart[i];
        }
        return -1.0/denom; // minus turns fitness into cost
    }
    else{
        double radic = 0;
        for(int i=0; i<14; i++){
            if(fitpart[i]<0){
                radic += pow(fitpart[i],2); // turns fitness into cost
            }
        }
        return sqrt(radic);
    }
}

///////////////////////////// marching cubes wrapper

void marchCubes(visualization_msgs::Marker* pmarker, int nTriangles,
                float voxelData[],
                float x0, float y0, float z0, float dx, float dy, float dz,
                int nx, int ny, int nz, float iso, float alpha){

    pmarker->points.resize(nTriangles*3);
    pmarker->colors.resize(nTriangles*3);

    Vector4d gridCell[8];
    uint32_t iTri = 0;

    float x=0; float y=0; float z=0;

    x=x0; for(int ix=0; ix<nx-1; ix++){y=y0; for(int iy=0; iy<ny-1; iy++){z=z0; for(int iz=0; iz<nz-1; iz++){

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

        int num = marchingCube(gridCell, iso, &(pmarker->points[3*iTri]));

        //bool degenerated = false;

        for(int it=0; it<num; it++){

            Vector3f v1(pmarker->points[3*(iTri+it)+0].x, pmarker->points[3*(iTri+it)+0].y, pmarker->points[3*(iTri+it)+0].z);
            Vector3f v2(pmarker->points[3*(iTri+it)+1].x, pmarker->points[3*(iTri+it)+1].y, pmarker->points[3*(iTri+it)+1].z);
            Vector3f v3(pmarker->points[3*(iTri+it)+2].x, pmarker->points[3*(iTri+it)+2].y, pmarker->points[3*(iTri+it)+2].z);

            /*if((v1-v2).norm()<1e-10 || (v1-v3).norm()<1e-10 || (v3-v2).norm()<1e-10){
                degenerated = true;
                printf("Degenerated!");
            }*/

            Vector3f normal = (v2-v1).cross(v3-v1);
            normal.normalize();
            normal[0] = normal[0]*0.5 + 0.5;
            normal[1] = normal[1]*0.5 + 0.5;
            normal[2] = normal[2]*0.5 + 0.5;

            for(int ic=0; ic<3; ic++){
                pmarker->colors[3*(iTri+it)+ic].r = normal[0];
                pmarker->colors[3*(iTri+it)+ic].g = normal[1];
                pmarker->colors[3*(iTri+it)+ic].b = normal[2];
                pmarker->colors[3*(iTri+it)+ic].a = alpha;
            }
        }

        iTri += num;

    z += dz;} y += dy;} x += dx;}
    breakloops:

    pmarker->points.resize(iTri*3);
    pmarker->colors.resize(iTri*3);


}

///////////////////////////// LwrFrame -> OCL array converter

void oclFromFrame(FLOAT pdst[], LwrFrame* psrc){
    pdst[0] = psrc->ori(0,0); pdst[4] = psrc->ori(0,1); pdst[ 8] = psrc->ori(0,2);  pdst[12] = psrc->pos(0);
    pdst[1] = psrc->ori(1,0); pdst[5] = psrc->ori(1,1); pdst[ 9] = psrc->ori(1,2);  pdst[13] = psrc->pos(1);
    pdst[2] = psrc->ori(2,0); pdst[6] = psrc->ori(2,1); pdst[10] = psrc->ori(2,2);  pdst[14] = psrc->pos(2);
    pdst[3] = 0;              pdst[7] = 0;              pdst[11] = 0;               pdst[15] = 1;
}
