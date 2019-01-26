
#include "LwrLibraryClasses.hpp"

/////////////////////
// class LwrJoints //
/////////////////////

LwrJoints::LwrJoints(){
    for(int i=0; i<7; i++){
        j[i]=0;
    }
}
LwrJoints::LwrJoints(double j1, double j2, double j3, double j4, double j5, double j6, double j7){
	j[0]=j1; j[1]=j2; j[2]=j3; j[3]=j4; j[4]=j5; j[5]=j6; j[6]=j7;
}

void LwrJoints::setJoints(double values[]){
    for(int i=0; i<7; i++){
        j[i]=values[i];
    }
}
void LwrJoints::setJoints(std::vector<double> &values){
    for(int i=0; i<7; i++){
        j[i]=values[i];
    }
}
void LwrJoints::setJoints(double j1, double j2, double j3, double j4, double j5, double j6, double j7){
    j[0]=j1; j[1]=j2; j[2]=j3; j[3]=j4; j[4]=j5; j[5]=j6; j[6]=j7;
}
void LwrJoints::setJoints(Matrix <double,7,1> &values){
	for(int i=0; i<7; i++){
        j[i]=values[i];
    }
}

void LwrJoints::getJoints(double values[]){
    for(int i=0; i<7; i++){
        values[i]=j[i];
    }
}
void LwrJoints::getJoints(std::vector<double> &values){
    for(int i=0; i<7; i++){
        values[i]=j[i];
    }
}
void LwrJoints::getJoints(Matrix <double,7,1> &values){
	for(int i=0; i<7; i++){
        values[i]=j[i];
    }
}

////////////////////
// class LwrFrame //
////////////////////

LwrFrame::LwrFrame(){
    ori=Matrix3d::Identity();
    pos=Vector3d(0,0,0);
}

void LwrFrame::setOri(double values[], bool rowwise){
    if(rowwise){
        ori << values[0],values[1],values[2],
               values[3],values[4],values[5],
               values[6],values[7],values[8];
    }
    else{
        ori << values[0],values[3],values[6],
               values[1],values[4],values[7],
               values[2],values[5],values[8];
    }
}
void LwrFrame::setOri(std::vector<double> &values, bool rowwise){
    setOri(values.data(),rowwise);
}
void LwrFrame::getOri(double values[], bool rowwise){
    if(rowwise){
        values[0]=ori(0,0); values[1]=ori(0,1); values[2]=ori(0,2);
        values[3]=ori(1,0); values[4]=ori(1,1); values[5]=ori(1,2);
        values[6]=ori(2,0); values[7]=ori(2,1); values[8]=ori(2,2);
    }
    else{
        values[0]=ori(0,0); values[3]=ori(0,1); values[6]=ori(0,2);
        values[1]=ori(1,0); values[4]=ori(1,1); values[7]=ori(1,2);
        values[2]=ori(2,0); values[5]=ori(2,1); values[8]=ori(2,2);
    }
}
void LwrFrame::getOri(std::vector<double> &values, bool rowwise){
    getOri(values.data(),rowwise);
}

void LwrFrame::setQuat(double w, double x,double y, double z){
    ori=(Matrix3d)Quaterniond(w,x,y,z);
}
void LwrFrame::setQuat(double values[]){
    setQuat(values[0],values[1],values[2],values[3]);
}
void LwrFrame::setQuat(std::vector<double> &values){
    setQuat(values[0],values[1],values[2],values[3]);
}
void LwrFrame::getQuat(double &w, double &x,double &y, double &z){
    Quaterniond q=(Quaterniond)ori;
    w=q.w();
    x=q.x();
    y=q.y();
    z=q.z();
}
void LwrFrame::getQuat(double values[]){
    getQuat(values[0],values[1],values[2],values[3]);
}
void LwrFrame::getQuat(std::vector<double> &values){
    getQuat(values[0],values[1],values[2],values[3]);
}

void LwrFrame::setPos(double x, double y, double z){
    pos << x,y,z;
}
void LwrFrame::setPos(double values[]){
    pos << values[0],values[1],values[2];
}
void LwrFrame::setPos(std::vector<double> &values){
    setPos(values.data());
}
void LwrFrame::getPos(double &x, double &y, double &z){
    x=pos[0];
    y=pos[1];
    z=pos[2];
}
void LwrFrame::getPos(double values[]){
    values[0]=pos[0];
    values[1]=pos[1];
    values[2]=pos[2];
}
void LwrFrame::getPos(std::vector<double> &values){
    values[0]=pos[0];
    values[1]=pos[1];
    values[2]=pos[2];
}

#ifdef LWR_LIBRARY_USE_KDL
void LwrFrame::setFrame(KDL::Frame &T){
    setOri(T.M.data,true);
    setPos(T.p.data,true);
}
#endif //LWR_LIBRARY_USE_KDL

LwrFrame LwrFrame::inverse(){
	LwrFrame result;
	result.ori=this->ori.transpose();
	result.pos=-result.ori*this->pos;
	return result;
}

/////////////
// classes //
/////////////

void LwrTwist::setTrans(double vx, double vy, double vz){
    trans << vx,vy,vz;
}
void LwrTwist::setTrans(double values[]){
    trans << values[0],values[1],values[2];
}
void LwrTwist::setTrans(std::vector<double> &values){
    setTrans(values.data());
}
void LwrTwist::getTrans(double &vx, double &vy, double &vz){
    vx=trans[0];
    vy=trans[1];
    vz=trans[2];
}
void LwrTwist::getTrans(double values[]){
    values[0]=trans[0];
    values[1]=trans[1];
    values[2]=trans[2];
}
void LwrTwist::getTrans(std::vector<double> &values){
    values[0]=trans[0];
    values[1]=trans[1];
    values[2]=trans[2];
}

void LwrTwist::setRot(double vx, double vy, double vz){
    rot << vx,vy,vz;
}
void LwrTwist::setRot(double values[]){
    rot << values[0],values[1],values[2];
}
void LwrTwist::setRot(std::vector<double> &values){
    setRot(values.data());
}
void LwrTwist::getRot(double &vx, double &vy, double &vz){
    vx=rot[0];
    vy=rot[1];
    vz=rot[2];
}
void LwrTwist::getRot(double values[]){
    values[0]=rot[0];
    values[1]=rot[1];
    values[2]=rot[2];
}
void LwrTwist::getRot(std::vector<double> &values){
    values[0]=rot[0];
    values[1]=rot[1];
    values[2]=rot[2];
}

void LwrTwist::computeFrom(LwrFrame &pose1, LwrFrame &pose2, double dt){
    Vector3d deltaPos=pose2.pos-pose1.pos;
    trans=deltaPos/dt;

    Matrix3d deltaOri=pose1.ori.transpose()*pose2.ori;
    AngleAxisd deltaOriAA;
    deltaOriAA.fromRotationMatrix(deltaOri);
    rot=pose1.ori*deltaOriAA.axis()*(deltaOriAA.angle()/dt);
}

LwrFrame LwrTwist::applyTo(LwrFrame &pose1, double dt){
    LwrFrame result;

    if(rot.norm()<=1e-10){
        return pose1;
    }
    AngleAxisd deltaOriAA(rot.norm()*dt,rot.normalized());
    result.ori=deltaOriAA*pose1.ori;

    result.pos=pose1.pos+trans*dt;

    return result;
}

///////////////
// operators //
///////////////

LwrFrame operator*(LwrFrame T1, LwrFrame T2){
	LwrFrame result;
	result.ori=T1.ori*T2.ori;
	result.pos=T1.ori*T2.pos+T1.pos;
	return result;
}
Vector3d operator*(LwrFrame T, Vector3d v){
	return T.ori*v + T.pos;
}

LwrErrorMsg operator|(LwrErrorMsg e1, LwrErrorMsg e2){
	return (LwrErrorMsg)((int)e1 | (int)e2);
}
LwrErrorMsg operator&(LwrErrorMsg e1, LwrErrorMsg e2){
	return (LwrErrorMsg)((int)e1 & (int)e2);
}

std::ostream& operator<<(std::ostream& stream, LwrJoints const& val){
	stream << "("
			<< val(0) << ", "
			<< val(1) << ", "
			<< val(2) << ", "
			<< val(3) << ", "
			<< val(4) << ", "
			<< val(5) << ", "
			<< val(6) << ") rad = ("
			<< val(0)/M_PI*180.0 << ", "
			<< val(1)/M_PI*180.0 << ", "
			<< val(2)/M_PI*180.0 << ", "
			<< val(3)/M_PI*180.0 << ", "
			<< val(4)/M_PI*180.0 << ", "
			<< val(5)/M_PI*180.0 << ", "
			<< val(6)/M_PI*180.0 << ") deg";
	return stream;
}
std::ostream& operator<<(std::ostream& stream, LwrFrame const& val){
	Eigen::Matrix<double,3,4> output;
	output << val.ori, val.pos;
	stream << output;
	return stream;
}
std::ostream& operator<<(std::ostream& stream, LwrTwist const& val){
	Eigen::Matrix<double,3,2> output;
	output << val.trans, val.rot;
	stream << "v in m/s, omega in rad/s:" << std::endl << output;
	return stream;
}
std::ostream& operator<<(std::ostream& stream, LwrXCart const& val){
	stream << "Pose:" << std::endl << val.pose << std::endl
			<< "Elbow angle: " << val.nsparam << " rad = "
			<< val.nsparam/M_PI*180.0 << " deg" << std::endl
			<< "Configuration: " << val.config << " (B"
			<< ((val.config&1)? "-":"+")
			<< ((val.config&2)? "-":"+")
			<< ((val.config&4)? "-":"+") << "F)";
	return stream;
}
std::ostream& operator<<(std::ostream& stream, LwrXTwist const& val){
	stream << val.twist << std::endl
			<< "Elbow speed: " << val.dnsparamdt << " rad/s";
	return stream;
}
std::ostream& operator<<(std::ostream& stream, LwrElbowInterval const& val){
	if(!val.valid){
		stream << "[]";
	}
	else{
		stream << "[" << val.lower << ", " << val.upper << "]";
	}
	return stream;
}

