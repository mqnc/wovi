
#include "LwrLibrary.hpp"
#include <iostream>
#include "derivator.hpp"

// TODO remove this, it's just to make syntax highlighting work
//#define FRI_CONNECTION

/*! \brief Link lengths of the LWR in meters
 */
const double Lwr::limbs[4]={0.31,0.4,0.39,0.078};

/*! \brief Joint limits of the LWR in radiants
 *
 *  (170deg for torsion joints, 120deg for torsion joints)
 */
const LwrJoints Lwr::jointLimits(
	170.0/180.0*M_PI,
		120.0/180.0*M_PI,
	170.0/180.0*M_PI,
		120.0/180.0*M_PI,
	170.0/180.0*M_PI,
		120.0/180.0*M_PI,
	170.0/180.0*M_PI);

/*! \brief x^2 (internal helper)*/
inline double square(double x){return x*x;}

/*! \brief cotangens (internal helper)*/
inline double cot(double x){return 1.0/tan(x);}

/*! \brief Constructor */
Lwr::Lwr():jointInterpolator(7){
	robotTimeMsr=0;
	dtMsr=0.001;
	dtSet=0.001;
#ifdef FRI_CONNECTION
	hfri=NULL;
#endif
	std::vector<double> buffer(7);
	for(int j=0; j<7; j++){buffer[j]=-jointLimits(j);}
	jointInterpolator.setPMin(buffer);
	for(int j=0; j<7; j++){buffer[j]=+jointLimits(j);}
	jointInterpolator.setPMax(buffer);

	for(int j=0; j<7; j++){buffer[j]=0.35;} // about 20deg/s[/s]
	jointInterpolator.setVMax(buffer);
	jointInterpolator.setAMax(buffer);

	jointInterpolator.setDt(dtSet);
	jointInterpolator.setMode(1);
}

/*! \brief Initialize FRI connection
 * and receive two packages by the robot in order to update position and velocity measurements
 */
bool Lwr::friConnect(int port){
#ifndef FRI_CONNECTION
	return false;
#else
	if(port==-1){hfri=new friRemote();}
	else{hfri=new friRemote(port);}

	friReceive();
	friReceive();

	return true; //friRemote calls exit(1) upon error, so if we arrive here we are already safe
#endif
}

/*! \brief Read robot data from FRI */
bool Lwr::friReceive(bool updateCartesian){ //TODO error treatment
#ifndef FRI_CONNECTION
	return false;
#else

	// receive package through FRI
	int result=hfri->doReceiveData();
	tFriMsrData msr=hfri->getMsrBuf();

	bool init= (robotTimeMsr==0); // true -> this is the first package we get, so don't compute velocities

	// read sample time
	dtSet=msr.intf.desiredCmdSampleTime;

	// read axis-specific values
	if(init){
		dtMsr=0;
		robotTimeMsr=msr.intf.timestamp;
		jointVelMsr.setJoints(0,0,0,0,0,0,0);
	}
	else{
		dtMsr=msr.intf.timestamp-robotTimeMsr;
		robotTimeMsr=msr.intf.timestamp;

		for(int j=0; j<7; j++){
			jointVelMsr[j]=(msr.data.msrJntPos[j]-jointPosMsr[j])/dtMsr;
		}
	}

	//std::cout << "measured: ";
	for(int j=0; j<7; j++){
		jointPosMsr[j]=msr.data.msrJntPos[j];
		//std::cout << jointPosMsr[j] << ", ";
		jointTrqMsr[j]=msr.data.msrJntTrq[j];
		jointTrqExt[j]=msr.data.estExtJntTrq[j];
	}
	//std::cout << std::endl;

	if(updateCartesian){
		LwrErrorMsg lwrerror;

		// compute forward kinematics
		lwrerror = forwardKinematics(xCartPosMsr, jointPosMsr);

		// compute Jacobians
		lwrerror = computeXJacobian(jacobianMsr, jointPosMsr);

		// compute Cartesian velocity from joint velocities
		lwrerror = forwardJacobian(xCartVelMsr,jacobianMsr,jointVelMsr);

		// compute Cartesian FT from joint torques
		lwrerror = xCartFTFromJointTorque(xCartFTMsr,jacobianMsr,jointTrqMsr);
		lwrerror = xCartFTFromJointTorque(xCartFTExt,jacobianMsr,jointTrqExt);
	}

	return true;
#endif
}

//todo: sntJacobian ins kartesische einbedingsen
//interpolate is noch nich definiert

bool Lwr::friCheckCmdMode(){
#ifndef FRI_CONNECTION
	return false;
#else
	return hfri->getState()==FRI_STATE_CMD;
#endif
}

/*! \brief Send robot commands through FRI */
bool Lwr::friSend(bool useCartesian){ //TODO error treatment
#ifndef FRI_CONNECTION
	return false;
#else

	int result;
	float fjoints[7];

	if(!useCartesian){

		for(int i=0;i<7;i++){
			fjoints[i]=jointPosNxt[i];
		}
		jointPosSnt=jointPosNxt;
	}
	else{
		LwrJoints joints;
		LwrErrorMsg lwrerror=inverseKinematics(joints,xCartPosNxt);

		for(int i=0;i<7;i++){
			fjoints[i]=joints[i];
		}
		jointPosSnt=joints;
	}


	result=hfri->doPositionControl(fjoints, false);
	hfri->doSendData();

	return true;
#endif
}

bool Lwr::friClose(){ //TODO error treatment
#ifndef FRI_CONNECTION
	return false;
#else
	delete hfri;
	return true;
#endif
}

void Lwr::interpolate(){
	std::vector <double> buffer(7);
	jointInterpolator.interpolate();

	jointInterpolator.getPNow(buffer);
	jointPosNxt.setJoints(buffer);
	jointInterpolator.getVNow(buffer);
	jointVelNxt.setJoints(buffer);
}

/*! \brief Compute Denavit-Hartenberg matrix from Denavit-Hartenberg parameters
 *
 *  d - offset along previous z to the common normal (for LWR the distance between hinge joints)\n
 *	theta - angle about previous z, from old x to new x (for LWR the joint angles)\n
 *  a - length of the common normal (for LWR 0)\n
 *  alpha - angle about common normal, from old z axis to new z axis (for LWR pi/2, -pi/2 or 0)
 */
LwrFrame Lwr::denavitHartenberg(double d, double theta, double a, double alpha){

	LwrFrame result;

	double ca=cos(alpha);
	double sa=sin(alpha);
	double ct=cos(theta);
	double st=sin(theta);

	result.ori << ct, -st*ca,  st*sa,
	              st,  ct*ca, -ct*sa,
	               0,     sa,     ca;

	result.pos << a*ct, a*st, d;

	return result;
}

/*! \brief Compute forward kinematics
 *
 *  xCartPose (output) - extended Cartesian position of the end-effector for input joint angles\n
 *	jointAngles (input) - joint angles
 */
LwrErrorMsg Lwr::forwardKinematics(LwrXCart& xCartPose, LwrJoints& jointAngles){

	LwrErrorMsg result=LWR_OK;

	LwrFrame T[7];
	T[0]=denavitHartenberg(limbs[0], jointAngles[0],0 ,  0.5*M_PI);
	T[1]=denavitHartenberg(       0, jointAngles[1],0 , -0.5*M_PI);
	T[2]=denavitHartenberg(limbs[1], jointAngles[2],0 , -0.5*M_PI);
	T[3]=denavitHartenberg(       0, jointAngles[3],0 ,  0.5*M_PI);
	T[4]=denavitHartenberg(limbs[2], jointAngles[4],0 ,  0.5*M_PI);
	T[5]=denavitHartenberg(       0, jointAngles[5],0 , -0.5*M_PI);
	T[6]=denavitHartenberg(limbs[3], jointAngles[6],0 ,       0);

	LwrFrame shoulder=T[0]*T[1];
	LwrFrame elbow=shoulder*T[2]*T[3];
	LwrFrame wrist=elbow*T[4]*T[5];
	xCartPose.pose=wrist*T[6];

	Vector3d xsw=wrist.pos-shoulder.pos;
	Vector3d xse=elbow.pos-shoulder.pos;

	Vector3d usin=Vector3d(0,0,1).cross(xsw);
	if(usin.norm() < 1e-7){
		// the Cartesian pose is still correct and stable, only the nsparam is ambiguous and might be nan
		result = LWR_WARNING | LWR_SINGULARITY;
	}
	usin.normalize();
	Vector3d ucos=xsw.cross(usin);
	ucos.normalize();

	xCartPose.nsparam=atan2(xse.dot(usin), xse.dot(ucos));

	xCartPose.config=
		(double)(jointAngles(1)<0) +
		(double)(jointAngles(3)<0)*2 +
		(double)(jointAngles(5)<0)*4;

	return result;

}

/*! \brief Compute inverse kinematics
 *
 *	jointAngles (output) - joint angles for input extended Cartesian position of the end-effector\n
 *  xCartPose (input) - extended Cartesian position of the end-effector
 */
LwrErrorMsg Lwr::inverseKinematics(LwrJoints& jointAngles, LwrXCart& xCartPose){

	LwrErrorMsg result=LWR_OK;

	LwrFrame mfw; // wrist pose in flange coordinates
	mfw.pos[2]=-limbs[3];

	LwrFrame mw=xCartPose.pose*mfw; // wrist pose

	Vector3d xw=mw.pos; // wrist pos
	Vector3d xs(0,0,limbs[0]); // shoulder pos

	Vector3d xf=xCartPose.pose.pos; // flange pos

	Vector3d xsw=xw-xs; // vector from shoulder to wrist
	double lsw=xsw.norm(); // length from shoulder to wrist

	if(lsw>=limbs[1]+limbs[2]){
		result=LWR_ERROR | LWR_TARGET_TOO_FAR;
		return result;
	}

	if(lsw<=fabs(limbs[1]-limbs[2])){
		result=LWR_ERROR | LWR_TARGET_TOO_CLOSE;
		return result;
	}

	// find the position of the elbow:
	Vector3d usw=xsw/lsw; // unit vector from shoulder to wrist
	Vector3d xwf=xf-xw; //vector from wrist to flange
	double cosphi=(lsw*lsw+limbs[1]*limbs[1]-limbs[2]*limbs[2]) / (2.0*lsw*limbs[1]); // cos of elbow angle
	double lseproj=cosphi*limbs[1]; // distance from shoulder to <elbow projected on <the line from shoulder to wrist>>
	double hse=sqrt(limbs[1]*limbs[1] - lseproj*lseproj); // distance between elbow and shoulder-wrist-line
	Vector3d xeproj=xs+usw*lseproj; // elbow position projected on the shoulder-wrist-line
	// (= point around which the elbow can rotate)

	// find direction vectors of the plane in which the elbow can rotate (usin and ucos):
	Vector3d x(1,0,0), y(0,1,0), z(0,0,1);

	Vector3d usin= z.cross(xsw);
	if(usin.norm()<1e-7){
		// this is actually not necessarily a real singularity,
		// just the here chosen nullspace parameter does not work in this situation
		result=LWR_ERROR | LWR_SINGULARITY; return result;}
	usin.normalize();
	Vector3d ucos= xsw.cross(usin);
	ucos.normalize();

	// actual elbow point
	Vector3d xe= xeproj+(cos(xCartPose.nsparam)*ucos+sin(xCartPose.nsparam)*usin)*hse;
	Vector3d xse= xe-xs;
	Vector3d xew= xw-xe;

	// joint axes
	Vector3d axs= xse.cross(z);
	if(axs.norm()<1e-7){
		result=LWR_ERROR | LWR_SINGULARITY; return result;}
	axs.normalize();
	Vector3d axe= xew.cross(xse);
	if(axe.norm()<1e-7){
		result=LWR_ERROR | LWR_SINGULARITY; return result;}
	axe.normalize();
	Vector3d axw= xwf.cross(xew);
	if(axw.norm()<1e-7){
		result=LWR_ERROR | LWR_SINGULARITY; return result;}
	axw.normalize();
	Vector3d axf= -xCartPose.pose.ori.col(1);

	// axes perpend. to joint axes and to a vector along the next link
	Vector3d axns= axs.cross(xse);
	axns.normalize();
	Vector3d axne= axe.cross(xew);
	axne.normalize();
	Vector3d axnw= axw.cross(xwf);
	axnw.normalize();
	Vector3d axnf= xCartPose.pose.ori.col(0);

	// joint angles
	jointAngles[0]=atan2(axs.dot(-x),axs.dot(y));
	jointAngles[1]=acos(xs.dot(xse)/limbs[0]/limbs[1]);
	jointAngles[2]=atan2(axe.dot(axns),-axe.dot(axs));
	jointAngles[3]=acos(xse.dot(xew)/limbs[1]/limbs[2]);
	jointAngles[4]=atan2(axw.dot(axne),-axw.dot(axe));
	jointAngles[5]=acos(xew.dot(xwf)/limbs[2]/limbs[3]);
	jointAngles[6]=atan2(-axf.dot(-axnw),axf.dot(-axw));

	// invert joints according to the selected config
	if((xCartPose.config & 1) > 0){
		jointAngles[1]=-jointAngles[1];
		jointAngles[0]=jointAngles[0]+M_PI;
		jointAngles[2]=jointAngles[2]+M_PI;
	}
	if((xCartPose.config & 2) > 0){
		jointAngles[3]=-jointAngles[3];
		jointAngles[2]=jointAngles[2]+M_PI;
		jointAngles[4]=jointAngles[4]+M_PI;
	}
	if((xCartPose.config & 4) > 0){
		jointAngles[5]=-jointAngles[5];
		jointAngles[4]=jointAngles[4]+M_PI;
		jointAngles[6]=jointAngles[6]+M_PI;
	}

	// modulo-ize angles so they are all between -pi and pi
	for(int j=0;j<=6;j+=2){
		while(jointAngles[j]>M_PI){jointAngles[j]-=2.0*M_PI;}
		while(jointAngles[j]<-M_PI){jointAngles[j]+=2.0*M_PI;}
	}

	// check for joint limits and singularities
	result=LWR_OK;

	for(int j=0;j<7;j++){
		if( fabs(jointAngles[j]) > jointLimits(j) ){
			result = result | LWR_WARNING | LWR_JOINTLIMIT;
		}
	}

	for(int j=1;j<=5;j+=2){
		if( fabs(jointAngles[j]) < 15.0/180.0*M_PI ){
			result = result | LWR_WARNING | LWR_CLOSE_TO_SINGULARITY;
		}
	}

	double overhead= acos(xsw.dot(z)/lsw);
	if(overhead < 15.0/180.0*M_PI || overhead > 165.0/180.0*M_PI){
		result = result | LWR_WARNING | LWR_CLOSE_TO_SINGULARITY;
	}

	return result;
}

/*! \brief Compute extended Jacobian matrix in base coordinate system
 *
 *	jacobian (output) - 7x7 Jacobian matrix for the current joint angles\n
 *  jointAngles (input) - jointAngles\n
 *  \n
 *  The 7th row describes the derivative of the null-space parameter with respect to the joint angles
 */
LwrErrorMsg Lwr::computeXJacobian(LwrXJacobian& jacobian, LwrJoints& jointAngles){

	// compute upper 6x7 standard jacobian

	LwrErrorMsg result=LWR_OK;

	LwrFrame T[7];
	T[0]=denavitHartenberg(limbs[0], jointAngles[0],0 ,  0.5*M_PI);
	T[1]=denavitHartenberg(       0, jointAngles[1],0 , -0.5*M_PI);
	T[2]=denavitHartenberg(limbs[1], jointAngles[2],0 , -0.5*M_PI);
	T[3]=denavitHartenberg(       0, jointAngles[3],0 ,  0.5*M_PI);
	T[4]=denavitHartenberg(limbs[2], jointAngles[4],0 ,  0.5*M_PI);
	T[5]=denavitHartenberg(       0, jointAngles[5],0 , -0.5*M_PI);
	T[6]=denavitHartenberg(limbs[3], jointAngles[6],0 ,  0);

	LwrFrame base; // identity
	LwrFrame shoulder=base*T[0];
	LwrFrame upperArm=shoulder*T[1];
	LwrFrame elbow=upperArm*T[2];
	LwrFrame lowerArm=elbow*T[3];
	LwrFrame wrist=lowerArm*T[4];
	LwrFrame flange=wrist*T[5];
	LwrFrame tool=flange*T[6];

	jacobian.block(0,0,3,1) <<     base.ori.col(2).cross(tool.pos -     base.pos);
	jacobian.block(0,1,3,1) << shoulder.ori.col(2).cross(tool.pos - shoulder.pos);
	jacobian.block(0,2,3,1) << upperArm.ori.col(2).cross(tool.pos - upperArm.pos);
	jacobian.block(0,3,3,1) <<    elbow.ori.col(2).cross(tool.pos -    elbow.pos);
	jacobian.block(0,4,3,1) << lowerArm.ori.col(2).cross(tool.pos - lowerArm.pos);
	jacobian.block(0,5,3,1) <<    wrist.ori.col(2).cross(tool.pos -    wrist.pos);
	jacobian.block(0,6,3,1) << 0,0,0;

	jacobian.block(3,0,3,1) <<     base.ori.col(2);
	jacobian.block(3,1,3,1) << shoulder.ori.col(2);
	jacobian.block(3,2,3,1) << upperArm.ori.col(2);
	jacobian.block(3,3,3,1) <<    elbow.ori.col(2);
	jacobian.block(3,4,3,1) << lowerArm.ori.col(2);
	jacobian.block(3,5,3,1) <<    wrist.ori.col(2);
	jacobian.block(3,6,3,1) <<   flange.ori.col(2);

	// compute extension for null-space parameter

	// joints 0,4,5 and 6 don't influence the null-space parameter
	jacobian(6,0)=0;
	jacobian(6,4)=0;
	jacobian(6,5)=0;
	jacobian(6,6)=0;

	for(int j=1; j<=3; j++){
		dvalue j1(jointAngles[1],0);
		dvalue j2(jointAngles[2],0);
		dvalue j3(jointAngles[3],0);

		if(j==1){j1.dx=1;} // compute derivative with respect to shoulder joint
		if(j==2){j2.dx=1;} // compute derivative with respect to upper arm joint
		if(j==3){j3.dx=1;} // compute derivative with respect to elbow joint

		dvalue sinj1=sin(j1);
		dvalue cosj1=cos(j1);
		dvalue sinj2=sin(j2);
		dvalue cosj2=cos(j2);
		dvalue sinj3=sin(j3);
		dvalue cosj3=cos(j3);

		dvector xe; // elbow position
		xe.v[0]=-limbs[1]*sinj1;
		xe.v[1]=dvalue(0,0);
		xe.v[2]=limbs[1]*cosj1;

		dvector xw; // wrist position
		xw.v[0]=-(limbs[1]+limbs[2]*cosj3)*sinj1 + limbs[2]*sinj3*cosj2*cosj1;
		xw.v[1]=limbs[2]*sinj3*sinj2;
		xw.v[2]=(limbs[1]+limbs[2]*cosj3)*cosj1 + limbs[2]*sinj3*cosj2*sinj1;

		dvector z,usin,ucos;

		z=dvector(dvalue(0,0),dvalue(0,0),dvalue(1,0));

		usin= cross(z,xw);
		ucos= cross(xw,usin);
		usin= usin/norm(usin);
		ucos= ucos/norm(ucos);

		dvalue nsparam= atan2(xe*usin,xe*ucos);

		jacobian(6,j)=nsparam.dx;
	}

	return result;
}

/*! \brief Transform a Jacobian into another coordinate system (7th row untouched)
 *
 *	jacobian (input) - Jacobian that has to be transformed\n
 *  ori (input) - transformation matrix with new orientation\n
 *	return value - transformed Jacobian\n
 *  \n
 *  J'(1..3,1..7) = T*J(1..3)\n
 *  J'(4..6,1..7) = T*J(4..6)\n
 *	J'(7,1..7) = J(7,1..7)
 */
LwrXJacobian Lwr::transformXJacobian(LwrXJacobian& jacobian, Matrix3d ori){

	LwrXJacobian result;
	result.block(0,0,3,7) << ori * jacobian.block(0,0,3,7);
	result.block(3,0,3,7) << ori * jacobian.block(3,0,3,7);
	result.block(7,0,1,7) << jacobian.block(7,0,1,7);

	return result;
}

/*! \brief Multiply joint velocities with an extended Jacobian matrix to compute the extended Cartesian twist
 *
 *	xCartTwist (output) - resulting extended Cartesian twist\n
 *  jacobian (input) - extended Jacobian matrix\n
 *  jointVelocities (input) - joint velocities\n
 *  \n
 *  twist = Jacobian * joint velocities
 */
LwrErrorMsg Lwr::forwardJacobian(LwrXTwist& xCartTwist, LwrXJacobian& jacobian, LwrJoints& jointVelocities){
	Matrix <double, 7,1> jv;
	jointVelocities.getJoints(jv);

	Matrix <double, 7,1> cv;
	cv=jacobian*jv;

	xCartTwist.twist.trans << cv.block(0,0,3,1);
	xCartTwist.twist.rot << cv.block(3,0,3,1);
	xCartTwist.dnsparamdt = cv(6);
}

/*! \brief Multiply extended Cartesian twist with an inverse extended Jacobian to compute the joint velocities
 *
 *  jointVelocities (output) - resulting joint velocities\n
 *  jacobian (input) - extended Jacobian matrix\n
 *	xCartTwist (input) - extended Cartesian twist\n
 *  \n
 *  joint velocities = Jacobian^(-1) * twist
 */
LwrErrorMsg Lwr::inverseJacobian(LwrJoints& jointVelocities, LwrXJacobian& jacobian, LwrXTwist& xCartTwist){
	Matrix <double, 7,1> cv;

	cv.block(0,0,3,1) << xCartTwist.twist.trans;
	cv.block(3,0,3,1) << xCartTwist.twist.rot;
	cv(6) = xCartTwist.dnsparamdt;

	Matrix <double, 7,1> jv;

	jv=jacobian.fullPivLu().solve(cv);

	jointVelocities.setJoints(jv);
}

/*! \brief Estimate Cartesian force/torque on flange from joint torque
 *
 *	xCartFT (output) - resulting force/torque\n
 *	jacobian (input) - extended Jacobian matrix\n
 *	jointTrq (input) - joint torques
 *	\n
 *	FT = J^T^(-1) * tau
 */
LwrErrorMsg Lwr::xCartFTFromJointTorque(LwrXTwist& xCartFT, LwrXJacobian& jacobian, LwrJoints& jointTrq){
	Matrix <double, 7,1> jv;
	jointTrq.getJoints(jv);

	Matrix <double, 7,1> cv;
	cv=jacobian.transpose().fullPivLu().solve(jv);

	xCartFT.twist.trans << cv.block(0,0,3,1);
	xCartFT.twist.rot << cv.block(3,0,3,1);
	xCartFT.dnsparamdt = cv(6);
}

/*! \brief Compute joint torque required to generate force/torque on flange
 *
 *	jointTrq (output) - resulting joint torques\n
 *	jacobian (input) - extended Jacobian matrix\n
 *	xCartFT (input) - force/torque\n
 *	\n
 *	tau = J^T * FT
 */
LwrErrorMsg Lwr::jointTorqueFromXCartFT(LwrJoints& jointTrq, LwrXJacobian& jacobian, LwrXTwist& xCartFT){
	Matrix <double, 7,1> cv;

	cv.block(0,0,3,1) << xCartFT.twist.trans;
	cv.block(3,0,3,1) << xCartFT.twist.rot;
	cv(6) = xCartFT.dnsparamdt;

	Matrix <double, 7,1> jv;

	jv=jacobian.transpose()*cv;

	jointTrq.setJoints(jv);
}

/*! \brief Modulo-ize an angle into the interval between -pi and pi (internal helper)
 *
 *  -pi <= result < pi\n
 *  (only efficient for small angles, implemented with while loop)
 */
inline double anglemod(double x){
	double y=x;
	while(y>=M_PI){y-=2.0*M_PI;}
	while(y<-M_PI){y+=2.0*M_PI;}
	return y;
}

/*! \brief Compare to intervals by their lower border in order to sort them (internal helper)
 */
bool operator <(LwrElbowInterval i1, LwrElbowInterval i2){ // for comparison in sorting
	// invalid intervals are largest, otherwise they are compared by lower bound
	if(!i1.valid){return false;}
	if(i1.valid && !i2.valid){return true;}
	if(i1.lower < i2.lower){return true;}
	return false;
}

/*! \brief Quicksort an array of intervals (internal helper)
 */
void Lwr::quicksortIntervals(LwrElbowInterval arr[], int left, int right) {

	int i = left, j = right;
	LwrElbowInterval tmp;
	LwrElbowInterval pivot = arr[(left + right) / 2];

	// partition
	while (i <= j) {
		while ( arr[i] < pivot)
			  i++;
		while ( pivot < arr[j] )
			  j--;
		if (i <= j) {
			  tmp = arr[i];
			  arr[i] = arr[j];
			  arr[j] = tmp;
			  i++;
			  j--;
		}
	}

	// recursion
	if (left < j){
	  quicksortIntervals(arr, left, j);}
	if (i < right){
	  quicksortIntervals(arr, i, right);}
}

/*! \brief Merge overlapping intervals in an array of intervals (internal helper)
 *
 *  dst (output) - resulting array (make sure it has at least as many elements as the source array)\n
 *  src (input) - source array\n
 *  \n
 *  The output is also sorted.\n
 *  \n
 *  Example:\n
 *  src = {[3,5], [invalid], [1,3], [7,9], [8,10]}\n
 *  dst = {[1,5], [7,10], [invalid], [invalid]
 */
void Lwr::mergeIntervals(LwrElbowInterval dst[], const LwrElbowInterval src[], const int n){

	int i,k;

	// copy intervals from src to dst with lower bound first
	for(i=0;i<n;i++){
		dst[i]=src[i];
		if(dst[i].lower>dst[i].upper){
			dst[i].lower=src[i].upper;
			dst[i].upper=src[i].lower;
		}
	}

	// sort intervals by their lower bound, invalid intervals will move to the end
	quicksortIntervals(dst,0,n-1);

	for(i=0;i<n-1;i++){ // go through all (but the last) intervals with index i
		if(!dst[i].valid){ // this interval has already been marked to be deleted
			continue;}
		k=i+1; // start going through further intervals with index k
		while(dst[k].lower <= dst[i].upper+1e-8 && k<n){ // check if interval k and interval i overlap or almost overlap
			if(dst[k].upper > dst[i].upper){ // expand interval i to include interval k if they overlap
				dst[i].upper=dst[k].upper;
			}
			dst[k].invalidate();
			k++;
		}
	}

	// move deleted intervals to the end
	quicksortIntervals(dst,0,n-1);
}

/*! \brief Compute blocked and reachable elbow intervals for half a robot (internal helper)
 *
 *  (see documentation for elbowIntervals)
 */
LwrErrorMsg Lwr::elbowIntervalsPart(
		LwrElbowInterval blockedPerJoint[3][4],
		Vector3d target,
		double l1, double l2,
		double jointLimitsPart[3],
		bool inv1, bool inv3){

	LwrErrorMsg result=LWR_OK;

	double tr=target.norm(); // target radius, distance from 0 to target

	if(tr>l1+l2){
		result=LWR_ERROR | LWR_TARGET_TOO_FAR;
		return result;
	}

	if(tr<=fabs(l1-l2)){
		result=LWR_ERROR | LWR_TARGET_TOO_CLOSE;
		return result;
	}

	if(fabs(target[0])<1e-7 && fabs(target[1])<1e-7){
		result=LWR_ERROR | LWR_SINGULARITY;
		return result;
	}

	double tpsi=atan2(target[1],target[0]); // azimuth of target
	double ttheta=asin(target[2]/tr); // altitude of target

	double rho=acos((l1*l1+tr*tr-l2*l2)/2.0/l1/tr); // angle between 0->target and first link

	double thetapole=0.5*M_PI-ttheta; // angle between 0->target and northpole

	double borders[4]={0,0,0,0}; // mathematical solutions of joint limits projected on the elbow circle
	// not necessarily valid solutions (like negative lengths that solve pythagoras but make no sense)
	// and it is not obvious which side of the border is the blocked side

	bool valid[4]={false,false,false,false}; // which border is a valid solution

	// auxiliary variables
	double alphamax;
	double radic;
	double phimax;
	double alphacheck;
	double buffer;
	bool buffer_b;

	////////////////////////
	// LIMITS FOR JOINT 1 //
	////////////////////////

	for(int i=0;i<2;i++){

		// if you draw lines on a sphere from the projected target to the northpole
		// to the projected elbow, alpha is the angle at the northpole
		if(!inv1){
			if(i==0){
				alphamax=jointLimitsPart[0]-tpsi;} // side 1
			else{
				alphamax=-jointLimitsPart[0]-tpsi;}	// side 2
		}
		else{
			if(i==0){
				alphamax=jointLimitsPart[0]-M_PI-tpsi;} // side 1
			else{
				alphamax=M_PI-jointLimitsPart[0]-tpsi;} // side 2
		}

		// spherical geometry
		radic= square(cot(alphamax)) + square(cos(thetapole)) - square(cot(rho)*sin(thetapole));
		// auxiliary variable under the root

		if(radic>=0){
			// solution 1, +sqrt
			phimax=2*atan( (cot(alphamax)+sqrt(radic)) / (cos(thetapole)+cot(rho)*sin(thetapole)) );
			alphacheck=atan2( sin(phimax) , cot(rho)*sin(thetapole)-cos(thetapole)*cos(phimax) );
			borders[0+2*i]=phimax;

			if(fabs(anglemod(alphacheck-alphamax))<0.5*M_PI){
				valid[0+2*i]=true;}

			// solution 2, -sqrt
			phimax=2*atan( (cot(alphamax)-sqrt(radic)) / (cos(thetapole)+cot(rho)*sin(thetapole)) );
			alphacheck=atan2( sin(phimax) , cot(rho)*sin(thetapole)-cos(thetapole)*cos(phimax) );
			borders[1+2*i]=phimax;

			if(fabs(anglemod(alphacheck-alphamax))<0.5*M_PI){
				valid[1+2*i]=true;}
		}
	}

	// for half of the sphere, mathematics swaps the order of the limits
	// swap back
	if(!inv1){
		if (tpsi>jointLimitsPart[0] || tpsi<jointLimitsPart[0]-M_PI){ // TODO: limit analysis!! (<? <=?)
			buffer=borders[1]; borders[1]=borders[0]; borders[0]=buffer;
			buffer_b=valid[1]; valid[1]=valid[0]; valid[0]=buffer_b;
		}

		if (tpsi<-jointLimitsPart[0] || tpsi>M_PI-jointLimitsPart[0]){ // TODO: limit analysis!! (<? <=?)
			buffer=borders[3]; borders[3]=borders[2]; borders[2]=buffer;
			buffer_b=valid[3]; valid[3]=valid[2]; valid[2]=buffer_b;
		}
	}
	else{
		if (!(tpsi>jointLimitsPart[0] || tpsi<jointLimitsPart[0]-M_PI)){ // TODO: limit analysis!! (<? <=?)
			buffer=borders[1]; borders[1]=borders[0]; borders[0]=buffer;
			buffer_b=valid[1]; valid[1]=valid[0]; valid[0]=buffer_b;
		}

		if (!(tpsi<-jointLimitsPart[0] || tpsi>M_PI-jointLimitsPart[0])){ // TODO: limit analysis!! (<? <=?)
			buffer=borders[3]; borders[3]=borders[2]; borders[2]=buffer;
			buffer_b=valid[3]; valid[3]=valid[2]; valid[2]=buffer_b;
		}
	}

	// assign borders to intervals
	if(valid[1] && valid[2]){
		if(borders[1]>borders[2]){
			blockedPerJoint[0][0].setBorders(borders[1],M_PI);
			blockedPerJoint[0][1].setBorders(-M_PI,borders[2]);
		}
		else{
			blockedPerJoint[0][0].setBorders(borders[1],borders[2]);
		}
	}

	if(valid[0] && valid[3]){
		if(borders[3]>borders[0]){
			blockedPerJoint[0][2].setBorders(borders[3],M_PI);
			blockedPerJoint[0][3].setBorders(-M_PI,borders[0]);
		}
		else{
			blockedPerJoint[0][2].setBorders(borders[3],borders[0]);
		}
	}

	if(valid[0] && valid[1] && ! valid[2] && ! valid[3]){
		if(borders[1]>borders[0]){
			blockedPerJoint[0][0].setBorders(borders[1],M_PI);
			blockedPerJoint[0][1].setBorders(-M_PI,borders[0]);
		}
		else{
			blockedPerJoint[0][0].setBorders(borders[1],borders[0]);
		}
	}

	if(valid[2] && valid[3] && ! valid[0] && ! valid[1]){
		if(borders[3]>borders[2]){
			blockedPerJoint[0][0].setBorders(borders[3],M_PI);
			blockedPerJoint[0][1].setBorders(-M_PI,borders[2]);
		}
		else{
			blockedPerJoint[0][0].setBorders(borders[3],borders[2]);
		}
	}

	if(!inv1){
		if(! valid[0] && ! valid[1] && ! valid[2] && ! valid[3] &&
			(tpsi>jointLimitsPart[0] || tpsi<-jointLimitsPart[0]) ){
				blockedPerJoint[0][0].setBorders(-M_PI,M_PI);
				result = result | LWR_WARNING | LWR_NO_SOLUTION_FOR_ELBOW;
		}
	}
	else{
		if(! valid[0] && ! valid[1] && ! valid[2] && ! valid[3] &&
			(tpsi>jointLimitsPart[0]-M_PI && tpsi<M_PI-jointLimitsPart[0]) ){
				blockedPerJoint[0][0].setBorders(-M_PI,M_PI);
				result = result | LWR_WARNING | LWR_NO_SOLUTION_FOR_ELBOW;
		}
	}

	////////////////////////
	// LIMITS FOR JOINT 2 //
	////////////////////////

	double cosphimax=(cos(jointLimitsPart[1])-cos(rho)*cos(thetapole))/sin(rho)/sin(thetapole);

	if(fabs(cosphimax)<1){
		phimax=acos(cosphimax);
		blockedPerJoint[1][0].setBorders(phimax,M_PI);
		blockedPerJoint[1][1].setBorders(-M_PI,-phimax);
	}
	if(cosphimax>=1){
		blockedPerJoint[1][0].setBorders(-M_PI,M_PI);
		result = result | LWR_WARNING | LWR_NO_SOLUTION_FOR_ELBOW;
	}

	////////////////////////
	// LIMITS FOR JOINT 3 //
	////////////////////////

	double adjq3min;

	if(!inv3){

		adjq3min=M_PI-jointLimitsPart[2];
		radic= square(cot(adjq3min)) + square(cos(rho)) - square(cot(thetapole)*sin(rho));

		if(radic>=0){
			phimax=2*atan( (cot(adjq3min)+sqrt(radic)) / (cos(rho)+cot(thetapole)*sin(rho)) );

			if(-M_PI<=phimax && phimax<=0){
				blockedPerJoint[2][0].setBorders(-phimax,M_PI);
				blockedPerJoint[2][1].setBorders(-M_PI,phimax);
			}
			phimax=2*atan( (cot(adjq3min)-sqrt(radic)) / (cos(rho)+cot(thetapole)*sin(rho)) );
			if(-M_PI<=phimax && phimax<=0){
				blockedPerJoint[2][2].setBorders(phimax,-phimax);
			}
		}
		else{
			if(cot(thetapole)*sin(rho)-cos(rho)<0){
				blockedPerJoint[2][0].setBorders(-M_PI,M_PI);
				result = result | LWR_WARNING | LWR_NO_SOLUTION_FOR_ELBOW;
			}
		}
	}
	else{// so if(inv3)

		// minimum angle for the adjacent to the third joint angle
		adjq3min=M_PI-jointLimitsPart[2];

		radic= square(cot(adjq3min)) + square(cos(rho)) - square(cot(thetapole)*sin(rho));
		if(radic>=0){
			phimax=2*atan( (cot(adjq3min)+sqrt(radic)) / (cos(rho)+cot(thetapole)*sin(rho)) );

			if(0<=phimax && phimax<=M_PI){
				blockedPerJoint[2][0].setBorders(phimax,M_PI);
				blockedPerJoint[2][1].setBorders(-M_PI,-phimax);
			}

			phimax=2*atan( (cot(adjq3min)-sqrt(radic)) / (cos(rho)+cot(thetapole)*sin(rho)) );

			if(0<=phimax && phimax<=M_PI){
				blockedPerJoint[2][2].setBorders(-phimax,phimax);
			}
		}
		else{
			// all or nothing feasable, check
			if(cot(thetapole)*sin(rho)-cos(rho)>0){
				blockedPerJoint[2][0].setBorders(-M_PI,M_PI);
				result = result | LWR_WARNING | LWR_NO_SOLUTION_FOR_ELBOW;
			}
		}
	}

    // //debug
    //std::cout << "Subresult: " << std::endl;
    //for(int j=0; j<3; j++){
    //    for(int i=0; i<4; i++){
    //        std::cout << blockedPerJoint[j][i] << "  ";
    //    }
    //    std::cout << std::endl;
    //}
    // //debug

	return result;
}

/*! \brief Compute blocked and reachable elbow intervals for a given Cartesian target
 *
 *  currentMargin (output) - clearance within the current null-space interval (no matter if blocked or reachable)\n
 *  reachable[11] (output) - array of intervals of reachable null-space parameters\n
 *  blocked[11] (output) - array of intervals of blocked null-space parameters\n
 *  blockedPerJoint[7][3] (output) - which joint blocks which intervals\n
 *  xCartPose (input) - extended Cartesian target pose for which the null-space intervals are to be computed\n
 *  \n
 *  please refer to the overall concept for more information
 */
LwrErrorMsg Lwr::elbowIntervals(
		LwrElbowInterval currentMargin,
		LwrElbowInterval reachable[11],
		LwrElbowInterval blocked[11],
		LwrElbowInterval blockedPerJoint[7][3],
		LwrXCart& xCartPose){

	// invalidate all intervals
	for(int j=0; j<11; j++){
		reachable[j].invalidate();
		blocked[j].invalidate();
	}
	for(int j=0; j<7; j++){
		for(int i=0; i<3; i++){
			blockedPerJoint[j][i].invalidate();
		}
	}

	LwrErrorMsg result=LWR_OK; // resulting error info
	LwrErrorMsg tempResult=LWR_OK; // error info for half of the arm

	bool inv1=(xCartPose.config & 1) >0; // joint inversions (180° rotation) due to configuration
	bool inv3=((xCartPose.config & 1) >0)^((xCartPose.config & 2) >0);
	bool inv5=((xCartPose.config & 2) >0)^((xCartPose.config & 4) >0);
	bool inv7=(xCartPose.config & 4) >0;

	LwrFrame mfw; // wrist pose in flange coordinates
	mfw.pos[2]=-limbs[3];

	LwrFrame mw=xCartPose.pose*mfw; // wrist pose

	Vector3d xw=mw.pos; // wrist pos
	Vector3d xs(0,0,limbs[0]); // shoulder pos

	// wrist position inside a cartesian csys located in the shoulder
	Vector3d xwfroms=mw.pos;

	xwfroms[0]= -xwfroms[0]; // for LWR joint configuration
	xwfroms[1]= -xwfroms[1];
	xwfroms[2]-= limbs[0];

	LwrElbowInterval blockedPerJointNormalPart[3][4]; // blocked elbow intervals for first three joints
	LwrElbowInterval blockedPerJointMirrorPart[3][4]; // blocked elbow intervals for last three joints
	LwrElbowInterval blockedPerJointMirrorPartMerged[3][4]; // intervals that covered -pi/pi before transformation are merged together
	double jointLimitsPart[3]={jointLimits(0),jointLimits(1),jointLimits(2)};

	tempResult=elbowIntervalsPart( // compute elbow intervals for the first three joints
			blockedPerJointNormalPart,
			xwfroms,
			limbs[1],
			limbs[2],
			jointLimitsPart,
			inv1,
			inv3);

	result = result | tempResult;
	if(result & LWR_ERROR){
		return result;
	}

	// for the last three joints, we 'flip' the robot and call the same function

	// shoulder of an inverse robot (flange and base flipped,
	// shoulder and wrist flipped, upper and lower arm flipped)
	LwrFrame mirrorxs=xCartPose.pose;
	for(int i=0;i<3;i++){
		for(int k=0;k<3;k++){
			if(k==1){continue;}
			mirrorxs.ori(i,k)=-mirrorxs.ori(i,k);
		}
	}
	mirrorxs.pos=xw;
	// mirrorshoulder is located at the wrist of the (non flipped) robot
	// -z points out the flange, +x is where the lower arm bends if joint 7 is 0° and
	// joint 6 deflects positively

	// mirrorwrist (shoulder) from mirrorshoulder
	Vector3d mirrorxwfroms=mirrorxs.inverse()*xs;

    //std::cout << mirrorxwfroms << std::endl;

	jointLimitsPart[0]=jointLimits(6);
	jointLimitsPart[1]=jointLimits(5);
	jointLimitsPart[2]=jointLimits(4);

	tempResult=elbowIntervalsPart( // compute elbow intervals for the last three joints
			blockedPerJointMirrorPart,
			mirrorxwfroms,
			limbs[2],
			limbs[1],
			jointLimitsPart,
			inv7,
			inv5);

	result = result | tempResult;
	if(result & LWR_ERROR){
		return result;
	}

	// now we have the blocked intervals on the elbow circle of the flipped robot
	// and we need to flip them back about a certain angle

	// compute offset angle
	LwrFrame ms;
	ms.pos[2]=limbs[0];

	Vector3d ex1=(mw.pos-ms.pos).cross(ms.ori.topRightCorner<3,1>());
	Vector3d ey1=ex1.cross(mw.pos-ms.pos);
	Vector3d ex2=(ms.pos-mw.pos).cross(mw.ori.topRightCorner<3,1>());

	ex1.normalize();
	ey1.normalize();

	double offset=atan2(ex2.dot(ey1),ex2.dot(ex1));

	int unused=-1;
	for(int j=0;j<3;j++){
		for(int k=0;k<4;k++){
			if(blockedPerJointMirrorPart[j][k].valid){
				blockedPerJointMirrorPart[j][k].setBorders(
						anglemod(offset-blockedPerJointMirrorPart[j][k].upper),
						anglemod(offset-blockedPerJointMirrorPart[j][k].lower));
			}
			else{
				unused=k; // mark this interval as unused (there is always at least one unused interval which will be needed later)
			}
		}
		for(int k=0;k<4;k++){
			if(blockedPerJointMirrorPart[j][k].lower>blockedPerJointMirrorPart[j][k].upper
					&& blockedPerJointMirrorPart[j][k].valid){
				// this interval now covers the -pi/pi barrier and needs to be split, use the unused interval
				blockedPerJointMirrorPart[j][unused].setBorders(-M_PI,blockedPerJointMirrorPart[j][k].upper);
				blockedPerJointMirrorPart[j][k].upper=M_PI;
			}
		}
	}

	// now we fill the result array
	for(int j=0;j<3;j++){
		quicksortIntervals(blockedPerJointNormalPart[j],0,3);
		for(int k=0;k<3;k++){
			blockedPerJoint[j][k]=blockedPerJointNormalPart[j][k];
		}

		mergeIntervals(blockedPerJointMirrorPartMerged[j],blockedPerJointMirrorPart[j],4);
		//quicksortIntervals(blockedPerJointMirrorPart[j],0,3);
		for(int k=0;k<3;k++){
			blockedPerJoint[6-j][k]=blockedPerJointMirrorPartMerged[j][k];
		}
	}

	// assign interval for elbow joint (blocks either all (if target too far or too close) or nothing)
	if(square(limbs[1]) + square(limbs[2]) + 2.0*limbs[1]*limbs[2]*cos(jointLimits(3)) > (xw-xs).dot(xw-xs)){
		blockedPerJoint[3][0].setBorders(-M_PI,M_PI);
		result = result | LWR_WARNING | LWR_NO_SOLUTION_FOR_ELBOW;
	}

	// merge all intervals
	if(result & LWR_NO_SOLUTION_FOR_ELBOW){
		blocked[0].setBorders(-M_PI,M_PI);
	}
	else{
		LwrElbowInterval blockedBig[21]; // array capable of holding all unmerged intervals

		// the following call is an ugly solution in terms of readability;
		// the whole blockedPerJoint[7][3] array is passed as the src[] argument
		mergeIntervals(blockedBig,blockedPerJoint[0],21);

        for(int i=0;i<11;i++){
            blocked[i]=blockedBig[i];
        }
	}

	// "compute" reachable intervals from blocked intervals
	int ir=-1;
	if(blocked[0].lower>-M_PI){
		ir++;
		reachable[0].setBorders(-M_PI,M_PI);
	}
	for(int ib=0;ib<11;ib++){
		if(blocked[ib].valid){
			if(ir>=0){
				reachable[ir].upper=blocked[ib].lower;
			}
			if(blocked[ib].upper<M_PI){
				ir++;
				reachable[ir].setBorders(blocked[ib].upper,M_PI);
			}
		}
	}

	// determine margin within current interval
	bool continueBlocked=false; // if we are inside a blocked interval that has to be continued behind -pi
	bool continueReachable=false; // if we are inside a reachable interval that has to be continued behind -pi
	for(int i=0;i<11;i++){
		if(blocked[i].valid){
			if(xCartPose.nsparam >= blocked[i].lower && xCartPose.nsparam >= blocked[i].upper){
				currentMargin=blocked[i];
                if(blocked[i].lower == -M_PI){
					continueBlocked=true;
				}
				else{
					break;
				}
			}
			if(continueBlocked && blocked[i].upper == M_PI){
				currentMargin.lower=blocked[i].lower;
				break;
			}
		}
		if(reachable[i].valid){
			if(xCartPose.nsparam >= reachable[i].lower && xCartPose.nsparam >= reachable[i].upper){
				currentMargin=reachable[i];
                if(reachable[i].lower == -M_PI){
					continueReachable=true;
				}
				else{
					break;
				}
			}
			if(continueReachable && reachable[i].upper == M_PI){
				currentMargin.lower=reachable[i].lower;
				break;
			}
		}
    }

	return result;
}



