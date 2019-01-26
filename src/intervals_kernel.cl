
#ifndef src2str
	#define src2str(src) #src
#endif

const char* intervalKernelSrc = src2str(

// n because just # would be interpreted by gcc, this way it is interpreted when sent to GPU


\n#define RETURN_INTERVALS___ \n


\n#ifndef DOUBLE_PRECISION \n
	\n#define FLOAT float \n
	\n#define FLOAT2 float2 \n
	\n#define FLOAT3 float3 \n
	\n#define FLOAT4 float4 \n
	\n#define FLOAT8 float8 \n
	\n#define FLOAT16 float16 \n
\n#else \n
	\n#pragma OPENCL EXTENSION cl_khr_fp64: enable \n
	\n#define FLOAT double \n
	\n#define FLOAT2 double2 \n
	\n#define FLOAT3 double3 \n
	\n#define FLOAT4 double4 \n
	\n#define FLOAT8 double8 \n
	\n#define FLOAT16 double16 \n
\n#endif \n

\n#define LWR_OK 0 \n
\n#define LWR_WARNING (1<<0) \n
\n#define LWR_ERROR (1<<1) \n
\n#define LWR_JOINTLIMIT (1<<2) \n
\n#define LWR_TARGET_TOO_FAR (1<<3) \n
\n#define LWR_TARGET_TOO_CLOSE (1<<4) \n
\n#define LWR_CLOSE_TO_SINGULARITY (1<<5) \n
\n#define LWR_SINGULARITY (1<<6) \n
\n#define LWR_NO_SOLUTION_FOR_ELBOW (1<<7) \n

\n#define RAD170 2.96705972839 \n
\n#define RAD120 2.09439510239 \n

\n#define SQUARE(X) (pow((X),2.0)) \n
\n#define COT(X) (1.0/tan(X)) \n

typedef struct{
	bool valid;
	FLOAT lower;
	FLOAT upper;
}t_interval;

\n#define SETBORDERS(INTERVAL, LOWER, UPPER) INTERVAL.lower=LOWER; INTERVAL.upper=UPPER; INTERVAL.valid=true; \n
// don't use SETBORDERS to swap the components (yes, took me ages to find this bug, I still like macros.)
\n#define INVALIDATE(INTERVAL) INTERVAL.lower=0; INTERVAL.upper=0; INTERVAL.valid=false; \n

FLOAT16 trafoMul(FLOAT16 a, FLOAT16 b){
	FLOAT16 c;

	c.s0 = dot(a.s048, b.s012);
	c.s1 = dot(a.s159, b.s012);
	c.s2 = dot(a.s26a, b.s012);
	c.s3 = 0;

	c.s4 = dot(a.s048, b.s456);
	c.s5 = dot(a.s159, b.s456);
	c.s6 = dot(a.s26a, b.s456);
	c.s7 = 0;

	c.s8 = dot(a.s048, b.s89a);
	c.s9 = dot(a.s159, b.s89a);
	c.sa = dot(a.s26a, b.s89a);
	c.sb = 0;

	c.sc = dot(a.s048c, b.scdef);
	c.sd = dot(a.s159d, b.scdef);
	c.se = dot(a.s26ae, b.scdef);
	c.sf = 1;

	return c;
}

inline FLOAT anglemod(FLOAT x){
	FLOAT y = fmod(x, 2.0*M_PI);
	if(y>=M_PI){y-=2.0*M_PI;}
	if(y<-M_PI){y+=2.0*M_PI;}
	return y;
}

// Compare two intervals by their lower border in order to sort them
inline bool smaller(t_interval a[], int i1, int i2, int n){
	if(i1>=n || i2>=n){return false;} // out of scope (we always have to sort 2^n elements)
	// invalid intervals are largest, otherwise they are compared by lower bound
	if(!a[i1].valid){return false;}
	if(a[i1].valid && !a[i2].valid){return true;}
	if(a[i1].lower < a[i2].lower){return true;}
	return false;
}

// exchange two intervals
inline void exchange(t_interval a[], int i1, int i2){
	t_interval tmp=a[i1];
	a[i1]=a[i2];
	a[i2]=tmp;
}

void sortIntervals(t_interval a[], int n) {
	int i;
	int j;
	for (i = 0; i < n -1; i++){
		for (j = 0; j < n - i - 1; j++){
			if (smaller(a, j+1, j, n)){
				exchange(a, j, j+1);
			}
		}
	}
}

// Merge overlapping intervals in an array of intervals
//
// dst (output) - resulting array (make sure it has at least as many elements as the source array)\n
// src (input) - source array
//
// The output is also sorted.
//
// Example:\n
// src = {[3,5], [invalid], [1,3], [7,9], [8,10]}
// dst = {[1,5], [7,10], [invalid], [invalid], [invalid]}
//
// the result is the biggest gap (or the smallest overlap, negative then)
FLOAT mergeIntervals(t_interval dst[], const t_interval src[], const int n){

/*
I TRIED TO FIND THE BIGGEST FREE INTERVAL (OR THE SMALLEST OVERLAP) BUT
IT NEVER SEEMED TO WORK FOR SOME REASON. THE COMMENT MESS IS A RESULT OF
THIS. MAYBE I GET TO THAT LATER. UNTIL THEN I WILL OPERATE ON THE REACHABLE
INTERVALS AT THE END OF THE MAIN FUNCTION. IT IS IMPOSSIBLE TO COMPUTE THE
SMALLEST OVERLAP THERE THOUGH.
*/

	int i;
	int k;
	FLOAT largestGap = -2*M_PI;

	// copy intervals from src to dst with lower bound first
	for(i=0;i<n;i++){
		dst[i]=src[i];
		if(dst[i].lower>dst[i].upper){
			dst[i].lower=src[i].upper;
			dst[i].upper=src[i].lower;
		}
	}

	// sort intervals by their lower bound, invalid intervals will move to the end
	sortIntervals(dst,n);

/*
	// find largest gap
	for(i=0;i<n-1;i++){ // go through all (but the last) intervals with index i
		if(!dst[i].valid){continue;} // this interval has already been marked to be deleted

		for(k=i+1;k<n;k++){ // start going through further intervals with index k
			if(!dst[k].valid){continue;}

			if(dst[i].upper > dst[k].upper){
				INVALIDATE(dst[k]);
				continue;
			}

			if(dst[i].valid && dst[k].valid && dst[k].upper > dst[i].upper && dst[k].lower-dst[i].upper>largestGap){
				largestGap = dst[k].lower-dst[i].upper;
			}
		}
	}
*/

	// merge intervals
	for(i=0;i<n-1;i++){ // go through all (but the last) intervals with index i
		if(!dst[i].valid){continue;} // this interval has already been marked to be deleted

		/*k=i+1; // start going through further intervals with index k

		while(dst[k].lower <= dst[i].upper+1e-6 && k<n){ // check if interval k and interval i overlap or almost overlap

			if(dst[k].upper > dst[i].upper){ // expand interval i to include interval k if they overlap
				dst[i].upper=dst[k].upper;
			}
			INVALIDATE(dst[k]);
			k++;
		}*/
		for(k=i+1;k<n;k++){ // check if interval k and interval i overlap or almost overlap
			if(!dst[k].valid){continue;}

			if(dst[k].lower > dst[i].upper+1e-6){break;} // stop merging and continue with i

			if(dst[k].upper > dst[i].upper){ // expand interval i to include interval k if they overlap
				dst[i].upper=dst[k].upper;
			}
			INVALIDATE(dst[k]);
		}
	}

	// move deleted intervals to the end
	sortIntervals(dst,n);

/*
	for(i=0;i<n-1;i++){ // go through all (but the last) intervals with index i
		if(!dst[i].valid){break;}
		k = i+1;
		if(!dst[k].valid){break;}

		if(dst[k].lower - dst[i].upper > largestGap){
			largestGap = dst[k].lower - dst[i].upper;
		}
	}
*/
	//if(dst[0].lower<-3.14 && dst[0].upper>3.14){largestGap = -1;}

	return largestGap;
}

// compute blocked and reachable elbow intervals for half a robot
ushort intervals_part(
		t_interval blockedPerJoint[3][4],
		FLOAT3 target,
		FLOAT l1,
		FLOAT l2,
		FLOAT jointLimitsPart[3],
		bool inv1,
		bool inv3){

	ushort result=LWR_OK;

	FLOAT tr = length(target); // target radius, distance from 0 to target

	if(tr>l1+l2){
		result=LWR_ERROR | LWR_TARGET_TOO_FAR;
		return result;
	}

	if(tr<=fabs(l1-l2)){
		result=LWR_ERROR | LWR_TARGET_TOO_CLOSE;
		return result;
	}

	if(fabs(target.x)<1e-7 && fabs(target.y)<1e-7){
		result=LWR_ERROR | LWR_SINGULARITY;
		return result;
	}

	FLOAT tpsi=atan2(target.y,target.x); // azimuth of target
	FLOAT ttheta=asin(target.z/tr); // altitude of target

	FLOAT rho=acos((l1*l1+tr*tr-l2*l2)/2.0/l1/tr); // angle between 0->target and first link
	FLOAT thetapole=0.5*M_PI-ttheta; // angle between 0->target and northpole

	FLOAT borders[4]; // mathematical solutions of joint limits projected on the elbow circle
	// not necessarily valid solutions (like negative lengths that solve pythagoras but make no sense)
	// and it is not obvious which side of the border is the blocked side
	bool valid[4]; // which border is a valid solution

	for(int i=0; i<4; i++){
		borders[i]=0;
		valid[i]=false;
	}

	// auxiliary variables
	FLOAT alphamax=0;
	FLOAT radic=0;
	FLOAT phimax=0;
	FLOAT alphacheck=0;
	FLOAT buffer=0;
	FLOAT aux1=0;
	FLOAT aux2=0;
	FLOAT aux3=0;
	bool buffer_b=false;

	////////////////////////
	// LIMITS FOR JOINT 1 //
	////////////////////////

	for(int i=0;i<2;i++){

		alphamax=((i==0)*2-1)* (jointLimitsPart[0]-M_PI*inv1)-tpsi;

		// spherical geometry
		radic= SQUARE(COT(alphamax)) + SQUARE(cos(thetapole)) - SQUARE(COT(rho)*sin(thetapole));
		// auxiliary variable under the root

		if(radic>=0){
			// solution 1, +sqrt
			aux1=COT(alphamax);
			aux2=sqrt(radic);
			aux3=(cos(thetapole)+COT(rho)*sin(thetapole));

			phimax=2*atan( (aux1+aux2) / aux3);
			alphacheck=atan2( sin(phimax) , COT(rho)*sin(thetapole)-cos(thetapole)*cos(phimax) );
			borders[0+2*i]=phimax;

			if(fabs(anglemod(alphacheck-alphamax))<0.5*M_PI){
				valid[0+2*i]=true;}

			// solution 2, -sqrt
			phimax=2*atan( (aux1-aux2) / aux3);
			alphacheck=atan2( sin(phimax) , COT(rho)*sin(thetapole)-cos(thetapole)*cos(phimax) );
			borders[1+2*i]=phimax;

			if(fabs(anglemod(alphacheck-alphamax))<0.5*M_PI){
				valid[1+2*i]=true;}
		}
	}

	if (inv1^  (tpsi>jointLimitsPart[0] || tpsi<jointLimitsPart[0]-M_PI)){ // TODO: limit analysis!! (<? <=?)
		buffer=borders[1]; borders[1]=borders[0]; borders[0]=buffer;
		buffer_b=valid[1]; valid[1]=valid[0]; valid[0]=buffer_b;
	}
	if (inv1^  (tpsi<-jointLimitsPart[0] || tpsi>M_PI-jointLimitsPart[0])){ // TODO: limit analysis!! (<? <=?)
		buffer=borders[3]; borders[3]=borders[2]; borders[2]=buffer;
		buffer_b=valid[3]; valid[3]=valid[2]; valid[2]=buffer_b;
	}


	// assign borders to intervals
	if(valid[1] && valid[2]){
		if(borders[1]>borders[2]){
			SETBORDERS(blockedPerJoint[0][0], borders[1], M_PI);
			SETBORDERS(blockedPerJoint[0][1],-M_PI,borders[2]);
		}
		else{
			SETBORDERS(blockedPerJoint[0][0],borders[1],borders[2]);
		}
	}

	if(valid[0] && valid[3]){
		if(borders[3]>borders[0]){
			SETBORDERS(blockedPerJoint[0][2],borders[3],M_PI);
			SETBORDERS(blockedPerJoint[0][3],-M_PI,borders[0]);
		}
		else{
			SETBORDERS(blockedPerJoint[0][2],borders[3],borders[0]);
		}
	}

	if(valid[0] && valid[1] && ! valid[2] && ! valid[3]){
		if(borders[1]>borders[0]){
			SETBORDERS(blockedPerJoint[0][0],borders[1],M_PI);
			SETBORDERS(blockedPerJoint[0][1],-M_PI,borders[0]);
		}
		else{
			SETBORDERS(blockedPerJoint[0][0],borders[1],borders[0]);
		}
	}

	if(valid[2] && valid[3] && ! valid[0] && ! valid[1]){
		if(borders[3]>borders[2]){
			SETBORDERS(blockedPerJoint[0][0],borders[3],M_PI);
			SETBORDERS(blockedPerJoint[0][1],-M_PI,borders[2]);
		}
		else{
			SETBORDERS(blockedPerJoint[0][0],borders[3],borders[2]);
		}
	}

	if(! valid[0] && ! valid[1] && ! valid[2] && ! valid[3] &&
		( !inv1 && (tpsi>jointLimitsPart[0] || tpsi<-jointLimitsPart[0]) ||
		   inv1 && (tpsi>jointLimitsPart[0]-M_PI && tpsi<M_PI-jointLimitsPart[0]) )
	){
			SETBORDERS(blockedPerJoint[0][0],-M_PI,M_PI);
			result = result | LWR_WARNING | LWR_NO_SOLUTION_FOR_ELBOW;
	}

	////////////////////////
	// LIMITS FOR JOINT 2 //
	////////////////////////

	FLOAT cosphimax=(cos(jointLimitsPart[1])-cos(rho)*cos(thetapole))/sin(rho)/sin(thetapole);

	if(fabs(cosphimax)<1){
		phimax=acos(cosphimax);
		SETBORDERS(blockedPerJoint[1][0],phimax,M_PI);
		SETBORDERS(blockedPerJoint[1][1],-M_PI,-phimax);
	}
	if(cosphimax>=1){
		SETBORDERS(blockedPerJoint[1][0],-M_PI,M_PI);
		result = result | LWR_WARNING | LWR_NO_SOLUTION_FOR_ELBOW;
	}

	////////////////////////
	// LIMITS FOR JOINT 3 //
	////////////////////////

	FLOAT adjq3min=0;
	FLOAT minusIfInv3=(!inv3)*2-1;

	// minimum angle for the adjacent to the third joint angle
	adjq3min=M_PI-jointLimitsPart[2];

	radic= SQUARE(COT(adjq3min)) + SQUARE(cos(rho)) - SQUARE(COT(thetapole)*sin(rho));

	if(radic>=0){
		phimax=2*atan( (COT(adjq3min)+sqrt(radic)) / (cos(rho)+COT(thetapole)*sin(rho)) );

		if(-M_PI*(!inv3)<=phimax && phimax<=M_PI*inv3){
			SETBORDERS(blockedPerJoint[2][0],-phimax*minusIfInv3,M_PI);
			SETBORDERS(blockedPerJoint[2][1],-M_PI,phimax*minusIfInv3);
		}
		phimax=2*atan( (COT(adjq3min)-sqrt(radic)) / (cos(rho)+COT(thetapole)*sin(rho)) );
		if(-M_PI*(!inv3)<=phimax && phimax<=M_PI*inv3){
			SETBORDERS(blockedPerJoint[2][2],phimax*minusIfInv3,-phimax*minusIfInv3);
		}
	}
	else{
		// all or nothing feasable, check
		if(inv3^ (COT(thetapole)*sin(rho)-cos(rho)<0)){
			SETBORDERS(blockedPerJoint[2][0],-M_PI,M_PI);
			result = result | LWR_WARNING | LWR_NO_SOLUTION_FOR_ELBOW;
		}
	}

	return result;
}


__kernel void intervals(
	__global short *error,
	__global short *intervals,
	__global float *cost,
	__global FLOAT16 *target,
	__global uchar *config,
	__global FLOAT16 *invTool) {

	// intervals has to be 64*n array of 4byte shorts
	// in the first 22 shorts the reachable intervals will be stored
	// the next 7*3*2 shorts store the blocked intervals per joint
	// one interval is stored as follows:
	// the lower bound is stored in the 4 least significant bytes
	// the upper bound is stored in the 4 most significant bytes
	// -32400 corresponds to -180deg
	//      0 corresponds to    0deg
	//  32400 corresponds to  180deg (so resolution is 180 steps per degree)
	//  32767 means the interval is not used

	const FLOAT BASE = 0.31;
	const FLOAT UPPER_ARM = 0.4;
	const FLOAT LOWER_ARM = 0.39;
	const FLOAT FLANGE = 0.078;
	const FLOAT8 JOINTLIMITS = (FLOAT8)(RAD170, RAD120, RAD170, RAD120, RAD170, RAD120, RAD170, 0);
	const FLOAT SINGULARITYWARNING = 15; // warning when this many degrees close to singularity

	// get the index of the current element
	int iPar = get_global_id(0); // index of parallel computations

	FLOAT16 TARGET = trafoMul(target[iPar], *invTool);

	//\n#define TARGET target[iPar]\n

	uchar cfg = config[iPar];

	t_interval reachable[11];
	t_interval blocked[11];
	t_interval blockedPerJoint[7][3];

	// invalidate all intervals
	for(int j=0; j<11; j++){
		INVALIDATE(reachable[j]);
		INVALIDATE(blocked[j]);
	}
	for(int j=0; j<7; j++){
		for(int i=0; i<3; i++){
			INVALIDATE(blockedPerJoint[j][i]);
		}
	}

	\n#ifdef RETURN_INTERVALS \n
	for(int j=0; j<64; j++){
		intervals[iPar*64 + j] = 32767; // == invalid
	}
	\n#endif \n

	ushort result=LWR_OK; // resulting error info
	error[iPar] = result;
	ushort tempResult=LWR_OK; // error info for half of the arm

	bool inv1=(cfg & 1) >0; // joint inversions (180deg rotation) due to configuration
	bool inv3=((cfg & 1) >0)^((cfg & 2) >0);
	bool inv5=((cfg & 2) >0)^((cfg & 4) >0);
	bool inv7=(cfg & 4) >0;

	FLOAT3 xw = TARGET.scde - FLANGE*TARGET.s89a; // wrist position
	FLOAT3 xs = (FLOAT3)(0,0,BASE); // shoulder position

	FLOAT dtarg = length(xw-xs);
	FLOAT mindist = sqrt(SQUARE(UPPER_ARM) + SQUARE(LOWER_ARM) + 2.0*UPPER_ARM*LOWER_ARM*cos(JOINTLIMITS.s3));
	FLOAT maxdist = UPPER_ARM+LOWER_ARM;

	if(dtarg > (mindist+maxdist)*0.5){
		cost[iPar] = maxdist-dtarg;
	}
	else{
		cost[iPar] = dtarg-mindist;
	}
	cost[iPar]*=100.0;

	//cost[iPar] = 5;

	// wrist position inside a cartesian csys located in the shoulder
	FLOAT3 xwfroms=xw;

	xwfroms.xy = -xwfroms.xy; // for LWR joint configuration
	xwfroms.z -= BASE;

	t_interval blockedPerJointNormalPart[3][4]; // blocked elbow intervals for first three joints
	t_interval blockedPerJointMirrorPart[3][4]; // blocked elbow intervals for last three joints
	t_interval blockedPerJointMirrorPartMerged[3][4]; // intervals that covered -pi/pi before transformation are merged together
	for(int j=0; j<3; j++){
		for(int i=0; i<4; i++){
			INVALIDATE(blockedPerJointNormalPart[j][i]);
			INVALIDATE(blockedPerJointMirrorPart[j][i]);
			INVALIDATE(blockedPerJointMirrorPartMerged[j][i]);
		}
	}

	FLOAT jointLimitsPart[3];
	jointLimitsPart[0] = JOINTLIMITS.s0;
	jointLimitsPart[1] = JOINTLIMITS.s1;
	jointLimitsPart[2] = JOINTLIMITS.s2;

	tempResult = intervals_part( // compute elbow intervals for the first three joints
			blockedPerJointNormalPart,
			xwfroms,
			UPPER_ARM,
			LOWER_ARM,
			jointLimitsPart,
			inv1,
			inv3);

	result = result | tempResult;
	if(result & LWR_ERROR){
		error[iPar] = result;
		return;
	}

	// for the last three joints, we 'flip' the robot and call the same function

	// shoulder of an inverse robot (flange and base flipped,
	// shoulder and wrist flipped, upper and lower arm flipped)

	// mirrorshoulder is located at the wrist of the (non flipped) robot
	// -z points out the flange, +x is where the lower arm bends if joint 7 is 0deg and
	// joint 6 deflects positively

	// mirrorwrist (shoulder) from mirrorshoulder

	//optimized matrix inversion and multiplication code following that is no longer understandable for the human mind
	FLOAT3 mirrorxwfroms;

	mirrorxwfroms.x = -TARGET.s2*BASE + TARGET.s0*xw.x + TARGET.s1*xw.y + TARGET.s2*xw.z;
	mirrorxwfroms.y = +TARGET.s6*BASE - TARGET.s4*xw.x - TARGET.s5*xw.y - TARGET.s6*xw.z;
	mirrorxwfroms.z = -TARGET.sa*BASE + TARGET.s8*xw.x + TARGET.s9*xw.y + TARGET.sa*xw.z;

	jointLimitsPart[0] = JOINTLIMITS.s6;
	jointLimitsPart[1] = JOINTLIMITS.s5;
	jointLimitsPart[2] = JOINTLIMITS.s4;

	tempResult = intervals_part( // compute elbow intervals for the last three joints
			blockedPerJointMirrorPart,
			mirrorxwfroms,
			LOWER_ARM,
			UPPER_ARM,
			jointLimitsPart,
			inv7,
			inv5);

	result = result | tempResult;
	if(result & LWR_ERROR){
		error[iPar] = result;
		return;
	}

	// now we have the blocked intervals on the elbow circle of the flipped robot
	// and we need to flip them back about a certain angle

	// compute offset angle
	FLOAT3 ex1 = normalize(cross(xw-xs, (FLOAT3)(0,0,1)));
	FLOAT3 ey1 = normalize(cross(ex1, xw-xs));
	FLOAT3 ex2 = normalize(cross(xs-xw, TARGET.s89a));

	FLOAT offset=atan2(dot(ex2, ey1), dot(ex2, ex1));
	FLOAT buffer=0;

	int unused=-1;
	for(int j=0;j<3;j++){
		for(int k=0;k<4;k++){
			if(blockedPerJointMirrorPart[j][k].valid){
				buffer = blockedPerJointMirrorPart[j][k].lower;
				blockedPerJointMirrorPart[j][k].lower = anglemod(offset-blockedPerJointMirrorPart[j][k].upper);
				blockedPerJointMirrorPart[j][k].upper = anglemod(offset-buffer);
			}
			else{
				unused=k; // mark this interval as unused (there is always at least one unused interval which will be needed later)
			}
		}
		for(int k=0;k<4;k++){
			if(blockedPerJointMirrorPart[j][k].lower > blockedPerJointMirrorPart[j][k].upper
					&& blockedPerJointMirrorPart[j][k].valid){
				// this interval now covers the -pi/pi barrier and needs to be split, use the unused interval
				SETBORDERS(blockedPerJointMirrorPart[j][unused],
						-M_PI,
						blockedPerJointMirrorPart[j][k].upper);
				blockedPerJointMirrorPart[j][k].upper=M_PI;
			}
		}
	}

	// now we fill the result array
	for(int j=0;j<3;j++){
		sortIntervals(blockedPerJointNormalPart[j],4);
		for(int k=0;k<3;k++){
			blockedPerJoint[j][k]=blockedPerJointNormalPart[j][k];
		}

		mergeIntervals(blockedPerJointMirrorPartMerged[j],blockedPerJointMirrorPart[j],4);
		for(int k=0;k<3;k++){
			blockedPerJoint[6-j][k]=blockedPerJointMirrorPartMerged[j][k];
			//blockedPerJoint[6-j][k]=blockedPerJointMirrorPart[j][k];
		}
	}

	// assign interval for elbow joint (blocks either all (if target too far or too close) or nothing)
	if(SQUARE(UPPER_ARM) + SQUARE(LOWER_ARM) + 2.0*UPPER_ARM*LOWER_ARM*cos(JOINTLIMITS.s3) > dot(xw-xs, xw-xs)){
		SETBORDERS(blockedPerJoint[3][0], -M_PI,M_PI);
		result = result | LWR_WARNING | LWR_NO_SOLUTION_FOR_ELBOW;
	}

	// merge all intervals
	t_interval blockedBig[21]; // array capable of holding all unmerged intervals

	// the following call is an ugly solution in terms of readability;
	// the whole blockedPerJoint[7][3] array is passed as the src[] argument

	FLOAT jointCost = mergeIntervals(blockedBig,blockedPerJoint[0],21);
	//if(jointCost < cost[iPar]){cost[iPar] = jointCost;}
	//cost[iPar] = jointCost;

	if(result & LWR_NO_SOLUTION_FOR_ELBOW){
		SETBORDERS(blocked[0], -M_PI,M_PI);
		//cost[iPar] = -1; // emergency solution
	}
	else{
		for(int i=0;i<11;i++){
			blocked[i]=blockedBig[i];
		}
	}

	// "compute" reachable intervals from blocked intervals
	int ir=-1;
	if(blocked[0].lower>-M_PI){
		ir++;
		SETBORDERS(reachable[0], -M_PI,M_PI);
	}
	for(int ib=0;ib<11;ib++){
		if(blocked[ib].valid){
			if(ir>=0){
				reachable[ir].upper=blocked[ib].lower;
			}
			if(blocked[ib].upper<M_PI){
				ir++;
				SETBORDERS(reachable[ir], blocked[ib].upper,M_PI);
			}
		}
	}


	int iRes = iPar*64;
	FLOAT biggestInterval = -1;
	FLOAT intervalSum = 0;
	FLOAT currentInterval = 0;

	for(int i=0;i<11;i++){
		if(reachable[i].valid){

			currentInterval = reachable[i].upper-reachable[i].lower;
			intervalSum += currentInterval;

			// special treatment around -pi|pi
			if(reachable[i].upper > M_PI-1e-6 && reachable[0].lower < -M_PI+1e-6){
				currentInterval += reachable[0].upper-reachable[0].lower;
			}

			if(currentInterval > biggestInterval){
				biggestInterval = currentInterval;
			}
			\n#ifdef RETURN_INTERVALS \n
			intervals[iRes++] = reachable[i].lower/M_PI*32400;
			intervals[iRes++] = reachable[i].upper/M_PI*32400;
			\n#endif \n
		}
		else{ // invalidation was performed at the beginning of the function
			iRes+=2;
		}
	}

	\n#ifdef RETURN_INTERVALS \n
	for(int j=0;j<7;j++){
		for(int i=0;i<3;i++){
			if(blockedPerJoint[j][i].valid){
				intervals[iRes++] = blockedPerJoint[j][i].lower/M_PI*32400;
				intervals[iRes++] = blockedPerJoint[j][i].upper/M_PI*32400;
			}
			else{
				iRes+=2;
			}
		}
	}
	\n#endif \n

	//jointCost = blockedPerJoint[0][0].upper - blockedPerJoint[0][0].lower - 0.5;

	//if(biggestInterval < cost[iPar]){cost[iPar] = biggestInterval;}
	if(intervalSum < cost[iPar]){cost[iPar] = intervalSum;}

	error[iPar] = result;/**/
	return;

}

);
