
#ifndef src2str
	#define src2str(src) #src
#endif

const char* ikRayKernelAutoElbowSrc = src2str(

// n because just # would be interpreted by gcc, this way it is interpreted when sent to GPU
 
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

bool ik(FLOAT8 *joints, FLOAT16 tcpTarget, FLOAT nsparam, uchar config, FLOAT16 invTool){

	const FLOAT BASE = 0.31;
	const FLOAT UPPER_ARM = 0.4;
	const FLOAT LOWER_ARM = 0.39;
	const FLOAT FLANGE = 0.078;

	FLOAT16 target = trafoMul(tcpTarget, invTool);
	//FLOAT16 target = tcpTarget;

	FLOAT3 xf = target.scde; // flange position
	FLOAT3 xw = xf - FLANGE*target.s89a; // wrist position
	FLOAT3 xs = (FLOAT3)(0,0,BASE); // shoulder position

	FLOAT3 xsw = xw-xs; // vector from shoulder to wrist
	FLOAT lsw = length(xsw); // distance from shoulder to wrist

	(*joints).s0 = (*joints).s1 = (*joints).s2 = (*joints).s3 =
	(*joints).s4 = (*joints).s5 = (*joints).s6 = (*joints).s7 =
	NAN;

	// check if target is within reach
	if(lsw >= UPPER_ARM + LOWER_ARM){
		return false;
	}
	if(lsw <= UPPER_ARM - LOWER_ARM){
		return false;
	}

	// find the position of the elbow:
	FLOAT3 usw = xsw/lsw; // unit vector from shoulder to wrist
	FLOAT3 xwf = xf-xw; //vector from wrist to flange
	FLOAT cosphi = (lsw*lsw + UPPER_ARM*UPPER_ARM - LOWER_ARM*LOWER_ARM) / (2.0*lsw*UPPER_ARM); // cos of elbow angle
	FLOAT lseproj = cosphi*UPPER_ARM; // distance from shoulder to <elbow projected on <the line from shoulder to wrist>>
	FLOAT hse = sqrt(UPPER_ARM*UPPER_ARM - lseproj*lseproj); // distance between elbow and shoulder-wrist-line
	FLOAT3 xeproj = xs + usw*lseproj; // elbow position projected on the shoulder-wrist-line
	// (= point around which the elbow can rotate)

	// find direction vectors of the plane in which the elbow can rotate (usin and ucos):
	FLOAT3 x = (FLOAT3)(1,0,0);
	FLOAT3 y = (FLOAT3)(0,1,0);
	FLOAT3 z = (FLOAT3)(0,0,1);

	FLOAT3 usin = cross(z,xsw);
	if(length(usin)<1e-7){
		// this is actually not necessarily a real singularity,
		// just the here chosen nullspace parameter does not work in this situation
		return false;
	}

	usin = normalize(usin);
	FLOAT3 ucos = cross(xsw,usin);
	ucos = normalize(ucos);

	// actual elbow point
	FLOAT3 xe = xeproj + (cos(nsparam)*ucos + sin(nsparam)*usin)*hse;
	FLOAT3 xse = xe-xs;
	FLOAT3 xew = xw-xe;

	// joint axes
	FLOAT3 axs = cross(xse,z);
	if(length(axs)<1e-7){
		return false;
	}
	axs = normalize(axs);

	FLOAT3 axe = cross(xew,xse);
	if(length(axe)<1e-7){
		return false;
	}
	axe = normalize(axe);

	FLOAT3 axw = cross(xwf,xew);
	if(length(axw)<1e-7){
		return false;
	}
	axw = normalize(axw);

	FLOAT3 axf = -target.s456;

	// axes perpend. to joint axes and to a vector along the next link
	FLOAT3 axns = normalize(cross(axs,xse));
	FLOAT3 axne = normalize(cross(axe,xew));
	FLOAT3 axnw = normalize(cross(axw,xwf));
	FLOAT3 axnf = target.s012;

	// joint angles
	(*joints).s0 = atan2(dot(axs,-x), dot(axs,y));
	(*joints).s1 = acos(dot(xs,xse)/BASE/UPPER_ARM);
	(*joints).s2 = atan2(dot(axe,axns),-dot(axe,axs));
	(*joints).s3 = acos(dot(xse,xew)/UPPER_ARM/LOWER_ARM);
	(*joints).s4 = atan2(dot(axw,axne),-dot(axw,axe));
	(*joints).s5 = acos(dot(xew,xwf)/LOWER_ARM/FLANGE);
	(*joints).s6 = atan2(-dot(axf,-axnw),dot(axf,-axw));
	(*joints).s7 = 0;

	// invert joints according to the selected config
	if((config & 1) > 0){
			(*joints).s1=-(*joints).s1;
			(*joints).s0=(*joints).s0+M_PI;
			(*joints).s2=(*joints).s2+M_PI;
	}
	if((config & 2) > 0){
			(*joints).s3=-(*joints).s3;
			(*joints).s2=(*joints).s2+M_PI;
			(*joints).s4=(*joints).s4+M_PI;
	}
	if((config & 4) > 0){
			(*joints).s5=-(*joints).s5;
			(*joints).s4=(*joints).s4+M_PI;
			(*joints).s6=(*joints).s6+M_PI;
	}

	FLOAT* pjnts;
	pjnts = (FLOAT*)joints; // to allow indexed access

	// modulo-ize angles so they are all between -pi and pi
	for(int j=0;j<=6;j+=2){
			while(pjnts[j]>M_PI){pjnts[j]-=2.0*M_PI;}
	}

	return true;
}

// costfunction, infinite in unreachable areas

FLOAT cost(FLOAT8 joints, FLOAT8 jointsBefore){

	FLOAT* pjnts = (FLOAT*)&joints; // to allow indexed access
	FLOAT* pjnts0 = (FLOAT*)&jointsBefore; // to allow indexed access
	FLOAT c=0;

	for (int j=0; j<7; j++){ // if the jump in the joint angles is too big, dont accept the solution
		if(fabs(pjnts[j]-pjnts0[j]) > 1.57){
			c = INFINITY;
			break;
		}
	}

	for (int j=0; j<7; j++){
		if(j%2 == 0){ // torsion joint
			if(fabs(pjnts[j])>=RAD170){
				c = INFINITY;
				break;
			}
			else{
				c += pow(atanh(pjnts[j]/RAD170), 2.0);
			}
		}
		else{ // hinge joint
			if(fabs(pjnts[j])>=RAD120){
				c = INFINITY;
				break;
			}
			else{
				c += pow(atanh(fabs(2.0*(pjnts[j]/RAD120))-1.0), 2.0);
			}
		}
	}

	return c;
}

// different costfunction, positive in unreachable areas

FLOAT cost2(FLOAT8 joints, FLOAT8 jointsBefore){

	const FLOAT k170 = 17.0/18.0;
	const FLOAT b170 = M_PI*k170/(1.0-2.0*k170);
	const FLOAT a170 = (k170-1.0)/k170 * b170*b170;
	const FLOAT c170 = -a170/b170;

	const FLOAT k120 = 12.0/18.0;
	const FLOAT b120 = M_PI*k120/(1.0-2.0*k120);
	const FLOAT a120 = (k120-1.0)/k120 * b120*b120;
	const FLOAT c120 = -a120/b120;

	const FLOAT k5 = 0.5/18.0;
	const FLOAT b5 = M_PI*k5/(1.0-2.0*k5);
	const FLOAT a5 = (k5-1.0)/k5 * b5*b5;
	const FLOAT c5 = -a5/b5;

	FLOAT fitpart[14];

	if(isnan(joints.s0)){
		return sqrt(10.0);
	}

	FLOAT* pjnts = (FLOAT*)&joints; // to allow indexed access
	FLOAT* pjnts0 = (FLOAT*)&jointsBefore; // to allow indexed access

	// fitness function designed so that
	// f(0)=1, f'(0)=0, f(180)=-1, f'(180)=0, f(120 or 170)=0

	// torsion joints:
	for(int j=0; j<7; j+=2){
		fitpart[j] = cos( a170/(fabs(pjnts[j])+b170) + c170);
	}
	// hinge joints:
	for(int j=1; j<7; j+=2){
		fitpart[j] = cos( a120/(fabs(pjnts[j])+b120) + c120);
	}
	// singularities (defined at +-5deg)
	fitpart[7] = -cos( a5/(fabs(pjnts[1])+b5) + c5);
	fitpart[8] = -cos( a5/(fabs(pjnts[3])+b5) + c5);
	fitpart[9] = -cos( a5/(fabs(pjnts[5])+b5) + c5);

	// distance to joints before (should be below 180deg to forbid reconfiguration)
	fitpart[10] = cos( 0.5*(pjnts[0]-pjnts0[0]));
	fitpart[11] = cos( 0.5*(pjnts[2]-pjnts0[2]));
	fitpart[12] = cos( 0.5*(pjnts[4]-pjnts0[4]));
	fitpart[13] = cos( 0.5*(pjnts[6]-pjnts0[6]));

	bool allGreater0 = true;

	for(int i=0; i<14; i++){
		if(fitpart[i]<=0){
			allGreater0 = false;
		}
	}

	if(allGreater0){
		FLOAT denom = 0;
		for(int i=0; i<14; i++){
			denom += 1.0/fitpart[i];
		}
		return -1.0/denom;
	}
	else{
		FLOAT radic = 0;
		for(int i=0; i<14; i++){
			if(fitpart[i]<0){
				radic += pow(fitpart[i],2);
			}
		}
		return sqrt(radic);
	}
}

// quaternion stuff

FLOAT16 q2mat(FLOAT4 q){ // w x y z
FLOAT16 R;

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
R.s0=1-2*q.s2*q.s2-2*q.s3*q.s3;  R.s4=2*q.s1*q.s2-2*q.s3*q.s0;    R.s8=2*q.s1*q.s3+2*q.s2*q.s0;   R.sc=0;
R.s1=2*q.s1*q.s2+2*q.s3*q.s0;    R.s5=1-2*q.s1*q.s1-2*q.s3*q.s3;  R.s9=2*q.s2*q.s3-2*q.s1*q.s0;   R.sd=0;
R.s2=2*q.s1*q.s3-2*q.s2*q.s0;    R.s6=2*q.s2*q.s3+2*q.s1*q.s0;    R.sa=1-2*q.s1*q.s1-2*q.s2*q.s2; R.se=0;
R.s3=0;                          R.s7=0;                          R.sb=0;                         R.sf=1;

return R;
}

FLOAT4 slerp(FLOAT4 q0, FLOAT4 q1, FLOAT t){
	if(dot(q0,-q1) > dot(q0,q1)){
		q1=-q1;
	}

	if(dot(q0,q1) > 1.0-1e-7){
		return q0;
	}

	FLOAT omega = acos(dot(q0,q1));

	FLOAT w0 = sin((1.0-t)*omega)/sin(omega);
	FLOAT w1 = sin(t*omega)/sin(omega);

	return normalize(w0*q0 + w1*q1);
}

__kernel void ikray(
	__global FLOAT *endParameters,
	__global FLOAT3 *rayEnds,
	__global FLOAT4 *rayColors,
	__global FLOAT3 *startPos,
	__global FLOAT4 *startOri,
	__global FLOAT16 *invTool,
	__global FLOAT *nsparam,
	__global uchar *config,
	__global FLOAT3 *targets,
	__global FLOAT4 *targetOris,
	__global uint *nsteps) {

	// get the index of the current element
	int i = get_global_id(0);

	FLOAT delta = 0.5; // check elbow parameter cw and ccw per step (in rad)
	int nElbowSteps = 8; // steps to find a good nsp for the step within +-delta of the nsp of the last step

	// current target along the ik ray
	FLOAT16 rayStepTarget = q2mat(*startOri);
	rayStepTarget.scde = *startPos;
	// last feasable target along the ik ray
	FLOAT16 lastOkTarget = rayStepTarget;
	// temporary interpolation parameter
	FLOAT q;
	FLOAT qp;
	FLOAT endParam=0;

	FLOAT8 jntsCurrent;
	FLOAT8 jntsLastElbowStep;
	FLOAT8 jntsLastRayStep;

	bool ikres;
	FLOAT nspCurrent = *nsparam;
	FLOAT nspLastElbowStep = nspCurrent;
	FLOAT nspLastRayStep = nspCurrent;

	FLOAT costCurrent = 0;
	FLOAT costLastElbowStep = 0;
	FLOAT costLastRayStep = 0;

	// ray steps: steps along the current ik probe ray
	// elbow steps: test elbow positions at the current step along the ray

	// start joints
	ikres = ik(&jntsLastRayStep, rayStepTarget, nspCurrent, *config, *invTool);
	costLastRayStep = cost2(jntsLastRayStep, jntsLastRayStep);

	if(costLastRayStep > 0){ // nothing todo, current pose unreachable
		rayEnds[i] = *startPos;
		return;
	}

	lastOkTarget = rayStepTarget;

	for(int rayStep = 1; rayStep <= *nsteps; rayStep++){ // step along the ray

		qp = (1.0*rayStep/(*nsteps)); // progress
		qp = pow(qp,1.7);
		rayStepTarget = q2mat(slerp(*startOri, targetOris[i], qp)); // current testing position
		//rayStepTarget = q2mat(targetOris[i]); // current testing position
		rayStepTarget.scde = *startPos*(1.0-qp) + targets[i]*qp;

		nspCurrent = nspLastRayStep;

		ikres = ik(&jntsCurrent, rayStepTarget, nspCurrent, *config, *invTool);
		costCurrent = cost2(jntsCurrent, jntsCurrent);

		FLOAT dir = 1.0;
		ikres = ik(&jntsCurrent, rayStepTarget, nspCurrent + 1e-5, *config, *invTool);
		if(cost2(jntsCurrent, jntsCurrent) > costCurrent){dir = -1.0;}

		costLastElbowStep = costCurrent;
		nspLastElbowStep = nspCurrent;

		for(int elbowStep = 1; elbowStep <= nElbowSteps; elbowStep++){

			q = ((FLOAT)elbowStep) / ((FLOAT)nElbowSteps);
			q = pow(q, 1.5);

			nspCurrent = nspLastRayStep + q*dir*delta;

			ikres = ik(&jntsCurrent, rayStepTarget, nspCurrent, *config, *invTool);
			costCurrent = cost2(jntsCurrent, jntsLastRayStep);

			if(costCurrent < costLastElbowStep){ // are we still going down?
				costLastElbowStep = costCurrent;
				nspLastElbowStep = nspCurrent;
				jntsLastElbowStep = jntsCurrent;
			}
			else{
				break; // we passed the minimum
			}
		}

		if(costLastElbowStep < 0){
			costLastRayStep = costLastElbowStep;
			jntsLastRayStep = jntsLastElbowStep;
			nspLastRayStep = nspLastElbowStep;
			lastOkTarget = rayStepTarget;
			endParam = qp;
		}
		else{
			break;
		}
	}

	rayEnds[i] = lastOkTarget.scde;

	q = endParam;
	endParameters[i] = endParam;

	FLOAT qAlpha = 1.4*q;
	if(qAlpha > 1.0){qAlpha = 1.0;}

	FLOAT qColor = exp(-pow(3.0*q,2.0));

	rayColors[i].xyz = (qColor)*(FLOAT3)(1,0.9,0.9) + (1-qColor)*(FLOAT3)(0,0.73,1);
	rayColors[i].w = 1.0 - qAlpha;

}

);
