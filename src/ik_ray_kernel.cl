
#ifndef src2str
	#define src2str(src) #src
#endif

const char* ikRayKernelSrc = src2str(

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


bool ik(FLOAT8 *joints, FLOAT16 target, FLOAT nsparam, uchar config){

	const FLOAT BASE = 0.31;
	const FLOAT UPPER_ARM = 0.4;
	const FLOAT LOWER_ARM = 0.39;
	const FLOAT FLANGE = 0.078;

	FLOAT3 xf = target.scde; // flange position
	FLOAT3 xw = xf - FLANGE*target.s89a; // wrist position
	FLOAT3 xs = (FLOAT3)(0,0,BASE); // shoulder position

	FLOAT3 xsw = xw-xs; // vector from shoulder to wrist
	FLOAT lsw = length(xsw); // distance from shoulder to wrist

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

__kernel void ikray(
	__global FLOAT3 *rayEnds,
	__global FLOAT16 *startPose,
	__global FLOAT *nsparam,
	__global uchar *config,
	__global FLOAT3 *targets,
	__global uint *nsteps) {

	// get the index of the current element
	int i = get_global_id(0);

	FLOAT16 stepTarget = *startPose;
	FLOAT16 lastOkTarget = *startPose;
	FLOAT q;
	FLOAT8 jnts;
	FLOAT* pjnts;
	pjnts = (FLOAT*)&jnts; // to allow indexed access
	bool ikres;
	bool reachedEnd=true;

	for(int step=1; step<*nsteps; step++){

		q = (1.0*step/(*nsteps));
		stepTarget.scde = (*startPose).scde*(1-q) + targets[i]*q;

		ikres = ik(&jnts, stepTarget, *nsparam, *config);

		if(ikres==false){
			reachedEnd=false;
		}

		for(int j=0; j<7; j+=2){
			if(fabs(pjnts[j]) > RAD170){
				reachedEnd=false;
			} // joint outside limit
		}
		for(int j=1; j<7; j+=2){
			if(fabs(pjnts[j]) > RAD120){
				reachedEnd=false;
			} // joint outside limit
		}

		if(!reachedEnd){break;}

		lastOkTarget = stepTarget;
	}

	if(reachedEnd){
		rayEnds[i] = (*startPose).scde;
	}
	else{
		rayEnds[i] = lastOkTarget.scde;
	}

	//q = i/400;
	//rayEnds[i] = (1.0-q)*stepTarget.scde + q*targets[i];

}

);
