function [rayEnd, endParameter, nspPath] = ikray(...
	startPose, invTool, nsparam, config, target)
	
	nsteps = 25; % steps along the ray
	delta = 0.5; % scout elbow parameter cw and ccw per step (in rad)
	nElbowSteps = 8; % steps to find a good nsp for the step within +-delta of the nsp of the last step
		
	% current target along the ik ray
	rayStepTarget = startPose;
	% last feasable target along the ik ray
	lastOkTarget = rayStepTarget;

	endParam=0;

	jntsCurrent = zeros(1,7);
	jntsLastElbowStep = zeros(1,7);
	jntsLastRayStep = zeros(1,7);
	
	nspCurrent = nsparam;
	nspPath = nsparam;
	nspLastElbowStep = nspCurrent;
	nspLastRayStep = nspCurrent;

	costCurrent = 0;
	costLastElbowStep = 0;
	costLastRayStep = 0;

	% ray steps: steps along the current ik probe ray
	% elbow steps: test elbow positions at the current step along the ray

	% start joints
	jntsLastRayStep = IK_matlab(rayStepTarget*invTool, nspCurrent, config);
	costLastRayStep = cost2(jntsLastRayStep, jntsLastRayStep);
	
	if costLastRayStep > 0 % nothing todo, current pose unreachable
		rayEnd = startPose(1:3,4);
		endParameter = 0;
		return;
	end

	lastOkTarget = rayStepTarget;

	for rayStep = 1:nsteps % step along the ray

		if rayStep == 10
			5798;
		end
		
		if rayStep == 11
			5798;
		end
		
		qp = rayStep/nsteps; % progress
		qp = qp^1.7;
		
		rayStepTarget = startPose;
		rayStepTarget(1:3,4) = startPose(1:3,4)*(1-qp) + target*qp;

		nspCurrent = nspLastRayStep;

		jntsCurrent = IK_matlab(rayStepTarget*invTool, nspCurrent, config);
		costCurrent = cost2(jntsCurrent, jntsLastRayStep);

		dir = 1.0;

		costLastElbowStep = costCurrent;
		nspLastElbowStep = nspCurrent;
		jntsLastElbowStep = jntsCurrent;
		
		jntsCurrent = IK_matlab(rayStepTarget*invTool, nspCurrent + 1e-5, config);
		if cost2(jntsCurrent, jntsLastRayStep) > costCurrent
			dir = -1.0;
		end

		for elbowStep = 1:nElbowSteps
			
			q = elbowStep / nElbowSteps;
			q = q^1.5;

			nspCurrent = nspLastRayStep + q*dir*delta;

			jntsCurrent = IK_matlab(rayStepTarget*invTool, nspCurrent, config);

			costCurrent = cost2(jntsCurrent, jntsLastRayStep);
			%costCurrent = cost2(jntsCurrent, jntsCurrent);

			if(costCurrent < costLastElbowStep) % are we still going down?
				costLastElbowStep = costCurrent;
				nspLastElbowStep = nspCurrent;
				jntsLastElbowStep = jntsCurrent;
			else
				break; % we passed the minimum
			end
		end

		if(costLastElbowStep < 0)
			costLastRayStep = costLastElbowStep;
			jntsLastRayStep = jntsLastElbowStep;
			nspLastRayStep = nspLastElbowStep;
			nspPath = [nspPath nspLastElbowStep];
			lastOkTarget = rayStepTarget;
			endParam = qp;
		else
			break;
		end
	end

	rayEnd = lastOkTarget(1:3,4);

	q = endParam;
	endParameter = endParam;

end