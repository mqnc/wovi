function joints=IK(target,nsparam,config)
% function joints=IK(target,nsparam,config)
% 
% all angles are in rad, joints are numbered from base (1)
% to hand flange (7), all joints 0 means the arm is fully
% stretched along z; the base stands at eye(4) pointing into
% z+ direction and plugs pointing into x+ direction
% 
% singularities are not considered and may cause errors or
% problems
% 
% target:   4x4 matrix specifying position and orientation
% 	of the hand flange
% 	target = tcptarget * inv(flange->tcp 4x4)
% nsparam:  nullspace parameter specifying the rotation of
% 	the elbow around a line between shoulder and wrist
% 	(ball), 0 means upwards
% config:   number between 0 and 7, selecting one of the 8
% 	mathematically possible solutions:
%       0 (=bin. 000) means that joint 2,4 and 6 have
%           positive deflection
%       1 (001) -> j2 negative, j4 positive, j6 positive
%       2 (010) -> j2 + , j4 - , j6 +
%       ...
% 
% joints: resulting jointangles, not checked if limits are
%   exceeded!!! std. limits:
%   [�170�,�120�,�170�,�120�,�170�,�120�,�170�]
%   if target is out of reach (too far or too close), the
%   result is [nan nan nan nan nan nan nan]
% 
% Please refer to LWR4_reference.pdf for illustration
% 
% by Mirko Kunze 2011
% mirko.kunze@kit.edu
%
% modified on 2012/02/06 by Mirko Kunze:
% joint 4 was inverted, which has been fixed

	% link lengths:
	% base -> shoulder -> elbow -> wrist -> flange
	lbs=0.31;
	lse=0.4;
	lew=0.39;
	lwf=0.078;

	% vectors between joints:
	mw=target*makehgtform('translate',[0,0,-lwf]); % matrix for wristpose
	xw=mw(1:3,4); % wrist target pos
	xs=[0;0;lbs]; % shoulder pos
	xf=target(1:3,4); % flange pos

	xsw=xw-xs; % vector from shoulder to wrist
	lsw=norm(xsw); % distance between shoulder and wrist
	usw=xsw/lsw; % unit vector from shoulder to wrist
	
	if lsw>lse+lew || lsw<abs(lse-lew) % no config possible, target out of reach (too far or too close)
		joints(1:7)=nan;
	else
		% finding the position of the elbow:
		
		xwf=xf-xw; % vector from wrist to flange
		cosphi= (lsw^2+lse^2-lew^2)/(2*lsw*lse); % cos of elbow angle
		lseproj= cosphi*lse; % distance from shoulder to elbow projected on the line from shoulder to wrist
		hse= sqrt(lse^2-lseproj^2); % distance between elbow and shoulder-wrist-line
		xeproj= xs+usw*lseproj; % elbow position projected on the shoulder-wrist-line
		% (= point around which the elbow can rotate)
		
		% direction vectors of the plane in which the elbow can rotate:
		usin= cross([0;0;1],xsw);
		usin= usin/norm(usin);
		ucos= cross(xsw,usin);
		ucos= ucos/norm(ucos);

		% actual elbow point
		xe= xeproj+(cos(nsparam)*ucos+sin(nsparam)*usin)*hse;
		xse= xe-xs;
		xew= xw-xe;
		
		% joint axes
		axs= cross(xse,[0;0;1]);
		axs= axs/norm(axs);
		axe= cross(xew,xse);
		axe= axe/norm(axe);
		axw= cross(xwf,xew);
		axw= axw/norm(axw);
		axf= -target(1:3,2);
		
		% axes perpend. to joint axes and to a vector along the next link
		axns= cross(axs,xse);
		axns= axns/norm(axns);
		axne= cross(axe,xew);
		axne= axne/norm(axne);
		axnw= cross(axw,xwf);
		axnw= axnw/norm(axnw);
		axnf= target(1:3,1);
		
		% joint angles
		joints(1)=atan2(dot(axs,[-1;0;0]),dot(axs,[0;1;0]));
		joints(2)=acos(dot(xs,xse)/lbs/lse);
		joints(3)=atan2(dot(axe,axns),-dot(axe,axs));
		joints(4)=acos(dot(xse,xew)/lse/lew);
		joints(5)=atan2(dot(axw,axne),-dot(axw,axe));
		joints(6)=acos(dot(xew,xwf)/lew/lwf);
		joints(7)=atan2(-dot(axf,-axnw),dot(axf,-axw));
		
		% invert joints according to the selected config
		if bitand(config,1)>0
			joints(2)=-joints(2);
			joints(1)=joints(1)+pi;
			joints(3)=joints(3)+pi;
		end
		if bitand(config,2)>0
			joints(4)=-joints(4);
			joints(3)=joints(3)+pi;
			joints(5)=joints(5)+pi;
		end
		if bitand(config,4)>0
			joints(6)=-joints(6);
			joints(5)=joints(5)+pi;
			joints(7)=joints(7)+pi;
		end
		
		% modulo-ize angles so they are all between -pi and pi
		joints=mod(joints+pi,2*pi)-pi;
		
	end
end
