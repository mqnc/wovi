function [m,nsparam,config]=FK_LWR4(joints)
% function [m,nsparam,config]=FK_LWR4(joints)
% forward kinematics for lwr4
%
% all angles are in rad, joints are numbered from base (1)
% to hand flange (7), all joints 0 means the arm is fully
% stretched along z; the base stands at eye(4) pointing into
% z+ direction and plugs pointing into x+ direction
%
% m[4x4x8]: transformation matrix for each link
%   1st is base (eye(4)), last is flange
% nsparam:  nullspace parameter returning the rotation of
%	the elbow around a line between shoulder and wrist
%	(ball), 0 means upwards
% config:   robot configuration, number between 0 and 7
%       0 (=bin. 000) means that joint 2,4 and 6 have
%           positive deflection
%       1 (001) -> j2 negative, j4 positive, j6 positive
%       2 (010) -> j2 + , j4 - , j6 +
%       ...
%
% by Mirko Kunze 2012
% mirko.kunze@kit.edu

m=dh2m(q2dh(joints));

config=double((joints(2)<0)+(joints(4)<0)*2+(joints(6)<0)*4);

xs=m(1:3,4,2); % shoulder pos
xe=m(1:3,4,4); % elbow pos
xw=m(1:3,4,6); % wrist pos

xsw=xw-xs;
xse=xe-xs;
usin= cross([0;0;1],xsw);
usin= usin/norm(usin);
ucos= cross(xsw,usin);
ucos= ucos/norm(ucos);

nsparam=atan2(dot(xse,usin),dot(xse,ucos));
	
end

%% Denavit-Hartenberg
function dh=q2dh(q)
	l=[0.31,0.4,0.39,0.078];
	
	dh=[q(1),	l(1),	0,		 pi/2;
		q(2),	0,		0,		-pi/2;
		q(3),	l(2),	0,		-pi/2;
		q(4),	0,		0,		 pi/2;
		q(5),	l(3),	0,		 pi/2;
		q(6),	0,		0,		-pi/2;
		q(7),	l(4),	0,		0];
end

function m=dh2m(dh)
	n=size(dh,1);
	m=zeros(4,4,n+1);
	m(:,:,1)=eye(4);
	
	for i=1:n
		m(:,:,i+1)=[	cos(dh(i,1)),	-sin(dh(i,1))*cos(dh(i,4)),	sin(dh(i,1))*sin(dh(i,4)),	dh(i,3)*cos(dh(i,1));
			sin(dh(i,1)),	cos(dh(i,1))*cos(dh(i,4)),	-cos(dh(i,1))*sin(dh(i,4)),	dh(i,3)*sin(dh(i,1));
			0,				sin(dh(i,4)),				cos(dh(i,4)),				dh(i,2);
			0,				0,							0,							1];
		
		m(:,:,i+1)=m(:,:,i)*m(:,:,i+1);
	end
end