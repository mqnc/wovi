function clickanalyze

figure(1)

fid = fopen('/home/mirko/catkin_ws/src/wovi/src/debug.bin','r');
J = fread(fid, [7,inf], 'double');
size(J)

n = 1002;
nsteps = 25;

j0 = J(:,1);
[T,~,config] = FK_LWR4(j0);
assignin('base','TCPstart',T(:,:,end));
p0 = T(1:3,4,end);
J(:,1) = [];

ps = [];
cs = [];

for i=1:1002
	jnts = J(:, (i-1)*nsteps+1:i*nsteps);
	jnts(:,jnts(1,:)==3)=[];
	hasnan = 0;
	
	if any(isnan(jnts(:)))
		jnts(:,isnan(jnts(1,:)))=[];
		hasnan = 1;
	end
	
	cs = [cs hasnan];
	
	T = FK_LWR4(jnts(:,end));
	p = T(1:3,4,end);
	ps = [ps p];
end

patch('xdata',ps(1,:), 'ydata',ps(2,:), 'zdata',ps(3,:), 'cdata',cs, ...
	'linestyle','none','marker', 'o', 'facevertexcdata',cs', ...
	'markerfacecolor','flat', 'facecolor','none', 'ButtonDownFcn', @cb);

line(p0(1),p0(2),p0(3),'marker','x','color','red','tag','focus');

axis equal

assignin('base','debugJoints',J);
assignin('base','nsteps',nsteps);
assignin('base','TCP',p0);
assignin('base','startJoints',j0);
assignin('base','config',config);

figure(2)

nnspfield = 100;
nstepsfield = 40;

h = pcolor(rand(nnspfield,nstepsfield));
set(h,'edgecolor', 'none', 'tag', 'nspfield');
x = get(h, 'xdata');
y = get(h, 'ydata');
x = (x-1)/(nstepsfield-1);
y = (y-1)/(nnspfield-1)*2*pi-pi;
set(h, 'xdata', x, 'ydata', y);
axis([0,1,-pi,pi])
shading interp

line([0,1],[0,0],'color',[1,1,1],'tag','nspline','linewidth',2);
line([0,1],[0,0],'color',[1,0,1],'tag','nsplineref','linewidth',2);

figure(3)

axis([0,1,-pi, pi]);

figure(1)
set(gcf,'position',[20,20,800,600])

figure(2)
set(gcf,'position',[900,20,800,600])

figure(3)
set(gcf,'position',[2000,20,800,600])

end

function cb(src,evt)

J = evalin('base','debugJoints');
nsteps = evalin('base','nsteps');
p0 = evalin('base','TCP');
j0 = evalin('base','startJoints');
config = evalin('base','config');
r = 0.7; % probe radius

ex = evt.IntersectionPoint(1);
ey = evt.IntersectionPoint(2);
ez = evt.IntersectionPoint(3);
x = get(src,'xdata');
y = get(src,'ydata');
z = get(src,'zdata');

d = (x-ex).^2 + (y-ey).^2 + (z-ez).^2;
[~,i] = min(d);
disp(i)
%i = 652; % ganz seltsam
%i = 330; % fieser Sprung
%i = 973; %totale Divergenz zw. Matlab und CL
%i = 68; % nice in lokalem Minimum
i=2;
p = [x(i), y(i), z(i)].';

target = p0 + (p-p0)/norm(p-p0)*r;

delete(findobj('tag', 'deleteme'));

set(findobj('tag','focus'),'xdata',[p0(1), target(1)], ...
	'ydata',[p0(2), target(2)],'zdata',[p0(3), target(3)]);

figure(2)

jnts = J(:, (i-1)*nsteps+1:i*nsteps);
jnts(:,jnts(1,:)==3)=[];
jnts = [j0, jnts];

q = ((0:26)/25).^1.7;

n = size(jnts, 2);

for i=1:n
	[~,nsp,~] = FK_LWR4(jnts(:,i));
	nsps(i) = nsp;
	line([q(i+1),q(i),q(i+1)], [nsp+0.5, nsp, nsp-0.5], 'color','white', ...
	'tag', 'deleteme');
end

set(findobj('tag','nspline'), 'xdata', q(1:n), 'ydata', nsps);

h = findobj('tag','nspfield');
nnspfield = size(get(h,'cdata'),1);
nstepsfield = size(get(h,'cdata'),2);

cdata = zeros(nnspfield, nstepsfield);

TCPstart = evalin('base','TCPstart');

[~,nsp0,~] = FK_LWR4(j0);
[rayEnd, endParameter, nspPath] = ikray(...
	TCPstart, eye(4), nsp0, config, target);

set(findobj('tag','nsplineref'), 'xdata', q(1:numel(nspPath)), 'ydata', nspPath);

%todo: minimas reinmalen

minima = [];

for istep=1:nstepsfield
	T = TCPstart;
	qq = (istep-1)/(nstepsfield-1);
	T(1:3,4) = TCPstart(1:3,4)*(1-qq) + target*qq;
	T(4,:) = [0 0 0 1];
		
	for insp=1:nnspfield
		nsp = (insp-1)/(nnspfield-1)*2*pi-pi;
		jtest = IK_matlab(T, nsp, config);
		ctest = cost2(jtest, jtest);
		cdata(insp, istep) = ctest;
		
		if insp>2
			if cdata(insp-1, istep)<cdata(insp, istep) && ...
					cdata(insp-1, istep)<cdata(insp-2, istep)
				minima = [minima; qq, nsp];
			end
		end
	end
end
%cdata(cdata > 0) = 0.1;
%cdata = cdata.^3;
cmax = max(cdata(:));
cmin = min(cdata(:));
%cdata(cdata < 0) = cdata(cdata < 0) * abs(cmax/cmin);

set(h,'cdata',cdata);
line(minima(:,1), minima(:,2), ...
	'linestyle', 'none', 'color', 'green', 'marker', '.', 'tag', 'deleteme')

%colormap(rand(100,3));

figure(3)
line(q(1:n), jnts, 'tag','deleteme')
legend('1','2','3','4','5','6','7')

end