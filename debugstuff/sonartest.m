function sonartest

figure

addpath /home/mirko/development/sphere_grid_library/

p = sphere_icos2_points(10);

flangeStart = [
  -0.394065584755768   0.736396656942949  -0.549938431603341  -0.501780326877307
   0.907555740206024   0.217310162566541  -0.359331144857217  -0.349342653983352
  -0.145103043835027  -0.640699818080630  -0.753955469362255   0.346950878465515
                   0                   0                   0                   1];

tool = [
   1    0    0    0
   0    1    0    0
   0    0    1 0.08
   0    0    0    1];
  

invTool = inv(tool);

tcpstart = flangeStart*tool;

tic
for i=1:size(p, 2);
	[rayEnd, qEnd] = ikray(tcpstart, invTool, 0, 2, tcpstart(1:3,4) + p(:,i)*0.7);
	rayEnds(1:3,i) = rayEnd;
end
toc

line(rayEnds(1,:),rayEnds(2,:),rayEnds(3,:), 'linestyle','none','marker','.');

copytobase;

axis equal

end

