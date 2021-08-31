clear
close all;
clc;

rng(2020);

%Generate path data
D   = 0.1;

%Now load and plot data
load ass2Data.mat;
map0 = map;
clear map;
map.Z = map0;

%Generate a grid map of a room
map.Neast  = size(map0,2);      %Number of East-axis points
map.Nnorth = size(map0,1);      %Number of North-axis points
map.Deast  = 0.2;               %East-axis cell dimension (in metres)
map.Dnorth = 0.2;               %North-axis cell dimension (in metres)
map.east0  = 0;                 %Origin offset East-axis
map.north0 = 0;                 %Origin offset North-axis
map.east   = map.east0:map.Deast:(map.east0+map.Deast*(map.Neast-1));
map.north  = map.north0:map.Dnorth:(map.north0+map.Dnorth*(map.Nnorth-1));
map        = generateMapNE(map);

laser.maxRange    = 120;
laser.resDeg      = 1;
laser.startDeg    = -45;
laser.stopDeg     =  45;
laser.w           = [0.7 0.2 0.09 0.01];
laser.sig         = 0.1;
laser.lambda      = 0.5;
laser.numScans    = 1+(laser.stopDeg - laser.startDeg)/laser.resDeg;
laser.forward0    = 0.0; %Laser forward offset (relative to body)
laser.right0      = 0.0; %Laser right offset (relative to body)
laser.angle_down0 = 0.0; %Laser down angle offset (relative to body - positive )



colormap('bone'); 
plot(map.eastpts,map.northpts,'ks','markersize',14,'markerfacecolor','k');
hold on;
daspect([1 1 1])
axis([-10 90 -10 60])
ax=axis;
xlabel('East direction')
ylabel('North direction')
pause(0.1)


N = length(F1);
y = lidarRanges;
u = [F1;F2];

%Store Data
z.y = y;
z.u = u;


%Setup robot model
param.nx    = 5;
param.D     = D;
param.m     = 20;
param.theta = 15;
param.l     = 0.5;
param.a     = 0.5;
param.bu    = 10;
param.br    = 10;
param.Q     = [1.5;1.5];
param.x0    = [50; 30; +pi/2+1.4711; 0; 0];
param.P0    = 1e-1*eye(5);
param.map   = map;
param.laser = laser;

%Options for the algorithm
opt.M       = 500; %Forward particle count

%Run a particle filter for localisation
g = localise(z,param,opt);

