clear;
clc;
close all;

% Create paths to allow access to nessecary files
addpath(genpath('../src'));
addpath(genpath('../Data'));

%% Define Paramaters and Simulation Conditions
% Boat Model Paramaters
param = boatParameters();

% Load Data for Map,Trjectory, etc.
load('mapdata');
param.spheroid = wgs84Ellipsoid('meters');
map.logodds = zeros(size(map.exp)); % Set log odds to zero as map is entirely unknown

% Simulation parameters
load('paramaters');
param.dt = 0.2;  % Discrete time step
param.T = 50;    % Total time

% Set initial state in the centre of the bottom gate facing right
x0 = [5;3;pi/2;0;0;0];

% Set time step and solver options
tHist   = 0:param.dt:param.T;  % Vector of time intervals used
options = odeset('MaxStep',0.005,'Events',@(t,x) checkCollision(t,x,param,map));
uHist = nan(4,length(tHist)); 

% Load Sensor Parameters
lidarParam = lidarParam();

% Set initial state estimate
[lw, xp] = boatInitialStateDistribution(3); %3 dummy particles to pass tests, not used
M = length(lw);

% Set Control inputs for MPC
uControl = [];
u = [0;0;0;0];

% Initialise history variables for plotting
uHist(:,1) = u;
x = zeros(6,length(tHist));
x(:,1) = x0;
lwHist = zeros(M,length(tHist));
lidarwHist = zeros(M,length(tHist));
GPSwHist = zeros(M,length(tHist));
IMUwHist = zeros(M,length(tHist));
lwHist(:,1) = lw(:);
xpHist = nan(6,M,length(tHist));
xpHist(:,:,1) = xp;
xMLHist = nan(6,length(tHist));

% Initialise output Video
vid = VideoWriter('Mapping.avi','Uncompressed AVI');
vid.FrameRate = 1/param.dt;
open(vid);

% Initialise Figure used for Animation
visFig = figure(1);
clf(1);
visFig.Color = 'w';
ax = axes(visFig,'FontSize',14);
hold(ax,'on');

% Plot Grid Map according to gray scale colour
[Egrid, Ngrid] = meshgrid(map.x,map.y);
Cgrid = 1./(1 + exp(map.logodds)); % Due to infs above this simply gives the true map with no sacle
colormap(ax,'gray');
caxis(ax,[0 1]);
gridFig = pcolor(ax,Egrid,Ngrid,Cgrid);
gridFig.EdgeColor = 'none';

% Create Objects to be animated
hP_R      = plot(ax,nan,nan,'k-'); %Right pontoon
hP_L      = plot(ax,nan,nan,'k-'); %Left pontoon
hC      = plot(ax,0,0,'k.'); % Centre of origin (assumed same as centre of mass)
tC      = text(ax,0,0,' C','FontSize',10,'Color','k'); %Centre of origin label
hold(ax,'off');
axis(ax,'equal');
axis(ax,[map.x(1),map.x(end),map.y(1),map.y(end)]);
xlabel(ax,'East [m]');
ylabel(ax,'North [m]');

% Define paramates or boat shape
L = 2*param.l;
W = 0.2;
space = param.d/2-W/2;

% Create points to draw boat shape
rPCb_R = [[0;space;0], [L/2;space;0], [L/2;space+W;0], [-L/2;space+W;0], [-L/2;space;0], [0;space;0]];
rPCb_L = -[[0;space;0], [L/2;space;0], [L/2;space+W;0], [-L/2;space+W;0], [-L/2;space;0], [0;space;0]];
circle = [cos(0:0.01:2*pi);sin(0:0.01:2*pi);zeros(size(0:0.01:2*pi))];
line = [[0,1];[0,0];[0,0]];

% Set sizes for vecotrs used in animation
rBNn = nan(3,length(tHist));
rCNn = nan(3,length(tHist));
rLNn = nan(3,length(tHist));
rRNn = nan(3,length(tHist));
rTLNn = nan(3,length(tHist));
rTRNn = nan(3,length(tHist));

% Initialise map for lidar measurements
map.z_use = map.exp;

% Run Simulation for given time length
for i = 1:length(tHist)-1
    try
        % Simulate boat motion for one time step
        func = @(t,x) boatDynamicsFast_take2(t,x,u,param);
        [~,xTemp,err,~,~] = ode45(func,[tHist(i) tHist(i+1)],x(:,i),options);
        xTemp = xTemp.';
        x(:,i+1) = xTemp(:,end);
        
        % Check for collision
        if ~isempty(err)
            warn = ['Collision Detected at N = ', num2str(x(1,i+1)),' E = ', num2str(x(2,i+1))];
            warning(warn);
            break;
        end
        
        % Simulate Lidar measurements
        [r] = lidarSimulation(x(:,i),map,lidarParam);
        
        % Update map based on these measurements
        map = mapUpdate(r,x(:,i),map,lidarParam);
        
        % Compute control required by MPC
        uControl = controlMPC(tHist(i+1),x(:,i+1),u,uControl,param,map);
        u = uControl(1:4);       
        uHist(:,i+1) = u;
        
        % Calculate Orientation of Pontoons
        Rnb = expm(x(3,i)*skew([0;0;1]));
        rBNn(:,i) = [x(1,i); x(2,i); 0];
        rCNn(:,i) = rBNn(:,i) + Rnb*param.rCBb;
        rPNn = rCNn(:,i) + Rnb*rPCb_R;
        rPNn1 = rCNn(:,i) + Rnb*rPCb_L;        
        
        % Update animation plot
        gridFig.CData(1:end,1:end) = 1./(1 + exp(map.logodds));        
        hP_L.XData = rPNn1(2,:);
        hP_L.YData = rPNn1(1,:);
        hP_R.XData = rPNn(2,:);
        hP_R.YData = rPNn(1,:);
        hC.XData = rCNn(2,i);
        hC.YData = rCNn(1,i);
        tC.Position = [rCNn(2,i),rCNn(1,i),0];
        
        % Ensure plot is drawn and wirte to video
        drawnow;
        writeVideo(vid,getframe(visFig));
        
    % Catch errors    
    catch hot_potato
        rethrow(hot_potato); 
    end
end

% Close the video to ensure it is saved properly
close(vid);