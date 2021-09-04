clear;
clc;
% close all;

%% Define Paramaters and Simulation Conditions
% Boat Model Paramaters
param = boatParameters();

% Load Data for Map,Trjectory, etc.
load('mapdata');
load('paramaters');
param.spheroid = wgs84Ellipsoid('meters');
map.logodds = log(map.exp) - log1p(-map.exp);     % Log odds of occupancy
% Simulation parameters
param.dt    = 0.2;         % Evaluation time interval (simulation may internally use smaller steps) [s]
param.T     = 50;         	% Total simulation time [s]

x0 = [5;3;pi/2;0;0;0];

tHist   = 0:param.dt:param.T;       % Specify times the output is returned
options = odeset('MaxStep',0.005,'Events',@(t,x) checkCollision(t,x,param,map));
uHist = nan(4,length(tHist)); 

lidarParam = lidarParam();
spacialParam = spatialDualInit();

% Set initial state estimate
[lw, xp] = boatInitialStateDistribution();
M = length(lw);

U = [];
u = [0;0;0;0];                      % Initial control action
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

vid = VideoWriter('Localisation.avi','Uncompressed AVI');
vid.FrameRate = 1/param.dt;
% vid.Quality = 100;
open(vid);


fig     = 1;
hf      = figure(fig);
clf(fig);
hf.Color = 'w';
ax      = axes(hf,'FontSize',14);
hold(ax,'on');

[Egrid, Ngrid] = meshgrid(map.x,map.y);
Cgrid = 1./(1 + exp(map.logodds));
colormap(ax,'gray');
caxis(ax,[0 1]);

hGrid   = pcolor(ax,Egrid,Ngrid,Cgrid);
hGrid.EdgeColor = 'none';

hRay    = plot(ax,nan(2,lidarParam.numScans),nan(2,lidarParam.numScans),'r-');
hP_R      = plot(ax,nan,nan,'k-'); %Right pontoon
hP_L      = plot(ax,nan,nan,'k-'); %Left pontoon
hC      = plot(ax,0,0,'k.'); % Centre of origin (assumed same as centre of mass)
tC      = text(ax,0,0,' C','FontSize',10,'Color','k'); %Centre of origin label
hPart   = plot(ax,nan(M,1),nan(M,1),'b.');
for q = 1:M
    hResamp(q)  = plot(ax,nan,nan,'r');
    hResampLine(q)  = plot(ax,nan,nan,'r');
end
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

for i = 1:length(tHist)-1
    try
        % Simulate one time step
        func = @(t,x) boatDynamicsFast_take2(t,x,u,param);
        [~,xTemp,err,~,~] = ode45(func,[tHist(i) tHist(i+1)],x(:,i),options);
        xTemp = xTemp.';
        x(:,i+1) = xTemp(:,end);
        t_sim(i) = tHist(i+1);
        
        % Check for collision
        if ~isempty(err)
            warn = ['Collision Detected at N = ', num2str(x(1,i+1)),' E = ', num2str(x(2,i+1))];
            warning(warn);
            break;
        end
        
        [ECEF,head] = spatialDualSimulation(x(:,i),spacialParam,'RTK','ECEF');
        [GPSwHist(:,i)] = GPSLikelihood(ECEF,xpHist(:,:,i),spacialParam,M);
        [IMUwHist(:,i)] = IMULikelihood(head,xpHist(:,:,i),spacialParam,M);
        
        lwHist(:,i) = GPSwHist(:,i) + IMUwHist(:,i);
        if(tHist(i) > 2)
            [r,Npts,Epts] = lidarSimulation(x(:,i),map,lidarParam);
            [lidarwHist(:,i)] = lidarLikelihood(r,xpHist(:,:,i),map,lidarParam,M);
            
            lwHist(:,i) = lidarwHist(:,i) + lwHist(:,i);
        end
        lwHist(:,i) = lwHist(:,i) - logSumExponential(lwHist(:,i));
%         lwHist_2(:,i) = GPSwHist(:,i) - logSumExponential(GPSwHist(:,i));

        xpPlot = xpHist(:,:,i); % Save particles before resampling (for plotting)
        
        %Generate indicies according to categorical distribution
        idx = randsample(length(lwHist(:,i)),M,true,exp(lwHist(:,i)));
        xpHist(:,:,i) = xpHist(:,idx,i);
        
        xpPlot2 = xpHist(:,:,i); % Save particles after resampling (for plotting)
        
        for k = 1:M
        xpHist(:,k,i+1) = processModel(tHist(i),xpHist(:,k,i),uHist(:,i),param,param.dt);
        end
        % Compute next control action
        U = controlMPC(tHist(i+1),x(:,i+1),u,U,param,map);
        
        % Store next control action to apply
        u = U(1:4);
       
        uHist(:,i+1) = u; % Save for plotting
        
        % Update plot
        Rnb = expm(x(3,i)*skew([0;0;1]));
        rBNn(:,i) = [x(1,i); x(2,i); 0];
        rCNn(:,i) = rBNn(:,i) + Rnb*param.rCBb;
        rLNn(:,i) = rBNn(:,i) + Rnb*param.rLCb;
        rRNn(:,i) = rBNn(:,i) + Rnb*param.rRCb;
        rTLNn(:,i) = rBNn(:,i) + Rnb*(param.rLCb-[(uHist(1,i)/1000)*cos(uHist(2,i));(uHist(1,i)/1000)*sin(uHist(2,i));0]);
        rTRNn(:,i) = rBNn(:,i) + Rnb*(param.rRCb-[(uHist(3,i)/1000)*cos(uHist(4,i));(uHist(3,i)/1000)*sin(uHist(4,i));0]);
        rPNn = rCNn(:,i) + Rnb*rPCb_R;
        rPNn1 = rCNn(:,i) + Rnb*rPCb_L;
        
        
        
        hGrid.CData(1:end,1:end) = 1./(1 + exp(map.logodds));        
        hP_L.XData = rPNn1(2,:);
        hP_L.YData = rPNn1(1,:);
        hP_R.XData = rPNn(2,:);
        hP_R.YData = rPNn(1,:);
        hC.XData = rCNn(2,i);
        hC.YData = rCNn(1,i);
        tC.Position = [rCNn(2,i),rCNn(1,i),0];
        hPart.XData = xpPlot(2,:);
        hPart.YData = xpPlot(1,:);
        for q = 1:M
            hResamp(q).XData = xpPlot2(2,q) + circle(1,:);
            hResamp(q).YData = xpPlot2(1,q) + circle(2,:);
            line_angle =  xpPlot2(1:3,q) + expm(xpPlot2(3,q)*skew([0;0;1]))*line;
            hResampLine(q).XData = line_angle(2,:);
            hResampLine(q).YData = line_angle(1,:);            
        end
        
%         for k = 1:lidarParam.numScans
%             hRay(k).XData = [x(2,i), Epts(k)];
%             hRay(k).YData = [x(1,i), Npts(k)];
%         end
        drawnow
        writeVideo(vid,getframe(hf));
    catch hot_potato
%         delete(wh);                 % Remove waitbar if error
        rethrow(hot_potato);        % Someone else's problem now
    end
%     waitbar(i/length(tHist),wh);    % Update waitbar
end

close(vid);
