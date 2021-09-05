% results = runtests('tests');
% assert(all([results.Passed]));
clear;
clc;
close all;

% Robot parameters
param = boatParameters();

% Simulation parameters
dt  = 0.05;         % Evaluation time interval (simulation may internally use smaller steps) [s]
T   = 10;         	% Total simulation time [s]
t   = 0:dt:T;       % Specify times the output is returned
options = odeset('MaxStep',0.005);

% Set initial conditions
x0 = boatInitialConditions_3DOF(param);

% Run the simulation
func = @(t,x) boatDynamics_3DOF(t,x,dummyInputs(t,x),param);
[t,x] = ode45(func,t,x0,options);
t = t.';
x = x.';

%% Plot history
N   = x(1,:);
E   = x(2,:);
psi = x(3,:);
u   = x(3,:);
v   = x(5,:);
w   = x(6,:);
c   = nan(1,length(t));
H   = nan(1,length(t));
uL  = nan(1,length(t));
uR  = nan(1,length(t));
for i = 1:length(t)
%     H(i) = robotEnergy(x(:,i),param);
    tau = dummyInputs(t(i),param);
    TL(i) = tau(1);
    alphaL(i) = tau(2);
    TR(i) = tau(3);
    alphaR(i) = tau(4);    
end

figure(1);clf
% title('Change of Vehicle States During Simulation');
subplot(3,3,1)
plot(t,N)
grid on
title('North position')
ylabel('N [m]')
subplot(3,3,4)
plot(t,E)
grid on
title('East position')
ylabel('E [m]')
subplot(3,3,7)
plot(t,psi*180/pi)
grid on
title('Yaw angle')
xlabel('Time [s]')

ylabel('r [\circ]')
subplot(3,3,2)
plot(t,u)
grid on
title('Surge velocity')
ylabel('u [m/s]')
subplot(3,3,5)
plot(t,v)
grid on
title('Sway velocity')
ylabel('v [m/s]')
subplot(3,3,8)
plot(t,w*180/pi)
grid on
title('Yaw angular velocity')
ylabel('r [\circ/s]')
xlabel('Time [s]')

subplot(4,3,3)
plot(t,TL)
grid on
title('Left Motor Force')
ylabel('\tau [N.m]')
subplot(4,3,6)
plot(t,alphaL)
grid on
title('Left Motor Angle')
ylabel('\tau [N.m]')
subplot(4,3,9)
plot(t,TR)
grid on
title('Right Motor Force')
ylabel('\tau [N.m]')
subplot(4,3,12)
plot(t,alphaR)
grid on
title('Left Motor Angle')
ylabel('\tau [N.m]')

%% Animation
fig     = 2;
hf      = figure(fig); clf(fig);
hf.Color = 'w';
ax      = axes(hf,'FontSize',14);
hold(ax,'on');
title('Animation of Vessel Movement');


hLt     = plot(ax,nan(size(E)),nan(size(N)),'r');
hRt     = plot(ax,nan(size(E)),nan(size(N)),'g');
% hSt     = plot(ax,nan(size(E)),nan(size(N)),'b');
hP      = plot(ax,nan,nan,'k-');
hP1      = plot(ax,nan,nan,'k-');
hL      = plot(ax,0,0,'r.');
hR      = plot(ax,0,0,'g.');
hTL      = plot(ax,nan,nan,'b-');
hTR      = plot(ax,nan,nan,'b-');
hB      = plot(ax,0,0,'k.');
hC      = plot(ax,0,0,'k.');
tL      = text(ax,0,0,' L','FontSize',10,'Color','r');
tR      = text(ax,0,0,' R','FontSize',10,'Color','g');
% tS      = text(ax,0,0,' S','FontSize',10,'Color','b');
tB      = text(ax,0,0,' C','FontSize',10,'Color','k');
% tC      = text(ax,0,0,' C','FontSize',10,'Color','k');

hold(ax,'off');
axis(ax,'equal');
axis(ax,[min(E)-5,max(E)+5,min(N)-5,max(N)+5]);
grid(ax,'on');
xlabel(ax,'East Displacement (m)');
ylabel(ax,'North Displacement (m)');

L = 2*param.l;
w = 0.2;
space = param.d/2-w/2;

theta = linspace(3*pi/2,pi/2,50);
% rPCb = [ [0;r;0], [a;r;0], [a;-r;0], [0;-r;0], r*[cos(theta);sin(theta);zeros(size(theta))] ];

rPCb = [[0;space;0], [L/2;space;0], [L/2;space+w;0], [-L/2;space+w;0], [-L/2;space;0], [0;space;0]];
rPCb1 = -[[0;space;0], [L/2;space;0], [L/2;space+w;0], [-L/2;space+w;0], [-L/2;space;0], [0;space;0]];

Se3     = skew([0;0;1]);

rBNn = nan(3,length(t));
rCNn = nan(3,length(t));
rLNn = nan(3,length(t));
rRNn = nan(3,length(t));
rTLNn = nan(3,length(t));
rTRNn = nan(3,length(t));
for i = 1:length(t)
    Rnb = expm(psi(i)*Se3);
    rBNn(:,i) = [N(i); E(i); 0];
    rCNn(:,i) = rBNn(:,i) + Rnb*param.rCBb;
    rLNn(:,i) = rBNn(:,i) + Rnb*param.rLCb;
    rRNn(:,i) = rBNn(:,i) + Rnb*param.rRCb;
    rTLNn(:,i) = rBNn(:,i) + Rnb*(param.rLCb-[(TL(i)/1000)*cos(alphaL(i));(TL(i)/1000)*sin(alphaL(i));0]);
    rTRNn(:,i) = rBNn(:,i) + Rnb*(param.rRCb-[(TR(i)/1000)*cos(alphaR(i));(TR(i)/1000)*sin(alphaR(i));0]);
    rPNn = rCNn(:,i) + Rnb*rPCb;
    hP.XData = rPNn(2,:);
    hP.YData = rPNn(1,:);
    
    rPNn1 = rCNn(:,i) + Rnb*rPCb1;
    hP1.XData = rPNn1(2,:);
    hP1.YData = rPNn1(1,:);
    
    hLt.XData = rLNn(2,:);
    hLt.YData = rLNn(1,:);
    hRt.XData = rRNn(2,:);
    hRt.YData = rRNn(1,:);
%     hSt.XData = rSNn(2,:);
%     hSt.YData = rSNn(1,:);
    
    hL.XData = rLNn(2,i);
    hL.YData = rLNn(1,i);
    tL.Position = [rLNn(2,i),rLNn(1,i),0];
    hR.XData = rRNn(2,i);
    hR.YData = rRNn(1,i);
    tR.Position = [rRNn(2,i),rRNn(1,i),0];
    hTL.XData = [rLNn(2,i),rTLNn(2,i)];
    hTL.YData = [rLNn(1,i),rTLNn(1,i)];
    hTR.XData = [rRNn(2,i),rTRNn(2,i)];
    hTR.YData = [rRNn(1,i),rTRNn(1,i)];
%     tS.Position = [rSNn(2,i),rSNn(1,i),0];
    hB.XData = rBNn(2,i);
    hB.YData = rBNn(1,i);
    tB.Position = [rBNn(2,i),rBNn(1,i),0];
    hC.XData = rCNn(2,i);
    hC.YData = rCNn(1,i);
    tC.Position = [rCNn(2,i),rCNn(1,i),0];
    pause(dt);
    drawnow
end



