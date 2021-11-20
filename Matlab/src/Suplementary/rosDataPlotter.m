clear;
clc;
close all;
% This script reads csv data files from saved Gazebo simulations and plots them

% Add path to stored data
addpath("~/NUMarine_ws/src/navigation/data"); % Path to file will vary depending on instalation

% Read in stored data
estimated = readmatrix('estimatedState.csv');
tru = readmatrix('trueState.csv');

% Extract data and exclude first few times steps (uniniutalised memory typically)
recIdx = 4:size(tru,1);

N = estimated(recIdx,1)';
E = estimated(recIdx,2)';
psi = estimated(recIdx,3)'*180/pi;
u = estimated(recIdx,4)';
v = estimated(recIdx,5)';
r = estimated(recIdx,6)';

Ntrue = tru(recIdx,1)';
Etrue = tru(recIdx,2)';
psitrue = tru(recIdx,3)'*180/pi;
utrue = tru(recIdx,4)';
vtrue = tru(recIdx,5)';
rtrue = tru(recIdx,6)';

timeSteps = size(recIdx,2);

t = 0:0.125:(timeSteps-1)/8;

% Plot Position Tracking
figure(1);
%Plot North Position
subplot(2,2,1);
plot(t,N,t,Ntrue);
legend("Estimated State","Actual State");
title("North Position");
xlabel("Time (s)");
ylabel("North (m) ");

%Plot East Position
subplot(2,2,2);
plot(t,E,t,Etrue);
legend("Estimated State","Actual State");
title("East Position");
xlabel("Time (s)");
ylabel("East (m)");

%Plot Psi Angle
subplot(2,2,3);
plot(t,psi,t,psitrue);
legend("Estimated State","Actual State");
title("Heading Angle");
xlabel("Time (s)");
ylabel("Heading Angle (\circ)");

%Plot Psi Angle
subplot(2,2,4);
plot(N,E,Ntrue,Etrue);
legend("Estimated State","Actual State");
title("N-E Combined 2D");
xlabel("East (m)");
ylabel("North (m)");

sgtitle("Estimated vs Actual States");


% Plot Errors
figure(2);
%Plot North Error
subplot(2,2,1);
plot(t,Ntrue-N);
title("Absolute Error in N");
xlabel("Time (s)");
ylabel("Error (m) ");
%Plot East Error
subplot(2,2,2);
plot(t,Etrue-E);
title("Absolute Error in E");
xlabel("Time (s)");
ylabel("Error (m)");
%Plot Psi Angle Error
subplot(2,2,3);
plot(t,psitrue-psi);
title("Absolute Error in \psi");
xlabel("Time (s)");
ylabel("Error (\circ)");
%Plot RMS position error
RMS = sqrt(((Ntrue-N).^2+(Etrue-E).^2)/2);
subplot(2,2,4);
plot(t,RMS);
title("RMS Position Error");
xlabel("Time (s)");
ylabel("Error (m)");
sgtitle("Error in States");

% Calculate errors
avgRMS = mean(RMS);
maxE = max(abs(Etrue-E));
maxN = max(abs(Ntrue-N));
avgPsi = mean(abs(psitrue-psi));

