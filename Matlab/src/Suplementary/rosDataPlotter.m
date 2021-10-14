clear;
clc;
close all;

addpath("~/NUMarine_ws/src/navigation/data");

estimated = readmatrix('estimatedState.csv');
tru = readmatrix('trueState.csv');

% recIdx = find(tru(:,1)>1e-10);
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
beep;

