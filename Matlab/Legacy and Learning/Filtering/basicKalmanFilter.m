clear;
clc;
close all;

%% Simulate a simple first-order system to define input output data
%Create input
N = 100;
t = 1:N;
u = input(t);

%Record state and output
y = zeros(1,N);
z = zeros(2,N+1);
z(:,1) = [5;1];

for t=1:N
    %Measurements
    y(:,t) = [1,0]*z(:,t)+10*randn;
    %Update system state
    z(:,t+1) = [1,0.1;0,1]*z(:,t) + [0;1]*u(:,t);
end

plot( y(:,1:N),'k-','linewidth',2);
title('True State Trajectory');
xlabel('Sample number'); 

%% Pretend we do not know what the system is and test the kalman filter
% Define model linear state space model for the system (note not exactly the real model as above)
A = [1,0.1;0,1];
B = [0;1];
C = [1,0];
D = 0;
% Define Q and R covariences
Q = diag([1,1]);
R = 1e-4;

% Define initial mean and covariance
mup(:,1)= [0;0];
Pp(:,:,1) = eye(2);

% Plot the distribution of the initial state and the true initial state
% xrange = linspace(-60,60,5000);
% plot(xrange,normpdf(xrange,mup(1,1),sqrt(Pp(:,:,1))),'b-','linewidth',2);
% hold on
% plot(xrange,normpdf(xrange,mup(2,1),sqrt(Pp(:,:,1))),'b-','linewidth',2);
%Now plot a line for the true initial state
% plot([z(1) z(1)],[0 1],'k-','linewidth',3);
% ax = [xrange(1) xrange(end) 0 0.6];
% axis(ax);
% title('Initial state pdf');
% xlabel('Range of pdf');

% Next we run the Kalman Filter iterations to produce
for t=1:N
    % filtered pdf: p(x_t | y_{1:t}) = N(muf(:,t) , Pf(:,:,t))
    muf(:,t) = mup(:,t) + Pp(:,:,t)*C'*((C*Pp(:,:,t)*C' + R)\(y(:,t) - C*mup(:,t) - D*u(:,t)));
    Pf(:,:,t) = Pp(:,:,t) - Pp(:,:,t)*C'*((C*Pp(:,:,t)*C' + R)\(C*Pp(:,:,t)));
    
    %Now plot the state pdf after correcting with the measurement
%     plot(xrange,normpdf(xrange,muf(:,t),sqrt(Pf(:,:,t))),'g-','linewidth',2);
%     hold on
%     axis(ax);
%     title(sprintf('Filtered position pdf at t=%4d',t))
%     xlabel('Range of pdf')
%     hold off;

    % predicted pdf: p(x_{t+1} | y_{1:t}) = N(mup(:,t+1) , Pp(:,:,t+1))
    mup(:,t+1) = A*muf(:,t) + B*u(:,t);
    Pp(:,:,t+1) = A*Pf(:,:,t)*A' + Q;
    
    %Now plot the state pdf after predicting forward
%     plot(xrange,normpdf(xrange,mup(:,t+1),sqrt(Pp(:,:,t+1))),'b-','linewidth',2);
%     hold on
    
    %Now plot a line for the true state and the measurement
%     plot([z(t+1) z(t+1)],[0 1],'k-','linewidth',3);
%     axis(ax);
%     title(sprintf('Predicted state pdf at t=%4d',t+1))
%     xlabel('Range of pdf')
%     pause(0.1)
end
figure;
% for t = 1:N
%     plot3(t*ones(size(xrange)),xrange,normpdf(xrange,muf(2,t),sqrt(Pf(1,1,t))),'b-');
%     hold on;
% end
% plot(1:N,z(2,1:N),'k-');
t = 1:N;
plot(t,muf(1,:),'b-',t,z(1,2:end),'k-');
figure;
plot(t,muf(2,:),'b-',t,z(2,2:end),'k-');

function u = input(t)
for i = 1:numel(t)
    if t(i)<=25
        u(:,i) = 2;
    elseif t(i)<=75 && t(i)>=50
        u(:,i) = -4;
    else
        u(:,i) = 0;
    end
end

end