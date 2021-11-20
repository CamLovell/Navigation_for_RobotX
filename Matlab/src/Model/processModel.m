function x = processModel(t,x,u0,param,dt)
% Standard deviation for induced noise
sigmaU = [2;0.01;2;0.01];
sigmapsi = deg2rad(0.1);

% Create more refined noise application for inputs
k = 50;
ds  = dt/k;
u = u0;
for i = 1:k
    dx = boatDynamicsFast_take2(t,x,u,param);
    x = x+dx*ds;
    u = normrnd(u0,sigmaU);
end

% Sample from the process model distribution
x = normrnd(x,[0.5;0.5;sigmapsi;0.001;0.001;0.001]);

end