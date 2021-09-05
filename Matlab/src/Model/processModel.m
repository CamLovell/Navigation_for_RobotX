function x = processModel(t,x,u0,param,dt)

sigmaU = [2;0.01;2;0.01];
sigmapsi = deg2rad(0.1);

k = 50;
ds  = dt/k;
u = u0;
for i = 1:k
    dx = boatDynamicsFast_take2(t,x,u,param);
    x = x+dx*ds;
    u = normrnd(u0,sigmaU);
end

% Draw a sample from process model p(x[k+1]|x[k])
x = normrnd(x,[0.5;0.5;sigmapsi;0.001;0.001;0.001]);

end