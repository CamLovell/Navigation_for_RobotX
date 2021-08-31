function [X] = IMUSimulation(x,param)
k = 1000/param.dt;
N = x(1);
E = x(2);
psi = x(3);
p = x(1);
q = x(2);
r = x(3);

psi_temp = psi;
for i = 1:k
    psi_temp = normrand(psi_temp,param.angSigma);
    angVel = normrand(psi_temp,param.angSigma);
end
end