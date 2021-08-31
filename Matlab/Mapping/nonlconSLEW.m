function [cineq,ceq] = nonlconSLEW(t1, x1, u0, U, param)

N = param.nHorizon; % Control horizon (set in controlMPC.m, not here)
% Actuator slew rates
umax = [1500;pi/2;1500;pi/2]*param.dt;
% Nonlinear equality constraints: ceq(x) == 0
ceq = []; % We don't have any for this problem, so leave it empty

% Nonlinear inequality constraints: cineq(x) <= 0
cineq = zeros(4*N,1);
u = zeros(4,1,N);

for k = 1:N                 % for each step in the control horizon
    u(:,k) = U(4*k-3:4*k);
    % Slew rate constraints
    if k == 1
        cineq((k-1)*4+1:k*4) = abs(u(:,k)-u0)-umax;
    else
        cineq((k-1)*4+1:k*4) = abs(u(:,k)-u(:,k-1))-umax;
    end
end
