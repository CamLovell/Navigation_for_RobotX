function V = errorMPC(t1, x1, u0, U, param) %#ok<INUSL>

dt = param.dt;
N = param.nHorizon; % Control horizon (set in controlMPC.m, not here)

% Penalty coefficients
qr           = 50^3;      % Penalty on position (N, E)
qpsi         = 50^3;      % Penalty on orientation (Psi)
sqrtruT      = 1/(100);   % Penalty on u1 and u3 (Thrust)
sqrtruAlpha  = 1;         % Penalty on u2 and u4 (Angle)

t = t1;
x = x1;
f = @boatDynamicsFast_take2;

for i = 1:N                 % for each step in the control horizon
    u = U(4*i-3:4*i);
    
    % Do one step of RK4 integration
    f1 = f(t,        x,           u, param);
    f2 = f(t + dt/2, x + f1*dt/2, u, param);
    f3 = f(t + dt/2, x + f2*dt/2, u, param);
    f4 = f(t + dt,   x + f3*dt,   u, param);
    x = x + (f1 + 2*f2 + 2*f3 + f4)*dt/6;
    t = t + dt;
    
    % Evaluate trajectory
    X = trajectoryEval(param.traj,t);
    rPNn = [X(1:2,1);0];
    psistar = X(3,1);

    % Unwrap angle
    temp = unwrap([x(3) psistar]);
    psistar = temp(2);
    
    % Extract states
    q = x(1:3);         % Displacement states
    psi = q(3);
    
    % Current position
    rCNn = [q(1);q(2);0];
    
    % Cost function
    V =  qr*(norm(rCNn(1:2) - rPNn(1:2)).^2) + qpsi*(psi - psistar).^2 + ...
            sqrtruT*u(1).^2 + sqrtruAlpha*u(2).^2 + sqrtruT*u(3).^2 + sqrtruAlpha*u(4).^2;
    
end