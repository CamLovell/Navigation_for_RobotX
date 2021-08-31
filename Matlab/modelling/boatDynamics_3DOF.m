function dx = boatDynamics(t, x, u, param)

% Extract and Assign inputs and paramaters
eta = x(1:3);
nu = x(4:6);
T1 = u(1);
alpha1 = u(2);
T2 = u(3);
alpha2 = u(4);

m = param.m;
Izz = param.Izz;
rCBb = param.rCBb;

% Rigid Body Mass and Corriolis matricies
MRB = [m,0,-m*rCBb(2);0,m,m*rCBb(1);-m*rCBb(2),m*rCBb(1),Izz];

CRB = [0,0,-m*nu(2); 0,0,m*nu(1);m*nu(2),-m*nu(1),0];

%Augmented Mass and Corriolis Matricies
MA = [param.dXu,param.dXv,param.dXr;param.dYu,param.dYv,param.dYr;param.dNu,param.dNv,param.dNr];

CA = [0,0,MA(2,:)*nu; 0,0,-MA(1,:)*nu;-MA(2,:)*nu,MA(1,:)*nu,0];

%Combined Mass and Corriolis Matricies
M = MRB + MA;
C = CRB + CA;

% Damping Matrix
magnu = abs(nu);
DNL = [param.Xuu*magnu(1),param.Xvv*magnu(2),param.Xrr*magnu(3);...
    param.Yuu*magnu(1),param.Yvv*magnu(2)+param.Yvr*magnu(3),param.Yrr*magnu(3)+param.Yrv*magnu(2);...
    param.Nuu*magnu(1),param.Nvv*magnu(2)+param.Nvr*magnu(3),param.Nrr*magnu(3)+param.Nrv*magnu(2)];
DL = [param.Xu,param.Xv,param.Xr;param.Yu,param.Yv,param.Yr;param.Nu,param.Nv,param.Nr];

D = DL + DNL;

% Reduced J matrix due to heave, pitch and roll being neglected
J = [cos(eta(3)),-sin(eta(3)),0;sin(eta(3)),cos(eta(3)),0;0,0,1];

% Generalised body forces and momoents
fTb = [T1*cos(alpha1)+T2*cos(alpha2);T1*sin(alpha1)+T2*sin(alpha2)];
mTCb = [-param.rLCb(2),param.rLCb(1),-param.rRCb(2),param.rRCb(1)]*[T1*cos(alpha1);T1*sin(alpha1);T2*cos(alpha2);T2*sin(alpha2)];

taub = [fTb;mTCb];

% Environental/Disturbance Forces
    % Will put in later

% State derivatives and assign to output vector
d_eta = J*nu;
d_nu =  M\(-C*nu-D*nu+taub);

dx = [d_eta;d_nu];

end

