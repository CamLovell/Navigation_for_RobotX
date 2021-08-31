function dx = boatDynamics_6DOF(t, x, u, param)

% Extract and Assign inputs and paramaters
eta = x(1:6);
nu = x(7:12);
T1 = u(1);
alpha1 = u(2);
T2 = u(3);
alpha2 = u(4);

vCNb = nu(1:3);
wCNb = nu(4:6);

m = param.m;
ICb = param.ICb;
rCBb = param.rCBb;



% Rigid Body Mass and Corriolis matricies
MRB = [m*eye(3),-m*skew(rCBb);m*skew(rCBb),ICb];

CRB = [m*skew(wCNb), -m*skew(wCNb)*skew(rCBb); m*skew(rCBb)*skew(wCNb), -skew(ICb*wCNb)];

%Augmented Mass and Corriolis Matricies
    % To Do, Move MA to Params as it is constant
MA = [param.dXu,param.dXv,param.dXw,param.dXp,param.dXq,param.dXr;...
    param.dYu,param.dYv,param.dYw,param.dYp,param.dYq,param.dYr;...
    param.dZu,param.dZv,param.dZw,param.dZp,param.dZq,param.dZr;
    param.dKu,param.dKv,param.dKw,param.dKp,param.dKq,param.dKr;...
    param.dMu,param.dMv,param.dMw,param.dMp,param.dMq,param.dMr;...
    param.dNu,param.dNv,param.dNw,param.dNp,param.dNq,param.dNr];

CA = [zeros(3,3),-skew(MA(1:3,1:3)*vCNb+MA(1:3,4:6)*wCNb);...
    -skew(MA(1:3,1:3)*vCNb+MA(1:3,4:6)*wCNb),-skew(MA(4:6,1:3)*vCNb+MA(4:6,4:6)*wCNb)];

%Combined Mass and Corriolis Matricies
M = MRB + MA;
C = CRB + CA;
% TO DO: Add 6_DOF Damping matricies
% Damping Matrix
magnu = abs(nu);
DNL = [param.Xuu*magnu(1),param.Xvv*magnu(2),param.Xrr*magnu(3);...
    param.Yuu*magnu(1),param.Yvv*magnu(2)+param.Yvr*magnu(3),param.Yrr*magnu(3)+param.Yrv*magnu(2);...
    param.Nuu*magnu(1),param.Nvv*magnu(2)+param.Nvr*magnu(3),param.Nrr*magnu(3)+param.Nrv*magnu(2)];
DL = [param.Xu,param.Xv,param.Xr;param.Yu,param.Yv,param.Yr;param.Nu,param.Nv,param.Nr];

D = DL + DNL;

% TO DO: Add 6_DOF J term
% Reduced J matrix due to heave, pitch and roll being neglected
J = [cos(eta(3)),-sin(eta(3)),0;sin(eta(3)),cos(eta(3)),0;0,0,1];

% TO DO: Add Pitch Heave and Roll Interatcions
% Generalised body forces and momoents
fTb = [T1*cos(alpha1)+T2*cos(alpha2);T1*sin(alpha1)+T2*sin(alpha2)];
mTCb = [-param.rLCb(2),param.rLCb(1),-param.rRCb(2),param.rRCb(1)]*[T1*cos(alpha1);T1*sin(alpha1);T2*cos(alpha2);T2*sin(alpha2)];

taub = [fTb;mTCb];

% TO DO: Add environmental forces (tau_wind, Tau_wave, etc)

% State derivatives and assign to output vector
d_eta = J*nu;
d_nu =  M\(-C*nu-D*nu+taub);

dx = [d_eta;d_nu];

end

