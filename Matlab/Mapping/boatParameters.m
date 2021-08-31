function param = boatParameters()

param.l     = 2;          % Longitudinal distance from B to wheels [m]
param.d     = 2;          % Distance between wheels [m]

param.rLCb	= [-param.l;-param.d/2;0];   % Implement this in Problem 1
param.rRCb 	= [-param.l;param.d/2;0];   % Implement this in Problem 1
param.rCBb = [0;0;0];

param.m     = 348.39;            % Robot mass [kg]
param.Izz    = 525.39;     % Yaw inertia [kg.m^2]
param.g     = 9.81;         % Acceleration due to gravity on Mars [m/s^2]

% Hydrodynamic terms
param.dXu = 516;
param.dYu = 0;
param.dNu = 0;

param.dXv = 0;
param.dYv = 991;
param.dNv = 288;

param.dXr = 0;
param.dYr = -1736;
param.dNr = 4486;

param.Xu = 120;
param.Yu = 0;
param.Nu = 0;

param.Xv = 0;
param.Yv = 884;
param.Nv = 552;

param.Xr = 0;
param.Yr = -202;
param.Nr = 1258;

param.Xuu = 85;
param.Xvv = 0;
param.Xrr = 0;

param.Yuu = 0;
param.Yvv = 696;
param.Yrr = 814;

param.Nuu = 0;
param.Nvv = -1516;
param.Nrr = -2884;

param.Yvr = 8361;
param.Nvr = -425;

param.Yrv = 307;
param.Nrv = -202;

param.dofIdx = [1 2 6];     % N, E, psi
