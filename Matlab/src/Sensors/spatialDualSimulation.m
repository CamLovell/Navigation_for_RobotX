function [Y_GPS,head] = spatialDualSimulation(X,param,sys,coord)

% Extract coordinates
N = X(1);
E = X(2);
psi = X(3);

% Select accuracy system
switch sys
    case "none"
        sigmaPos = param.GPSsigma;
    case "SBAS"
        sigmaPos = param.GPSsigma_SBAS;
    case "RTK"
        sigmaPos = param.GPSsigma_RTK;
    otherwise
        error("unrecognised GPS system");
end

% Calculate GPS position and extract from normal distribution
Y_GPS_true = Rzyx(151.219855,-(90-33.857481),0)*[N;E;0] + [-4647135;2552687;3533330];
Y_GPS = normrnd(Y_GPS_true,sigmaPos);

% Extract heading from normal distribution 
head = normrnd(psi,param.sigmaHead);
end
