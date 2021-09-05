function [Y_GPS,head] = spatialDualSimulation(X,param,sys,coord)
N = X(1);
E = X(2);
psi = X(3);
p = X(4);
q = X(5);
r = X(6);
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

switch coord
    case "ECEF"
        Y_GPS = Rzyx(151.219855,-(90-33.857481),0)*[N;E;0] + [-4647135;2552687;3533330]; % Will probably just stick with this for now
    case "Geodesic"
        
    otherwise
        error("Coordinate system undefined");
end
% Straight up NED reading for now, will convert to simulating ECEF
head = normrnd(psi,param.sigmaHead);
end
