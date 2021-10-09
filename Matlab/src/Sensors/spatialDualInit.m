function [param] = spatialDualInit()

    % Initialise tge GPS Paramaters
    param.GPSsigma = 1.2;
    param.GPSsigma_SBAS = 0.5;
    param.GPSsigma_RTK = 0.08;
    
    temp = Rzyx(151.219855,-(90-33.857481),0)*[param.GPSsigma_RTK;param.GPSsigma_RTK;0.1];
    
    param.GPSsigma_RTK = max(abs(temp(1:2)));
    param.sigmaHead = deg2rad(0.1);
    param.h0 = 0;
    
end