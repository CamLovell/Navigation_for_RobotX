function [param] = spatialDualInit()

    % Initialise tge GPS Paramaters
    param.GPSsigma = 1.2;
    param.GPSsigma_SBAS = 0.5;
    param.GPSsigma_RTK = 0.008;
    param.sigmaHead = deg2rad(0.1);
    param.h0 = 0;
    
end