function [param] = spatialDualInit()
    % Initialise the GPS Paramaters
    
    % Accuracies depending on which system is used
    param.GPSsigma = 1.2;
    param.GPSsigma_SBAS = 0.5;
    param.GPSsigma_RTK = 0.08;     
    
    % Convert to ECEF frame
    temp = Rzyx(151.219855,-(90-33.857481),0)*[param.GPSsigma_RTK;param.GPSsigma_RTK;0.1];    
    param.GPSsigma_RTK = max(abs(temp(1:2)));
    
    % Heading uncertainty 
    param.sigmaHead = deg2rad(0.1);
    
    % assumed height of 2D plane
    param.h0 = 0;
    
end