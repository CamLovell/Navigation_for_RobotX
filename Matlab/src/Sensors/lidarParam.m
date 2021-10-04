function [param] = lidarParam()
    %NOTE: Real Lidar returns an array of measurments for each sensor

    %Assumed/Known Measurments
    % Firing Position in 1/10400 of a rotation (~0.0346deg)
    % Range Measurments(3) for each beam(8) 0 Indicates invalid point (below min or above max)
    % Intensities agasin (3)(8) (Maybe use these to weighted scale the recieved measuements into 1?)
    % Status (8) (Should be 0, if not do not true this laser)
    % Time Stamp for each measurment
    
    % M8 Paramaters from Data Sheet and User Guide
    param.maxRange = 150;
    param.min = 1;
    param.sigma = 0.05;
    param.res = 0.003;
    param.freqRot = 10;
    param.freqFire = 53828;
%     param.rotUnit = 360/10400;
    param.beamAngle = [-0.318505,-0.2692,-0.218009,-0.165195,-0.111003,-0.0557982,0.0,0.0557982];
    param.resDeg      = (360/10400);
%     param.resDeg      = 1;
    param.startDeg    = 0;
    param.stopDeg     = 360-param.resDeg;
%     param.stopDeg     = param.startDeg+579*param.resDeg;
    param.weights     = [0.7 0.2 0.09 0.01];
    param.lambda      = 0.5;
    param.numScans    = floor(1+(param.stopDeg - param.startDeg)/param.resDeg);
    param.forward0    = 0.0; %Laser forward offset (relative to body)
    param.right0      = 0.0; %Laser right offset (relative to body)
    param.angle_down0 = 0.0; %Laser down angle offset (relative to body - positive )
    param.theta = (param.startDeg:param.resDeg:param.stopDeg)*pi/180; 
    
    % Map update stuff
    param.hitDepth = 0.5;
    param.hitChange = 2*param.res;
    param.passChange = 1*param.res;

end