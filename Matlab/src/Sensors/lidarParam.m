function [param] = lidarParam()
    % M8 Paramaters from Data Sheet and User Guide
    param.maxRange = 150; % Max Range
    param.min = 1; % Min Range
    param.sigma = 0.03; % Used in likelihood model
    param.res = 0.03; % Used in mapping
    param.resDeg      = (360/10400); % Angular resolution
    param.startDeg    = 0; % First firing location
    param.stopDeg     = 360-param.resDeg; % Last firing location
    param.weights     = [0.85 0.09 0.06];
    param.lambda      = 0.5; % Exponential distribution parameter
    param.numScans    = floor(1+(param.stopDeg - param.startDeg)/param.resDeg); % Total number of scan lines
    param.forward0    = 0.0; %Laser forward offset (relative to body)
    param.right0      = 0.0; %Laser right offset (relative to body)
    param.angle_down0 = 0.0; %Laser down angle offset (relative to body - positive )
    param.theta = (param.startDeg:param.resDeg:param.stopDeg)*pi/180; % Vector of firing angles for the LiDAR
    
    % Map update stuff
    param.hitDepth = 0.5;
    param.hitChange = 2*param.res;
    param.passChange = 1*param.res;

end