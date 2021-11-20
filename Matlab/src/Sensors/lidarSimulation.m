function [r] = lidarSimulation(x,map,param)
    % Extract states into structure for scan function
    pose.N = x(1);
    pose.E = x(2);
    pose.psi = x(3);

    % Optain "true" ranges to obstacles via ray tracing
    true_range = lidarScanNE(map,param,pose);

    %Simulate noise and missed hits
    r = zeros(size(param.numScans));
    type = randsample(3,param.numScans,true,param.weights);

    for i = 1:param.numScans
        switch type(i)
            case 1 % Hit
                r(i) = normrnd(true_range(i),param.sigma);
            case 2% Short
                r(i) = -log(rand)/param.lambda;
            case 3% Rand
                r(i) = param.maxRange*rand;   
            otherwise %Somehow something else has happened
                r(i) = 0;
        end

        % Assign a measurement of 0 if generated range is invalid
        if r(i)>=param.maxRange||r(i)<=param.min
             r(i) = 0; % M8 lidar returns 0 invalid measurements
        end
    end
end