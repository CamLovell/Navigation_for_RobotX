%Likelihood Model for the Quanergy M8 LIDAR
function [w] = lidarLikelihood(y,x,map,params,M)

map.z_use = map.mapped; %Ensure "seen" map is used for particle likelihoods

% Assign actual Measurments
measured = y; 

% Initialise particle weights
w = zeros(M,1);

% Loop through all particles
for k = 1:M
    % Assign states to pose structure for scan function
    pose.N = x(1,k);
    pose.E = x(2,k);
    pose.psi = x(3,k);
    
    % Calculate grid square in which boat exists
    Nidx = round(((pose.N-map.y(1))/(map.y(end)-map.y(1)))*(length(map.y)-1)+1);
    Eidx = round(((pose.E-map.x(1))/(map.x(end)-map.x(1)))*(length(map.x)-1)+1);
    
    % Ensure not outside the map
    if(Nidx>601||Eidx>2001)
        Nidx=601;
        Eidx=2001;
    end
    
    % Only perform ray tracing when not occupied
    if(map.z_use(Nidx,Eidx)~=1)
        % Calculate expected measurements
        expected = lidarScanNE(map,params,pose);
        for i = 1:size(measured,2)
            if measured(i)>params.min && measured(i)<=params.maxRange
                p_hit = (1/sqrt(2*pi*params.sigma^2))*exp(-0.5*((measured(i)-expected(i))^2)/params.sigma^2);
            else
                p_hit = 0;
            end

            % Likelihood of Short Hit given current particle
            if measured(i) <= expected(i) && measured(i) > 0
                p_short = params.lambda*exp(-params.lambda*measured(i))/(1-params.lambda*exp(-params.lambda*expected(i)));
            else
                p_short = 0;
            end

            % Likelihood of Random Return given current particle
            if measured(i)>=0 && measured(i)<=params.maxRange
                p_rand = 1/params.maxRange;
            else
                p_rand = 0;
            end
                        
            % Use LSE to calculate the sum of non-log weights and add to current particle weight
            tempW = [reallog(params.weights(1)*p_hit),reallog(params.weights(2)*p_rand),reallog(params.weights(3)*p_short)];
            w(k) = w(k) + logSumExponential(tempW);
        end
    else
        % Set likelihood to "impossible" if inside occupied square
        w(k) = -inf;
    end
end
    % Normalise lidar likelihoods
    w = w - logSumExponential(w);
end