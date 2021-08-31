%%Likelihood Model for the Quanergy M8 LIDAR
% This function takes in the measurements for a
function [w] = lidarLikelihood(y,x,map,params,M)

if nargin < 5
    M = 1;
end
measured = y; %Actual Measurments

w = zeros(M,1);
for k = 1:M
    pose.N = x(1,k);
    pose.E = x(2,k);
    pose.psi = x(3,k);
    expected = lidarScanNE(map,params,pose); % NEED TO LOOK AT THE FORM OF INPUTS FOR ADRIAN'S SCAN FUNCTION
%     expected = expected(y~=0);
    for i = 1:size(measured,2)
        % Likelihood of hit given current particle % May need to include the
        % division by integral
%         if measured(i)<=params.min && measured(i)> 0
%             warning('test');
%         end
        if measured(i)>params.min && measured(i)<=params.maxRange
            p_hit = (1/sqrt(2*pi*params.sigma^2))*exp(-0.5*((measured(i)-expected(i))^2)/params.sigma^2);
        else
            p_hit = 0;
        end
        
        % Likelihood of Max Range given current particle // do not add in max hits
        % NOTE: The M8 lidar returns exactly 0 for invalid measurements (i.e. above max range)
        if measured(i) == 0 || measured(i) >= 150
%             p_max = 1;
            continue;
        else
            p_max = 0;
        end
        
        % Likelihood of Short Hit given current particle
        if measured(i) <= expected(i) && measured(i) > 0
            p_short = params.lambda*exp(-params.lambda*expected(i))/(1-params.lambda*exp(-params.lambda*expected(i)));
        else
            p_short = 0;
        end
        
        % Likelihood of Random Return given current particle
        if measured(i)>0 && measured(i)<=params.maxRange
            p_rand = 1/params.maxRange;
        else
            p_rand = 0;
        end
        
        % Add all likelihoods to get true likelihood
        tempW = [reallog(params.weights(1)*p_hit),reallog(params.weights(2)*p_max),reallog(params.weights(3)*p_rand),reallog(params.weights(4)*p_short)];
%         tempMax = max(tempW);
%         w(k) = w(k) + tempMax + reallog(sum(exp(tempW-tempMax),2));
           w(k) = w(k) + logSumExponential(tempW);
%         w(k)
%         w(k)
    end
end
w(k);
end