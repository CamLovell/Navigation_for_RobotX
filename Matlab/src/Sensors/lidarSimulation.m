function [r,N,E] = lidarSimulation(x,map,param)
% Function to simulate the measurments obtained by the lidar
pose.N = x(1);
pose.E = x(2);
pose.psi = x(3);

%Optain true rages to obstacles
Nidx = round(((pose.N-map.y(1))/(map.y(end)-map.y(1)))*(length(map.y)-1)+1);
Eidx = round(((pose.E-map.x(1))/(map.x(end)-map.x(1)))*(length(map.x)-1)+1);
if(map.logodds(Nidx,Eidx) < 0)
%     error('Particle Generated in Ocupied Cell');
    a = 1;
end
true_range = lidarScanNE(map,param,pose);

%Simulate noise and missed hits
r = zeros(size(param.numScans));
type = randsample(4,param.numScans,true,param.weights);
% type = ones(size(type));
for i = 1:param.numScans
    if true_range(i)>=param.maxRange||true_range(i)<=param.min
        type(i) = 2;
    end
    switch type(i)
        case 1 %Hit
            r(i) = normrnd(true_range(i),param.sigma);
        case 2 %Max
            r(i) = 0;
        case 3%Short
            r(i) = -log(rand)/param.lambda;
        case 4%Rand
            r(i) = param.maxRange*rand;   
        otherwise %Somehow something else has happened
            r(i) = 0;
    end
end

% if nargout>1
%    for i = 1: 
% end

theta = pose.psi - param.theta;
for i = 1:param.numScans
    N(i) = pose.N + r(i)*sin(theta(i));
    E(i) = pose.E + r(i)*cos(theta(i));
end

end