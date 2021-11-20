function map = mapUpdate(y,x,map,param)

    % Extract states
    N = x(1);
    E = x(2);
   
    % Extract Grid centres from map data
    N_cent = map.y(1:end-1)+map.dy;
    E_cent = map.x(1:end-1)+map.dx;
    
    % Calculate maximum possible scan distance on the given map
    maxDist = hypot(N_cent(end)-N_cent(1),E_cent(end)-E_cent(1));
    
    % Measurement intervals at which scan lines are discretised
    measInt = 0:param.res:maxDist;
    
    % Discritise lines into N and E components
    N_meas = N + measInt.*cos(param.theta(:));
    E_meas = E + measInt.*sin(param.theta(:));
    
    % Check which of these points is inside the map
    inMap = N_meas < N_cent(end) & N_meas > N_cent(1) & E_meas < E_cent(end) & E_meas > E_cent(1);
    
    % Find indicies corresponding to the grid squares in which discritised points exist
    Nidx = round(((N_meas-N_cent(1))/(N_cent(end)-N_cent(1)))*(length(N_cent)-1)+1);
    Eidx = round(((E_meas-E_cent(1))/(E_cent(end)-E_cent(1)))*(length(E_cent)-1)+1);
    
    % Find Measurements inside useable range
    isUseable = reshape(find(y < param.maxRange),1,[]);
    
    % Loop through useable measurements 
    for i = isUseable
        % Loop through each discritesed point for this measurement
        for j = 1:length(measInt)
            % Check if point exists in the map
            if inMap(i,j)
                % Perform 0 range trick, -inf ensures case never passes so is trick not used
                if y(i)==-inf
                    map.logodds(Nidx(i,j),Eidx(i,j)) = map.logodds(Nidx(i,j),Eidx(i,j)) - param.passChange/5;
                    
                elseif measInt(j) < y(i) % If discitised point befor measurement
                    map.logodds(Nidx(i,j),Eidx(i,j)) = map.logodds(Nidx(i,j),Eidx(i,j)) - param.passChange;
                    
                elseif(measInt(j) < y(i) + param.hitDepth && y(i)~=0) % If discitised point between measurement and hit depth
                    map.logodds(Nidx(i,j),Eidx(i,j)) = map.logodds(Nidx(i,j),Eidx(i,j)) + param.hitChange;

                else % If discitised point after hit depth
                    break; % Break to next iteration
                end
            else
                break; % Break to next iteration
            end 
        end
    end
    
    % Satruate log odds
    map.logodds(map.logodds > 50) = 50;
    map.logodds(map.logodds < -50) = -50;
    
    % Assign odds to used variable
    map.mapped = map.logodds;
    
    % Perform Threshold checking
    map.mapped(map.mapped > 0) = 1;
    map.mapped(map.mapped <= 0) = 0;
end