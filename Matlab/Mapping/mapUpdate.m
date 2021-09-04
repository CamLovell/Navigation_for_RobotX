function map = mapUpdate(y,x,map,param)

    % Extract states
    N = x(1);
    E = x(2);
    psi = x(3);
    
    % Extract grid lines from map data
    N_line = map.y;
    E_line = map.x;
    
    % Extract Grid centres from map data (grid centres are used as they correspond more accurately to the map grid
    N_cent = N_line(1:end-1)+map.dy;
    E_cent = E_line(1:end-1)+map.dx;
    
    % logodds is the current log odds of each grid square
    map.logodds;
    
    % max range is tha max lidar range
    param.maxRange;
    
    % theta is the vector of angles at which scanlines are released
    theta = param.theta;
%     theta = param.theta;
    % ds is lidar sampling resolution (check lidar init from localisation and/or data sheet
    param.res;
    
    % "H" depth of hit zone (tuneable paramater, less than thinnest wall,greater than grid size)
    param.hitDepth;
    
    % ehit and epass refer to change in log likelihoods given hit or pass of a given grid space                               
    param.hitChange;
    param.passChange;
    
    % "hypot" function is used to avoid underflow and overflow in the sqrt sum of squares calculation
    maxDist = hypot(N_line(end)-N_line(1),E_line(end)-E_line(1));
    % s gives an array of distances for which a hit/pass measurment is effectively taken, based on the Lidar resolution
    measInt = 0:param.res:maxDist;
    
    % Add s to the N and E positions of the state (assuming known), adjusted by the scan line angles from theta
    N_meas = N + measInt.*cos(theta(:));
    E_meas = E + measInt.*sin(theta(:));
    % This gives all the points along a scan line at the lidar resolution, at each of these points a "measurment" is taken
    % Find which of these points are in the map, between first and last gridlines for the given state direction
    inMap = N_meas < N_line(end) & N_meas > N_line(1) & E_meas < E_line(end) & E_meas > E_line(1);
    
    % Find indicies of map at which each of the N and E "measurment" points fall on the map
    Nidx = round(((N_meas-N_line(1))/(N_line(end)-N_line(1)))*(length(N_line)-1)+1);
    Eidx = round(((E_meas-E_line(1))/(E_line(end)-E_line(1)))*(length(E_line)-1)+1);
    
    isUseable = reshape(find(y < param.maxRange),1,[]); % Returns indicies for which the measurments are within the speced range
    % Loop through non-maxRange measurements 
    for i = isUseable
        % nested loop through length of measurment array
        
        for j = 1:length(measInt)
            % Nested loops give the indicies of the grid spaces in matrix
            if inMap(i,j)
                % If measurment array distance is less than current measuremnt take away pass change
                if Nidx(i,j) == 0 || Eidx(i,j) == 0 && measInt(j) < y(i)
                Nidx(i,j);
                Eidx(i,j);
                end
                if y(i)==0
                    map.logodds(Nidx(i,j),Eidx(i,j)) = map.logodds(Nidx(i,j),Eidx(i,j)) - param.passChange/5;
                elseif measInt(j) < y(i)
                    Nidx(i,j)*map.dy;
                    Eidx(i,j)*map.dx;
                    map.logodds(Nidx(i,j),Eidx(i,j)) = map.logodds(Nidx(i,j),Eidx(i,j)) - param.passChange;
                
                    % else if less than measuremnt plus depth of hit zone add hit change
                elseif measInt(j) < y(i) + param.hitDepth
                    map.logodds(Nidx(i,j),Eidx(i,j)) = map.logodds(Nidx(i,j),Eidx(i,j)) + param.hitChange;
                % else (greater than measurment with hit ray added) dont change anything as no info is given
                else
                    break;
                end
            else
                break;
            end 
        end
    end
    
    % Satruate log odds to desired max or min (ensures they do not become irreversibly small or large from repeated bad measurments
    map.logodds(map.logodds > 50) = 50;
    map.logodds(map.logodds < -50) = -50;
    
    map.mapped = map.logodds;
    
    map.mapped(map.mapped > 0) = 1;
    map.mapped(map.mapped <= 0) = 0;
    
    % Look at edge detection stuff??
           
end