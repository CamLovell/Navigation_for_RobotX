function [value,isterminal,direction] = checkCollision(t,x,param,map)

value = 1;
isterminal = 1;
direction = 0;
% Extract States
N   = x(1,:);
E   = x(2,:);
psi = x(3,:);

rCNn = [N;E;0];
Rnb = expm(psi*skew([0;0;1]));

persistent rPCb_R_all
persistent rPCb_L_all
% Define the outline of the boat
if isempty(rPCb_R_all) && isempty(rPCb_L_all)
    L = 2*param.l;
    W = 0.2;
    space = param.d/2-W/2;
    
    rPCb_R_all = [[0;space;0], [L/2;space;0], [L/2;space+W;0], [-L/2;space+W;0], [-L/2;space;0], [0;space;0]];
    rPCb_L_all = -[[0;space;0], [L/2;space;0], [L/2;space+W;0], [-L/2;space+W;0], [-L/2;space;0], [0;space;0]];
end
% Shift outline to the N coordinate frame
rPNn_all = [Rnb*rPCb_R_all + rCNn,Rnb*rPCb_L_all + rCNn];

% Check if any of the locations along the boat outline are on an ocupied grid space
for i = 1:size(rPNn_all,2)
    rONn = rPNn_all(:,i);
    Nidx = find(map.y <= rONn(1), 1, 'last');
    Eidx = find(map.x <= rONn(2), 1, 'last');
    if isempty(Nidx) || isempty(Eidx)
        continue
    end
    if map.expFull(Nidx,Eidx)
        value = 0;
        return
    end
end