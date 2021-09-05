function R = Rzyx(psi,theta,phi)
    RZ = [cosd(psi),-sind(psi),0;sind(psi),cosd(psi),0;0,0,1];
    RY = [cosd(theta),0,sind(theta);0,1,0;-sind(theta),0,cosd(theta)];
    RX = [1,0,0;0,cosd(phi),-sind(phi);0,sind(phi),cosd(phi)];
    
    R = RZ*RY*RX;
end