function dx = boatDynamicsFast_take2(t, x, u, param)
% Precompute terms
magnu = abs(x(4:6));
c1 = cos(u(2));
c2 = cos(u(4));
s1 = sin(u(2));
s2 = sin(u(4));
a = u(1)*s1+u(3)*s2-((696*magnu(2)+8361*magnu(3)+884)*x(5)+(-167.61*x(4)+814*magnu(3)+307*magnu(2)-202)*x(6));
b = u(1)*(-param.rLCb(2)*c1+param.rLCb(1)*s1)+u(3)*(-param.rRCb(2)*c2+param.rRCb(1)*s2)-((-475*x(5)+1736*x(6))*x(4)+(-1516*magnu(2)-425*magnu(3)+552)*x(5)+(-2884*magnu(3)-202*magnu(2)+1258)*x(6));

% Calcualte state derivative
dx = [x(4)*cos(x(3)) - x(5)*sin(x(3)) ;...
    x(5)*cos(x(3)) + x(4)*sin(x(3)) ;...
    x(6);...
    0.001156885202281*(u(1)*c1+u(3)*c2-((85*magnu(1)+120)*x(4)+(642.61*x(5)-1736*x(6))*x(6)));...
    0.000694851544311*a+0.000240704132172*b;...
    -0.000039932482757*a+0.000185712389164*b];

end

