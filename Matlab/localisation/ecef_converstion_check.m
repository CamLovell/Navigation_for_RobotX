clear;
clc;
close all;

%Get the WGS-84 ellipsoid model
wgs84 = wgs84Ellipsoid('meters');
%Plot the ellisoid
figure('Renderer','opengl')
ax = axesm('globe','Geoid',wgs84,'Grid','on', ...
'GLineWidth',1,'GLineStyle','-',...
'Gcolor','b','Galtitude',100);
ax.Position = [0 0 1 1];
axis equal off;
view(3);
%Plot the ECEF principle axes
l1=line([0 12000000],[0 0],[0 0]);
l2=line([0 0],[0 12000000],[0 0]);
l3=line([0 0],[0 0],[0 12000000]);
set(l1,'color','k','linewidth',2);
set(l2,'color','k','linewidth',2);
set(l3,'color','k','linewidth',2);

%Continued from before.....
load topo
geoshow(topo,topolegend,'DisplayType','texturemap')
demcmap(topo)
land = shaperead('landareas','UseGeoCoords',true);
plotm([land.Lat],[land.Lon],'Color','black')
rivers = shaperead('worldrivers','UseGeoCoords',true);
plotm([rivers.Lat],[rivers.Lon],'Color','blue')
%ES Building
lat = 45 * pi / 180;
long = 135 * pi / 180;
h = 200;
% WGS-84 parameters
a = 6378137;
e = 0.08181919;
Rn = a / (1 - e^2 * (sin(lat))^2)^0.5; % Normal Radius
%Compute point in Geocentric coordinates
rNOe = [(Rn+h)*cos(lat)*cos(long);
(Rn+h)*cos(lat)*sin(long);
(Rn*(1-e^2) + h)*sin(lat)];
%Draw a line through the point
ll=line([0 2*rNOe(1)],[0 2*rNOe(2)],[0 2*rNOe(3)]);
set(ll,'color','r','linewidth',2);

% Continued from before…
% Generate y and z data
[y z] = meshgrid(-1e6:2e5:1e6);
x = 0*y + a + 200; % set x
%Plot the surface
surf(x,y,z,0.9*ones(size(z)))

%Continued… z
%Draw the NE plane centred at UoN ES Building
[x y] = meshgrid(-1e6:2e5:1e6); % Generate y and z data
z = 0*y;
%rotate to new lat,long
R= Rzyx(135,-90-45,0);
[sx,sy]=size(x);
xyz = [x(:) y(:) z(:)]*R.';
x=rNOe(1) + reshape(xyz(:,1),sx,sy);
y=rNOe(2) + reshape(xyz(:,2),sx,sy);
z=rNOe(3) + reshape(xyz(:,3),sx,sy);
surf(x,y,z,0.9*ones(size(z))) %Plot the surface