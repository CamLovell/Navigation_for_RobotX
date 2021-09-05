clear;
clc;
close all;

hf      = figure(1);
clf(1);
hf.Color = 'w';
ax      = axes(hf,'FontSize',14);
hold(ax,'on');



circle = [cos([0:0.01:2*pi]);sin([0:0.01:2*pi]);zeros(size(0:0.01:2*pi))];
line = expm(deg2rad(45)*skew([0;0;1]))*[[0,1];[0,0];zeros(1,2)];

x = [1,2];
y = [1,2];

hPart(1)   = plot(ax,nan,nan,'b');
hPart(2)  = plot(ax,nan,nan,'r');
% hPart2  = plot(ax,nan,nan,'b');

hPart(1).XData = circle(1,:)
hPart(1).YData = circle(2,:)
hPart(2).XData = line(1,:)
hPart(2).YData = line(2,:)
% hPart2(2).XData = [2,3];
% hPart2(2).YData = y;