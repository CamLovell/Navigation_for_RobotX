clear;
clc;
close all;
n = 40;
map = zeros(n,n);

map(:,1) = 1;
map(:,end) = 1;
map(1,:) = 1;
map(end,:) = 1;

map(28:31,1) = 0;

map([1:21],[5:6]) = 1;
map([20:end],[20]) = 1;
map([10:15],[25:30]) = 1;
map([30:35],[28:35]) = 1;


pcolor([0:n],[0:n],[[1-map,ones(n,1)];[ones(1,n+1)]]);
colormap('gray(2)');
title('Example 2D Grid Map','FontSize',16,'Interpreter','latex');
xlabel('$x$ Direction','FontSize',14,'Interpreter','latex');
ylabel('$y$ Direction','FontSize',14,'Interpreter','latex');