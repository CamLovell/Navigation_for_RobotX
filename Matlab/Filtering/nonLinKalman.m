clear;
clc;
close all;

n = 2;
N = 1000;

sqrtsigma = chol(diag([1e-4,100]));
sigma = sqrtsigma*sqrtsigma.';
test  = sqrtm(sigma)-sqrtsigma;
mean = [1;90];

x = sqrtsigma.'*randn(2,N) + mean;
y = nan(size(x));
y(1,:) = x(1,:).*cosd(x(2,:));
y(2,:) = x(1,:).*sind(x(2,:));
figure
plot(x(1,:),x(2,:),'+');
xlabel('x(1,:)')
ylabel('x(2,:)')
figure
plot(y(1,:),y(2,:),'+');
xlabel('y(1,:)')
ylabel('y(2,:)')