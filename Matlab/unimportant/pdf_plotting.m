clear;
clc;
close all;

mu = 0;
sigma = 3;
x = [-10:0.1:10];

y = normpdf(x,mu,sigma);

plot(x,y,'k');
hold on;
plot([mu,mu],[0,max(y)],'b--');
plot([mu,mu]+sigma,[0,y(find(x == mu+sigma))],'b--');
axis([-10 10 0 0.15]);
annotation('doublearrow',([0.35,mu+sigma-0.325]+10)/20,[y(find(x == mu+sigma)),y(find(x == mu+sigma))]/0.14);
text(mu+sigma/2,y(find(x == mu+sigma))+0.003,'\sigma');
txt = text(-1.1,0.1375,'$x_t = \mu$','FontSize',11,'Interpreter','latex');
title('Sample Normal Distribution','FontSize',16,'Interpreter','latex');
xlabel('$x_t$','FontSize',14,'Interpreter','latex');
ylabel('$p(x_t)$','FontSize',14,'Interpreter','latex');