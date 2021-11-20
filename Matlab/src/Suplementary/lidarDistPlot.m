clear;
clc;
close all;
%% This script simply lots the likelihood model used for the Quanergy M8
y = -10:0.1:160;
y_m = 75;

for i = 1:numel(y)
    if y_m > 10 && y(i) <= 150
        p_hit = (1/sqrt(2*pi*5^2))*exp(-0.5*((y(i)-y_m)^2)/5^2);
    else
        p_hit = 0;
    end

    if y(i) <= y_m && y(i) > 10
        p_short = 0.025*exp(-0.025*y(i))/(1-0.025*exp(-0.025*y_m));
    else
        p_short = 0;
    end

    if y(i)>10 && y(i)<=150
        p_rand = 1/150;
    else
        p_rand = 0;
    end
    p(i) = p_hit + p_short + p_rand;
end

plot(y,p,'-k');
title("Measurment Likelihood for Quanergy M8 LiDAR");
xlabel("Range");
ylabel("Likelihood");
xticks([10,75,150]);
xticklabels({'y_{min}','y^*','y_{max}'});
yticks([]);
axis([0,160,0,0.1]);