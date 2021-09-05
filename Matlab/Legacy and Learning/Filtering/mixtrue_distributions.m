 clear;
 clc;
 close all;
 
%Number of samples
L = 10000;
%Weights (sum to 1)
w = [0.4 0.2 0.1 0.3];
%Draw L samples from categorical
i = randsample(length(w),L,true,w);
%Generate Histogram
s = zeros(length(w),1)
for k=1:length(w)
s(k) = sum(i==k)/L;
end
bar(1:length(w),s,1)