function [muz,Pz] = UnscentedTransform(h,mu,P)
%First generate the sigma points
n = size(P,1);
sP = chol(P).';
p = sqrt(n)*sP;
s = mu + p;
s = [s mu-p];
%Propagate through the nonlinearity
for i=1:2*n
    z(:,i) = h(s(:,i));
end
%Generate the sample mean and covariance
muz = sum(z,2)/(2*n);
Pz = (z - muz)*(z - muz).'/(2*n);

