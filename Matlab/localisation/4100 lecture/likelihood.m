function [w] = likelihood(y,x,map,laser)

M = size(x,2);
w = zeros(M,1);

for i=1:M

    %Set the current pose
    pose.north = x(1,i);
    pose.east  = x(2,i);
    pose.psi   = 180*x(3,i)/pi;
    
    %Compute predicted scan ranges
    [rr] = scanNE(map,laser,pose);

    %Now compute the likelihood for each of the models

    %Hit
    c1 = 1/sqrt(2*pi*laser.sig^2);
    t1 = (y(:) - rr(:))/laser.sig;
    p1 = c1*exp(-0.5*t1.*t1);

    %Short
    c2 = 1./(1 - exp(-laser.lambda*rr(:)));
    p2 = (y(:) <= rr(:)).*c2.*laser.lambda.*exp(-laser.lambda*y(:));

    %Max
    c3 = 1;
    p3 = c3.*(abs(y(:) - laser.maxRange) <= 0.01 );

    %Rand
    c4 = 1/laser.maxRange;
    p4 = c4.*ones(length(y),1);
    
    %Now compute the likelihood
    lh = laser.w(1)*p1 + laser.w(2)*p2 + laser.w(3)*p3 + laser.w(4)*p4;

    w(i) = sum(log(lh));
end