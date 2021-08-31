function g = localise(z,mod,o)

g = mod;
M = o.M;
N = length(z.y);

%Create some room for filtered particles
g.xf = zeros(g.nx,M,N);

y  = z.y;
u  = z.u;

m  = g.m;
l  = g.l;
J  = g.theta;
a  = g.a;
bu = g.bu;
br = g.br;

%Initial particles
x = g.x0 + sqrtm(g.P0)*randn(g.nx,M);

%Run the main loop
for t=1:N
    %Calculate the log-weights
    w       = likelihood(y(:,t),x,g.map,g.laser);

    %Plot the predicted particles as blue dots
    if t>1
        delete(hfp);
    end 
    hfp = plot(x(2,:),x(1,:),'b.');
    
    t
    
    %Normalise v log-weights
    mw      = max(w);
    w       = w - mw - log(sum(exp(w - mw)));
    
    %Generate indicies according to categorical distribution     
    ii = randsample(length(w),M,true,exp(w));
    xx = x(:,ii);
    u1 = u(1,t) + g.Q(1)*randn(1,M);  %Input noise
    u2 = u(2,t) + g.Q(2)*randn(1,M);  %Input noise
    e1 = xx(4,:).*cos(xx(3,:)); 
    e2 = xx(4,:).*sin(xx(3,:));
    e3 = xx(5,:);
    e4 = (-bu*xx(4,:) -m*l*xx(5,:).^2 + u1 + u2)/m; 
    e5 = ((m*l*xx(4,:)-br).*xx(5,:) - a*u1 + a*u2)/(J+m*l^2);        
    x  = xx + g.D*[e1;e2;e3;e4;e5] + 0.1*randn(g.nx,M); %extra noise to compensate for Euler model
    
    %Store the filtered particles
    g.xf(:,:,t) = xx;
    
    %Plot the filtered particles as red circles    
    if t>1
        delete(hf);
    end 
    hf  = plot(xx(2,:),xx(1,:),'ro','linewidth',3); 
    pause(0.00001);
end

