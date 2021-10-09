function w = GPSLikelihood(y,x,params,M)
 N_part = x(1,:);
 E_part = x(2,:);
 D_part = params.h0*ones(size(E_part));
 w = zeros(M,1);
 
 expected = Rzyx(151.219855,-(90-33.857481),0)*[N_part;E_part;D_part] + [-4647135;2552687;3533330];
 for k = 1:M   
     
    P = (1./sqrt(2.*pi.*params.GPSsigma_RTK.^2)).*exp(-0.5.*((y(1:2)-expected(1:2,k)).^2)./params.GPSsigma_RTK.^2);
%     P_East = (1/sqrt(2*pi*params.GPSsigma^2))*exp(-0.5*((E_measured-E_part(k))^2)/params.GPSsigma^2);
    
%     temp_w = [reallog(P_North),reallog(P_East)];
    w(k) = logSumExponential(reallog(P));
 end
end