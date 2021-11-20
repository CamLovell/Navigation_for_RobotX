function w = GPSLikelihood(y,x,params,M)
 % Extract state components
 N_part = x(1,:);
 E_part = x(2,:);
 D_part = params.h0*ones(size(E_part)); % assumed height of NE plane
 
 % Initialise Weights
 w = zeros(M,1);
 
 % Calculate expected measurements according to ECEF coordinate transform
 expected = Rzyx(151.219855,-(90-33.857481),0)*[N_part;E_part;D_part] + [-4647135;2552687;3533330];
 for k = 1:M   
    % Calculate particle weight using N and E components
    P = (1./sqrt(2.*pi.*params.GPSsigma_RTK.^2)).*exp(-0.5.*((y(1:2)-expected(1:2,k)).^2)./params.GPSsigma_RTK.^2);
    
    % Add calculated weights to particle weight
    w(k) = sum(reallog(P));
 end
end