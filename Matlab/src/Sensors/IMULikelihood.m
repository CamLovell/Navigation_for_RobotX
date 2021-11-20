function w = IMULikelihood(y,x,params,M)
 % Extract relevant states
 psi_part = x(3,:);
 
 % Initialise Weights
 w = zeros(M,1);

 for k = 1:M   
    % Calculate particle weight using psi and measurement
    P = (1./sqrt(2.*pi.*params.sigmaHead.^2)).*exp(-0.5.*((y-psi_part(k)).^2)./params.sigmaHead.^2);
    
    % Add calculated weights to particle weight
    w(k) = reallog(P);
 end
end
