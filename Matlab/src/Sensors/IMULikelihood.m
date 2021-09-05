function w = IMULikelihood(y,x,params,M)
 psi_part = x(3,:);
 w = zeros(M,1);

 for k = 1:M   
     
    P = (1./sqrt(2.*pi.*params.sigmaHead.^2)).*exp(-0.5.*((y-psi_part(k)).^2)./params.sigmaHead.^2);
    
    w(k) = reallog(P);
 end
end
