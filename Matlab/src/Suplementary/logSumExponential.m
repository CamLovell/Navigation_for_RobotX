function w = logSumExponential(logWeights)
    % Calculate log sum exponential of given input log weights
    W_max = max(logWeights);
    w = W_max + reallog(sum(exp(logWeights-W_max)));
    
    % Prevent nan issues carying over
    if(isnan(w))
        w=-inf;
    end
end