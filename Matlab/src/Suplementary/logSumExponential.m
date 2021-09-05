function w = logSumExponential(logWeights)
% Takes log weights as inputs and outputs the normalised log weights via the log sum exponential trick
    W_max = max(logWeights);
    w = W_max + reallog(sum(exp(logWeights-W_max)));
end