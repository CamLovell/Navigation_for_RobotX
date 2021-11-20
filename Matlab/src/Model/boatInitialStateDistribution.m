function [lw,xp] = boatInitialStateDistribution(M)
    % Ensure number of Particles is divisible by 3
    assert(mod(M,3)==0);

    % Create particles with initially even weights
    relw = ones(1,M);

    % Calculate log weights for return
    lw	= log(relw/sum(relw));

    % Create initial mixture distribution
    xp  = [[2+rand(1,M/3)*6,12+rand(1,M/3)*6,22+rand(1,M/3)*6];1+rand(1,M)*14;pi/2+(pi/16)*randn(1,M);zeros(1,M);zeros(1,M);zeros(1,M)];
end