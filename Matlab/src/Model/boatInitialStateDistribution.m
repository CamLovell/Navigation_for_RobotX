function [lw,xp] = boatInitialStateDistribution()

M 	= 300;                        % number of particles

relw = ones(1,M);
lw	= log(relw/sum(relw));          % log weights (one value per particle)
% xp  = [[2+rand(1,M/3)*6,12+rand(1,M/3)*6,22+rand(1,M/3)*6];1+rand(1,M)*14;pi/2+(pi/16)*randn(1,M);zeros(1,M);zeros(1,M);zeros(1,M)]; 	% samples (one column per particle)
xp  = [3+rand(1,M)*4;1+rand(1,M)*4;pi/2+deg2rad(0.01)*randn(1,M);zeros(1,M);zeros(1,M);zeros(1,M)]; 	% samples (one column per particle)