function U = controlMPC(t1,x1,u0,U,param,map)

m = size(u0,1);

switch param.scenario
    case 'path'
        N = 10;                  % Control horizon for following path
    case 'goal'
        N = 10;                  % Control horizon for no path
    otherwise
        error('Unknown scenario');
end

param.nHorizon = N; % Control horizon

if isempty(U)
    U = [u0;zeros(m*(param.nHorizon-1),1)];
else
    % warm start
    U = [U(m+1:end);zeros(m,1)];
end

switch param.scenario
    % Compute control for path following
    case 'path'
        options = optimoptions('fmincon','Display','none','Algorithm','sqp',...
            'MaxFunctionEvaluations',1e5,...
            'MaxIterations',1000);
        err     = @(U) errorMPC(t1,x1,u0,U,param);
        Aineq   = [];
        bineq   = [];
        Aeq     = [];
        beq     = [];
        lb      = repmat([-2000;-1;-2000;-1],N,1);
        ub      = repmat([3000; 1;3000; 1],N,1);
        nonlcon = @(U) nonlconSLEW(t1,x1,u0,U,param);     
        U       = fmincon(err,U,Aineq,bineq,Aeq,beq,lb,ub,nonlcon,options);
    % Compute control for end goal   
    case 'goal'
        options = optimoptions('fmincon','Display','none','Algorithm','sqp',...
            'MaxFunctionEvaluations',1e5,...
            'MaxIterations',1000);
        cost = @(U) costMPC(t1,x1,u0,U,param);
        Aineq   = [];
        bineq   = [];
        Aeq     = [];
        beq     = [];
        lb      = repmat([-2000;-1;-2000;-1],N,1);
        ub      = repmat([3000; 1;3000; 1],N,1);
        nonlcon = @(U) nonlconMPC(t1,x1,u0,U,param,map);
        U       = fmincon(cost,U,Aineq,bineq,Aeq,beq,lb,ub,nonlcon,options);
    otherwise
        error('Unknown scenario');
end


