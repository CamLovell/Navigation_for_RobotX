# using LinearAlgebra
include("robotDynamics.jl")
function costMPC(t1,x1,u0,U,param)

    dt = param.dt
    N = param.nHorizon # Control horizon (set in controlMPC.m, not here)

    V = 0

    t = t1
    x = x1
    e = zeros(5*N,1)

    for k = 1:N                 # for each step in the control horizon
        u = U[(2*k-1:2*k)]

        # Do one step of RK4 integration
        f1 = robotDynamics(t,        x,           u, param)
        f2 = robotDynamics((t + dt/2), (x + f1*dt/2), u, param)
        f3 = robotDynamics((t + dt/2), (x + f2*dt/2), u, param)
        f4 = robotDynamics((t + dt),   (x + f3*dt),   u, param)
        x  = x + (f1 + 2*f2 + 2*f3 + f4)*dt/6
        t  = t + dt

        # Add contribution to cost function for kth step
        #     V = V + ru*norm(u)^2;
        #     if t>=5
        #         V = V + qr*norm(rENn-rCNn)^2 + qpsi*(q(3)-psistar)^2;
        #     end
        if t>=5
            q = x[(3:5)]
            qr = 120
            qpsi = 50

            rENn = [0.75;1.7;0]
            psistar = 0
            rCNn = [q[(1)]+param.c*cos(q[(3)]);q[(2)]+param.c*sin(q[(3)]);0]

            epose = [qr*(rENn[(1:2)]-rCNn[(1:2)]);qpsi*(q[(3)]-psistar)]
        else
            epose = [0;0;0]
        end
        ru = 10
        e[(5k-4):5k] = [epose;ru*u]
    end

    V = e'*e
    return V[1]
end
