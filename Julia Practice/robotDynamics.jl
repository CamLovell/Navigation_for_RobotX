function robotDynamics(t, x, u, param)

    p3 = [param.mominv12*x[1]+param.mominv13*x[2];param.mominv22*x[1]+param.mominv23*x[2];param.mominv32*x[1]+param.mominv33*x[2]];

    nu3 = param.M3inv*p3;

    beta = param.betaT/max(sqrt(nu3[1]^2+(nu3[2]-param.s*nu3[3])^2),1e-3);

    c = cos(x[5]);
    s = sin(x[5]);
    l = param.l;

    RRterm = param.RRterm;

    dh1 = (param.Mrinvij*x[1]+param.Mrinvji*x[2]);
    dh2 = (param.Mrinvji*x[1]+param.Mrinvij*x[2]);

    a = param.a;
    b = param.b;

    dx = [(a+b)*u[1]+ (a-b)*u[2]+(-beta*(1+RRterm))*dh1 + (2*p3[2]-beta*(-1+RRterm))*dh2;
        (-a+b)*u[1]+ (-a-b)*u[2]+(-2*p3[2]-beta*(-1+RRterm))*dh1 + (-beta*(1+RRterm))*dh2;
        (c+l*s)*dh1 + (-c+l*s)*dh2;
        (s-l*c)*dh1 + (-s-l*c)*dh2;
        dh1 + dh2]

        return dx
end
