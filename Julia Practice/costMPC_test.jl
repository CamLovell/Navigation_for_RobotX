include("robotDynamics.jl")
include("costMPC.jl")

# function test(z,x,paramater)
#     println(x[3])
#     println(z)
#     println(paramater.l)
# end

someArray = [1,2,3,4]
someVar = 52
struct whytho
    a
    b
    dt
    nHorizon
    c
    l
    mominv12
    mominv13
    mominv22
    mominv23
    mominv32
    mominv33
    betaT
    M3inv
    RRterm
    Mrinvij
    Mrinvji
    s
end

M3inv = [0.333333333333333 0 0;0 0.600000000000000 -2.666666666666667;0 -2.666666666666667 26.666666666666668]

params = whytho(25,3.75,0.1,60,0.1,0.2,0.5,-0.5,-2.222222,-2.222222,0.0556,0.0556,0.8904,M3inv,0.0575,3.787,3.6204,0.05)

# test(1.255332,[1.75,1,1.42454,1,1],params)
# display(robotDynamics(1,[1,2,3,4,5],[6,7],params))
u0 = [0.1;0.1]
U = [u0;zeros(2*(params.nHorizon-1),1)]
tot = 0.0
for i = 1:(5*10^2)
    global tot += costMPC(1,[(1/i),1,1,1,1],[1,1],U,params)
end
display(tot)
# costMPC(1,[(1/i),1,1,1,1],[1,1],U,params)
