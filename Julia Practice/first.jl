using Printf
using Statistics
using Plots

x = 1:10; y = rand(10,2); # These are the plotting data
display(plot(x, y, title="variables in title? $(y[1,2])", label = ["first" "second"], xlabel="jsut some shit",ylabel="asgasdgt"))
# Time to add axis labels
#fuck this is simple shit
println(y[1,2])
savefig("testfig.pdf")
