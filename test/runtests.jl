using HierarchicalDriving
using Base.Test

using MCVI
using Base.Test
import MCVI: init_lower_action, lower_bound, upper_bound
using POMDPs
using GenerativeModels

using POMDPModels # for LightDark1d

# write your own tests here
#@test 1 == 2
include("run_random.jl")

#include("runMCVI.jl")

#@test test_solve_right()
