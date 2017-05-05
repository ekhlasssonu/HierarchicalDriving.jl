using HierarchicalDriving
using Base.Test

using Base.Test
using POMDPs
using GenerativeModels

using MCTS
# using MCVI
# import MCVI: init_lower_action, lower_bound, upper_bound

using POMDPModels # for LightDark1d

# write your own tests here
#@test 1 == 2
include("run_random.jl")

#include("run_mcts.jl")

#include("runMCVI.jl")

#@test test_solve_right()
