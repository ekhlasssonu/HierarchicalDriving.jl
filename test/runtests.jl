using HierarchicalDriving
using Base.Test

using Base.Test
using POMDPs

using MCTS
# using MCVI
# import MCVI: init_lower_action, lower_bound, upper_bound

using POMDPModels # for LightDark1d


#include("nb_cache.jl")

#@time include("run_gen_ul_tran.jl")

include("run_single_agent_occgrid.jl")

# write your own tests here
#@test 1 == 2
#include("run_random.jl")

#include("run_single_agent_grid.jl")

#include("run_mcts.jl")

#@test test_solve_right()
