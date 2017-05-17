using HierarchicalDriving
using POMDPToolbox
using POMDPs
using MCTS
using DataFrames

p = LowLevelMDP()
rng = MersenneTwister(3)
s = initial_state(p,rng)

@code_warntype generate_sr(p,s,1,rng)
