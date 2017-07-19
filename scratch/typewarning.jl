using HierarchicalDriving
using POMDPToolbox
using POMDPs
using MCTS
using DataFrames

p = LowLevelMDP()
rng = MersenneTwister(3)
s = initial_state(p,rng)

# @code_warntype generate_s(p,s,1,rng)
function g(p, s, i, rng)
  sp = generate_s(p, s, i, rng)
  return sp.neighborhood[1][1].model
end

@code_warntype g(p, s, 1, rng)
