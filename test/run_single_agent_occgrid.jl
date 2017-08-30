using HierarchicalDriving
using POMDPToolbox
using DiscreteValueIteration

using JLD

println("Testing SingleAgentOccGridMDP")

n_ld_blocks = 1
n_adj_blocks = 1
cellLength = 75.0

for n_agents in 40:20:160
  println("NumAgents = $n_agents")
  p = SingleAgentOccGridMDP(n_ld_blocks, n_adj_blocks, n_agents, road_segment, cellLength, 100.0, "../scratch/TranProb_$(convert(Int64, cellLength))_$(n_ld_blocks)_$(n_adj_blocks).jld", "../scratch/ActionRwd_$(convert(Int64, cellLength))_$(n_ld_blocks)_$(n_adj_blocks).jld")
  normalize_egoTranProb(p)

  sim = RolloutSimulator()

  println(p.num_ld_nbr_blocks, " ", p.num_adj_nbr_blocks, " ", p.n_agents, " ", p.cellLength, " ", p.n_v_cells, " ", p.goal_reward, " ", p.discount)
  println(p.goal)
  println(n_states(p))

  #sts = states(p)
  #i = 1
  #for st in sts
  #  println(i, " ", st, " ", state_index(p,st))
  #  i += 1
  #end

  N = 10
  #rsum = 0.0
  #for i in 1:N
  #    rsum += simulate(sim, p, RandomPolicy(p))
  #end
  #println("random: $(rsum/N)")

  solver = ValueIterationSolver()
  policy = solve(solver, p, verbose=true)
  println(typeof(policy))
  #println("Policy:")
  #println(policy)
  println("Simulating: ")
  rsum = 0.0
  for i in 1:N
    println("Simulation# ", i)
    rsum += simulate(sim, p, policy)
  end
  println("value iteration: $(rsum/N)")

  save("../scratch/SingleAgentOccGridPolicy_$(n_agents)_$(convert(Int64, cellLength))_$(n_ld_blocks)_$(n_adj_blocks).jld", "policy", policy)
end
