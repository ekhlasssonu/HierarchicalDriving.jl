using HierarchicalDriving
using POMDPToolbox
using DiscreteValueIteration

using JLD

println("Testing SingleAgentOccGridMDP")

n_ld_blocks = 1
n_adj_blocks = 1
cell_length = 75.0
nbr_cell_length = 25.0

for n_agents in 40:20:160
  println("NumAgents = $n_agents")
  p = SingleAgentOccGridMDP(n_agents, road_segment, cell_length, nbr_cell_length, n_ld_blocks, n_adj_blocks, 100.0, "../scratch/SingleAgentOccGrid/TranProb_$(convert(Int64, cell_length))_$(convert(Int64,nbr_cell_length))_$(n_ld_blocks)_$(n_adj_blocks).jld", "../scratch/SingleAgentOccGrid/ActionRwd_$(convert(Int64, cell_length))_$(convert(Int64,nbr_cell_length))_$(n_ld_blocks)_$(n_adj_blocks).jld")
  normalize_egoTranProb(p)

  sim = RolloutSimulator()

  println(p.num_ld_nbr_blocks, " ", p.num_adj_nbr_blocks, " ", p.n_agents, " ", p.cell_length, " ", p.n_v_cells, " ", p.goal_reward, " ", p.discount)
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
  pol = policy.policy
  act_map = ["cr", "kl", "cl"]
  #println(typeof(policy))
  println("Policy:")
  for dist in p.n_v_cells:-1:1
    for ln in n_lanes(p):-1:1
      for ld_dist in 1:p.num_ld_nbr_blocks + 1
        for occLt in 1:2^p.num_adj_nbr_blocks
          for occRt in 1:2^p.num_adj_nbr_blocks
            st_idx = occRt + 2^p.num_adj_nbr_blocks * ((occLt-1) + 2^p.num_adj_nbr_blocks * ((ld_dist - 1) + (p.num_ld_nbr_blocks+1) * ((dist - 1) + p.n_v_cells * (ln - 1))))
            print(act_map[pol[st_idx]]," ")
          end
        end
      end
      print("\t")
    end
    println()
  end

  #println(pol)
  println("Simulating: ")
  rsum = 0.0
  for i in 1:N
    println("Simulation# ", i)
    rsum += simulate(sim, p, policy)
  end
  println("value iteration: $(rsum/N)")

  save("../scratch/SingleAgentOccGrid/SingleAgentOccGridPolicy_$(n_agents)_$(convert(Int64, cell_length))_$(convert(Int64,nbr_cell_length))_$(n_ld_blocks)_$(n_adj_blocks).jld", "policy", policy)
end
