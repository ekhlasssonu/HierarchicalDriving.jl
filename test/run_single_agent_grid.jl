using HierarchicalDriving
using POMDPToolbox
using DiscreteValueIteration

using JLD

println("Testing SingleAgentGridMDP")

p = SingleAgentGridMDP(75.0)

sim = RolloutSimulator()

N = 100
rsum = 0.0
for i in 1:N
    rsum += simulate(sim, p, RandomPolicy(p))
end
println("random: $(rsum/N)")

solver = ValueIterationSolver()
policy = solve(solver, p, verbose=false)
#println(typeof(policy))
println("Policy:")
println(policy)
#println("Simulating: ")
rsum = 0.0
for i in 1:N
  #println("Simulation# ", i)
    rsum += simulate(sim, p, policy)
end
println("value iteration: $(rsum/N)")

save("../scratch/SingleAgentGrid/SingleAgentGridPolicy.jld", "policy", policy)

#println(states(p))

#policy2 = load("../scratch/SingleAgentGridPolicy.jld", "policy")
#println(states(policy2.mdp))
println("run_single_agent_grid.jl executed")
