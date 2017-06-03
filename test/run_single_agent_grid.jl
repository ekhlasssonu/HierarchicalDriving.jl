using HierarchicalDriving
using POMDPToolbox
using DiscreteValueIteration

println("Testing SingleAgentGridMDP")

p = SingleAgentGridMDP()

sim = RolloutSimulator()

N = 100
rsum = 0.0
for i in 1:N
    rsum += simulate(sim, p, RandomPolicy(p))
end
println("random: $(rsum/N)")

solver = ValueIterationSolver()
policy = solve(solver, p, verbose=true)
rsum = 0.0
for i in 1:N
    rsum += simulate(sim, p, policy)
end
println("value iteration: $(rsum/N)")
