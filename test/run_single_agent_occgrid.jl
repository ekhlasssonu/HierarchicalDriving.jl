using HierarchicalDriving
using POMDPToolbox
using DiscreteValueIteration

using JLD

println("Testing SingleAgentOccGridMDP")

p = SingleAgentOccGridMDP(50, road_segment, 24, 100.0, "../scratch/TranProb_Heur.jld", "../scratch/ActionRwd_Heur.jld")
normalize_egoTranProb(p)
#println("egoTranProb = \n",p.egoTranProb)
#println("egor_s_a = \n",p.ego_r_s_a)

#=states = states(p)
println("Num States = ", n_states(p))
rng = MersenneTwister(779)
st = initial_state(p, rng)
println(state_index(p,st),"\t",st)
println(states[state_index(p,st)])

d = SAOccGridDist(p, st, 1)
next_states = iterator(d)

sum = 0.0
for sp in next_states
  #println("next state: ",state_index(p,sp)," - ",sp)
  sum += pdf(d, sp)
end
println("Sum = ", sum)

println("sum = ", checkProb(p))=#

#=for i in 1:10
  sp = rand(rng, d)
  println("sp = ", sp)

  prob = pdf(d, sp)
  println("tran prob = ", prob)
end=#
sim = RolloutSimulator()

N = 100
rsum = 0.0
for i in 1:N
    rsum += simulate(sim, p, RandomPolicy(p))
end
println("random: $(rsum/N)")

solver = ValueIterationSolver()
policy = solve(solver, p, verbose=true)
println(typeof(policy))
println("Policy:")
println(policy)
println("Simulating: ")
rsum = 0.0
for i in 1:N
  println("Simulation# ", i)
    rsum += simulate(sim, p, policy)
end
println("value iteration: $(rsum/N)")

save("../scratch/SingleAgentOccGridPolicy.jld", "policy", policy)
