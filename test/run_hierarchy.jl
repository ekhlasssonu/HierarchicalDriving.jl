p = SimulationMDP()

#Type 1
#=n_v_cells1 = 8
u_goalReward1 = 50.0
u_p_next_cell = 0.9
u_p_desired_lane = 0.9

policyFileName = "../scratch/SingleAgentGridPolicy.jld"
#hf1 = HierarchicalFramework1(p,n_v_cells1, u_goalReward1, u_p_next_cell, u_p_desired_lane)
hf1 = HierarchicalFramework1(p, policyFileName)
#println(hf1)
rng = MersenneTwister(223)
hp1 = HierarchicalPolicy1(hf1)
#println(hp1)
s = initial_state(p, rng)
#printGlobalPhyState(s, p)
#printNeighborCache(s)
println()
a = action(hp1, s)
println("action = ",a)=#

#Type2
n_v_cells2 = 24
u_goalReward2 = 100.0
tranFileName = "../scratch/TranProb_Heur.jld"
rwdFileName = "../scratch/ActionRwd_Heur.jld"
policyFileName = "../scratch/SingleAgentOccGridPolicy.jld"
#hf2 = HierarchicalFramework2(p, n_v_cells2, u_goalReward2, tranFileName, rwdFileName, policyFileName)
hf2 = HierarchicalFramework2(p, policyFileName)
#println(hf2)
rng = MersenneTwister(223)
hp2 = HierarchicalPolicy2(hf2)
#println(hp2)
s = initial_state(p, rng)
printGlobalPhyState(s, p)
#printNeighborCache(s)
println()
a = action(hp2, s)
println("action = ",a)
