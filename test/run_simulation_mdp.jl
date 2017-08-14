p = SimulationMDP()
#solver = MCTSSolver()
solver = DPWSolver(depth=p.HORIZON,
                               exploration_constant=10.0,
                               n_iterations=1_000,
                               k_action=10.0,
                               alpha_action=1/10,
                               k_state=5.0,
                               alpha_state=1/10,
                               #estimate_value=RolloutEstimator(subintentional_lowlevel_policy(p))
                              )
function runSims()
  println("Flat MDP Test")

  policy = solve(solver, p)
  for i in 1:10
    print("\r$i")
    hr = HistoryRecorder(max_steps = 20)
    simulate(hr, p, policy)
  end
  println()
end

#@time runSims()

rng = MersenneTwister(978)
s = initial_state(p, rng)

println("State initialized")
printGlobalPhyState(s, p)

println("Neighbors:")
printNeighborCache(s)

sp = generate_s(p, s, 1, rng)

println("Neighbors:")
printNeighborCache(sp)

println("State Updated")
printGlobalPhyState(sp, p)
