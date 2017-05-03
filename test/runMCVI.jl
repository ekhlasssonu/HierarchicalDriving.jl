using POMDPs

using POMDPToolbox

importall MCVI

function test_solve_right()
    prob = ChangeLaneRightPOMDP()
    sim = MCVISimulator()

    #num_iter=1, num_particles=100, obs_branching=8, num_states=500, num_prune_obs=1000, num_eval_belief=5000, num_obs=50
    solver = MCVISolver(sim, nothing, 1, 10, 8, 50, 100, 50, 5, ChangeLaneRightLowerBound(sim.rng), ChangeLaneRightUpperBound(sim.rng))
    println("Solving...")
    policy = solve(solver, prob)
    println("...Solved")
    up = updater(policy)
    reward = simulate(sim, prob, policy, up, up.root)
    println("Reward:", reward)
    return true
end

#=function test_solve_left()
    prob = ChangeLaneLeftPOMDP()
    sim = MCVISimulator()

    solver = MCVISolver(sim, nothing, 1, 100, 8, 500, 1000, 5000, 50, ChangeLaneLeftLowerBound(sim.rng), ChangeLaneLeftUpperBound(sim.rng))
    println("Solving...")
    policy = solve(solver, prob)
    println("...Solved")
    up = updater(policy)
    reward = simulate(sim, prob, policy, up, up.root)
    println("Reward:", reward)
    return true
end

function test_solve_m25()
    prob = MaintainAt25POMDP()
    sim = MCVISimulator()

    solver = MCVISolver(sim, nothing, 1, 100, 8, 500, 1000, 5000, 50, MaintainAt25LowerBound(sim.rng), MaintainAt25UpperBound(sim.rng))
    println("Solving...")
    policy = solve(solver, prob)
    println("...Solved")
    up = updater(policy)
    reward = simulate(sim, prob, policy, up, up.root)
    println("Reward:", reward)
    return true
end

function test_solve_m23()
    prob = MaintainAt23POMDP()
    sim = MCVISimulator()

    solver = MCVISolver(sim, nothing, 1, 100, 8, 500, 1000, 5000, 50, MaintainAt23LowerBound(sim.rng), MaintainAt23UpperBound(sim.rng))
    println("Solving...")
    policy = solve(solver, prob)
    println("...Solved")
    up = updater(policy)
    reward = simulate(sim, prob, policy, up, up.root)
    println("Reward:", reward)
    return true
end

function test_solve_m27()
    prob = MaintainAt27POMDP()
    sim = MCVISimulator()

    solver = MCVISolver(sim, nothing, 1, 100, 8, 500, 1000, 5000, 50, MaintainAt27LowerBound(sim.rng), MaintainAt27UpperBound(sim.rng))
    println("Solving...")
    policy = solve(solver, prob)
    println("...Solved")
    up = updater(policy)
    reward = simulate(sim, prob, policy, up, up.root)
    println("Reward:", reward)
    return true
end
=#
