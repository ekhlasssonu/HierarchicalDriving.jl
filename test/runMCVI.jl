using POMDPs

using POMDPToolbox

importall MCVI

function test_solve_right()
    prob = ChangeLaneRightPOMDP()
    sim = MCVISimulator()

    solver = MCVISolver(sim, nothing, 1, 100, 8, 500, 1000, 5000, 50, ChangeLaneRightLowerBound(sim.rng), ChangeLaneRightUpperBound(sim.rng))
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
