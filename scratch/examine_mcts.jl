using HierarchicalDriving
using POMDPToolbox
using POMDPs
using MCTS
using DataFrames
using JLD
using Reel
using ProgressMeter

@everywhere using POMDPToolbox

mdp = LowLevelMDP(0.9,
                  [0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH],
                  CarPhysicalState((0.0, 1.0 * LANE_WIDTH/2.0, AVG_HWY_VELOCITY)),
                  (CarPhysicalState((0.0, 3.0 * LANE_WIDTH/2.0 - 0.5, AVG_HWY_VELOCITY - 0.5)),
                   CarPhysicalState((150.0, 3.0 * LANE_WIDTH/2.0 + 0.5, AVG_HWY_VELOCITY + 0.5))),
                  5.0, -50.0, 0.0, -3.0, -2.0, -0.5, HierarchicalDriving.getFrameList())

heur = subintentional_policy(mdp)

policies = Dict{String, Policy}(
    "random" => RandomPolicy(mdp),
    "heuristic" => heur,
    "mcts" => begin
        solver = DPWSolver(depth=20,
                           exploration_constant=10.0,
                           n_iterations=10_000,
                           k_action=5.0,
                           alpha_action=1/10,
                           k_state=5.0,
                           alpha_state=1/10,
                          )
        solve(solver, mdp)
    end
)

problems = Dict{String, Any}(
    "default" => mdp
)

seed = 27
policy = policies["heuristic"]
sim = HistoryRecorder(max_steps=40,
                      show_progress=true,
                      rng=MersenneTwister(seed))
HierarchicalDriving.setrng!(policy, MersenneTwister(seed+10000))

hist = simulate(sim, mdp, policy)
@show discounted_reward(hist)

frames = Frames(MIME("image/png"), fps=5)
@showprogress "Creating gif..." for s in state_hist(hist)
    push!(frames, (mdp, s))
end

filename = string(tempname(), "_hd_run.gif")
write(filename, frames)
println(filename)
run(`setsid gifview $filename`)


#=
sim = PmapSimulator(HistoryRecorder(max_steps=40)) do r::SimResult
    hist = r[:history]
    return (:reward => discounted_reward(hist),
            :n_steps => n_steps(hist))
end # do block defines what to do with the simulation result. Should be a collection of pairs

s = SimSet(problems=problems,
           policies=policies,
           simulator=sim,
           n=1000
          )
# basically just a dict holding options
           
sets = [
    SimSet(s, "heuristic", "default"),
    SimSet(s, "random", "default"),
    SimSet(s, "mcts", "default")
]

df = run(sets)

filename = joinpath("data", "compare_$(Dates.format(now(), "E_d_u_HH_MM")).jld")
println("saving to $filename")
@save(filename, df)

using Plots
# unicodeplots()
gr()
histogram(df[:reward])
gui()

means = by(df, :policy_key) do df
    DataFrame(reward=mean(df[:reward]), n_steps=mean(df[:n_steps]))
end
=#
