using HierarchicalDriving
using POMDPToolbox
using POMDPs
using MCTS
using DataFrames
using JLD

@everywhere using POMDPToolbox

mdp = LowLevelMDP(0.9,
                  [0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH],
                  CarPhysicalState((0.0, 1.0 * LANE_WIDTH/2.0, AVG_HWY_VELOCITY)),
                  (CarPhysicalState((10.0, 3.0 * LANE_WIDTH/2.0 - 0.5, AVG_HWY_VELOCITY - 0.5)),
                   CarPhysicalState((100.0, 3.0 * LANE_WIDTH/2.0 + 0.5, AVG_HWY_VELOCITY + 0.5))),
                  50.0, -500.0, 0.0, -3.0, -2.0, -1.0, HierarchicalDriving.getFrameList())

heur = subintentional_policy(mdp)

policies = Dict{String, Policy}(
    "random" => RandomPolicy(mdp),
    "heuristic" => heur,
    "mcts" => begin
        solver = DPWSolver(depth=40,
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


sim = PmapSimulator(HistoryRecorder(max_steps=40)) do r::SimResult
    hist = r[:history]
    return (:reward => discounted_reward(hist),
            :n_steps => n_steps(hist))
end # do block defines what to do with the simulation result. Should be a collection of pairs

s = SimSet(problems=problems,
           policies=policies,
           simulator=sim,
           n=1
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
@save(filename, df, problems, policies)

using Plots
# unicodeplots()
gr()
histogram(df[:reward])
gui()

means = by(df, :policy_key) do df
    DataFrame(reward=mean(df[:reward]), n_steps=mean(df[:n_steps]))
end
