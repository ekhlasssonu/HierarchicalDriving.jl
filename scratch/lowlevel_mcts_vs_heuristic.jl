using HierarchicalDriving
using POMDPToolbox
using POMDPs
using MCTS
using DataFrames

@everywhere using POMDPToolbox

mdp = LowLevelMDP()
heur = subintentional_policy(mdp)

policies = Dict{String, Policy}(
    "random" => RandomPolicy(mdp),
    "heuristic" => heur,
    "mcts" => begin
        solver = DPWSolver(depth=20,
                           exploration_constant=10.0,
                           n_iterations=100,
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




sim = PmapSimulator(HistoryRecorder(max_steps=10)) do r::SimResult
    hist = r[:history]
    return (:reward => discounted_reward(hist),)
end # do block defines what to do with the simulation result. Should be a collection of pairs

s = SimSet(problems=problems,
           policies=policies,
           simulator=sim,
           n=10
          )
# basically just a dict holding options
           
sets = [
    SimSet(s, "heuristic", "default"),
    SimSet(s, "random", "default"),
    SimSet(s, "mcts", "default")
]

df = run(sets)

means = by(df, :policy_key, df -> mean(df[:reward]))
println(means)
