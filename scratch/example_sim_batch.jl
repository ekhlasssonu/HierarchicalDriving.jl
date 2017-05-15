using HierarchicalDriving
using POMDPToolbox
using POMDPs

@everywhere using POMDPToolbox

mdp = LowLevelMDP()
heur = subintentional_policy(mdp)

policies = Dict{String, Policy}(
    "random" => RandomPolicy(mdp),
    "heuristic" => heur
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
    SimSet(s, "random", "default")
]

df = run(sets)

println(df)

# you can then rerun one of the simulations
r = 14
row = df[r, :]
println("Rerunning $r")
println(row)
hist = rerun(row, SimSet(s, "random", "default"))
@show discounted_reward(hist)
