using HierarchicalDriving
using POMDPToolbox

mdp = LowLevelMDP()
heur = subintentional_policy(mdp)

solvers = Dict{String, Any}(
    "random" => RandomSolver(),
    "heuristic" => heur
) # can be solvers or policies

problems = Dict{String, Any}(
    "default" => mdp
)

sim = PmapSimulator(HistoryRecorder(max_steps=10)) do mdp, hist
    return (:reward => discounted_reward(hist),)
end # do block defines what to do with the simulation result. Should be a collection of pairs

s = SimSet(problems=problems,
           solvers=solvers,
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
row = df[14, :]
println("Rerunning")
println(row)
hist = rerun(row, SimSet(s, "random", "default"))
@show discounted_reward(hist)
