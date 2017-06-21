using POMDPs

using POMDPToolbox

println("LowLevelHeuristics")
p6 = LowLevelMDP()
for i in 1:10
  print("\r$i")
  policy = subintentional_lowlevel_policy(p6)
  hr = HistoryRecorder(max_steps = 10)
  simulate(hr, p6, policy)
end
println()

println("SimulationMDP")
p7 = SimulationMDP()
for i in 1:10
  print("\r$i")
  policy = subintentional_simulation_policy(p7)
  hr = HistoryRecorder(max_steps = 60)
  simulate(hr, p7, policy)
end
println()
