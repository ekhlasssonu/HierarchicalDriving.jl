using POMDPs

using POMDPToolbox

println("Right")
p = ChangeLaneRightPOMDP()
for i in 1:100
  print("\r$i")
  policy = RandomPolicy(p)
  hr = HistoryRecorder(max_steps = 20)
  simulate(hr, p, policy)
end
println()
#=
println("Left")
p1 = ChangeLaneLeftPOMDP()
for i in 1:100
  print("\r$i")
  policy = RandomPolicy(p1)
  hr = HistoryRecorder(max_steps = 20)
  simulate(hr, p1, policy)
end
println()

println("M25")
p3 = MaintainAt25POMDP()
for i in 1:100
  print("\r$i")
  policy = RandomPolicy(p3)
  hr = HistoryRecorder(max_steps = 20)
  simulate(hr, p3, policy)
end
println()

println("M27")
p4 = MaintainAt27POMDP()
for i in 1:100
  print("\r$i")
  policy = RandomPolicy(p4)
  hr = HistoryRecorder(max_steps = 20)
  simulate(hr, p4, policy)
end
println()

println("M23")
p2 = MaintainAt23POMDP()
for i in 1:100
  print("\r$i")
  policy = RandomPolicy(p2)
  hr = HistoryRecorder(max_steps = 20)
  simulate(hr, p2, policy)
end
println()
=#
println("Generic Low Level")
p5 = LowLevelMDP()
for i in 1:10000
  print("\r$i")
  policy = RandomPolicy(p5)
  hr = HistoryRecorder(max_steps = 20)
  simulate(hr, p5, policy)
end
println()
