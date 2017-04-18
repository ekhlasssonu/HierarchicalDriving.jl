using POMDPs
p = ChangeLaneRightPOMDP()

using POMDPToolbox

policy = RandomPolicy(p)
hr = HistoryRecorder(max_steps = 20)
simulate(hr, p, policy)
