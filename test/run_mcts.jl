println("MCTS Policy")
p5 = LowLevelMDP()
solver = MCTSSolver()
policy = solve(solver, p5)
for i in 1:10
  print("\r$i")
  hr = HistoryRecorder(max_steps = 20)
  simulate(hr, p5, policy)
end
