numSims = 100
for n in 40:20:160

  open("../scratch/FlatSimulationSubintentional_new.txt", "a") do f
    println("*************************************NumAgents = $n*************************************")
    write(f, "*************************************NumAgents = $n*************************************\n")
    avg_distance_to_goal = 0.0
    avg_time_to_goal = 0.0
    avg_distance_from_lane_center = 0.0
    numSuccess = 0
    avg_hard_braking_rate = 0.0
    avg_induced_hard_brake_rate = 0.0
    avg_planning_time = 0.0
    avg_collision_rate = 0.0

    p = SimulationMDP(n)

    for i in 1:numSims
      println("\tSimulation no.: $i")
      rng = MersenneTwister(i*29+1)
      policy = subintentional_simulation_policy(p, rng)
      hr = HistoryRecorder(max_steps = 150, rng = rng)
      t1 = time_ns()
      hist = simulate(hr, p, policy)
      t2 = time_ns()
      avg_planning_time += (t2 - t1)/1.0e9
      st_hist = state_hist(hist)
      act_hist = action_hist(hist)
      finState = st_hist[end]

      if finState.terminal == 1
        avg_collision_rate += 1
      end
      num_induced_hard_brake = 0
      cum_distance_from_lane_center = 0.0
      for st in st_hist
        egoState = st.ego
        cum_distance_from_lane_center += get_distance_from_lane_center(egoState, p)

        if check_induced_hardbraking(st, p)
          num_induced_hard_brake += 1
        end
      end
      avg_distance_from_lane_center += cum_distance_from_lane_center/length(st_hist)
      avg_induced_hard_brake_rate += num_induced_hard_brake/length(st_hist)

      num_hard_braking = 0
      for act in act_hist
        if act == length(EgoActionSpace().actions)
          num_hard_braking += 1
        end
      end
      avg_hard_braking_rate += num_hard_braking/length(act_hist)

      if finState.terminal == 2
        avg_distance_to_goal += finState.ego.state[1]
        avg_time_to_goal += length(hist) * p.TIME_STEP
        numSuccess += 1
      end

    end

    avg_distance_to_goal /= numSuccess
    avg_time_to_goal /= numSuccess
    successRate = 1.0*numSuccess/numSims
    avg_distance_from_lane_center /= numSims
    tot_hard_brake_rate = avg_hard_braking_rate + avg_induced_hard_brake_rate
    avg_hard_braking_rate /= numSims
    avg_induced_hard_brake_rate /= numSims
    tot_hard_brake_rate /= numSims
    avg_planning_time /= numSims
    avg_collision_rate /= numSims

    println("Average distance to goal = $avg_distance_to_goal")
    println("Average time to goal = $avg_time_to_goal")
    println("Success Rate = $successRate")
    println("Average distance from lane center = $avg_distance_from_lane_center")
    println("Average hard braking rate = $avg_hard_braking_rate")
    println("Average induced hard braking rate = $avg_induced_hard_brake_rate")
    println("Total hard braking rate = $tot_hard_brake_rate\n")
    println("Average planning time = $avg_planning_time\n")
    println("Average collision rate = $avg_collision_rate\n")

    write(f,"Average distance to goal = $avg_distance_to_goal\n")
    write(f,"Average time to goal = $avg_time_to_goal\n")
    write(f,"Success Rate = $successRate\n")
    write(f,"Average distance from lane center = $avg_distance_from_lane_center\n")
    write(f,"Average hard braking rate = $avg_hard_braking_rate\n")
    write(f,"Average induced hard braking rate = $avg_induced_hard_brake_rate\n")
    write(f,"Total hard braking rate = $tot_hard_brake_rate\n")
    write(f,"Average planning time = $avg_planning_time\n")
    write(f,"Average collision rate = $avg_collision_rate\n")
  end
end



#=for n in 100:20:160
  open("../scratch/FlatSimulationMCTSDPW.txt", "a") do f
    println("*************************************NumAgents = $n*************************************")
    write(f, "*************************************NumAgents = $n*************************************\n")

    avg_distance_to_goal = 0.0
    avg_time_to_goal = 0.0
    avg_distance_from_lane_center = 0.0
    numSuccess = 0
    avg_hard_braking_rate = 0.0
    avg_induced_hard_brake_rate = 0.0
    avg_planning_time = 0.0
    avg_collision_rate = 0.0

    p = SimulationMDP(n)
    for i in 1:numSims
      println("Simulation $i")
      rng = MersenneTwister(i*29+1)
      t1 = time_ns()
      solver = DPWSolver(depth=p.HORIZON,
                                     exploration_constant=10.0,
                                     n_iterations=1_500,
                                     k_action=10.0,
                                     alpha_action=1/10,
                                     k_state=5.0,
                                     alpha_state=1/10,
                                     #estimate_value=RolloutEstimator(subintentional_lowlevel_policy(p))
                                    )
      policy = solve(solver, p)
      hr = HistoryRecorder(max_steps = 150, rng = rng)
      hist = simulate(hr, p, policy)
      t2 = time_ns()
      avg_planning_time += (t2 - t1)/1.0e9
      st_hist = state_hist(hist)
      act_hist = action_hist(hist)
      finState = st_hist[end]

      if finState.terminal == 1
        avg_collision_rate += 1
      end
      num_induced_hard_brake = 0
      cum_distance_from_lane_center = 0.0
      for st in st_hist
        egoState = st.ego
        cum_distance_from_lane_center += get_distance_from_lane_center(egoState, p)

        if check_induced_hardbraking(st, p)
          num_induced_hard_brake += 1
        end
      end
      avg_distance_from_lane_center += cum_distance_from_lane_center/length(st_hist)
      avg_induced_hard_brake_rate += num_induced_hard_brake/length(st_hist)

      num_hard_braking = 0
      for act in act_hist
        if act == length(EgoActionSpace().actions)
          num_hard_braking += 1
        end
      end
      avg_hard_braking_rate += num_hard_braking/length(act_hist)

      if finState.terminal == 2
        avg_distance_to_goal += finState.ego.state[1]
        avg_time_to_goal += length(hist) * p.TIME_STEP
        numSuccess += 1
      end

    end
    avg_distance_to_goal /= numSuccess
    avg_time_to_goal /= numSuccess
    successRate = 1.0*numSuccess/numSims
    avg_distance_from_lane_center /= numSims
    tot_hard_brake_rate = avg_hard_braking_rate + avg_induced_hard_brake_rate
    avg_hard_braking_rate /= numSims
    avg_induced_hard_brake_rate /= numSims
    tot_hard_brake_rate /= numSims
    avg_planning_time /= numSims
    avg_collision_rate /= numSims

    println("Average distance to goal = $avg_distance_to_goal")
    println("Average time to goal = $avg_time_to_goal")
    println("Success Rate = $successRate")
    println("Average distance from lane center = $avg_distance_from_lane_center")
    println("Average hard braking rate = $avg_hard_braking_rate")
    println("Average induced hard braking rate = $avg_induced_hard_brake_rate")
    println("Total hard braking rate = $tot_hard_brake_rate\n")
    println("Average planning time = $avg_planning_time\n")
    println("Average collision rate = $avg_collision_rate\n")

    write(f,"Average distance to goal = $avg_distance_to_goal\n")
    write(f,"Average time to goal = $avg_time_to_goal\n")
    write(f,"Success Rate = $successRate\n")
    write(f,"Average distance from lane center = $avg_distance_from_lane_center\n")
    write(f,"Average hard braking rate = $avg_hard_braking_rate\n")
    write(f,"Average induced hard braking rate = $avg_induced_hard_brake_rate\n")
    write(f,"Total hard braking rate = $tot_hard_brake_rate\n")
    write(f,"Average planning time = $avg_planning_time\n")
    write(f,"Average collision rate = $avg_collision_rate\n")
  end

end=#
