numSims = 100

for n in 40:20:160
  #=
  open("../scratch/Hierarchical1.txt", "a") do f

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

    #Type 1
    policyFileName = "../scratch/SingleAgentGridPolicy.jld"
    #hf1 = HierarchicalFramework1(p,n_v_cells1, u_goalReward1, u_p_next_cell, u_p_desired_lane)
    hf1 = HierarchicalFramework1(p, policyFileName)
    #println(hf1)
    for i in 1:numSims
      println("\tSimulation no.: $i")
      rng = MersenneTwister(i*29+1)

      hp1 = HierarchicalPolicy1(hf1,rng)
      #println(hp1)
      #s = initial_state(p, rng)
      #printGlobalPhyState(s, p.roadSegment)
      #printNeighborCache(s)
      #println()
      #a = action(hp1, s)
      #println("action = ",a)

      hr = HistoryRecorder(max_steps = 150, rng = rng)
      @time (hist = simulate(hr, p, hp1))
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
        println("\t\tSuccess!!!\t\t Planning Time = $(hp1.planningTime)")
        avg_distance_to_goal += finState.ego.state[1]
        avg_time_to_goal += length(hist) * p.TIME_STEP
        numSuccess += 1
      else
        println("\t\tFailure: Final y = $(finState.ego.state[2])\t\t Planning Time = $(hp1.planningTime)")
      end
      avg_planning_time += hp1.planningTime
    end


    avg_distance_to_goal /= numSuccess
    avg_time_to_goal /= numSuccess
    successRate = 1.0*numSuccess/numSims
    avg_distance_from_lane_center /= numSims
    avg_hard_braking_rate /= numSims
    avg_induced_hard_brake_rate /= numSims
    avg_planning_time /= numSims
    avg_collision_rate /= numSims

    println("Average distance to goal = $avg_distance_to_goal")
    println("Average time to goal = $avg_time_to_goal")
    println("Success Rate = $successRate")
    println("Average distance from lane center = $avg_distance_from_lane_center")
    println("Average hard braking rate = $avg_hard_braking_rate")
    println("Average induced hard braking rate = $avg_induced_hard_brake_rate")

    write(f,"Average distance to goal = $avg_distance_to_goal\n")
    write(f,"Average time to goal = $avg_time_to_goal\n")
    write(f,"Success Rate = $successRate\n")
    write(f,"Average distance from lane center = $avg_distance_from_lane_center\n")
    write(f,"Average hard braking rate = $avg_hard_braking_rate\n")
    write(f,"Average induced hard braking rate = $avg_induced_hard_brake_rate\n")
    write(f,"Average planning time = $avg_planning_time\n")
    write(f,"Average collision rate = $avg_collision_rate\n")
  end
  =#


  for cellLength in [75]
    open("../scratch/Hierarchical2_$(cellLength).txt", "a") do f

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
      policyFileName = "../scratch/SingleAgentOccGridPolicy_$(n)_$(cellLength)_1_1.jld"

      hf2 = HierarchicalFramework2(p, policyFileName)
      for i in 1:numSims
        println("\tSimulation no.: $i")
        rng = MersenneTwister(i*29+1)

        hp2 = HierarchicalPolicy2(hf2,rng)
        hr = HistoryRecorder(max_steps = 150, rng = rng)

        @time (hist = simulate(hr, p, hp2))
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
          println("\t\tSuccess!!!\t\t Planning Time = $(hp2.planningTime)")
          avg_distance_to_goal += finState.ego.state[1]
          avg_time_to_goal += length(hist) * p.TIME_STEP
          numSuccess += 1
        else
          println("\t\tFailure: Final y = $(finState.ego.state[2])\t\t Planning Time = $(hp2.planningTime)")
        end
        avg_planning_time += hp2.planningTime

      end
      avg_distance_to_goal /= numSuccess
      avg_time_to_goal /= numSuccess
      successRate = 1.0*numSuccess/numSims
      avg_distance_from_lane_center /= numSims
      avg_hard_braking_rate /= numSims
      avg_induced_hard_brake_rate /= numSims
      avg_planning_time /= numSims
      avg_collision_rate /= numSims

      println("Average distance to goal = $avg_distance_to_goal")
      println("Average time to goal = $avg_time_to_goal")
      println("Success Rate = $successRate")
      println("Average distance from lane center = $avg_distance_from_lane_center")
      println("Average hard braking rate = $avg_hard_braking_rate")
      println("Average induced hard braking rate = $avg_induced_hard_brake_rate")

      write(f,"Average distance to goal = $avg_distance_to_goal\n")
      write(f,"Average time to goal = $avg_time_to_goal\n")
      write(f,"Success Rate = $successRate\n")
      write(f,"Average distance from lane center = $avg_distance_from_lane_center\n")
      write(f,"Average hard braking rate = $avg_hard_braking_rate\n")
      write(f,"Average induced hard braking rate = $avg_induced_hard_brake_rate\n")
      write(f,"Average planning time = $avg_planning_time\n")
      write(f,"Average collision rate = $avg_collision_rate\n")
    end
  end
end
