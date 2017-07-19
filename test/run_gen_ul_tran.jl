using POMDPs

using POMDPToolbox

using JLD

println("Generating Transition Function for ULTranFuncGenMDP")

road_segment = RoadSegment((-100.0, 500.0),[0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH])

n_agents = 40

#ll_timestep = 0.2

#ll_horizon = 20
n_iter = 20
#agents = [10, 40, 70, 100]
agents = [40]

cell_length = 25.0

n_v_cells = round(Int64, ceil(length(road_segment)/cell_length))
numLanes = n_lanes(road_segment)

gen = SingleAgentOccGridMDP_TGenerator(road_segment, 25.0)

tranProb = zeros(Float64, (3, 4, 8, 8, 3, 5))  #Pr(Δln, Δdist|a, ldDist, occLt, occRt)
actionRwd = zeros(Float64, (3,4,8,8))
fill!(tranProb, 0.2)
#Experiments will be carried out for varying number of agents

#Initial position of the ego vehicle
egoLane = 2
egoDist = 2
initEgoPos = AgentGridLocation(egoLane, egoDist)
for a in -1:1
  #println("a = $a")
  for ldCell in 0:3
    #println("\tldCell = $ldCell")
    for occLt in 1:8
      #println("\t\toccLt = $occLt")
      for occRt in 1:8
        #println("\t\t\toccRt = $occRt")
        ulInitState = ImmGridOccSt(initEgoPos, ldCell, tuple(int2BoolArray(occLt, 3)...), tuple(int2BoolArray(occRt, 3)...))
        for i in 1:n_iter
          print("\ra = $a ldCell = $ldCell occLt = $occLt occRt = $occRt iterNo: $i    ")
          rng = MersenneTwister(i)
          llMDP, gblSt = initialize_LowLevelMDP_gblSt(gen, ulInitState, a, rng)
          #println("initEgoState = ", gblSt.ego)
          solver = DPWSolver(depth=llMDP.HORIZON,
                         exploration_constant=10.0,
                         n_iterations=1_000,
                         k_action=10.0,
                         alpha_action=1/10,
                         k_state=5.0,
                         alpha_state=1/10,
                         estimate_value=RolloutEstimator(subintentional_lowlevel_policy(llMDP))
                        )

          policy = solve(solver, llMDP)
          hr = HistoryRecorder(max_steps = llMDP.HORIZON, rng = rng)
          hist = simulate(hr, llMDP, policy, gblSt)
          finEgoState = state_hist(hist)[end].ego
          finEgoPos = getCarGridLocation(gen, finEgoState)
          #println("finEgoState = ",finEgoState, " finEgoPos = ", finEgoPos)
          laneOffset = finEgoPos.lane - initEgoPos.lane
          distOffset = finEgoPos.distance - initEgoPos.distance
          rwd = discounted_reward(hist)
          #println("distOffset = ", distOffset)
          if distOffset >= 5 || clamp(initEgoPos.lane + a, 0, numLanes) != finEgoPos.lane
            println("\n\t\tStartLane = ", initEgoPos.lane, " TargetLane = ", clamp(initEgoPos.lane + a, 0, numLanes), " FinalLane = ", finEgoPos.lane, " distOffset = ", distOffset, " Reward = ", rwd)
          end
          distOffset = clamp(distOffset, 0, 4)

          #print("\t\tStartLane = ", initEgoPos.lane, " TargetLane = ", clamp(initEgoPos.lane + a, 0, numLanes), " FinalLane = ", finEgoPos.lane, " distOffset = ", distOffset)
          tranProb[a+2,ldCell+1,occLt,occRt,laneOffset+2,distOffset+1] += 1
          actionRwd[a+2,ldCell+1,occLt,occRt] += rwd
        end
        for laneOffset in -1:1
          for distOffset in 0:4
            tranProb[a+2,ldCell+1,occLt,occRt,laneOffset+2,distOffset+1] /= n_iter
            actionRwd[a+2,ldCell+1,occLt,occRt] /= n_iter
          end
        end
      end
    end
  end
end

println("tranProb = ", tranProb)

save("../scratch/TranProb.jld", "tranProb", tranProb)
save("../scratch/ActionRwd.jld", "r_s_a", actionRwd)
#t2 = load("../scratch/TranProb.jld", "tranProb")

#println("t2 = ", t2)
