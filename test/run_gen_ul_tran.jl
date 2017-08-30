using POMDPs

using POMDPToolbox

using JLD

println("Generating Transition Function for ULTranFuncGenMDP")

num_ld_nbr_blocks = 1
num_adj_nbr_blocks = 1
cellLength = 75.0
n_v_cells = convert(Int64,ceil(length(road_segment)/cellLength))
numLanes = n_lanes(road_segment)

n_iter = 20

gen = SingleAgentOccGridMDP_TGenerator(road_segment, cellLength)

tranProb = zeros(Float64, (3, num_ld_nbr_blocks+1, 2^num_adj_nbr_blocks, 2^num_adj_nbr_blocks, 3, 4))  #Pr(Δln, Δdist|a, ldDist, occLt, occRt)
actionRwd = zeros(Float64, (3,num_ld_nbr_blocks+1,2^num_adj_nbr_blocks, 2^num_adj_nbr_blocks))
fill!(tranProb, 0.2)
#Experiments will be carried out for varying number of agents

#Initial position of the ego vehicle
egoLane = 2
egoDist = 3
initEgoPos = AgentGridLocation(egoLane, egoDist)
for a in -1:1
  #println("a = $a")
  for ldCell in 0:num_ld_nbr_blocks
    #println("\tldCell = $ldCell")
    for occLt in 1:2^num_adj_nbr_blocks
      #println("\t\toccLt = $occLt")
      for occRt in 1:2^num_adj_nbr_blocks
        #println("\t\t\toccRt = $occRt")
        ulInitState = ImmGridOccSt(initEgoPos, ldCell, int2BoolArray(occLt, num_adj_nbr_blocks), int2BoolArray(occRt, num_adj_nbr_blocks))
        for i in 1:n_iter
          print("\ra = $a ldCell = $ldCell occLt = $occLt occRt = $occRt iterNo: $i    ")
          rng = MersenneTwister(i)
          llMDP, gblSt = initialize_LowLevelMDP_gblSt(gen, ulInitState, a, rng)
          println()
          printGlobalPhyState(gblSt, llMDP.roadSegment)
          #println("initEgoState = ", gblSt.ego)
          solver = DPWSolver(depth=llMDP.HORIZON,
                         exploration_constant=10.0,
                         n_iterations=1_500,
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
          if distOffset > 3 || clamp(initEgoPos.lane + a, 0, numLanes) != finEgoPos.lane
            println("\n\t\tStartLane = ", initEgoPos.lane, " TargetLane = ", clamp(initEgoPos.lane + a, 0, numLanes), " FinalLane = ", finEgoPos.lane, " distOffset = ", distOffset, " Reward = ", rwd)
          end
          distOffset = clamp(distOffset, 0, 3)

          #print("\t\tStartLane = ", initEgoPos.lane, " TargetLane = ", clamp(initEgoPos.lane + a, 0, numLanes), " FinalLane = ", finEgoPos.lane, " distOffset = ", distOffset)
          tranProb[a+2,ldCell+1,occLt,occRt,laneOffset+2,distOffset+1] += 1
          actionRwd[a+2,ldCell+1,occLt,occRt] += rwd
        end
        for laneOffset in -1:1
          for distOffset in 0:3
            tranProb[a+2,ldCell+1,occLt,occRt,laneOffset+2,distOffset+1] /= n_iter
            actionRwd[a+2,ldCell+1,occLt,occRt] /= n_iter
          end
        end
      end
    end
  end
end

tDimensions = size(tranProb)
for a in -1:1
  for ldDist in 0:tDimensions[2]-1
    for lt_occ_int in 1:tDimensions[3]
      for rt_occ_int in 1:tDimensions[4]
        sum = 0.0
        for d_ln in -1:1
          for d_dist in 0:tDimensions[6]-1
            sum += tranProb[a+2, ldDist+1, lt_occ_int, rt_occ_int, d_ln+2, d_dist+1]
          end
        end
        for d_ln in -1:1
          for d_dist in 0:tDimensions[6]-1
            tranProb[a+2, ldDist+1, lt_occ_int, rt_occ_int, d_ln+2, d_dist+1] /= sum
          end
        end
      end
    end
  end
end

#println("tranProb = ", tranProb)

save("../scratch/TranProb_$(convert(Int64,cellLength))_$(num_ld_nbr_blocks)_$(num_adj_nbr_blocks).jld", "tranProb", tranProb)
save("../scratch/ActionRwd_$(convert(Int64,cellLength))_$(num_ld_nbr_blocks)_$(num_adj_nbr_blocks).jld", "r_s_a", actionRwd)
#t2 = load("../scratch/TranProb.jld", "tranProb")

#println("t2 = ", t2)
