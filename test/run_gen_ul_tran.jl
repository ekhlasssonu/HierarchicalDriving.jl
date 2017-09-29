using POMDPs

using POMDPToolbox

using JLD

println("Generating Transition Function for SingleAgentOccGridMDP using ULTranFuncGenMDP")

num_ld_nbr_blocks = 1
num_adj_nbr_blocks = 1
cell_length = 75.0
nbr_cell_length = 25.0
n_v_cells = convert(Int64,ceil(length(road_segment)/cell_length))
numLanes = n_lanes(road_segment)

n_iter = 50

n_d_ln = 3
max_d_dist = 2
gen = SingleAgentOccGridMDP_TGenerator(road_segment, cell_length, nbr_cell_length)

tranProb = zeros(Float64, (3, num_ld_nbr_blocks+1, 2^num_adj_nbr_blocks, 2^num_adj_nbr_blocks, n_d_ln, max_d_dist + 1))  #Pr(Δln, Δdist|a, ld_dist, occLt, occRt)
actionRwd = zeros(Float64, (3,num_ld_nbr_blocks+1,2^num_adj_nbr_blocks, 2^num_adj_nbr_blocks))
fill!(tranProb, 0.2)
#NOTE: Experiments will be carried out for varying number of agents

#Initial position of the ego vehicle
egoLane = 2
egoDist = 3
initEgoPos = AgentGridLocation(egoLane, egoDist)
for a in -1:1
  #println("a = $a")
  for ld_dist in 1:num_ld_nbr_blocks + 1
    #println("\tldCell = $ld_dist")
    for occLt in 1:2^num_adj_nbr_blocks
      #println("\t\toccLt = $occLt")
      for occRt in 1:2^num_adj_nbr_blocks
        #println("\t\t\toccRt = $occRt")
        ulInitState = ImmGridOccSt(initEgoPos, ld_dist, int2BoolArray(occLt, num_adj_nbr_blocks), int2BoolArray(occRt, num_adj_nbr_blocks))
        iterations = n_iter
        for i in 1:iterations
          print("\ra = $a ld_dist = $ld_dist occLt = $occLt occRt = $occRt iterNo: $i    ")
          rng = MersenneTwister(i)
          llMDP, gblSt = initialize_LowLevelMDP_gblSt(gen, ulInitState, a, rng)
          println("\n Initial Config: ")
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
          #Ignore Collisions here.
          #TODO: Fix this. If collision happens, it happens.
          if state_hist(hist)[end].terminal == 1
            println("\n * \n * \n *")
            #iterations += 1
            #continue
          end

          finEgoPos = getCarGridLocation(gen, finEgoState)
          #println("finEgoState = ",finEgoState, " finEgoPos = ", finEgoPos)
          laneOffset = finEgoPos.lane - initEgoPos.lane
          distOffset = finEgoPos.distance - initEgoPos.distance
          rwd = discounted_reward(hist)
          #println("distOffset = ", distOffset)
          if distOffset > max_d_dist + 1 || clamp(initEgoPos.lane + a, 0, numLanes) != finEgoPos.lane
            println("\n\t\tStartLane = ", initEgoPos.lane, " TargetLane = ", clamp(initEgoPos.lane + a, 0, numLanes), " FinalLane = ", finEgoPos.lane, " distOffset = ", distOffset, " Reward = ", rwd)
          end
          distOffset = clamp(distOffset, 0, max_d_dist + 1)

          #print("\t\tStartLane = ", initEgoPos.lane, " TargetLane = ", clamp(initEgoPos.lane + a, 0, numLanes), " FinalLane = ", finEgoPos.lane, " distOffset = ", distOffset)
          tranProb[a+2,ld_dist,occLt,occRt,laneOffset+2,distOffset+1] += 1
          actionRwd[a+2,ld_dist,occLt,occRt] += rwd
        end
        for laneOffset in -1:1
          for distOffset in 0:max_d_dist
            tranProb[a+2,ld_dist,occLt,occRt,laneOffset+2,distOffset+1] /= n_iter
            actionRwd[a+2,ld_dist,occLt,occRt] /= n_iter
          end
        end
      end
    end
  end
end

tDimensions = size(tranProb)
for a in -1:1
  for ld_dist in 1:tDimensions[2]
    for lt_occ_int in 1:tDimensions[3]
      for rt_occ_int in 1:tDimensions[4]
        sum = 0.0
        for d_ln in -1:1
          for d_dist in 0:tDimensions[6]-1
            sum += tranProb[a+2, ld_dist, lt_occ_int, rt_occ_int, d_ln+2, d_dist+1]
          end
        end
        for d_ln in -1:1
          for d_dist in 0:tDimensions[6]-1
            tranProb[a+2, ld_dist, lt_occ_int, rt_occ_int, d_ln+2, d_dist+1] /= sum
          end
        end
      end
    end
  end
end

save("../scratch/SingleAgentOccGrid/TranProb_$(convert(Int64,cell_length))_$(convert(Int64,nbr_cell_length))_$(num_ld_nbr_blocks)_$(num_adj_nbr_blocks).jld", "tranProb", tranProb)
save("../scratch/SingleAgentOccGrid/ActionRwd_$(convert(Int64,cell_length))_$(convert(Int64,nbr_cell_length))_$(num_ld_nbr_blocks)_$(num_adj_nbr_blocks).jld", "r_s_a", actionRwd)
#t2 = load("../scratch/TranProb.jld", "tranProb")

#println("t2 = ", t2)
