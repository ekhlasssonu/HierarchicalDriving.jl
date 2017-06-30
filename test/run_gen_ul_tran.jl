using POMDPs

using POMDPToolbox

println("Generating Transition Function for ULTranFuncGenMDP")

road_segment = RoadSegment((-100.0, 500.0),[0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH])

n_agents = 40

ll_timestep = 0.2

ll_horizon = 0.3

n_iter = 1
#agents = [10, 40, 70, 100]
agents = [40]

cell_length = 25.0

n_v_cells = round(Int64, ceil(length(road_segment)/cell_length))
numLanes = n_lanes(road_segment)

gen = SingleAgentOccGridMDP_TGenerator(road_segment, 25.0)

tranProb = zeros(Float64, (4, 8, 8, 3, 3, 8))  #Pr(Δln, Δdist|ldDist, occLt, occRt, a)
#Experiments will be carried out for varying number of agents
for n_agents_idx in 1:length(agents)
  n_agents = agents[n_agents_idx]
  #Initial position of the ego vehicle
  egoLane = 2
  egoDist = 2
  initEgoPos = AgentGridLocation(egoLane, egoDist)
  for a in -1:1
    println("a = $a")
    for ldCell in 0:3
      println("\tldCell = $ldCell")
      for occLt in 1:8
        println("\t\toccLt = $occLt")
        for occRt in 1:8
          println("\t\t\toccRt = $occRt")
          ulInitState = ImmGridOccSt(initEgoPos, ldCell, tuple(int2BoolArray(occLt, 3)...), tuple(int2BoolArray(occRt, 3)...))
          for i in 1:n_iter
            println("\t\t\t\titerNo: $i")
            rng = MersenneTwister(i)
            llMDP, gblSt = initialize_LowLevelMDP_gblSt(gen, ulInitState, a, rng)
            solver = DPWSolver(depth=llMDP.HORIZON,
                           exploration_constant=10.0,
                           n_iterations=1_500,
                           k_action=10.0,
                           alpha_action=1/10,
                           k_state=5.0,
                           alpha_state=1/10,

                          )

            policy = solve(solver, llMDP)
            hr = HistoryRecorder(max_steps = llMDP.HORIZON, rng = rng)
            hist = simulate(hr, llMDP, policy, gblSt)
            finEgoState = state_hist(hist)[end].ego
            finEgoPos = getCarGridLocation(gen, finEgoState)
            laneOffset = finEgoPos.lane - initEgoPos.lane
            distOffset = finEgoPos.distance - initEgoPos.distance
            distOffset = clamp(distOffset, 0 , 7)

            println("\t\t\t\t\tStartLane = ", initEgoPos.lane, " TargetLane = ", clamp(initEgoPos.lane + a, 0, numLanes), " FinalLane = ", finEgoPos.lane)

            tranProb[ldCell+1,occLt,occRt,a+2,laneOffset+2,distOffset+1] += 1
          end
          for laneOffset in 1:3
            for distOffset in 1:8
              tranProb[ldCell+1,occLt,occRt,a+2,laneOffset,distOffset] /= n_iter
            end
          end
        end
      end
    end
  end
  println("#numAgents = ", n_agents)
  println("tranProb = ", tranProb)
end
