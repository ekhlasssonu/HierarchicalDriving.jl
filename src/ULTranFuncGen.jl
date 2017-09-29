#Generate a LowLevelMDP given UpperLevelMDP, Upper level state and macro action.
type SingleAgentOccGridMDP_TGenerator #Bad name
  road_segment::RoadSegment
  cell_length::Float64
  nbr_cell_length::Float64
  frameList::Array{LowLevelCarFrameL0,1}
end

SingleAgentOccGridMDP_TGenerator(road_segment::RoadSegment, cell_length::Float64, nbr_cell_length::Float64) = SingleAgentOccGridMDP_TGenerator(road_segment, cell_length, nbr_cell_length, getFrameList())

function getCarGridLocation(gen::SingleAgentOccGridMDP_TGenerator, phySt::CarPhysicalState)
  x = phySt.state[1]
  y = phySt.state[2]
  lane = getLaneNo(y, gen.road_segment)
  x_adjusted = x - gen.road_segment.x_boundary[1]
  distance = Int64(ceil(x_adjusted/gen.cell_length))
  return AgentGridLocation(lane, distance)
end

function initialize_LowLevelMDP_gblSt(gen::SingleAgentOccGridMDP_TGenerator, ulInitState::ImmGridOccSt, a::Int64, rng::AbstractRNG)
  rs = gen.road_segment
  lb_x = rs.x_boundary[1]
  ub_x = rs.x_boundary[2]
  cell_length = gen.cell_length
  nbr_cell_length = gen.nbr_cell_length
  numLanes = n_lanes(rs)
  egoGrid = ulInitState.egoGrid
  egoLane = egoGrid.lane
  egoDist = egoGrid.distance
  init_y = getLaneCenter(rs, egoLane) + randn(rng) * LANE_WIDTH/12.0
  init_x = lb_x + ((egoDist - 1) + rand(rng)) * cell_length
  init_xdot = randn(rng) * VEL_STD_DEV + AVG_HWY_VELOCITY
  initSt = CarPhysicalState((init_x, init_y, init_xdot))

  egoGoalLane = clamp(egoLane + a, 0, numLanes)

  fin_y = getLaneCenter(rs, egoGoalLane)

  fin_y_lb = fin_y - 0.5
  fin_y_ub = fin_y + 0.5

  fin_x_lb = lb_x + egoDist * cell_length
  fin_x_ub = ub_x

  finSt_lb = CarPhysicalState((fin_x_lb, fin_y_lb, AVG_HWY_VELOCITY - 5.0))
  finSt_ub = CarPhysicalState((fin_x_ub, fin_y_ub, AVG_HWY_VELOCITY + 5.0))

  llMDP = LowLevelMDP(ll_discount, ll_TIME_STEP, sim_TIME_STEP, ll_HORIZON, rs, initSt, (finSt_lb, finSt_ub),
                          ll_goalReward, ll_collisionCost, ll_y_dev_cost, ll_hardbrakingCost, ll_discomfortCost, ll_velocityDeviationCost, getFrameList())

  #Generate initial global state for llMDP
  #ego state is initSt
  neighborhood = Array{Array{CarLocalIS,1}}(numLanes)
  for i in 1:numLanes
    neighborhood[i] = Array{CarLocalIS,1}()
  end

  #leader in the same lane.
  ldr_dist = 0*CAR_LENGTH + (ulInitState.ld_distance - 1 + rand(rng)) * nbr_cell_length
  if ldr_dist < 1.8 * CAR_LENGTH
    ldr_dist = min(ldr_dist + 1.8 * CAR_LENGTH, nbr_cell_length)
  end
  ldr_x = init_x + ldr_dist
  ldr_y = getLaneCenter(rs, egoLane)
  ldrProbDist = NTuple{3,NormalDist}((NormalDist(ldr_x, 0.01), NormalDist(ldr_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
  intentionArray = zeros(Float64, numLanes)
  egoLane - 1 > 0 ? intentionArray[egoLane-1] += 0.3 : intentionArray[egoLane] += 0.3
  intentionArray[egoLane] += 0.4
  egoLane + 1 <= numLanes ? intentionArray[egoLane+1] += 0.3 : intentionArray[egoLane] += 0.3
  ldrState = randCarLocalISL0(rng, ldrProbDist, intentionArray, gen.frameList)
  push!(neighborhood[egoLane], ldrState)

  #Right occupancy
  if egoLane-1 > 0
    occ_rt = ulInitState.occ_r
    adj_idx = convert(Int64, floor(length(occ_rt)/2))+1
    for i in 1:length(occ_rt)
      i_distance_offset = (-1)^(i) * convert(Int64, floor(i/2))
      i_idx = adj_idx - (-1)^(i) * convert(Int64, floor(i/2))
      if occ_rt[i_idx]
        i_lane = egoLane-1
        i_x = init_x - nbr_cell_length/2.0 + (i_distance_offset + rand(rng)) * nbr_cell_length
        i_y = getLaneCenter(rs, i_lane)
        i_ProbDist = NTuple{3,NormalDist}((NormalDist(i_x, 0.1), NormalDist(i_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))

        intentionArray = zeros(Float64, numLanes)
        i_lane - 1 > 0 ? intentionArray[i_lane-1] += 0.3 : intentionArray[i_lane] += 0.3
        intentionArray[i_lane] += 0.4
        i_lane + 1 <= numLanes ? intentionArray[i_lane+1] += 0.3 : intentionArray[i_lane] += 0.3
        ldrState = randCarLocalISL0(rng, i_ProbDist, intentionArray, gen.frameList)
        push!(neighborhood[i_lane], ldrState)
      end
    end
  end

  #LeftOccupancy
  if egoLane+1 <= numLanes
    occ_lt = ulInitState.occ_l
    adj_idx = convert(Int64, floor(length(occ_lt)/2))+1
    for i in 1:length(occ_lt)
      i_distance_offset = (-1)^(i) * convert(Int64, floor(i/2))
      i_idx = adj_idx - (-1)^(i) * convert(Int64, floor(i/2))
      if occ_lt[i_idx]
        i_lane = egoLane+1
        i_x = init_x - nbr_cell_length/2.0 + (i_distance_offset + rand(rng)) * nbr_cell_length
        i_y = getLaneCenter(rs, i_lane)
        i_ProbDist = NTuple{3,NormalDist}((NormalDist(i_x, 0.1), NormalDist(i_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))

        intentionArray = zeros(Float64, numLanes)
        i_lane - 1 > 0 ? intentionArray[i_lane-1] += 0.3 : intentionArray[i_lane] += 0.3
        intentionArray[i_lane] += 0.4
        i_lane + 1 <= numLanes ? intentionArray[i_lane+1] += 0.3 : intentionArray[i_lane] += 0.3
        ldrState = randCarLocalISL0(rng, i_ProbDist, intentionArray, gen.frameList)
        push!(neighborhood[i_lane], ldrState)
      end
    end
  end

  initGblSt = GlobalStateL1(0, initSt, neighborhood)
  return (llMDP, initGblSt)
end
