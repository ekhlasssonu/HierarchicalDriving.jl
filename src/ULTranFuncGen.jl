#Generate a LowLevelMDP given UpperLevelMDP, Upper level state and macro action.
type SingleAgentOccGridMDP_TGenerator #Bad name
  road_segment::RoadSegment
  cell_length::Float64
  frameList::Array{LowLevelCarFrameL0,1}
end

SingleAgentOccGridMDP_TGenerator(road_segment::RoadSegment, cell_length::Float64) = SingleAgentOccGridMDP_TGenerator(road_segment::RoadSegment, cell_length::Float64, getFrameList())

function getCarGridLocation(gen::SingleAgentOccGridMDP_TGenerator, phySt::CarPhysicalState)
  x = phySt.state[1]
  y = phySt.state[2]
  lane = getLaneNo(y, gen.road_segment)
  x_offset = x - gen.road_segment.x_boundary[1]
  distance = round(Int64, ceil(x_offset/gen.cell_length))
  return AgentGridLocation(lane, distance)
end

function initialize_LowLevelMDP_gblSt(gen::SingleAgentOccGridMDP_TGenerator, ulInitState::ImmGridOccSt, a::Int64, rng::AbstractRNG)
  rs = gen.road_segment
  cell_length = gen.cell_length
  numLanes = n_lanes(rs)
  egoGrid = ulInitState.egoGrid
  egoLane = egoGrid.lane
  egoDist = egoGrid.distance
  init_y = getLaneCenter(rs, egoLane) + randn(rng) * LANE_WIDTH/12.0
  init_x = ((egoDist - 1) + rand(rng)) * cell_length
  init_xdot = randn(rng) * VEL_STD_DEV + AVG_HWY_VELOCITY
  initSt = CarPhysicalState((init_x, init_y, init_xdot))

  egoGoalLane = clamp(egoLane + a, 0, numLanes)

  fin_y = getLaneCenter(rs, egoGoalLane)

  fin_y_lb = fin_y - 0.5
  fin_y_ub = fin_y + 0.5

  fin_x_lb = egoDist * cell_length
  fin_x_ub = (egoDist + 3) * cell_length

  finSt_lb = CarPhysicalState((fin_x_lb, fin_y_lb, AVG_HWY_VELOCITY - 0.5))
  finSt_ub = CarPhysicalState((fin_x_ub, fin_y_ub, AVG_HWY_VELOCITY + 0.5))

  llMDP = LowLevelMDP(ll_discount, ll_TIME_STEP, ll_HORIZON, rs, initSt, (finSt_lb, finSt_ub),
                          ll_goalReward, ll_collisionCost, ll_y_dev_cost, ll_hardbrakingCost, ll_discomfortCost, ll_velocityDeviationCost, getFrameList())

  #Generate initial global state for llMDP
  #ego state is initSt
  neighborhood = Array{Array{CarLocalIS,1}}(numLanes)

  for i in 1:numLanes
    neighborhood[i] = Array{CarLocalIS,1}()
  end
  for ln in 1:numLanes
    if ln == egoLane + 1      #left lane: use occ_lt
      occ_lt = [ulInitState.occ_l...]
      for idx in length(occ_lt):-1:1
        present = occ_lt[idx]
        if present
          init_x = (egoDist - 1 + idx-2) * cell_length + rand(rng) * cell_length  #mean of initial x
          init_y = egoLane * LANE_WIDTH + LANE_WIDTH/2.0   #mean of initial y
          init_xdot = AVG_HWY_VELOCITY
          carProbDensity = NTuple{3,NormalDist}((NormalDist(init_x, 0.1), NormalDist(init_y, LANE_WIDTH/6.0), NormalDist(init_xdot, VEL_STD_DEV)))
          intentionArray = zeros(Float64, numLanes)
          intentionArray[ln] = 0.7
          intentionArray[ln-1] = 0.3
          carState = randCarLocalISL0(rng, carProbDensity, intentionArray, gen.frameList)
          push!(neighborhood[ln], carState)
        end
      end

    elseif ln == egoLane  #same lane: use occ_rt
      ld_dist = ulInitState.ld_distance
      #Ensure that the initial state is not a collison state
      while true
        init_x = (egoDist - 1 + ld_dist + rand(rng)) * cell_length
        abs(init_x - initSt.state[1]) > 2 * CAR_LENGTH && break
      end
      init_y = (2 * egoLane - 1) * LANE_WIDTH/2
      init_xdot = AVG_HWY_VELOCITY
      carProbDensity = NTuple{3,NormalDist}((NormalDist(init_x, 0.1), NormalDist(init_y, LANE_WIDTH/6.0), NormalDist(init_xdot, VEL_STD_DEV)))
      intentionArray = zeros(Float64, numLanes)
      ln - 1 > 0 ? intentionArray[ln-1] += 0.3 : intentionArray[ln] += 0.3
      intentionArray[ln] += 0.4
      ln + 1 <= numLanes ? intentionArray[ln+1] += 0.3 : intentionArray[ln] += 0.3
      carState = randCarLocalISL0(rng, carProbDensity, intentionArray, gen.frameList)
      push!(neighborhood[ln], carState)

    elseif ln == egoLane - 1  # right lane: use ld_distance
      occ_rt = [ulInitState.occ_r...]
      for idx in length(occ_rt):-1:1
        present = occ_rt[idx]
        if present
          init_x = (egoDist - 1 + idx-2) * cell_length + rand(rng) * cell_length  #mean of initial x
          init_y = egoLane * LANE_WIDTH - LANE_WIDTH/2.0   #mean of initial y
          init_xdot = AVG_HWY_VELOCITY
          carProbDensity = NTuple{3,NormalDist}((NormalDist(init_x, 0.1), NormalDist(init_y, LANE_WIDTH/6.0), NormalDist(init_xdot, VEL_STD_DEV)))
          intentionArray = zeros(Float64, numLanes)
          intentionArray[ln] = 0.7
          intentionArray[ln+1] = 0.3
          carState = randCarLocalISL0(rng, carProbDensity, intentionArray, gen.frameList)
          push!(neighborhood[ln], carState)
        end
      end
    else            # other lanes: random population or empty
      nothing
    end
  end
  initGblSt = GlobalStateL1(0, initSt, neighborhood)
  return (llMDP, initGblSt)
end

type DummyGBLStLLMDP <: POMDPs.MDP{GlobalStateL1, Int64}
  llMDP::LowLevelMDP
  gblSt::GlobalStateL1
end

function DummyGBLStLLMDP(gen::SingleAgentOccGridMDP_TGenerator, ulInitState::ImmGridOccSt, a::Int64, rng::AbstractRNG)
  llMDP, gblSt = initialize_LowLevelMDP_gblSt(gen, ulInitState, a, rng)
  return DummyGBLStLLMDP(llMDP, gblSt)
end

function initial_state(p::DummyGBLStLLMDP, rng::AbstractRNG)
  return p.gblSt
end

function reward(p::DummyGBLStLLMDP, s::GlobalStateL1, a::Int, rng::AbstractRNG)
  return reward(p.llMDP, s, a, rng)
end

function generate_s(p::DummyGBLStLLMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  return generate_s(p.llMDP, s, a, rng)
end

function generate_sr(p::DummyGBLStLLMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  return generate_sr(p.llMDP, s, a, rng)
end

function generate_sprime_ego(gen::SingleAgentOccGridMDP_TGenerator, ulInitState::ImmGridOccSt, a::Int64, solver::DPWSolver, rng::AbstractRNG)
  mdp_st = initialize_LowLevelMDP_gblSt(gen, ulInitState)
  p = mdp_st[1]

  policy1 = solve(solver, p)
  hr1 = HistoryRecorder(max_steps = p.llMDP.HORIZON, rng = rng)
  hist1 = simulate(hr1, p, policy1);
  finEgoState = hist1[end].ego
  return getCarGridLocation(p.ulMDP, finEgoState)
end
