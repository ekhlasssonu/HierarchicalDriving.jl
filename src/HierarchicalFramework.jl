using DiscreteValueIteration

using JLD


abstract HierarchicalFramework

type HierarchicalFramework1 <: HierarchicalFramework
  sim_mdp::SimulationMDP
  high_mdp::SingleAgentGridMDP
  policy::ValueIterationPolicy
end

function HierarchicalFramework1(sim::SimulationMDP, n_v_cells::Int64, u_goalReward::Float64, u_p_next_cell::Float64, u_p_desired_lane::Float64)
  roadSegment = sim.roadSegment
  goalUB = sim.egoTargetState[2]
  cellLength = length(roadSegment)/n_v_cells

  x_ub = goalUB.state[1]
  y_ub = goalUB.state[2]
  lane_ub = getLaneNo(y_ub, roadSegment)
  x_ub_offset = x_ub - roadSegment.x_boundary[1]
  distance_ub = convert(Int64, ceil(x_ub_offset/cellLength))

  goalCell = AgentGridLocation(lane_ub, distance_ub)
  u_discount = sim.discount_factor^ll_HORIZON

  up = SingleAgentGridMDP(roadSegment, n_v_cells, goalCell, u_goalReward, u_p_next_cell, u_p_desired_lane, u_discount)
  return HierarchicalFramework1(sim, up)
end
function HierarchicalFramework1(sim::SimulationMDP, policyFileName::String)
  policy = load(policyFileName, "policy")
  up = policy.mdp
  @assert up.roadSegment == sim.roadSegment
  return HierarchicalFramework1(sim, up, policy)
end

type HierarchicalPolicy1 <: Policy
  hf1::HierarchicalFramework1
  upper_level_state::Nullable{AgentGridLocation}
  macro_action_idx::Int64
  t::Int64
  rng::AbstractRNG
end

HierarchicalPolicy1(hf1::HierarchicalFramework1) =  HierarchicalPolicy1(hf1, Nullable{AgentGridLocation}(), 0, 0, MersenneTwister(982019))
HierarchicalPolicy1(hf1::HierarchicalFramework1, rng::AbstractRNG) =  HierarchicalPolicy1(hf1, Nullable{AgentGridLocation}(), 0, 0, rng)

function action(hp1::HierarchicalPolicy1, gblSt::GlobalStateL1)
  sim_mdp = hp1.hf1.sim_mdp
  upper_level_mdp = hp1.hf1.high_mdp
  upper_level_policy = hp1.hf1.policy

  println("1")
  #Extract LowLevelMDP state with immediate neighbors
  low_level_gbl_st = calcLowLevelGblSt(gblSt, sim_mdp)
  println("2")

  egoState = gblSt.ego

  if isnull(hp1.upper_level_state)
    println("3")
    #Extract upper level state
    hp1.upper_level_state = getCarGridLocation(upper_level_mdp, egoState)
    #Get macro action from upper_level_policy
    upper_level_state_idx = state_index(upper_level_mdp, get(hp1.upper_level_state))
    hp1.macro_action_idx = upper_level_policy.policy[upper_level_state_idx]
  end
  println("4")
  upper_level_state = get(hp1.upper_level_state)
  macro_action_idx = hp1.macro_action_idx
  macro_action = upper_level_policy.action_map[macro_action_idx]

  # Define low_level_mdp with goal and time step
  println("5")
  egoLane = upper_level_state.lane
  numLanes = n_lanes(sim_mdp)
  targetLane = clamp(egoLane + macro_action, 0, numLanes)
  target_y = getLaneCenter(sim_mdp.roadSegment, targetLane)
  target_distance = upper_level_state.distance + 1
  target_x_lb, target_x_ub =  get_x_bounds(upper_level_mdp, target_distance)

  low_level_mdp = LowLevelMDP(ll_discount, ll_TIME_STEP, ll_HORIZON, sim_mdp.roadSegment, egoState,
                                (CarPhysicalState((target_x_lb, target_y - 0.5, AVG_HWY_VELOCITY - 2.5)),
                                 CarPhysicalState((target_x_ub, target_y + 0.5, AVG_HWY_VELOCITY + 2.5))),
                                ll_goalReward, ll_collisionCost, ll_y_dev_cost, ll_hardbrakingCost, ll_discomfortCost, ll_velocityDeviationCost, getFrameList())

  println("6")
  if checkForCollision(low_level_gbl_st, low_level_mdp)
    low_level_gbl_st.terminal = 1
  end
  if checkTargetCoordinates(low_level_gbl_st, low_level_mdp)
    low_level_gbl_st.terminal = 2
  end

  println("7")
  if isterminal(low_level_gbl_st, low_level_mdp) || hp1.t == low_level_mdp.HORIZON
    println("8")
    hp1.t = 0
    hp1.upper_level_state =  Nullable{AgentGridLocation}()
    hp1.macro_action_idx = 0
    return 0
  end
  println("9")
  # Solve LowLevelMDP to get elementary action
  low_level_solver = DPWSolver(depth=low_level_mdp.HORIZON,
                 exploration_constant=10.0,
                 n_iterations=1_000,
                 k_action=10.0,
                 alpha_action=1/10,
                 k_state=5.0,
                 alpha_state=1/10,
                 estimate_value=RolloutEstimator(subintentional_lowlevel_policy(low_level_mdp))
                )
  low_level_policy = solve(low_level_solver, low_level_mdp)
  low_level_hr = HistoryRecorder(max_steps = low_level_mdp.HORIZON, rng = hp1.rng)
  println("10")

  #hist1 = simulate(low_level_hr, low_level_mdp, low_level_policy)
  #act = action_hist(hist1)[1]
  act = action(low_level_policy, low_level_gbl_st)
  println("11")

  hp1.t += 1

  return act
end




type HierarchicalFramework2 <: HierarchicalFramework
  sim_mdp::SimulationMDP
  high_mdp::SingleAgentOccGridMDP
  policy::ValueIterationPolicy
end

function HierarchicalFramework2(sim::SimulationMDP, n_v_cells::Int64, goal_reward::Float64, tranFileName::String, rwdFileName::String, policyFileName::String)
  roadSegment = sim.roadSegment
  cellLength = length(roadSegment)/n_v_cells
  goalLB = sim.egoTargetState[1]
  goalUB = sim.egoTargetState[2]
  x_lb = goalLB.state[1]
  y_lb = goalLB.state[2]
  lane_lb = getLaneNo(y_lb, roadSegment)
  x_lb_offset = x_lb - roadSegment.x_boundary[1]
  distance_lb = convert(Int64, ceil(x_lb_offset/cellLength))
  x_ub = goalUB.state[1]
  y_ub = goalUB.state[2]
  lane_ub = getLaneNo(y_ub, roadSegment)
  x_ub_offset = x_ub - roadSegment.x_boundary[1]
  distance_ub = convert(Int64, ceil(x_ub_offset/cellLength))
  cellLength = length(roadSegment)/n_v_cells

  goalCells = Array(AgentGridLocation, (distance_ub - distance_lb + 1)*(lane_ub - lane_lb + 1))
  idx = 1
  for ln in lane_lb:lane_ub
    for dist in distance_lb:distance_ub
      goalCells[idx] = AgentGridLocation(ln, dist)
      idx += 1
    end
  end

  up = SingleAgentOccGridMDP(sim.n_agents, roadSegment, n_v_cells, goalCells, goal_reward, tranFileName, rwdFileName)

  policy = load(policyFileName, "policy")
  @assert up.roadSegment == sim.roadSegment "Mismatching road segments"
  return HierarchicalFramework2(sim, up, policy)
end

function HierarchicalFramework2(sim::SimulationMDP, policyFileName::String)
  policy = load(policyFileName, "policy")
  up = policy.mdp
  @assert up.roadSegment == sim.roadSegment
  return HierarchicalFramework2(sim, up, policy)
end

type HierarchicalPolicy2 <: Policy
  hf2::HierarchicalFramework2
  upper_level_state::Nullable{ImmGridOccSt}
  macro_action_idx::Int64
  t::Int64
  rng::MersenneTwister
  updateHorizon::Bool
end

HierarchicalPolicy2(hf2::HierarchicalFramework2) =  HierarchicalPolicy2(hf2, Nullable{ImmGridOccSt}(), 0, 0, MersenneTwister(982019), false)

function action(hp2::HierarchicalPolicy2, gblSt::GlobalStateL1)
  if isterminal(hp2.hf2.sim_mdp, gblSt) #Either overall success or collision, nothing to do
    return 0
  end

  sim_mdp = hp2.hf2.sim_mdp
  upper_level_mdp = hp2.hf2.high_mdp
  upper_level_policy = hp2.hf2.policy

  println("1")
  #Extract LowLevelMDP state with immediate neighbors
  low_level_gbl_st = calcLowLevelGblSt(gblSt, sim_mdp)
  println("2")
  egoState = gblSt.ego
  if isnull(hp2.upper_level_state)
    println("3")
    # Extract upper level state
    #TODO: Check if bug exists
    hp2.upper_level_state = extractImmGridOccSt(upper_level_mdp, gblSt)
    upper_level_state_idx = state_index(upper_level_mdp, get(hp2.upper_level_state))
    hp2.macro_action_idx = upper_level_policy.policy[upper_level_state_idx]
  end
  println("4")
  #Get macro action from upper_level_policy
  upper_level_state = get(hp2.upper_level_state)
  macro_action_idx = hp2.macro_action_idx
  macro_action = upper_level_policy.action_map[macro_action_idx]

  # Define low_level_mdp with goal and time step
  egoLane = upper_level_state.egoGrid.lane
  numLanes = n_lanes(sim_mdp)
  targetLane = clamp(egoLane + macro_action, 0, numLanes)
  target_y = getLaneCenter(sim_mdp.roadSegment, targetLane)
  println("5")

  target_distance = upper_level_state.egoGrid.distance + 4
  if target_distance > upper_level_mdp.n_v_cells
    target_distance = upper_level_mdp.n_v_cells
  end
  dummy, target_x_ub =  get_x_bounds(upper_level_mdp, target_distance)
  dummy, target_x_lb =  get_x_bounds(upper_level_mdp, upper_level_state.egoGrid.distance)
  println("6")

  low_level_mdp = LowLevelMDP(ll_discount, ll_TIME_STEP, ll_HORIZON, sim_mdp.roadSegment, egoState,
                                (CarPhysicalState((target_x_lb, target_y - 0.5, AVG_HWY_VELOCITY - 2.5)),
                                 CarPhysicalState((target_x_ub, target_y + 0.5, AVG_HWY_VELOCITY + 2.5))),
                                ll_goalReward, ll_collisionCost, ll_y_dev_cost, ll_hardbrakingCost, ll_discomfortCost, ll_velocityDeviationCost, getFrameList())

  if checkForCollision(low_level_gbl_st, low_level_mdp)
    low_level_gbl_st.terminal = 1
  end
  if checkTargetCoordinates(low_level_gbl_st, low_level_mdp)
    low_level_gbl_st.terminal = 2
  end
  println("7")

  if isterminal(low_level_gbl_st, low_level_mdp) || hp2.t == low_level_mdp.HORIZON
    println("8")
    hp2.t = 0
    hp2.upper_level_state =  Nullable{AgentGridLocation}()
    hp2.macro_action_idx = 0
    return 0
  end
  println("9")
  #TODO: Solve LowLevelMDP to get elementary action
  low_level_solver = DPWSolver(depth=low_level_mdp.HORIZON,
                 exploration_constant=10.0,
                 n_iterations=1_000,
                 k_action=10.0,
                 alpha_action=1/10,
                 k_state=5.0,
                 alpha_state=1/10,
                 estimate_value=RolloutEstimator(subintentional_lowlevel_policy(low_level_mdp))
                )
  println("10")
  low_level_policy = solve(low_level_solver, low_level_mdp)
  low_level_hr = HistoryRecorder(max_steps = low_level_mdp.HORIZON, rng = hp2.rng)
  #hist1 = simulate(low_level_hr, low_level_mdp, low_level_policy)
  #act = action_hist(hist1)[1]
  act = action(low_level_policy, low_level_gbl_st)
  println("11")

  hp2.t += 1

  return act
end
