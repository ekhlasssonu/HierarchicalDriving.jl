using DiscreteValueIteration

using JLD


abstract HierarchicalFramework

type HierarchicalFramework1 <: HierarchicalFramework
  sim_mdp::SimulationMDP
  high_mdp::SingleAgentGridMDP
  policy::ValueIterationPolicy
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
  planningTime::Float64
end

HierarchicalPolicy1(hf1::HierarchicalFramework1) =  HierarchicalPolicy1(hf1, Nullable{AgentGridLocation}(), 0, 0, MersenneTwister(982019), 0.0)
HierarchicalPolicy1(hf1::HierarchicalFramework1, rng::AbstractRNG) =  HierarchicalPolicy1(hf1, Nullable{AgentGridLocation}(), 0, 0, rng, 0.0)

function action(hp1::HierarchicalPolicy1, gblSt::GlobalStateL1)
  #println("Step $(hp1.t)")
  sim_mdp = hp1.hf1.sim_mdp
  upper_level_mdp = hp1.hf1.high_mdp
  upper_level_policy = hp1.hf1.policy

  #Extract LowLevelMDP state with immediate neighbors
  low_level_gbl_st = calcLowLevelGblSt(gblSt, sim_mdp)
  #println("LowLevelGlobalState: ")
  #printGlobalPhyState(low_level_gbl_st, sim_mdp.roadSegment)

  t1 = time_ns()
  egoState = gblSt.ego
  #print("\t egoState: $(egoState.state) \t")

  if isnull(hp1.upper_level_state)
    #Extract upper level state
    hp1.upper_level_state = getCarGridLocation(upper_level_mdp, egoState)
    #Get macro action from upper_level_policy
    upper_level_state_idx = state_index(upper_level_mdp, get(hp1.upper_level_state))
    hp1.macro_action_idx = upper_level_policy.policy[upper_level_state_idx]
  end
  upper_level_state = get(hp1.upper_level_state)
  macro_action_idx = hp1.macro_action_idx
  macro_action = upper_level_policy.action_map[macro_action_idx]

  y_dev_cost = -2.0
  if macro_action != 0
    y_dev_cost = 0.0
  end
  #print("macro_action: $macro_action \t")

  # Define low_level_mdp with goal and time step
  egoLane = upper_level_state.lane
  numLanes = n_lanes(sim_mdp)
  targetLane = clamp(egoLane + macro_action, 0, numLanes)
  target_y = getLaneCenter(sim_mdp.roadSegment, targetLane)
  #print("targetLane = $targetLane, target_y = $target_y \t")
  target_distance = upper_level_state.distance + 1
  target_x_lb, target_x_ub =  get_x_bounds(upper_level_mdp, target_distance)

  vel_lb = sim_mdp.egoTargetState[1].state[3]
  vel_ub = sim_mdp.egoTargetState[2].state[3]

  low_level_mdp = LowLevelMDP(ll_discount, ll_TIME_STEP, ll_HORIZON, sim_mdp.roadSegment, egoState,
                                (CarPhysicalState((target_x_lb, target_y - 0.5, vel_lb)),
                                 CarPhysicalState((sim_mdp.roadSegment.x_boundary[2], target_y + 0.5, vel_ub))),
                                ll_goalReward, ll_collisionCost, y_dev_cost, ll_hardbrakingCost, ll_discomfortCost, ll_velocityDeviationCost, getFrameList())


  if checkForCollision(low_level_gbl_st, low_level_mdp)
    println("Collision Occured!!!")
    low_level_gbl_st.terminal = 1
    gblSt.terminal = 1
    return 0
  end
  #print("target_x = [$(low_level_mdp.egoTargetState[1].state[1]), $(low_level_mdp.egoTargetState[2].state[1])]")
  if checkTargetCoordinates(low_level_gbl_st, low_level_mdp)
    #println("LowLevel Success: ")
    low_level_gbl_st.terminal = 2
  end

  if (low_level_gbl_st.terminal == 1 || low_level_gbl_st.terminal == 2 || hp1.t >= ul_TIME_STEP/low_level_mdp.TIME_STEP)
    #println("**********5**********")
    hp1.t = 0
    hp1.upper_level_state =  Nullable{AgentGridLocation}()
    hp1.macro_action_idx = 0
    return 0
  end
  # Solve LowLevelMDP to get elementary action
  low_level_solver = DPWSolver(depth=low_level_mdp.HORIZON,
                 exploration_constant=10.0,
                 n_iterations=1_500,
                 k_action=10.0,
                 alpha_action=1/10,
                 k_state=5.0,
                 alpha_state=1/10,
                 estimate_value=RolloutEstimator(subintentional_lowlevel_policy(low_level_mdp))
                )
  low_level_policy = solve(low_level_solver, low_level_mdp)
  #low_level_hr = HistoryRecorder(max_steps = low_level_mdp.HORIZON, rng = hp1.rng)

  #hist1 = simulate(low_level_hr, low_level_mdp, low_level_policy)
  #act = action_hist(hist1)[1]
  act = action(low_level_policy, low_level_gbl_st)

  hp1.t += 1
  t2 = time_ns()
  #println("act = $(EgoActionSpace().actions[act])")
  hp1.planningTime += (t2 - t1)/1.0e9
  return act
end

type HierarchicalFramework2 <: HierarchicalFramework
  sim_mdp::SimulationMDP
  high_mdp::SingleAgentOccGridMDP
  policy::ValueIterationPolicy
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
  rng::AbstractRNG
  planningTime::Float64
end

HierarchicalPolicy2(hf2::HierarchicalFramework2, rng::AbstractRNG = MersenneTwister(982019)) =  HierarchicalPolicy2(hf2, Nullable{ImmGridOccSt}(), 0, 0, rng, 0.0)

function action(hp2::HierarchicalPolicy2, gblSt::GlobalStateL1)
  #println("Step $(hp1.t)")
  sim_mdp = hp2.hf2.sim_mdp
  upper_level_mdp = hp2.hf2.high_mdp
  upper_level_policy = hp2.hf2.policy

  #Extract LowLevelMDP state with immediate neighbors
  low_level_gbl_st = calcLowLevelGblSt(gblSt, sim_mdp)

  t1 = time_ns()
  egoState = gblSt.ego

  if isnull(hp2.upper_level_state)
    hp2.upper_level_state = extractImmGridOccSt(upper_level_mdp, gblSt)
    upper_level_state_idx = state_index(upper_level_mdp, get(hp2.upper_level_state))
    hp2.macro_action_idx = upper_level_policy.policy[upper_level_state_idx]
  end
  #Get macro action from upper_level_policy
  upper_level_state = get(hp2.upper_level_state)
  macro_action_idx = hp2.macro_action_idx
  macro_action = upper_level_policy.action_map[macro_action_idx]

  #println("Current Ego Car Loc:", upper_level_state.egoGrid)
  #printGlobalPhyState(low_level_gbl_st, sim_mdp.roadSegment)
  #println("Macro action:", macro_action)

  y_dev_cost = -2.0
  if macro_action != 0
    y_dev_cost = 0.0
  end

  # Define low_level_mdp with goal and time step
  egoLane = upper_level_state.egoGrid.lane
  numLanes = n_lanes(sim_mdp)
  targetLane = clamp(egoLane + macro_action, 0, numLanes)
  target_y = getLaneCenter(sim_mdp.roadSegment, targetLane)

  target_distance = upper_level_state.egoGrid.distance + 2
  if target_distance > upper_level_mdp.n_v_cells
    target_distance = upper_level_mdp.n_v_cells
  end
  dummy, target_x_ub =  get_x_bounds(upper_level_mdp, target_distance)
  dummy, target_x_lb =  get_x_bounds(upper_level_mdp, upper_level_state.egoGrid.distance)

  vel_lb = sim_mdp.egoTargetState[1].state[3]
  vel_ub = sim_mdp.egoTargetState[2].state[3]

  low_level_mdp = LowLevelMDP(ll_discount, ll_TIME_STEP, ll_HORIZON, sim_mdp.roadSegment, egoState,
                                (CarPhysicalState((target_x_lb, target_y - 0.5, vel_lb)),
                                 CarPhysicalState((sim_mdp.roadSegment.x_boundary[2], target_y + 0.5, vel_ub))),
                                ll_goalReward, ll_collisionCost, y_dev_cost, ll_hardbrakingCost, ll_discomfortCost, ll_velocityDeviationCost, getFrameList())

  if checkForCollision(low_level_gbl_st, low_level_mdp)
    println("Collision Occured!!!")
    low_level_gbl_st.terminal = 1
    gblSt.terminal = 1
    return 0
  end
  if checkTargetCoordinates(low_level_gbl_st, low_level_mdp)
    low_level_gbl_st.terminal = 2
  end

  if (low_level_gbl_st.terminal == 1 || low_level_gbl_st.terminal == 2 || hp2.t >= ul_TIME_STEP/low_level_mdp.TIME_STEP)
    hp2.t = 0
    hp2.upper_level_state =  Nullable{ImmGridOccSt}()
    hp2.macro_action_idx = 0
    return 0
  end
  # Solve LowLevelMDP to get elementary action
  low_level_solver = DPWSolver(depth=low_level_mdp.HORIZON,
                 exploration_constant=10.0,
                 n_iterations=1_500,
                 k_action=10.0,
                 alpha_action=1/10,
                 k_state=5.0,
                 alpha_state=1/10,
                 estimate_value=RolloutEstimator(subintentional_lowlevel_policy(low_level_mdp))
                )
  low_level_policy = solve(low_level_solver, low_level_mdp)
  #low_level_hr = HistoryRecorder(max_steps = low_level_mdp.HORIZON, rng = hp2.rng)
  #hist1 = simulate(low_level_hr, low_level_mdp, low_level_policy)
  #act = action_hist(hist1)[1]
  act = action(low_level_policy, low_level_gbl_st)

  hp2.t += 1
  t2 = time_ns()
  hp2.planningTime += (t2 - t1)/1.0e9
  return act
end
