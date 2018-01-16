abstract type DiscreteActionDrivingMDP <: POMDPs.MDP{GlobalStateL1, Int64} end
type LowLevelMDP <: DiscreteActionDrivingMDP
  discount_factor::Float64
  TIME_STEP::Float64
  SIM_TIME_STEP::Float64
  COLLISION_CUSHION::Float64
  HORIZON::Int64
  roadSegment::RoadSegment #numLanes + 1 values
  egoStartState::CarPhysicalState
  egoTargetState::NTuple{2, CarPhysicalState} #Two values with lb and ub on target physical state
  goalReward::Float64
  collisionCost::Float64
  steeringCost::Float64
  hardbrakingCost::Float64
  discomfortCost::Float64
  velocityDeviationCost::Float64
  frameList::Array{LowLevelCarFrameL0,1}
end

#=
LowLevelMDP() = LowLevelMDP(0.9, 0.2, 20,
                            [0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH],
                            CarPhysicalState((0.0, 3.0 * LANE_WIDTH/2.0, AVG_HWY_VELOCITY)),
                            (CarPhysicalState((0.0, 5.0 * LANE_WIDTH/2.0 - 0.5, AVG_HWY_VELOCITY - 0.5)), CarPhysicalState((500.0, 5.0 * LANE_WIDTH/2.0 + 0.5, AVG_HWY_VELOCITY + 0.5))),
                            5.0, -50.0, -1.0, -3.0, -2.0, -0.5, getFrameList())
=#
LowLevelMDP() = LowLevelMDP(ll_discount, ll_TIME_STEP, sim_TIME_STEP, COLLISION_CUSHION, ll_HORIZON,
                            RoadSegment((-100.0, 500.0),[0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH]),
                            CarPhysicalState((0.0, 1.0 * LANE_WIDTH/2.0, AVG_HWY_VELOCITY)),
                            (CarPhysicalState((10.0, 3.0 * LANE_WIDTH/2.0 - 0.5, AVG_HWY_VELOCITY - 0.5)),
                             CarPhysicalState((100.0, 3.0 * LANE_WIDTH/2.0 + 0.5, AVG_HWY_VELOCITY + 0.5))),
                            ll_goalReward, ll_collisionCost, ll_y_dev_cost, ll_hardbrakingCost, ll_discomfortCost, ll_velocityDeviationCost, getFrameList())

discount(p::LowLevelMDP) = p.discount_factor

#function isterminal(p::LowLevelMDP, st::GlobalStateL1{CarLocalIS{ParamCarModelL0}})
#  st.terminal > 0 ? true : false
#end
function isterminal(p::LowLevelMDP, st::GlobalStateL1)
  st.terminal == 1 ? true : false #For planning purpose, keep only collision as termination criteria
end
function isterminal(st::GlobalStateL1)
  st.terminal == 1 ? true : false
end
#From actions
n_actions(p::LowLevelMDP) = length(actions(p))
actions(::LowLevelMDP) = EgoActionSpace()

n_lanes(p::LowLevelMDP) = n_lanes(p.roadSegment)

function getLaneNo(y::Float64, p::LowLevelMDP)
  return getLaneNo(y, p.roadSegment)
end

function getLaneNo(phySt::CarPhysicalState, p::LowLevelMDP)
  y = phySt.state[2]
  return getLaneNo(y,p)
end

function getLaneCenter(phySt::CarPhysicalState, p::LowLevelMDP)
  laneNo = getLaneNo(phySt, p)
  return getLaneCenter(p.roadSegment, laneNo)
end

function printState(p::LowLevelMDP, s::GlobalStateL1)
  egoState = s.ego
  egoLane = getLaneNo(egoState, p)
  for ln in 1:length(s.neighborhood)
    print("Lane No.: $ln ")
    carNo = 1
    for carIS in s.neighborhood[ln]
      if (ln == egoLane) && (carIS.physicalState.state[1] < egoState.state[1])
        print("Ego:",egoState.state, "\t")
      end
      print("carNo $carNo ",carIS.physicalState.state,"\t")
      carNo += 1
    end
    println()
  end
end
function checkForCollision(gblISL1::GlobalStateL1, p::LowLevelMDP, safety_dist::Float64=0.0)
  egoState = gblISL1.ego
  y = egoState.state[2]
  if y > p.roadSegment.laneMarkings[end] || y < p.roadSegment.laneMarkings[1]
    #println("OOB Error: y = $(gblISL1.ego.state)\t\t")
    return true
  end
  nbrhood = gblISL1.neighborhood
  numLanes = length(nbrhood)
  for ln in 1:numLanes
    for carIS in nbrhood[ln]
      if collision(egoState, carIS, safety_dist)
        return true
      end
    end
  end
  return false
end

function checkForCollision(curr_gblIS::GlobalStateL1, a::CarAction, nbr_a::Array{Array{CarAction,1},1}, next_gblIS::GlobalStateL1, p::LowLevelMDP, rng::AbstractRNG, safety_dist::Float64=0.0)
    curr_egoState = curr_gblIS.ego
    y = curr_egoState.state[2]
    if y > p.roadSegment.laneMarkings[end] || y < p.roadSegment.laneMarkings[1]
      #println("OOB Error: y = $(gblISL1.ego.state)\t\t")
      return true
    end
    next_egoState = next_gblIS.ego
    y = next_egoState.state[2]
    if y > p.roadSegment.laneMarkings[end] || y < p.roadSegment.laneMarkings[1]
      #println("OOB Error: y = $(gblISL1.ego.state)\t\t")
      return true
    end
  curr_nbrs_st = curr_gblIS.neighborhood
  next_nbrs_st = next_gblIS.neighborhood
  #For each agent check if collision happens with that agent during the transition
  for ln in 1:length(n_lanes(p))
    for oa_idx in 1:length(curr_nbrs_st[ln])
      curr_ego_st = CarPhysicalState(curr_gblIS.ego)
      next_ego_st = CarPhysicalState(next_gblIS.ego)

      curr_oa_st = curr_nbrs_st[ln][oa_idx].physicalState
      next_oa_st = next_nbrs_st[ln][oa_idx].physicalState
      oa_act = nbr_a[ln][oa_idx]
      #Easy elimination criteria
      #if the car is far off in the same direction both in start and end state, skip
      if abs(curr_ego_st.state[1] - curr_oa_st.state[1]) > 1.5 * CAR_LENGTH && abs(next_ego_st.state[1] - next_oa_st.state[1]) > 1.5 * CAR_LENGTH && (curr_ego_st.state[1] - curr_oa_st.state[1])/(next_ego_st.state[1] - next_oa_st.state[1]) > 0.0
        continue
      end
      #if same thing on the y axis
      if abs(curr_ego_st.state[2] - curr_oa_st.state[2]) > 1.5 * CAR_WIDTH && abs(next_ego_st.state[2] - next_oa_st.state[2]) > 1.5 * CAR_WIDTH && (curr_ego_st.state[2] - curr_oa_st.state[2])/(next_ego_st.state[2] - next_oa_st.state[2]) > 0.0
        continue
      end
      time_remaining = p.TIME_STEP
      #print("~")
      while time_remaining > 0
        #propagate both cars
        #NOTE: Should I worry about rng? Probably not if safety_dist is large enough
        curr_ego_st = propagateCar(curr_ego_st, a, p.SIM_TIME_STEP, rng, (TRN_NOISE_X, TRN_NOISE_Y, TRN_NOISE_XDOT))
        curr_oa_st = propagateCar(curr_oa_st, oa_act, p.SIM_TIME_STEP, rng, (TRN_NOISE_X, TRN_NOISE_Y, TRN_NOISE_XDOT))

        #check collision
        if collision(curr_ego_st, curr_oa_st, safety_dist)
          #print("l.")
          return true
        end

        time_remaining -= p.SIM_TIME_STEP
      end
    end
  end
  return false
end

function checkTargetCoordinates(gblISL1::GlobalStateL1, p::LowLevelMDP)
  egoState = gblISL1.ego
  x = egoState.state[1]
  y = egoState.state[2]
  xdot = egoState.state[3]
  targetLB = p.egoTargetState[1]
  targetUB = p.egoTargetState[2]

  if (targetLB.state[1] < x) && (x < targetUB.state[1]) && (targetLB.state[2] < y) && (y < targetUB.state[2])
    return true
  end
  return false
end

function sortbyx(cars_in_the_lane::Array{CarLocalIS,1})
  sorted = Array{CarLocalIS,1}()
  for carIS in cars_in_the_lane
    carPhySt = carIS.physicalState
    x = carPhySt.state[1]
    y = carPhySt.state[2]

    numCarsAhead = 0
    while numCarsAhead < length(sorted)
      otherCarIS = sorted[numCarsAhead+1]
      otherCarPhySt = otherCarIS.physicalState
      xp = otherCarPhySt.state[1]
      if xp < x
        break
      end
      numCarsAhead += 1
    end
    if numCarsAhead == length(sorted)
      push!(sorted, carIS)
    else
      temp = splice!(sorted, numCarsAhead+1:length(sorted))
      push!(sorted, carIS)
      append!(sorted,temp)
    end
  end
  return sorted
end
function sortintolanes(neighborhood::Array{Array{CarLocalIS,1},1}, p::LowLevelMDP)
  numLanes = length(neighborhood)
  sorted = Array{Array{CarLocalIS,1},1}(numLanes)

  for ln in 1:numLanes
    sorted[ln] = Array{CarLocalIS,1}()
  end
  for ln in 1:numLanes
    for carIS in neighborhood[ln]
      carPhySt = carIS.physicalState
      carLane = getLaneNo(carPhySt, p)
      if carLane < 1
        carLane = 1
        #println("carLane = 0")
      elseif carLane > numLanes
        carLane = numLanes
        #println("carLane > numLanes")
      end

      push!(sorted[carLane],carIS)
    end
  end
  return sorted
end

function sortNeighborhood(neighborhood::Array{Array{CarLocalIS,1},1}, p::LowLevelMDP)
  numLanes = length(neighborhood)
  if numLanes != n_lanes(p)
    println("[sortNeighborhood] Incorrect number of lanes.")
  end

  sorted = sortintolanes(neighborhood, p)

  for ln in 1:numLanes
    sorted[ln] = sortbyx(sorted[ln])
  end
  return sorted
end

function check_induced_hardbraking(globalISL1::GlobalStateL1, p::LowLevelMDP)
  laneCenters = getLaneCenters(p.roadSegment)

  egoState = globalISL1.ego
  egoLane = getLaneNo(egoState, p)
  numLanes = length(globalISL1.neighborhood)
  if numLanes != n_lanes(p)
    println("[check_induced_hardbraking] Incorrect numLanes. ")
  end

  for carIS in globalISL1.neighborhood[egoLane]
    carPhySt = carIS.physicalState
    if carPhySt.state[1] > egoState.state[1]  #Not interested in cars ahead of the ego vehicle
      continue
    end
    carModel = carIS.model
    x = carPhySt.state[1]
    xdot = carPhySt.state[3]
    dxdot = xdot - egoState.state[3]
    g = egoState.state[1] - x - CAR_LENGTH

    ddotx = get_idm_accln(carIS.model.frame.longitudinal, xdot, dxdot, g)
    if ddotx <= -6.0
      return true
    end
    break # Only interested in car immediately behind ego car.
  end

  return false
end

function updateNeighborState(globalISL1::GlobalStateL1, p::LowLevelMDP, rng::AbstractRNG)
  laneCenters = getLaneCenters(p.roadSegment)

  egoState = globalISL1.ego
  egoLane = getLaneNo(egoState,p)
  numLanes = length(globalISL1.neighborhood)
  if numLanes != n_lanes(p)
    println("[updateNeighborState] Incorrect numLanes. ")
  end
  if egoLane < 1 || egoLane > numLanes
    println("[updateNeighborState] Incorrect egoLane index, $egoLane. $egoState, y = ", egoState.state[2])
  end
  updatedNeighborhood = Array{Array{CarLocalIS,1},1}(numLanes)
  nbr_acts = Array{Array{CarAction,1},1}(numLanes)

  for ln in 1:numLanes #For every lane
    numCars = length(globalISL1.neighborhood[ln])
    updatedNeighborhood[ln] = Array{CarLocalIS,1}()
    nbr_acts[ln] = Array{CarAction,1}(numCars)
    ldCarIS = Nullable{CarLocalIS}()
    for carIdx in 1:numCars #For every car in the lane
      carIS = globalISL1.neighborhood[ln][carIdx] #Initial IS of the current car under consideration
      #carAct = actions[ln][carIdx] #action performed by current car

      carPhySt = carIS.physicalState
      carModel = carIS.model
      carFrame = carModel.frame
      targetLane = clamp(carModel.targetLane, 1, numLanes)
      currNode = carModel.currNode
      fsm = carFrame.policy

      updatedFrame = carFrame

      x = carPhySt.state[1]
      xdot = carPhySt.state[3]
      dxdot = 0.0
      g = AVG_GAP
      if ln != egoLane
        if !isnull(ldCarIS)
          dxdot = xdot - ldCarIS.physicalState.state[3]
          g = ldCarIS.physicalState.state[1] - x - CAR_LENGTH
        end
      else
        if isnull(ldCarIS)
          if (egoState.state[1] > x)
            g = egoState.state[1] - x - CAR_LENGTH
            dxdot = xdot - egoState.state[3]
          end
        else
          if (egoState.state[1] > x) && (ldCarIS.physicalState.state[1] > egoState.state[1])
            g = egoState.state[1] - x - CAR_LENGTH
            dxdot = xdot - egoState.state[3]
          elseif (egoState.state[1] > ldCarIS.physicalState.state[1])
            g = ldCarIS.physicalState.state[1] - x - CAR_LENGTH
            dxdot = xdot - ldCarIS.physicalState.state[3]
          end
        end
      end

      ddotx = get_idm_accln(carIS.model.frame.longitudinal, xdot, dxdot, g)

      ddotx < -6.0 ? ddotx = -6.0 : nothing

      #Compute ydot
      y = carPhySt.state[2]
      target_y = laneCenters[targetLane]

      nextLane = ln
      #TODO: This is a hack
      if targetLane == 0
        targetLane = ln
      end
      #End of hack. The targetLane algo should be redone?
      if targetLane > ln
        nextLane = ln+1
      elseif targetLane < ln
        nextLane = ln-1
      end

      nxtFlPhySt = Nullable{CarPhysicalState}()
      nxtLdPhySt = Nullable{CarPhysicalState}()

      #Update car's model, frame remains same, target lane remains same, only currNode changes
      #Generate observation for other car's lateral motion
      edgeLabel = "Undetermined"
      #If reached target
      #println("Target y = ", target_y, " y = ", y, " target lane = ", targetLane, " lane = ", ln)
      if (abs(target_y - y) < LANE_WIDTH/16.0) && (targetLane == ln) #Second part is important
        edgeLabel = "Reached"
        #else
      elseif targetLane == ln #If already in the targetLane then might as well finish moving to center
        edgeLabel = "SafeSmooth"
      else
        #Check smooth and safe, first check if nextLeading and nextFollowing are known
        for nxtLnIS in globalISL1.neighborhood[nextLane]
            if nxtLnIS.physicalState.state[1] >= carPhySt.state[1]
              nxtLdPhySt = nxtLnIS.physicalState
            end
            if nxtLnIS.physicalState.state[1] <= carPhySt.state[1]
              nxtFlPhySt = nxtLnIS.physicalState
              break
            end
        end
        if egoLane == nextLane
            if egoState.state[1] >= carPhySt.state[1]
              if isnull(nxtLdPhySt) || (nxtLdPhySt.state[1] > egoState.state[1])
                  nxtLdPhySt = egoState
              end
            end
            if egoState.state[1] <= carPhySt.state[1]
              if isnull(nxtFlPhySt) || (nxtFlPhySt.state[1] < egoState.state[1])
                nxtFlPhySt = egoState
              end
            end
        end
        isSmooth = false
        isSafe = false
        if !isnull(nxtLdPhySt)
          isSmooth = isLaneChangeSmooth(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtLdPhySt, ddotx)
          if !isSmooth
            edgeLabel = "UnsafeOrUnsmooth"
          end
        end
        if !isnull(nxtFlPhySt)
          isSafe = isLaneChangeSafe(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtFlPhySt)
          if !isSafe
            edgeLabel = "UnsafeOrUnsmooth"
          end
        end

        if (isSafe && isSmooth)
            edgeLabel = "SafeSmooth"
        end
      end
      #println("EdgeLabel = ", edgeLabel)
      updatedNode = currNode

      rnd = Base.rand(rng)
      #println("rnd = ", rnd)
      cumProb = 0.0
      for nextNode in fsm.nodeSet
        #println("NextNode = ", nextNode.nodeLabel)
        cumProb += get(fsm.transitionProb,(currNode, FSM_Edge(edgeLabel), nextNode),0.0)
        #println("CumProb = ", cumProb)
        if rnd < cumProb
          updatedNode = nextNode
          break
        end
      end

      currNode = updatedNode
      rnd = Base.rand(rng)
      ydotCumProb = [0.0,0.0,0.0]

      ydotCumProb[1] = get(fsm.actionProb, (currNode, 2.0), 0.0)
      ydotCumProb[2] = get(fsm.actionProb, (currNode, 0.0), 0.0) + ydotCumProb[1]
      ydotCumProb[3] = get(fsm.actionProb, (currNode, -2.0), 0.0)+ ydotCumProb[2]

      ydot = 0.0
      if rnd < ydotCumProb[1]
        ydot = 2.0  #Move towards desired lane
      elseif rnd < ydotCumProb[2]
        ydot = 0.0  #Keep lane
      else
        ydot = -2.0 #Abort motion to next lane and move to center of current lane
      end
      #print("CurrNode: ",currNode.nodeLabel," cLane = $ln tLane = $targetLane ")

      if ydot < 0.0 #Reverse direction to center of current lane
        target_y = laneCenters[ln]
        ydot = -ydot  #Sign is immaterial, target_y matters
      end

      if target_y != y
        ydot = min(ydot, abs(target_y - y)/p.TIME_STEP)
        ydot *= (target_y - y)/abs(target_y - y)  #By convention
      else
        ydot = 0.0
      end
      #println("x = $x, y = $y, target_y = $target_y, ydot = $ydot")
      updatedCarPhySt = propagateCar(carPhySt, CarAction(ddotx, ydot), p.TIME_STEP, rng, (2 * TRN_NOISE_X, 2 * TRN_NOISE_Y, 2 * TRN_NOISE_XDOT))

      #println("cNode = ",currNode.nodeLabel," edgeLabel = ",edgeLabel, " nNode = ",updatedNode.nodeLabel)
      updatedModel = ParamCarModelL0(targetLane, updatedFrame, updatedNode)
      updatedIS = CarLocalIS{ParamCarModelL0}(updatedCarPhySt, updatedModel)
      push!(updatedNeighborhood[ln], updatedIS)
      nbr_acts[ln][carIdx] = CarAction(ddotx, ydot)

      ldCarIS = carIS

    end
  end

  return updatedNeighborhood, nbr_acts

end

type CarNormalDist{T <: POMDPs.MDP}
  problem::T
  probDensity::Array{Array{NTuple{3, NormalDist},1},1}   #2D array of normal distribution
end

function initial_state_distribution(p::LowLevelMDP)
  numLanes = n_lanes(p)
  egoState = p.egoStartState
  ego_x = egoState.state[1]
  egoLane = getLaneNo(egoState, p)

  probDensity = Array{Array{NTuple{3, NormalDist},1},1}(numLanes)

  laneCenters = getLaneCenters(p.roadSegment)

  for ln in 1:numLanes
    probDensity[ln] = Array{NTuple{3, NormalDist},1}()
    mean_y = laneCenters[ln]

    if ln == egoLane
      #ldCarDist = NTuple{3,NormalDist}((NormalDist(ego_x + AVG_GAP, AVG_GAP/6.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
      #flCarDist = NTuple{3,NormalDist}((NormalDist(ego_x - AVG_GAP, AVG_GAP/6.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
    elseif abs(ln - egoLane)%2 == 1
      #ldCarDist = NTuple{3,NormalDist}((NormalDist(ego_x + AVG_GAP/2.0, AVG_GAP/6.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
      #flCarDist = NTuple{3,NormalDist}((NormalDist(ego_x - AVG_GAP/2.0, AVG_GAP/6.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
      ldCarDist = NTuple{3,NormalDist}((NormalDist(ego_x + 0.0, 4.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
      #flCarDist = NTuple{3,NormalDist}((NormalDist(ego_x - 0.0, 4.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
    else
      #ldCarDist = NTuple{3,NormalDist}((NormalDist(ego_x + AVG_GAP, AVG_GAP/6.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
      #flCarDist = NTuple{3,NormalDist}((NormalDist(ego_x - AVG_GAP, AVG_GAP/6.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
      ldCarDist = NTuple{3,NormalDist}((NormalDist(ego_x + 0.0, 4.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
      #flCarDist = NTuple{3,NormalDist}((NormalDist(ego_x - 0.0, 4.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
    end
    push!(probDensity[ln], ldCarDist)
    #push!(probDensity[ln], flCarDist)
  end
  return CarNormalDist{LowLevelMDP}(p, probDensity)
end

function rand(rng::AbstractRNG, d::CarNormalDist{LowLevelMDP})
  problem = d.problem
  egoState = problem.egoStartState
  numLanes = n_lanes(problem)
  neighborhood = Array{Array{CarLocalIS,1}}(numLanes)

  for i in 1:numLanes
    neighborhood[i] = Array{CarLocalIS,1}()
  end

  for ln in 1:numLanes
    for carProbDensity in d.probDensity[ln]
      rnd = rand(rng)
      if (rnd > 0.4)
        intentionArray = zeros(Float64, numLanes)
        intentionArray[ln] = 0.4
        if ln-1 > 0
          intentionArray[ln-1] = 0.3
        else
          intentionArray[ln] += 0.3
        end
        if ln+1 <= numLanes
          intentionArray[ln+1] = 0.3
        else
          intentionArray[ln] += 0.3
        end
        #print("Generate: ln = $ln, ")
        carState = randCarLocalISL0(rng, carProbDensity, intentionArray, problem.frameList)

        push!(neighborhood[ln], carState)
      end
    end
  end
  return (GlobalStateL1(0, egoState, neighborhood))
end

#Generate next state
function generate_s(p::LowLevelMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  return first(generate_sr(p, s, a, rng))
end

function generate_sr(p::LowLevelMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  actionSet = EgoActionSpace()
  act = actionSet.actions[a]
  if s.terminal == 1
    #println("End generate_s")
    return s, 0.0
  end
  #If initial state has reached terminal without being marked:
  if checkForCollision(s, p, p.COLLISION_CUSHION)
    #println("Collision generate_s")
    return GlobalStateL1(1, CarPhysicalState(s.ego.state), s.neighborhood), p.collisionCost
  end

  reward = 0.0
  init_egoSt = s.ego
  targetLB = p.egoTargetState[1]
  targetUB = p.egoTargetState[2]

  next_egoSt = propagateCar(s.ego, act, p.TIME_STEP, rng, (TRN_NOISE_X, TRN_NOISE_Y, TRN_NOISE_XDOT))
  neighborhood, nbr_acts = updateNeighborState(s, p, rng)

  sp = GlobalStateL1(0, next_egoSt, neighborhood)
  if checkForCollision(s, act, nbr_acts, sp, p, rng, p.COLLISION_CUSHION)
    return GlobalStateL1(1, next_egoSt, neighborhood), p.collisionCost
  #elseif checkForCollision(s, act, nbr_acts, sp, p, rng, p.COLLISION_CUSHION)
    #reward += p.collisionCost/5.0
  end
  sortNeighborhood(neighborhood, p)
  #sp = GlobalStateL1(0, next_egoSt, neighborhood)

  if s.terminal == 0 && checkTargetCoordinates(s,p)
    reward += p.goalReward
    sp.terminal = 2
  end
  if s.terminal == 2
    sp.terminal = 2
  end

  xdot = init_egoSt.state[3]
  if (targetLB.state[3] > xdot)
    reward += (abs(xdot - targetLB.state[3]) * p.velocityDeviationCost)
  elseif (xdot > targetUB.state[3])
    reward += (abs(xdot - targetUB.state[3]) * p.velocityDeviationCost)
  end

  if act.ddot_x <= -4.0
    reward += p.hardbrakingCost
  end

  if check_induced_hardbraking(s, p)
    reward += p.discomfortCost
  end

  #Penalize for deviating from lane center
  currLaneCenter = getLaneCenter(init_egoSt, p)
  reward += p.steeringCost * (abs(currLaneCenter - init_egoSt.state[2])) * p.TIME_STEP

  return sp, reward
end

function initial_state(p::LowLevelMDP, rng::AbstractRNG)
  isd = initial_state_distribution(p)
  return rand(rng, isd)
end

#Heuristics approach
type subintentional_lowlevel_policy <: Policy
  egoFrame::LowLevelCarFrameL0
  problem::LowLevelMDP
end

function subintentional_lowlevel_policy(p::LowLevelMDP)
  fsm = createFSM()
  idmNormal = createIDM_aggressive()
  mobilNormal = createMOBIL_aggressive()
  egoFrame = LowLevelCarFrameL0(idmNormal, mobilNormal, fsm, CAR_LENGTH, CAR_WIDTH)

  return subintentional_lowlevel_policy(egoFrame, p)
end

function action(si_policy::subintentional_lowlevel_policy, gblSt::GlobalStateL1)
  problem = si_policy.problem
  egoFrame = si_policy.egoFrame

  lon = egoFrame.longitudinal
  lat = egoFrame.lateral

  egoState = gblSt.ego
  egoLane = getLaneNo(egoState, problem)
  x = egoState.state[1]
  y = egoState.state[2]
  xdot = egoState.state[3]

  g= AVG_GAP
  dxdot = 0.0

  neighborhood = gblSt.neighborhood

  #target_x = (problem.egoTargetState[1].state[1] + problem.egoTargetState[2].state[1])/2.0

  #IDM acceleration
  ldcarstate = Nullable{CarPhysicalState}()
  for carIS in neighborhood[egoLane]
    if carIS.physicalState.state[1] > x
      ldcarstate = carIS.physicalState
    else
      break
    end
  end

  if (!isnull(ldcarstate))
    g = ldcarstate.state[1] - x - CAR_LENGTH
    dxdot = xdot - ldcarstate.state[3]
  end

  ddotx = get_idm_accln(lon, xdot, dxdot, g)
  #println("ddotx = ", ddotx)

  if ddotx < -4.0
    ddotx = -6.0
  elseif ddotx < -1.0
    ddotx = -2.0
  elseif ddotx < 2.0
    ddotx = 0.0
  else
    ddotx = 2.0
  end

  #Lateral motion determined by target_y and MOBIL
  target_y = (problem.egoTargetState[1].state[2] + problem.egoTargetState[2].state[2])/2.0

  targetLane = getLaneNo(target_y, problem)

  nextLane = egoLane
  if targetLane > egoLane
    nextLane += 1
  elseif targetLane < egoLane
    nextLane -= 1
  end

  ydot = 0.0
  if (target_y - y) != 0
    ydot = (target_y - y)/abs(target_y - y) * 2.0
  end
  if nextLane == egoLane  #Move to desired y
    if abs(target_y - y) < LANE_WIDTH/16
      ydot = 0.0
    end
  else
    nxtFlPhySt = Nullable{CarPhysicalState}()
    nxtLdPhySt = Nullable{CarPhysicalState}()

    for nxtLnIS in neighborhood[nextLane]
      if nxtLnIS.physicalState.state[1] >= x
        nxtLdPhySt = nxtLnIS.physicalState
      end
      if nxtLnIS.physicalState.state[1] <= x
        nxtFlPhySt = nxtLnIS.physicalState
        break
      end
    end
    isSafe = true
    isSmooth = true
    #Check if unsafe
    if !isnull(nxtFlPhySt)
      isSafe = isLaneChangeSafe(lat, lon, egoState, nxtFlPhySt)
    end
    if !isnull(nxtLdPhySt)
      isSmooth = isLaneChangeSmooth(lat, lon, egoState, nxtLdPhySt, ddotx)
    end
    if !isSafe || !isSmooth
      ydot = 0.0
    end
  end

  egoAct = CarAction(ddotx, ydot)
  actIdx = 0
  egoActions = EgoActionSpace()
  for act in egoActions.actions
    actIdx += 1
    if act == egoAct
      break
    end
  end
  return actIdx

end
