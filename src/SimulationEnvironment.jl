type SimulationMDP <: DiscreteActionDrivingMDP
  discount_factor::Float64
  TIME_STEP::Float64
  HORIZON::Int64
  n_agents::Int64
  roadSegment::RoadSegment
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

SimulationMDP() = SimulationMDP(0.99, 0.3, 60, 40,
                                  RoadSegment((-100.0, 500.0),[0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH]),
                                  CarPhysicalState((0.0, 1.0 * LANE_WIDTH/2.0, AVG_HWY_VELOCITY)),
                                  (CarPhysicalState((425.0, 7.0 * LANE_WIDTH/2.0 - 0.5, AVG_HWY_VELOCITY - 2.5)),
                                   CarPhysicalState((500.0, 7.0 * LANE_WIDTH/2.0 + 0.5, AVG_HWY_VELOCITY + 2.5))),
                                  ll_goalReward*4, ll_collisionCost*4, ll_y_dev_cost, ll_hardbrakingCost, ll_discomfortCost, ll_velocityDeviationCost, getFrameList() )

discount(p::SimulationMDP) = p.discount_factor
function isterminal(p::SimulationMDP, st::GlobalStateL1)
  st.terminal > 0 ? true : false
end
n_actions(p::SimulationMDP) = length(actions(p))
actions(::SimulationMDP) = EgoActionSpace()
n_lanes(p::SimulationMDP) = length(p.roadSegment.laneMarkings)-1
#Function to generate initial state of Simulation Environment
#TODO: Remove duplicates if possible.
function getLaneNo(y::Float64, p::SimulationMDP)
  return getLaneNo(y, p.roadSegment)
end

function getLaneNo(phySt::CarPhysicalState, p::SimulationMDP)
  y = phySt.state[2]
  return getLaneNo(y,p)
end

function getLaneCenter(phySt::CarPhysicalState, p::SimulationMDP)
  laneNo = getLaneNo(phySt, p)
  return getLaneCenter(p.roadSegment, laneNo)
end

function printState(p::SimulationMDP, s::GlobalStateL1)
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

function printGlobalPhyState(s::GlobalStateL1, p::SimulationMDP)
  egoState = s.ego
  egoLane = getLaneNo(egoState, p)
  for ln in 1:length(s.neighborhood)
    print("Lane No.: $ln ")
    colNo = -100
    carNo = length(s.neighborhood[ln])
    egoPrinted = false
    for carNo in length(s.neighborhood[ln]):-1:1
      carIS = s.neighborhood[ln][carNo]

      a = ""
      if (!egoPrinted) && (ln == egoLane) && (carIS.physicalState.state[1] > egoState.state[1])
        ego_x = egoState.state[1]
        while(colNo < ego_x)
          print(" ")
          colNo += 1
        end
        a = @sprintf("_%0.3f,%0.3f,%0.3f",egoState.state[1],egoState.state[2],egoState.state[3])
        colNo += length(a)
        print(a)
        egoPrinted = true
      end
      car_x = carIS.physicalState.state[1]
      while(colNo < car_x)
        print(" ")
        colNo += 1
      end
      b = @sprintf("%0.3f,%0.3f,%0.3f",carIS.physicalState.state[1],carIS.physicalState.state[2],carIS.physicalState.state[3])
      colNo += length(b)
      print(b)
      carNo += 1
    end
    println()
  end
end


function sortintolanes(neighborhood::Array{Array{CarLocalIS,1},1}, p::SimulationMDP)
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

function sortNeighborhood(neighborhood::Array{Array{CarLocalIS,1},1}, p::SimulationMDP)
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

function n_cars(gblSt::GlobalStateL1, ln::Int64, p::SimulationMDP)
  return length(gblSt.neighborhood[ln])
end

#Return the car leading and following car indices respectively
function binary_search(cars::Vector{CarLocalIS}, phySt::CarPhysicalState)
  if length(cars) == 0
    return [0,0]
  end
  car_x = phySt.state[1]
  lead_idx = 1
  lead_x = cars[1].physicalState.state[1]
  trail_idx = length(cars)
  trail_x = cars[end].physicalState.state[1]

  while trail_idx > lead_idx
    mid_idx = convert(Int, floor((lead_idx+trail_idx)/2))
    if cars[mid_idx].physicalState.state[1] > car_x
      lead_idx = mid_idx+1
    else
      trail_idx = mid_idx
    end
  end
  if cars[trail_idx].physicalState.state[1] == car_x
    return [trail_idx, trail_idx]
  end
  if cars[trail_idx].physicalState.state[1] < car_x
    trail_idx -= 1
  end
  return [trail_idx, trail_idx+1]
end

function initializeNbCache(gblSt::GlobalStateL1, p::SimulationMDP)
  gblSt._neighbor_cache = calcImmediateNeighborCache(gblSt, p)
end

function printNeighborCache(gblSt::GlobalStateL1)
  if length(keys(gblSt._neighbor_cache))==0
    println("Empty neighbor cache")
    return
  end
  egoState = gblSt.ego
  neighborhood = gblSt.neighborhood
  for ln in 1:length(neighborhood)
    for carIdx in length(neighborhood[ln]):-1:1
      carIS = neighborhood[ln][carIdx]
      carPhySt = carIS.physicalState
      println("Lane # $ln, car# $carIdx: state: ", carPhySt)
      carImmNeighbors = gblSt._neighbor_cache[carPhySt]
      for lane in 1:length(carImmNeighbors)
        print("\t\t")
        for nbrIdx in 1:length(carImmNeighbors[lane])
          a = @sprintf("%0.3f,%0.3f,%0.3f",carImmNeighbors[lane][nbrIdx].state[1],carImmNeighbors[lane][nbrIdx].state[2],carImmNeighbors[lane][nbrIdx].state[3])
          print(a,"; \t\t")
        end
        println()
      end
      println()
    end
  end
end

function getImmediateNeighbors(gblSt::GlobalStateL1, p::SimulationMDP, phySt::CarPhysicalState)::Array{SVector{2,CarPhysicalState}}
  if haskey(gblSt._neighbor_cache, phySt)
    return gblSt._neighbor_cache[phySt]
  else
    return calcImmediateNeighbors(gblSt, p, phySt)
  end
end

function calcImmediateNeighborCache(s::GlobalStateL1, p::SimulationMDP)::NbCache
  nb_cache = NbCache() #Map car indexed by CarPhysicalState to CarPhysicalState of Leading and following cars in each lane
  numLanes = n_lanes(p.roadSegment)
  egoState = s.ego
  egoLane = getLaneNo(egoState, p)
  neighborhood = s.neighborhood

  lastProcessedIdx = zeros(Int64, numLanes)
  finished = fill(false, numLanes)

  for ln in 1:numLanes
    if length(neighborhood[ln]) == 0
      finished[ln] = true
    end
  end

  while !all(finished)
    currCar = CarPhysicalState((-Inf, -1.0, AVG_HWY_VELOCITY)) #Overall furthest ahead
	  currCar_lane = 0
	  for ln in 1:numLanes
	    if !finished[ln]  #If lane is not already fully processed
        if currCar.state[1] < neighborhood[ln][lastProcessedIdx[ln]+1].physicalState.state[1]
          currCar = neighborhood[ln][lastProcessedIdx[ln]+1].physicalState
          currCar_lane = ln
        end
      end
    end

    @assert currCar.state[2] > 0.0 #Indeed within the road segment in y

    nbhd = Array(SVector{2,CarPhysicalState}, numLanes)
    for ln in 1:numLanes
      nbhd_ln = Array(CarPhysicalState, 2)
      if lastProcessedIdx[ln] < 1
        nbhd_ln[1] = CarPhysicalState((Inf, getLaneCenter(p.roadSegment, currCar_lane), AVG_HWY_VELOCITY))
      else
        nbhd_ln[1] = neighborhood[ln][lastProcessedIdx[ln]].physicalState
      end

      if  lastProcessedIdx[ln] >= length(neighborhood[ln])
        nbhd_ln[2] = CarPhysicalState((-Inf, getLaneCenter(p.roadSegment, currCar_lane), AVG_HWY_VELOCITY))
      elseif ln != currCar_lane
        nbhd_ln[2] = neighborhood[ln][lastProcessedIdx[ln]+1].physicalState
      else
        if lastProcessedIdx[ln] == length(neighborhood[ln])-1
          nbhd_ln[2] = CarPhysicalState((-Inf, getLaneCenter(p.roadSegment, currCar_lane), AVG_HWY_VELOCITY))
        else
          nbhd_ln[2] = neighborhood[ln][lastProcessedIdx[ln]+2].physicalState
        end
      end

      if ln == egoLane
        if nbhd_ln[1].state[1] > egoState.state[1] && egoState.state[1] > currCar.state[1]
          nbhd_ln[1] = egoState
        elseif nbhd_ln[2].state[1] < egoState.state[1] && egoState.state[1] < currCar.state[1]
          nbhd_ln[2] = egoState
        end
      end

      nbhd[ln] = SVector{2,CarPhysicalState}(nbhd_ln)
    end
    nb_cache[currCar] = nbhd

    lastProcessedIdx[currCar_lane] += 1
    if lastProcessedIdx[currCar_lane] == length(neighborhood[currCar_lane])
      finished[currCar_lane] = true
    end
  end

  nb_cache[egoState] = calcImmediateNeighbors(s, p, egoState)
  return nb_cache
end

function calcImmediateNeighbors(gblSt::GlobalStateL1, p::SimulationMDP, phySt::CarPhysicalState)::Array{SVector{2,CarPhysicalState}}
  carLane = getLaneNo(phySt, p)
  egoState = gblSt.ego
  egoLane = getLaneNo(egoState, p)
  car_x = phySt.state[1]
  numLanes = n_lanes(p)
  imm_neighbor = Array{Array{CarPhysicalState,1},1}(numLanes)
  for ln in 1:n_lanes(p)
    imm_neighbor[ln] = Array{CarPhysicalState,1}()

    arr = gblSt.neighborhood[ln]
    if length(arr) == 0  #Empty lane: Add two cars at infinite distance from current car
      ldr = CarPhysicalState((Inf, getLaneCenter(p.roadSegment, ln), AVG_HWY_VELOCITY))
      trlr = CarPhysicalState((-Inf, getLaneCenter(p.roadSegment, ln), AVG_HWY_VELOCITY))
      push!(imm_neighbor[ln], ldr)
      push!(imm_neighbor[ln], trlr)
    else
      nbr_indices = binary_search(arr, phySt) #[lead_idx, trail_idx]
      lead_idx = nbr_indices[1]
      trail_idx = nbr_indices[2]
      #Add imm. Leading car
      if lead_idx < 1 #Probably never happens
        push!(imm_neighbor[ln], CarPhysicalState((Inf, getLaneCenter(p.roadSegment, ln), AVG_HWY_VELOCITY)))
      #Lead car can be the same as the current car
      else
        if (arr[lead_idx].physicalState == phySt)
          lead_idx -= 1
        end
        if lead_idx < 1
          push!(imm_neighbor[ln], CarPhysicalState((Inf, getLaneCenter(p.roadSegment, ln), AVG_HWY_VELOCITY)))
        else
          push!(imm_neighbor[ln], arr[lead_idx].physicalState)
        end
      end

      #Add imm. trailing car
      if length(arr) < trail_idx #Happens when all vehicles are leading (or the same)
        push!(imm_neighbor[ln], CarPhysicalState((-Inf, getLaneCenter(p.roadSegment, ln), AVG_HWY_VELOCITY)))
      else
        if (arr[trail_idx].physicalState == phySt)
          trail_idx += 1
        end
        if length(arr) < trail_idx
          push!(imm_neighbor[ln], CarPhysicalState((-Inf, getLaneCenter(p.roadSegment, ln), AVG_HWY_VELOCITY)))
        else
          push!(imm_neighbor[ln], arr[trail_idx].physicalState)
        end
      end
    end

    # Handle ego lane
    if ln == egoLane
      if (imm_neighbor[ln][1].state[1] > egoState.state[1]) && (egoState.state[1] > phySt.state[1])
        imm_neighbor[ln][1] = egoState
      end

      if (imm_neighbor[ln][2].state[1] < egoState.state[1]) && (egoState.state[1] < phySt.state[1])
        imm_neighbor[ln][2] = egoState
      end
    end

  end
  return SVector{2, CarPhysicalState}[SVector{2}(v) for v in imm_neighbor]
end

function calcLowLevelGblSt(gblSt::GlobalStateL1, p::SimulationMDP)
  egoState = gblSt.ego
  numLanes = n_lanes(p)
  imm_neighbor = Array{Array{CarLocalIS,1},1}(numLanes)
  for ln in 1:numLanes
    imm_neighbor[ln] = Array{CarLocalIS,1}()

    arr = gblSt.neighborhood[ln]
    if !isempty(arr)
      nbr_indices = binary_search(arr, egoState) #[lead_idx, trail_idx]
      lead_idx = nbr_indices[1]
      trail_idx = nbr_indices[2]
      #Add imm. Leading car
      if lead_idx >= 1
        push!(imm_neighbor[ln], arr[lead_idx])
      end

      #Add imm. trailing car
      if length(arr) >= trail_idx
        push!(imm_neighbor[ln], arr[trail_idx])
      end
    end
  end
  return GlobalStateL1(egoState, imm_neighbor)
end


#Code to propagate
function updateCarIS(is::CarLocalIS, gblSt::GlobalStateL1, p::SimulationMDP, rng::AbstractRNG)
  numLanes = n_lanes(p)

  carPhySt = is.physicalState
  carLane = getLaneNo(carPhySt, p)
  laneCenters = getLaneCenters(p.roadSegment)
  carModel = is.model
  targetLane = carModel.targetLane
  carFrame = carModel.frame
  currNode = carModel.currNode
  fsm = carFrame.policy

  imm_neighbors = getImmediateNeighbors(gblSt, p, carPhySt)
  # Accln:
  ldrPhySt = imm_neighbors[carLane][1]
  x = carPhySt.state[1]
  xdot = carPhySt.state[3]

  dxdot = xdot - ldrPhySt.state[3]
  g = ldrPhySt.state[1] - x - CAR_LENGTH

  ddot_x = get_idm_accln(is.model.frame.longitudinal, xdot, dxdot, g)

  y = carPhySt.state[2]

  #println("\t\t\t carLane = $carLane targetLane = $targetLane")
  #Figure out edge labels
  if targetLane == 0  #TargetLane not fixed yet
    cgRtPossible = false
    rtLane = carLane - 1
    if rtLane > 0 #Lane exists now see if lane change is possible
      nxtLdPhySt = imm_neighbors[rtLane][1]
      nxtFlPhySt = imm_neighbors[rtLane][2]

      isSmooth = isLaneChangeSmooth(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtLdPhySt, ddot_x)
      isSafe = isLaneChangeSafe(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtFlPhySt)

      if isSmooth && isSafe
        cgRtPossible = true
      end
    end

    cgLtPossible = false
    ltLane = carLane + 1
    if ltLane <= numLanes #Lane exists now see if lane change is possible
      nxtLdPhySt = imm_neighbors[ltLane][1]
      nxtFlPhySt = imm_neighbors[ltLane][2]

      isSmooth = isLaneChangeSmooth(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtLdPhySt, ddot_x)
      isSafe = isLaneChangeSafe(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtFlPhySt)

      if isSmooth && isSafe
        cgLtPossible = true
      end
    end
    cgRtPossible ? (cgLtPossible ? ((rand(rng) < 0.5) ? targetLane = rtLane : targetLane = ltLane) : targetLane = rtLane) : (cgLtPossible ? targetLane = ltLane : targetLane = carLane)
    #carModel.targetLane = targetLane #Not needed here
    #println("\t\t\t\t updated targetLane = $targetLane")
  end
  target_y = laneCenters[targetLane]
  #println("\t\t\t target_y = $target_y")

  edgeLabel = "Undetermined"
  if carLane == targetLane
    if (abs(target_y - y) < LANE_WIDTH/16.0)
      edgeLabel = "Reached"
      targetLane = 0 #Should reflect in original, ready for next transition, handled later
    else
      edgeLabel = "SafeSmooth"
    end
  else
    nxtLdPhySt = imm_neighbors[targetLane][1]
    nxtFlPhySt = imm_neighbors[targetLane][2]

    isSmooth = isLaneChangeSmooth(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtLdPhySt, ddot_x)
    isSafe = isLaneChangeSafe(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtFlPhySt)

    (isSafe && isSmooth) ? edgeLabel = "SafeSmooth" : edgeLabel = "UnsafeOrUnsmooth"
  end
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

  #target Lane has been initialized
  if ydot < 0.0 #Reverse direction to center of current lane
    targetLane = carLane
    ydot = -ydot  #Sign is immaterial, target_y matters
  end

  if target_y != y
    ydot = min(ydot, abs(target_y - y)/p.TIME_STEP)
    ydot *= (target_y - y)/abs(target_y - y)  #By convention
  else
    ydot = 0.0
  end
  updatedCarPhySt = propagateCar(carPhySt, CarAction(ddot_x, ydot), p.TIME_STEP, rng, (TRN_NOISE_X, TRN_NOISE_Y, TRN_NOISE_XDOT))


  targetLane == 0 ? updatedNode = fsm.nodeSet[1] : nothing
  updatedModel = ParamCarModelL0(targetLane, carFrame, updatedNode)  #Reflects targetLane = 0 here.
  return CarLocalIS(updatedCarPhySt, updatedModel)
end


function updateOtherCarsStates(gblSt::GlobalStateL1, p::SimulationMDP, rng::AbstractRNG)
  #println("updateOtherCarsStates")

  numLanes = n_lanes(p)
  oa_states = Array{Array{CarLocalIS,1},1}(numLanes)
  for ln in 1:numLanes
    #println("\t Lane: $ln")
    oa_states[ln] = Array{CarLocalIS,1}()
    i = 0
    for carIS in gblSt.neighborhood[ln]
      i += 1
      #println("\t\t Car#: $i")
      push!(oa_states[ln], updateCarIS(carIS, gblSt, p, rng))
    end
  end
  return sortNeighborhood(oa_states,p)
end
function checkForCollision(gblISL1::GlobalStateL1, p::SimulationMDP)
  egoState = gblISL1.ego
  egoLane = getLaneNo(egoState, p)
  y = egoState.state[2]
  if y > p.roadSegment.laneMarkings[end] || y < p.roadSegment.laneMarkings[1]
    return true
  end
  imm_neighbors = getImmediateNeighbors(gblISL1, p, egoState)

  numLanes = length(imm_neighbors)
  for ln in -1:1
    if egoLane + ln < 1 || egoLane + ln > numLanes
      continue
    end
    for carIS in imm_neighbors[egoLane + ln]
      if collision(egoState, carIS)
        return true
      end
    end
  end
  return false
end

function checkTargetCoordinates(gblISL1::GlobalStateL1, p::SimulationMDP)
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

#TODO: Redo
function check_induced_hardbraking(globalISL1::GlobalStateL1, p::SimulationMDP)
  laneCenters = getLaneCenters(p.roadSegment)

  egoState = globalISL1.ego
  egoLane = getLaneNo(egoState, p)
  imm_neighbors = getImmediateNeighbors(globalISL1, p, egoState)
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
    if ddotx < -6.0
      return true
    end
    break # Only interested in car immediately behind ego car.
  end

  return false
end

function generate_s(p::SimulationMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  if s.terminal > 0 || a < 1 || a > n_actions(p)
    #println("End generate_s")
    return s
  end

  if checkForCollision(s, p)
    #println("Collision generate_s")
    return GlobalStateL1(1, CarPhysicalState(s.ego.state), s.neighborhood, s._neighbor_cache)
  end
  if checkTargetCoordinates(s,p)
    return GlobalStateL1(2, CarPhysicalState(s.ego.state), s.neighborhood,s._neighbor_cache)
  end

  actionSet = EgoActionSpace()
  act = actionSet.actions[a]

  #println("EgoState = ", s.ego)
  egoState = propagateCar(s.ego, act, p.TIME_STEP, rng, (TRN_NOISE_X, TRN_NOISE_Y, TRN_NOISE_XDOT))
  #println("EgoStatePrime = ", egoState)
  #println("End update egoState $egoState. Begin update neighborhood")
  neighborhood = updateOtherCarsStates(s, p, rng)
  #println("End update neighborhood")
  sp = GlobalStateL1(0, egoState, neighborhood)
  initializeNbCache(sp,p)
  return sp
end

function reward(p::SimulationMDP, s::GlobalStateL1, a::Int, rng::AbstractRNG)
  actionSet = EgoActionSpace()
  act = actionSet.actions[a]
  if (s.terminal > 0 )
    #println("End reward")
    #println("Reward = ", 0.0)
    return 0.0
  end

  targetLB = p.egoTargetState[1]
  targetUB = p.egoTargetState[2]
  reward = 0.0
  egoSt = s.ego

  if checkForCollision(s, p)
    #println("End reward")
    #println("Reward = ",p.collisionCost)
    return p.collisionCost
  end

  if checkTargetCoordinates(s,p)
    reward += p.goalReward
  end

  xdot = egoSt.state[3]
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

  currLaneCenter = getLaneCenter(egoSt, p)
  reward += p.steeringCost * (abs(currLaneCenter - egoSt.state[2])) * p.TIME_STEP

  return reward
end

function generate_sr(p::SimulationMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  #print("\rBegin generate_sor")
  sp = generate_s(p, s, a, rng)

  r = reward(p,s,a,rng)
  #r = reward(p, s, a, sp)
  #print("\rEnd generate_sor")
  return sp, r
end

function populate_lane_x_positions(x_boundary::NTuple{2,Float64}, agents_per_lane::Int64, rng::AbstractRNG)
  x_positions = Vector{Float64}()
  x_ub = x_boundary[2]
  x_lb = x_boundary[1]
  avg_dist = (x_ub-x_lb)/agents_per_lane
  push!(x_positions, x_ub - (avg_dist/4.0 * rand(rng))) #Some random distance from x_ub
  for ag in 2:agents_per_lane
    gap = avg_dist
    next_x = x_positions[end] - gap
    if next_x > x_lb
      push!(x_positions, next_x)
    end
  end
  return x_positions
end

function initial_state_distribution(p::SimulationMDP, rng::AbstractRNG)
  numAgents = p.n_agents
  numLanes = n_lanes(p)
  laneCenters = getLaneCenters(p.roadSegment)

  agents_per_lane = round(Int64, numAgents/numLanes)
  rs = p.roadSegment
  probDensity = Array{Array{NTuple{3, NormalDist},1},1}(numLanes)
  egoState = p.egoStartState
  ego_x = egoState.state[1]
  egoLane = getLaneNo(egoState,p)
  for ln in 1:numLanes
    probDensity[ln] = Array{NTuple{3, NormalDist},1}()

    x_positions = populate_lane_x_positions(rs.x_boundary, agents_per_lane, rng)

    mean_y = laneCenters[ln]
    for carId in 1:length(x_positions)
      #Don't add car that causes collision in first state
      if (ln == egoLane) && (abs(ego_x - x_positions[carId]) < CAR_LENGTH)
        continue
      end
      carDist = NTuple{3,NormalDist}((NormalDist(x_positions[carId], AVG_GAP/6.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
      push!(probDensity[ln], carDist)
    end
  end
  return CarNormalDist{SimulationMDP}(p, probDensity)
end

#rand substitute.
function rand(rng::AbstractRNG, d::CarNormalDist{SimulationMDP})
  problem = d.problem
  egoState = problem.egoStartState
  numLanes = n_lanes(problem)
  neighborhood = Array{Array{CarLocalIS,1}}(numLanes)

  for ln in 1:numLanes
    neighborhood[ln] = Array{CarLocalIS,1}()
  end

  intentionArray = Vector{Float64}()  #Keeping empty
  for ln in 1:numLanes
    for carProbDensity in d.probDensity[ln]
      carState = randCarLocalISL0(rng, carProbDensity, intentionArray, problem.frameList)
      if abs(egoState.state[1] - carState.physicalState.state[1]) > 2*CAR_LENGTH || abs(egoState.state[2] - carState.physicalState.state[2]) > 1.4*CAR_WIDTH
        push!(neighborhood[ln], carState)
      end
    end
  end
  s = (GlobalStateL1(0, egoState, neighborhood))
  initializeNbCache(s,problem)
  return s
end

function initial_state(p::SimulationMDP, rng::AbstractRNG)
  isd = initial_state_distribution(p, rng)
  return rand(rng, isd)
end

type subintentional_simulation_policy <: Policy
  egoModel::ParamCarModelL0
  problem::SimulationMDP
  rng::MersenneTwister # could be changed to RNG <: AbstractRNG
end

function subintentional_simulation_policy(p::SimulationMDP, rng::AbstractRNG=MersenneTwister(706432))
  fsm = createFSM()
  idmNormal = createIDM_normal()
  mobilNormal = createMOBIL_normal()
  egoFrame = LowLevelCarFrameL0(idmNormal, mobilNormal, fsm, CAR_LENGTH, CAR_WIDTH)
  targetLane = 0
  node = fsm.nodeSet[1]

  egoModel = ParamCarModelL0(targetLane, egoFrame, node)

  return subintentional_simulation_policy(egoModel, p, rng)
end

function action(si_policy::subintentional_simulation_policy, gblSt::GlobalStateL1)
  problem = si_policy.problem
  egoModel = si_policy.egoModel
  egoFrame = egoModel.frame
  egoNode = egoModel.currNode

  lon = egoFrame.longitudinal
  lat = egoFrame.lateral
  fsm = egoFrame.policy

  egoState = gblSt.ego
  egoLane = getLaneNo(egoState, problem)
  x = egoState.state[1]
  y = egoState.state[2]
  xdot = egoState.state[3]

  laneCenters = getLaneCenters(problem.roadSegment)
  goal_y = (problem.egoTargetState[1].state[2] + problem.egoTargetState[2].state[2])/2.0
  goalLane = getLaneNo(goal_y, problem.roadSegment)

  imm_neighbors = getImmediateNeighbors(gblSt, problem, egoState)
  targetLane = egoModel.targetLane

  ldrPhySt = imm_neighbors[egoLane][1]
  dxdot = xdot - ldrPhySt.state[3]
  g = ldrPhySt.state[1] - x - CAR_LENGTH

  ddot_x = get_idm_accln(lon, xdot, dxdot, g)

  if ddot_x < -4.0
    ddot_x = -6.0
  elseif ddot_x < -1.0
    ddot_x = -2.0
  elseif ddot_x < 2.0
    ddot_x = 0.0
  else
    ddot_x = 2.0
  end

  rnd = Base.rand(si_policy.rng)
  ydotCumProb = [0.0,0.0,0.0]


  ydotCumProb[1] = get(fsm.actionProb, (egoNode, 2.0), 0.0)
  ydotCumProb[2] = get(fsm.actionProb, (egoNode, 0.0), 0.0) + ydotCumProb[1]
  ydotCumProb[3] = get(fsm.actionProb, (egoNode, -2.0), 0.0)+ ydotCumProb[2]

  ydot = 0.0
  if rnd < ydotCumProb[1]
    ydot = 2.0  #Move towards desired lane
  elseif rnd < ydotCumProb[2]
    ydot = 0.0  #Keep lane
  else
    ydot = -2.0 #Abort motion to next lane and move to center of current lane
  end

  #If targetLane is 0 then check for safety and start moving
  if targetLane == 0
    nxtLane = egoLane
    if goalLane > egoLane
      nxtLane += 1
    elseif goalLane < egoLane
      nxtLane -= 1
    end
    if (nxtLane != egoLane)
      nxtLdPhySt = imm_neighbors[nxtLane][1]
      nxtFlPhySt = imm_neighbors[nxtLane][2]

      isSmooth = isLaneChangeSmooth(lat, lon, egoState, nxtLdPhySt, ddot_x)
      isSafe = isLaneChangeSafe(lat, lon, egoState, nxtFlPhySt)

      if isSmooth && isSafe
        targetLane = nxtLane
      else
        targetLane = egoLane
      end
    end
  end

  if ydot < 0.0 #Reverse direction to center of current lane
    targetLane = carLane
    ydot = -ydot  #Sign is immaterial, target_y matters
  end
  target_y = laneCenters[targetLane]

  if target_y != y
    ydot = min(ydot, abs(target_y - y)/problem.TIME_STEP)
    ydot *= (target_y - y)/abs(target_y - y)  #By convention
  else
    ydot = 0.0
  end
  updatedCarPhySt = propagateCar(egoState, CarAction(ddot_x, ydot), problem.TIME_STEP, si_policy.rng, (TRN_NOISE_X, TRN_NOISE_Y, TRN_NOISE_XDOT))

  edgeLabel = "Undetermined"
  if egoLane == targetLane
    if (abs(target_y - y) < LANE_WIDTH/16.0)
      edgeLabel = "Reached"
      targetLane = 0 #Should reflect in original, ready for next transition, handled later
    else
      edgeLabel = "SafeSmooth"
    end
  else
    nxtLdPhySt = imm_neighbors[targetLane][1]
    nxtFlPhySt = imm_neighbors[targetLane][2]

    isSmooth = isLaneChangeSmooth(lat, lon, egoState, nxtLdPhySt, ddot_x)
    isSafe = isLaneChangeSafe(lat, lon, egoState, nxtFlPhySt)

    (isSafe && isSmooth) ? edgeLabel = "SafeSmooth" : edgeLabel = "UnsafeOrUnsmooth"
  end
  updatedNode = egoNode

  rnd = Base.rand(si_policy.rng)
  #println("rnd = ", rnd)
  cumProb = 0.0
  for nextNode in fsm.nodeSet
    #println("NextNode = ", nextNode.nodeLabel)
    cumProb += get(fsm.transitionProb,(egoLane, FSM_Edge(edgeLabel), nextNode),0.0)
    #println("CumProb = ", cumProb)
    if rnd < cumProb
      updatedNode = nextNode
      break
    end
  end
  targetLane == 0 ? updatedNode = fsm.nodeSet[1] : nothing
  updatedModel = ParamCarModelL0(targetLane, egoFrame, updatedNode)  #Reflects targetLane = 0 here.
  si_policy.egoModel = updatedModel

  egoAct = CarAction(ddot_x, ydot)
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

#Code to gridify
