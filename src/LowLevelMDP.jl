type LowLevelMDP <:POMDPs.MDP{GlobalStateL1, Int64}
  discount_factor::Float64
  nbrLaneMarkings::Array{Float64,1} #numLanes + 1 values
  egoStartState::CarPhysicalState
  egoTargetState::NTuple{2, CarPhysicalState} #Two values with lb and ub on target physical state
  goalReward::Float64
  collisionCost::Float64
  movementCost::Float64
  hardbrakingCost::Float64
  discomfortCost::Float64
  velocityDeviationCost::Float64
  frameList::Array{CarFrameL0,1}
end

LowLevelMDP() = LowLevelMDP(0.9, [0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH], CarPhysicalState((0.0, 3.0 * LANE_WIDTH/2.0, AVG_HWY_VELOCITY)), (CarPhysicalState((0.0, 5.0 * LANE_WIDTH/2.0 - 0.5, AVG_HWY_VELOCITY - VEL_STD_DEV)), CarPhysicalState((100.0, 5.0 * LANE_WIDTH/2.0 + 0.5, AVG_HWY_VELOCITY + VEL_STD_DEV))), 5.0, -50.0, 0.0, -3.0, -2.0, -0.5, getFrameList())

discount(p::LowLevelMDP) = p.discount_factor
isterminal(::LowLevelMDP, act::Int64) = act == length(EgoActionSpace().actions)

function isterminal(p::LowLevelMDP, st::GlobalStateL1)
  st.terminal > 0 ? true : false
end
#From actions
n_actions(p::LowLevelMDP) = length(actions(p))
actions(::LowLevelMDP) = EgoActionSpace()

n_lanes(p::LowLevelMDP) = length(p.nbrLaneMarkings)-1

function getLaneNo(phySt::CarPhysicalState, p::LowLevelMDP)
  y = phySt.state[2]

  for j = 1:length(p.nbrLaneMarkings)
    if y < p.nbrLaneMarkings[j]
      return j-1
    end
  end
  return length(p.nbrLaneMarkings)
end

function sortNeighborhood(neighborhood::Array{Array{CarLocalISL0,1},1}, p::LowLevelMDP)
  numLanes = length(neighborhood)
  if numLanes != n_lanes(p)
    println("[sortNeighborhood] Incorrect number of lanes.")
  end

  sorted = Array{Array{CarLocalISL0,1},1}(numLanes)

  for ln in 1:numLanes
    sorted[ln] = Array{CarLocalISL0,1}()
  end
  for ln in 1:numLanes
    for carIS in neighborhood[ln]
      carPhySt = carIS.physicalState
      x = carPhySt.state[1]
      y = carPhySt.state[2]

      #Find the lane to which the car belongs
      carLane = getLaneNo(carPhySt, p)
      if carLane < 1
        carLane = 1
        #println("carLane = 0")
      elseif carLane > numLanes
        carLane = numLanes
        #println("carLane > numLanes")
      end
      if length(sorted[carLane]) == 0
        push!(sorted[carLane], carIS)
      else
        numCarsAhead = 0
        while numCarsAhead < length(sorted[carLane])
          otherCarIS = sorted[carLane][numCarsAhead+1]
          otherCarPhySt = otherCarIS.physicalState
          xp = otherCarPhySt.state[1]
          if xp < x
            break
          end
          numCarsAhead += 1
        end
        if numCarsAhead == length(sorted[carLane])
          push!(sorted[carLane], carIS)
        else
          temp = splice!(sorted[carLane], numCarsAhead+1:length(sorted[carLane]))
          push!(sorted[carLane], carIS)
          append!(sorted[carLane],temp)
        end
      end

    end
  end
  #=println("Sorted neighborhood")
  for ln in 1:length(sorted)
    print("Lane no. ", ln, " : ")
    for carIS in sorted[ln]
      print(carIS.physicalState.state[1], " ")
    end
    println()
  end=#
  return sorted
end

function check_induced_hardbraking(globalISL1::GlobalStateL1, p::LowLevelMDP)
  laneCenters = []
  for ln in 2:length(p.nbrLaneMarkings)
    push!(laneCenters, (p.nbrLaneMarkings[ln]+p.nbrLaneMarkings[ln-1])/2.0)
  end

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
    carModel = carIS.modelL0
    x = carPhySt.state[1]
    xdot = carPhySt.state[3]
    dxdot = egoState.state[3] - xdot
    g = egoState.state[1] - x

    ddotx = get_idm_accln(carIS.modelL0.frame.longitudinal, xdot, dxdot, g)
    if ddotx < -4.0
      return true
    end
    break # Only interested in car immediately behind ego car.
  end

  return false
end

function getOtherCarsAction(globalISL1::GlobalStateL1, p::LowLevelMDP, rng::AbstractRNG)
  laneCenters = []
  for ln in 2:length(p.nbrLaneMarkings)
    push!(laneCenters, (p.nbrLaneMarkings[ln]+p.nbrLaneMarkings[ln-1])/2.0)
  end

  egoState = globalISL1.ego
  egoLane = getLaneNo(egoState, p)
  numLanes = length(globalISL1.neighborhood)
  if numLanes != n_lanes(p)
    println("[getOtherCarsAction] Incorrect numLanes. ")
  end

  actions = Array{Array{CarAction,1},1}(numLanes)
  for ln in 1:numLanes
    actions[ln] = Array{CarAction,1}()
    carNo = 1
    ldCarIS = Nullable{CarLocalISL0}()
    for carIS in globalISL1.neighborhood[ln]
      carPhySt = carIS.physicalState
      carModel = carIS.modelL0
      x = carPhySt.state[1]
      xdot = carPhySt.state[3]
      dxdot = 0.0
      g = AVG_GAP
      #Longitudinal acceleration
      if (ln != egoLane)
        if !isnull(ldCarIS)#carNo != 1
          dxdot = ldCarIS.physicalState.state[3] - xdot
          g = ldCarIS.physicalState.state[1] - x
        end
      else
        if (isnull(ldCarIS))  && (egoState.state[1] > x) #car is the first in the lane in neighborhood but ego vehicle is ahead of it
          g = egoState.state[1] - x
          dxdot = egoState.state[3] - xdot
        elseif (!isnull(ldCarIS)) && (egoState.state[1] > x) && (ldCarIS.physicalState.state[1] > egoState.state[1])
          g = egoState.state[1] - x
          dxdot = egoState.state[3] - xdot
        elseif (!isnull(ldCarIS)) && (egoState.state[1] > ldCarIS.physicalState.state[1])
          g = ldCarIS.physicalState.state[1] - x
          dxdot = ldCarIS.physicalState.state[3] - xdot
        end
      end

      ddotx = get_idm_accln(carIS.modelL0.frame.longitudinal, xdot, dxdot, g)

      #NOTE: IDM acceleration part is correct.

      #Lateral velocity
      tLn = carModel.targetLane
      carFrame = carModel.frame
      fsm = carFrame.policy
      carModelNode = carModel.currNode

      #Sample action from current node
      #Next node sampled from current node
      rnd = Base.rand(rng)
      ydotCumProb = [0.0,0.0,0.0]

      ydotCumProb[1] = get(fsm.actionProb, (carModelNode, 2.0), 0.0)
      ydotCumProb[2] = get(fsm.actionProb, (carModelNode, 0.0), 0.0) + ydotCumProb[1]
      ydotCumProb[3] = get(fsm.actionProb, (carModelNode, -2.0), 0.0)+ ydotCumProb[2]

      ydot = 0.0
      if rnd < ydotCumProb[1]
        ydot = 2.0
      elseif rnd < ydotCumProb[2]
        ydot = 0.0
      else
        ydot = -2.0
      end
      y = carPhySt.state[2]
      target_y = laneCenters[tLn]

      if target_y != y
        ydot *= (target_y - y)/abs(target_y - y)  #By convention
      end
      #println("Node No.", carModelNode.nodeLabel, ", Lane No. $ln, Target Lane = $tLn, Car No. $carNo, curr_y $y, target_y $target_y, ydot $ydot, ddotx = $ddotx")
      push!(actions[ln], CarAction(ddotx, ydot))

      carNo += 1
      ldCarIS = carIS
    end
  end

  return actions
end


function updateNeighborState(globalISL1::GlobalStateL1, p::LowLevelMDP, rng::AbstractRNG)
  laneCenters = []
  for ln in 2:length(p.nbrLaneMarkings)
    push!(laneCenters, (p.nbrLaneMarkings[ln]+p.nbrLaneMarkings[ln-1])/2.0)
  end

  egoLane = getLaneNo(globalISL1.ego,p)
  numLanes = length(globalISL1.neighborhood)
  if numLanes != n_lanes(p)
    println("[updateNeighborState] Incorrect numLanes. ")
  end
  updatedNeighborhood = Array{Array{CarLocalISL0,1},1}(numLanes)

  actions = getOtherCarsAction(globalISL1, p, rng) #This gives all cars' (sampled) action in the order they are on neighborhood

  for ln in 1:numLanes #For every lane
    numCars = length(globalISL1.neighborhood[ln])
    updatedNeighborhood[ln] = Array{CarLocalISL0,1}()
    for carIdx in 1:numCars #For every car in the lane
      nxtFlPhySt = Nullable{CarPhysicalState}()
      nxtLdPhySt = Nullable{CarPhysicalState}()

      carIS = globalISL1.neighborhood[ln][carIdx] #Initial IS of the current car under consideration
      carAct = actions[ln][carIdx] #action performed by current car

      carPhySt = carIS.physicalState
      carModel = carIS.modelL0
      carFrame = carModel.frame
      targetLane = carModel.targetLane
      currNode = carModel.currNode

      updatedFrame = carFrame

      nextLane = ln
      if targetLane > ln
        nextLane = ln+1
      elseif targetLane < ln
        nextLane = ln-1
      end

      target_y = laneCenters[targetLane]
      y = carPhySt.state[2]
      ydot = 0.0
      if target_y > y
        ydot = min(carAct.dot_y, (target_y - y)/TIME_STEP)
      else
        ydot = max(carAct.dot_y, (target_y - y)/TIME_STEP)
      end

      xddot = carAct.ddot_x
      updatedCarPhySt = propagateCar(carPhySt, CarAction(xddot, ydot), TIME_STEP, rng, (TRN_NOISE_X, TRN_NOISE_Y, TRN_NOISE_XDOT))

      #Update car's model, frame remains same, target lane remains same, only currNode changes
      carFSM = carFrame.policy
      #Generate observation for other car's lateral motion
      edgeLabel = "Undetermined"
      #If reached target
      #println("Target y = ", target_y, " y = ", y, " target lane = ", targetLane, " lane = ", ln)
      if abs(target_y - y) < LANE_WIDTH/16.0
        edgeLabel = "Reached"
        #else
      elseif targetLane == ln #If already in the targetLane then might as well finish moving to center
        edgeLabel = "SafeSmooth"
      else
        #Check smooth and safe, first check if nextLeading and nextFollowing are known
        for nxtLnIS in globalISL1.neighborhood[nextLane]
            if nxtLnIS.physicalState.state[1] >= carPhySt.state[1]
                nxtLdPhySt = nxtLnIS.physicalState
            elseif nxtLnIS.physicalState.state[1] <= carPhySt.state[1]
                nxtFlPhySt = nxtLnIS.physicalState
                break
            end
        end
        if egoLane == nextLane
            if globalISL1.ego.state[1] > carPhySt.state[1]
                if isnull(nxtLdPhySt) || (nxtLdPhySt.state[1] > globalISL1.ego.state[1])
                    nxtLdPhySt = globalISL1.ego
                end
            elseif globalISL1.ego.state[1] < carPhySt.state[1]
                if isnull(nxtFlPhySt) || (nxtFlPhySt.state[1] < globalISL1.ego.state[1])
                  nxtFlPhySt = globalISL1.ego
                end
            end
        end
        if (!isnull(nxtLdPhySt) && !isnull(nxtFlPhySt))
            isSmooth = isLaneChangeSmooth(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtLdPhySt, xddot)
            isSafe = isLaneChangeSafe(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtFlPhySt)
            if (isSafe && isSmooth)
                edgeLabel = "SafeSmooth"
            else
                edgeLabel = "UnsafeOrUnsmooth"
            end
        end
      end
      #println("EdgeLabel = ", edgeLabel)
      updatedNode = currNode

      rnd = Base.rand(rng)
      #println("rnd = ", rnd)
      cumProb = 0.0
      for nextNode in carFSM.nodeSet
        #println("NextNode = ", nextNode.nodeLabel)
        cumProb += get(carFSM.transitionProb,(currNode, FSM_Edge(edgeLabel), nextNode),0.0)
        #println("CumProb = ", cumProb)
        if rnd < cumProb
          updatedNode = nextNode
          break
        end
      end
      #println("CurrNode = ",currNode.nodeLabel," UpdatedNode = ",updatedNode.nodeLabel)
      updatedModel = CarModelL0(targetLane, updatedFrame, updatedNode)
      updatedIS = CarLocalISL0(updatedCarPhySt, updatedModel)
      push!(updatedNeighborhood[ln], updatedIS)
    end
  end

  return sortNeighborhood(updatedNeighborhood, p)

end

type LowLevelNormalDist
  problem::LowLevelMDP
  probDensity::Array{Array{NTuple{3, NormalDist},1},1}   #2D array of normal distribution
end

function initial_state_distribution(p::LowLevelMDP)
  numLanes = n_lanes(p)
  egoState = p.egoStartState
  ego_x = egoState.state[1]
  egoLane = getLaneNo(egoState, p)

  maxCars = 2 * ones(UInt64, numLanes)
  probDensity = Array{Array{NTuple{3, NormalDist},1},1}(numLanes)

  laneCenters = []
  for ln in 2:length(p.nbrLaneMarkings)
    push!(laneCenters, (p.nbrLaneMarkings[ln]+p.nbrLaneMarkings[ln-1])/2.0)
  end

  for ln in 1:numLanes
    probDensity[ln] = Array{NTuple{3, NormalDist},1}()
    mean_y = laneCenters[ln]

    if ln != egoLane
      ldCarDist = NTuple{3,NormalDist}((NormalDist(ego_x + AVG_GAP/2.0, AVG_GAP/6.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
      flCarDist = NTuple{3,NormalDist}((NormalDist(ego_x - AVG_GAP/2.0, AVG_GAP/6.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
    else
      ldCarDist = NTuple{3,NormalDist}((NormalDist(ego_x + AVG_GAP, AVG_GAP/6.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
      flCarDist = NTuple{3,NormalDist}((NormalDist(ego_x - AVG_GAP, AVG_GAP/6.0), NormalDist(mean_y, LANE_WIDTH/6.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
    end
    push!(probDensity[ln], ldCarDist)
    push!(probDensity[ln], flCarDist)
  end
  return LowLevelNormalDist(p, probDensity)
end

function rand(rng::AbstractRNG, d::LowLevelNormalDist)
  problem = d.problem
  egoState = problem.egoStartState
  numLanes = n_lanes(problem)
  neighborhood = Array{Array{CarLocalISL0,1}}(numLanes)

  for i in 1:numLanes
    neighborhood[i] = Array{CarLocalISL0,1}()
  end

  for ln in 1:numLanes
    for carProbDensity in d.probDensity[ln]
      rnd = rand(rng)
      if (rnd > 0.4)
        intentionArray = zeros(Float64, numLanes)
        intentionArray[ln] = 0.6
        if ln-1 > 0
          intentionArray[ln-1] = 0.2
        else
          intentionArray[ln] += 0.2
        end
        if ln+1 <= numLanes
          intentionArray[ln+1] = 0.2
        else
          intentionArray[ln] += 0.2
        end
        carState = randCarLocalISL0(rng, carProbDensity, intentionArray, problem.frameList)

        push!(neighborhood[ln], carState)
      end
    end
  end
  return (GlobalStateL1(0, egoState, neighborhood))
end

#Generate next state
function generate_s(p::LowLevelMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  #println("Begin generate_s")
  actionSet = EgoActionSpace()
  act = actionSet.actions[a]
  if s.terminal > 0
    #println("End generate_s")
    return s
  end
  if a == length(actionSet.actions)
    #println("Terminal action generate_s")
    return GlobalStateL1(1, CarPhysicalState(s.ego.state), s.neighborhood)
  end
  if checkForCollision(s)
    #println("Collision generate_s")
    return GlobalStateL1(1, CarPhysicalState(s.ego.state), s.neighborhood)
  end

  #println("Begin update egoState")
  egoState = propagateCar(s.ego, act, TIME_STEP, rng, (TRN_NOISE_X, TRN_NOISE_Y, TRN_NOISE_XDOT))
  #println("End update egoState. Begin update neighborhood")
  neighborhood = updateNeighborState(s, p, rng)
  #println("End update neighborhood")
  sp = GlobalStateL1(0, egoState, neighborhood)
  targetLB = p.egoTargetState[1]
  targetUB = p.egoTargetState[2]
  if (targetLB.state[1] < egoState.state[1]) && (egoState.state[1] < targetUB.state[1]) && (targetLB.state[2] < egoState.state[2]) && (egoState.state[2] < targetUB.state[2])
    sp = GlobalStateL1(2, egoState, neighborhood)
  end
  #println("End generate_s")
  return sp
end

#Generate reward
function reward(p::LowLevelMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  #println("Begin reward")
  actionSet = EgoActionSpace()
  act = actionSet.actions[a]
  if (s.terminal > 0 )
    #println("End reward")
    return 0.0
  end
  if a == length(actionSet.actions)
    #println("End reward")
    return p.collisionCost - 5.0  #Just giving it the minimum value. Not sure how MCVI treats the value
  end

  reward = 0.0
  egoSt = s.ego
  nbrhood = s.neighborhood
  if checkForCollision(s)
    #println("End reward")
    return p.collisionCost
  end

  #Goal defined as a range
  targetLB = p.egoTargetState[1]
  targetUB = p.egoTargetState[2]
  if (targetLB.state[1] < egoSt.state[1]) && (egoSt.state[1] < targetUB.state[1]) && (targetLB.state[2] < egoSt.state[2]) && (egoSt.state[2] < targetUB.state[2])
    reward += p.goalReward
  end
  if act.ddot_x <= -4.0
    reward += p.hardbrakingCost
  end
  if check_induced_hardbraking(s, p)
    reward += p.discomfortCost
  end

  xdot = egoSt.state[3]
  if (targetLB.state[3] > egoSt.state[3])
    reward += (abs(xdot - targetLB.state[3]) * p.velocityDeviationCost)
  elseif (egoSt.state[3] > targetUB.state[3])
    reward += (abs(xdot - targetUB.state[3]) * p.velocityDeviationCost)
  end

  #println("End reward")
  return reward
end

function generate_sr(p::LowLevelMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  #print("\rBegin generate_sor")
  sp = generate_s(p, s, a, rng)

  r = reward(p,s,a,rng)
  #print("\rEnd generate_sor")
  return sp, r
end

function initial_state(p::LowLevelMDP, rng::AbstractRNG)
  #println("Begin initial_state")
  isd = initial_state_distribution(p)
  #println("End initial_state")
  return rand(rng, isd)
end

type LowLevelLowerBound
    rng::AbstractRNG
end

type LowLevelUpperBound
    rng::AbstractRNG
end

function lower_bound(lb::LowLevelLowerBound, p::LowLevelMDP, s::GlobalStateL1)
    return p.collisionCost
end

function upper_bound(ub::LowLevelUpperBound, p::LowLevelMDP, s::GlobalStateL1)
    return p.goalReward
end

function init_lower_action(p::ChangeLaneRightPOMDP)
    length(EgoActionSpace())
end
