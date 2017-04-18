#include("Global.jl")

#import Base: ==, +, *, -, copy
#import Random.rand

#Modeling parameters
NUM_INTENTIONS = 4 #Only 4 possible target lanes, movement is fixed based on


using POMDPs

function getLaneNo(phySt::CarPhysicalState)
  if phySt.state[2] < -LANE_WIDTH
    return 1
  elseif phySt.state[2] < 0
    return 2
  elseif phySt.state[2] < LANE_WIDTH
    return 3
  else
    return 4
  end
end


#Get car action given global state and model
function getOtherCarsAction(globalISL1::GlobalStateL1, rng::AbstractRNG)
  #=
  Change next part for different maneuvers
  =#
  laneCenters = [-3*LANE_WIDTH/2, -LANE_WIDTH/2, LANE_WIDTH/2, 3*LANE_WIDTH/2]
  #=
  End of variable part
  =#
  egoState = globalISL1.ego
  egoLane = getLaneNo(egoState)
  numLanes = length(globalISL1.neighborhood)
  actions = Array{Array{CarAction,1},1}(numLanes)
  for ln in 1:numLanes
    actions[ln] = Array{CarAction,1}()
    carNo = 1
    for carIS in globalISL1.neighborhood[ln]
      carPhySt = carIS.physicalState
      carModel = carIS.modelL0
      x = carPhySt.state[1]
      xdot = carPhySt.state[3]
      dxdot = 0.0
      g = AVG_GAP
      #Longitudinal acceleration
      if (ln != egoLane)
        if carNo != 1
          dxdot = globalISL1.neighborhood[numLanes][carNo-1].physicalState.state[3] - xdot
          g = globalISL1.neighborhood[numLanes][carNo-1].physicalState.state[1] - x
        end
      else
        if (carNo == 1)  && (egoState.state[1] > x) #car is the first in the lane in neighborhood but ego vehicle is ahead of it
          g = egoState.state[1] - x
          dxdot = egoState.state[3] - xdot
        elseif (carNo != 1) && (egoState.state[1] > x) && (globalISL1.neighborhood[numLanes][carNo-1].physicalState.state[1] > egoState.state[1])
          g = egoState.state[1] - x
          dxdot = egoState.state[3] - xdot
        elseif (carNo != 1) && (egoState.state[1] > globalISL1.neighborhood[numLanes][carNo-1].physicalState.state[1])
          g = globalISL1.neighborhood[numLanes][carNo-1].physicalState.state[1] - x
          dxdot = globalISL1.neighborhood[numLanes][carNo-1].physicalState.state[3] - xdot
        end
      end

      ddotx = get_idm_accln(carIS.modelL0.frame.longitudinal, xdot, dxdot, g)

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
        ydot = -2.0
      elseif rnd < ydotCumProb[2]
        ydot = 0.0
      else
        ydot = 2.0
      end
      y = carPhySt.state[2]
      target_y = laneCenters[tLn]

      ydot *= (target_y - y)/abs(target_y - y)  #By convention

      push!(actions[ln], CarAction(ddotx, ydot))

      carNo += 1
    end
  end

  return actions
end

function updateNeighborState(globalISL1::GlobalStateL1, rng::AbstractRNG)
  #Maneuver specific
  laneCenters = [-3*LANE_WIDTH/2, -LANE_WIDTH/2, LANE_WIDTH/2, 3*LANE_WIDTH/2]

  #Maneuver independent
  egoLane = getLaneNo(globalISL1.ego)
  numLanes = length(globalISL1.neighborhood)
  updatedNeighborhood = Array{Array{CarLocalISL0,1},1}(numLanes)

    nxtFlPhySt = Nullable{CarPhysicalState}()
    nxtLdPhySt = Nullable{CarPhysicalState}()

  actions = getOtherCarsAction(globalISL1, rng) #This gives all cars' (sampled) action in the order they are on neighborhood

  for ln in 1:numLanes #For every lane
    numCars = length(globalISL1.neighborhood[ln])
    updatedNeighborhood[ln] = Array{CarLocalISL0,1}()
    for carIdx in 1:numCars #For every car in the lane
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
      ydot = 0
      if target_y > y
        ydot = min(carAct.dot_y, (target_y - y)/TIME_STEP)
      else
        ydot = max(carAct.dot_y, (target_y - y)/TIME_STEP)
      end

      xddot = carAct.ddot_x
      updatedCarPhySt = propagateCar(carPhySt, CarAction(xddot, ydot), TIME_STEP, rng, (TRN_NOISE_X, TRN_NOISE_Y, TRN_NOISE_XDOT))

      #TODO Update car's model, frame remains same, target lane remains same, only currNode changes
      carFSM = carFrame.policy
      #Generate observation for other car's lateral motion
      edgeLabel = "Undetermined"
      #If reached target
      #println("Target y = ", target_y, " y = ", y, " target lane = ", targetLane, " lane = ", ln)
      if abs(target_y - y) < 0.25
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
      #println("CurrNode = ",currNode.nodeLabel," UpdatedNode = ",updatedNode)
      updatedModel = CarModelL0(targetLane, updatedFrame, updatedNode)
      updatedIS = CarLocalISL0(updatedCarPhySt, updatedModel)
      push!(updatedNeighborhood[ln], updatedIS)
    end
  end

  return sortNeighborhood(updatedNeighborhood)

end

#=
     Change Lane to Right.
     y component of the state is taken relative to the lane marking
     The values of medians are given as Parameters of

=#
type ChangeLaneRightPOMDP <:POMDPs.POMDP{GlobalStateL1, Int64, EgoObservation}
  discount_factor::Float64
  goalReward::Float64       #Based on distance from target_y
  collisionCost::Float64    #Negative for collision, zero otherwise
  movementCost::Float64     #Every action has a negative (zero) cost associated to it, could be different from next one
  hardbrakingCost::Float64  #Greater cost for hardbraking than any other action
  discomfortCost::Float64   #Third party discomfort cost. If third party brakeshard.
  velocityCost::Float64     #Proportional to deviation in velocity from target_vel
  target_y::Float64         #Desired lateral position
  target_vel::Float64       #Target velocity
end

ChangeLaneRightPOMDP() = ChangeLaneRightPOMDP(0.9, -0.5, -50.0, 0.0, -3.0, -2.0, -0.2, LANE_WIDTH/2.0, AVG_HWY_VELOCITY)
discount(p::ChangeLaneRightPOMDP) = p.discount_factor
isterminal(::ChangeLaneRightPOMDP, act::Int64) = act == length(EgoActionSpace().actions)
#Needs to be updated. No need for absent
function isterminal(p::ChangeLaneRightPOMDP, st::GlobalStateL1)
  st.egoState.absent ? true : false
end
#From actions
n_actions(p::ChangeLaneRightPOMDP) = length(actions(p))
actions(::ChangeLaneRightPOMDP) = EgoActionSpace()

#=
Generate state

=#
type ChangeLaneRightNormalStateDist
  egoDist::NTuple{3,NormalDist}
  leftLeadingDist::NTuple{3,NormalDist}
  currLeadingDist::NTuple{3,NormalDist}
  rightLeadingDist::NTuple{3,NormalDist}
  rightFollowingDist::NTuple{3,NormalDist}
  farRightNearestDist::NTuple{3,NormalDist}
end

Base.eltype(::ChangeLaneRightNormalStateDist) = GlobalStateL1

function initial_state_distribution(p::ChangeLaneRightPOMDP)
  egoDist = NTuple{3,NormalDist}((NormalDist(0.0,0.0), NormalDist(-LANE_WIDTH/2.0, LANE_WIDTH * 3.0/16.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
  lLDist  = NTuple{3,NormalDist}((NormalDist(AVG_GAP/2.0,10.0), NormalDist(- 3 * LANE_WIDTH/2.0, LANE_WIDTH * 3.0/16.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
  cLDist  = NTuple{3,NormalDist}((NormalDist(AVG_GAP,10.0), NormalDist(-LANE_WIDTH/2.0, LANE_WIDTH * 3.0/16.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
  rLDist  = NTuple{3,NormalDist}((NormalDist(AVG_GAP/2.0,10.0), NormalDist(LANE_WIDTH/2.0, LANE_WIDTH * 3.0/16.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
  rFDist  = NTuple{3,NormalDist}((NormalDist(-AVG_GAP/2.0,10.0), NormalDist(LANE_WIDTH/2.0, LANE_WIDTH * 3.0/16.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
  fRNDist = NTuple{3,NormalDist}((NormalDist(0.0,10.0), NormalDist(3 * LANE_WIDTH/2.0, LANE_WIDTH * 3.0/16.0), NormalDist(AVG_HWY_VELOCITY, VEL_STD_DEV)))
  return(ChangeLaneRightNormalStateDist(egoDist, lLDist, cLDist, rLDist, rFDist, fRNDist))
end

function rand(rng::AbstractRNG, d::ChangeLaneRightNormalStateDist, frameList::Array{CarFrameL0,1}=getFrameList())
  egoState = randCarPhysicalState(rng, d.egoDist, true)

  neighborhood = Array{Array{CarLocalISL0,1}}(NUM_INTENTIONS)
  #TODO: There has to be a better way
  for i in 1:NUM_INTENTIONS
    neighborhood[i] = Array{CarLocalISL0,1}()
  end

  leftLeadingState = randCarLocalISL0(rng, d.leftLeadingDist, [0.5, 0.2, 0.2, 0.1], frameList)
  #Assert that it is indeed ahead of ego vehicle
  if leftLeadingState.physicalState.state[1] < egoState.state[1]
    leftLeadingState.physicalState.state = (egoState.state[1] + AVG_GAP/2.0, leftLeadingState.physicalState.state[2], leftLeadingState.physicalState.state[3])
  end
  push!(neighborhood[1], leftLeadingState)

  currLeadingState = randCarLocalISL0(rng, d.currLeadingDist, [0.2, 0.5, 0.2, 0.1], frameList)
  #Assert that it is indeed ahead of ego vehicle
  if currLeadingState.physicalState.state[1] < egoState.state[1]
    currLeadingState.physicalState.state = (egoState.state[1] + AVG_GAP/2.0, currLeadingState.physicalState.state[2], currLeadingState.physicalState.state[3])
  end
  push!(neighborhood[2], currLeadingState)

  rightLeadingState = randCarLocalISL0(rng, d.rightLeadingDist, [0.1, 0.2, 0.5, 0.2], frameList)
  #Assert that it is indeed ahead of ego vehicle
  if rightLeadingState.physicalState.state[1] < egoState.state[1]
    rightLeadingState.physicalState.state = (egoState.state[1] + AVG_GAP/2.0, rightLeadingState.physicalState.state[2], rightLeadingState.physicalState.state[3])
  end
  push!(neighborhood[3], rightLeadingState)

  rightFollowingState = randCarLocalISL0(rng, d.rightFollowingDist, [0.0, 0.2, 0.6, 0.2], frameList)
  #Assert that it is indeed behind ego vehicle
  if rightFollowingState.physicalState.state[1] > egoState.state[1]
    rightFollowingState.physicalState.state = (egoState.state[1] - AVG_GAP/2.0, rightFollowingState.physicalState.state[2], rightFollowingState.physicalState.state[3])
  end
  push!(neighborhood[3], rightFollowingState)

  farRightClosestState = randCarLocalISL0(rng, d.farRightNearestDist, [0.0, 0.1, 0.3, 0.6], frameList)
  push!(neighborhood[4], farRightClosestState)

  return(GlobalStateL1(egoState, neighborhood))
end


#Generate observation
function generate_o(p::ChangeLaneRightPOMDP, s::Union{GlobalStateL1,Void}, a::Union{Int64,Void}, sp::GlobalStateL1, rng::AbstractRNG)
  numLanes = length(sp.neighborhood)
  nbrObs = Array{Array{CarPhysicalState,1},1}(numLanes)

  egoPos = sp.ego

  for ln in 1:numLanes
    nbrObs[ln] = Array{CarPhysicalState,1}()
    for car in sp.neighborhood[ln]
      phySt = car.physicalState
      x = phySt.state[1]
      y = phySt.state[2]
      xdot = phySt.state[3]

      noise_x = randn(rng) * OBS_NOISE_X
      noise_y = randn(rng) * OBS_NOISE_Y
      noise_xdot = randn(rng) * OBS_NOISE_XDOT
      x += noise_x
      y += noise_y
      xdot += noise_xdot

      carObs = CarPhysicalState(phySt.absent, (x,y,xdot))
      push!(nbrObs[ln], carObs)
    end
  end

  return EgoObservation(egoPos, nbrObs)
end

#Generate next state
function generate_s(p::ChangeLaneRightPOMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  actionSet = EgoActionSpace()
  act = actionSet.actions[a]
  if s.ego.absent
    return s
  end
  if a == length(actionSet.actions)
    return GlobalStateL1(CarPhysicalState(true, s.ego.state), s.neighborhood)
  end
  if checkForCollision(s)
    return GlobalStateL1(CarPhysicalState(true, s.ego.state), s.neighborhood)
  end

  egoState = propagateCar(s.ego, act, TIME_STEP, rng, (TRN_NOISE_X, TRN_NOISE_Y, TRN_NOISE_XDOT))
  neighborhood = updateNeighborState(s, rng)

  sp = GlobalStateL1(egoState, neighborhood)

  return sp
end

#Generate reward
function reward(p::ChangeLaneRightPOMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  actionSet = EgoActionSpace()
  act = actionSet.actions[a]
  if (s.ego.absent )
    return 0.0
  end
  if a == length(actionSet.actions)
    return p.collisionCost
  end

  reward = 0.0
  egoSt = s.ego
  nbrhood = s.neighborhood
  #TODO: More stuff here
  if checkForCollision(s)
    return p.collisionCost
  end
  #if abs(egoSt.state[2] - p.target_y) < 0.5
    reward += (p.goalReward * abs(egoSt.state[2] - p.target_y))
  #end
  if act.ddot_x <= -4.0
    reward += p.hardbrakingCost
  end
  nbrActions = getOtherCarsAction(s, rng) #Randomness is iffy but IDM part is constant, so no problem there
  discomfort = false
  for ln in 1:length(nbrActions)
    for carAct in nbrActions[ln]
      if carAct.ddot_x <= -4.0
        reward += p.discomfortCost
        discomfort = true
        break
      end
    end
    if discomfort
      break
    end
  end
  xdot = egoSt.state[3]
  reward += (abs(xdot - p.target_vel) * p.velocityCost)
  return reward
end

#TODO: Needs to be refined
function generate_sor(p::ChangeLaneRightPOMDP, s::GlobalStateL1, a::Int64, rng::AbstractRNG)
  sp = generate_s(p, s, a, rng)
  o = generate_o(p, nothing, nothing, sp, rng)

  r = reward(p,s,a,rng)

  return sp, o, r
end

function initial_state(p::ChangeLaneRightPOMDP, rng::AbstractRNG)
  isd = initial_state_distribution(p)
  return rand(rng, isd)
end


type ChangeLaneRightLowerBound
    rng::AbstractRNG
end

type ChangeLaneRightUpperBound
    rng::AbstractRNG
end

function lower_bound(lb::ChangeLaneRightLowerBound, p::ChangeLaneRightPOMDP, s::GlobalStateL1)
    return p.collisionCost
end

function upper_bound(ub::ChangeLaneRightUpperBound, p::ChangeLaneRightPOMDP, s::GlobalStateL1)
    return 0.0
end

function init_lower_action(p::ChangeLaneRightPOMDP)
    length(EgoActionSpace())
end
