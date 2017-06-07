#include("FSM.jl")
#include("DrivingParams.jl")

#import Base: ==, +, *, -, copy, Random, hash
#importall POMDPs, MCVI

abstract Action
abstract State
abstract Frame
abstract Model #Associated with a Frame and contains a belief/node of the controller

abstract LocalIS
abstract CarState <: State
abstract CarFrame <: Frame
abstract CarModel <: Model




type CarAction <: Action
  ddot_x::Float64
  dot_y::Float64
end
==(a1::CarAction, a2::CarAction) = (a1.ddot_x == a2.ddot_x) && (a1.dot_y == a2.dot_y)
Base.hash(a1::CarAction, h::UInt64=zero(UInt64)) = hash(a1.ddot_x, hash(a1.dot_y,h))
Base.copy(a1::CarAction) = CarAction(a1.ddot_x, a1.dot_y)


type CarPhysicalState <: CarState
  #absent::Bool
  state::NTuple{3,Float64} #<x, y, \dot{x} >.
end
#CarPhysicalState(state::NTuple{3,Float64}) = CarPhysicalState(false, state)
CarPhysicalState(st::CarPhysicalState) = CarPhysicalState(st.state)
==(s1::CarPhysicalState, s2::CarPhysicalState) = (s1.state == s2.state)
>(s1::CarPhysicalState, s2::CarPhysicalState) = (s1.state[1] > s2.state[2])
<(s1::CarPhysicalState, s2::CarPhysicalState) = (s1.state[1] < s2.state[2])
Base.hash(s::CarPhysicalState, h::UInt64=zero(UInt64)) = hash(s.state,h)
Base.copy(s::CarPhysicalState) = CarPhysicalState(s.state)

collision(s1::CarPhysicalState, s2::CarPhysicalState) = (abs(s1.state[1] - s2.state[1]) < CAR_LENGTH) && (abs(s1.state[2] - s2.state[2]) < CAR_WIDTH)


type LowLevelCarFrameL0 <: CarFrame
  longitudinal::IDMParam
  lateral::MOBILParam
  policy::FSM{Int64, Float64, String} #Edge label is string for now
  carLength::Float64
  carWidth::Float64
end
LowLevelCarFrameL0(long::IDMParam, lat::MOBILParam, pol::FSM{Int64, Float64, String}) = LowLevelCarFrameL0(long, lat, pol, CAR_LENGTH, CAR_WIDTH)
==(f1::LowLevelCarFrameL0, f2::LowLevelCarFrameL0) = (f1.longitudinal == f2.longitudinal) && (f1.lateral == f2.lateral) && (f1.policy == f2.policy) && (f1.carLength == f2.carLength) && (f1.carWidth == f2.carWidth)
Base.hash(f::LowLevelCarFrameL0, h::UInt64=zero(UInt64)) = hash(f.longitudinal, hash(f.lateral, hash(f.policy, hash(f.carLength, hash(f.carWidth, h)))))
Base.copy(f::LowLevelCarFrameL0) = LowLevelCarFrameL0(f.longitudinal, f.lateral, f.policy, f.carLength, f.carWidth)

type LowLevelCarModelL0 <: CarModel
  targetLane::Int64
  frame::LowLevelCarFrameL0
  currNode::FSM_Node{Int64} #Node in car's policy FSM, equivalent to belief
end
==(m1::LowLevelCarModelL0, m2::LowLevelCarModelL0) = (m1.targetLane == m2.targetLane) && (m1.frame == m2.frame) && (m1.currNode == m2.currNode)
Base.hash(m::LowLevelCarModelL0, h::UInt64=zero(UInt64)) = hash(m.targetLane, hash(m.frame, hash(m.currNode, h)))
Base.copy(m::LowLevelCarModelL0) = LowLevelCarModelL0(m.targetLane, m.frame, m.currNode)

type UpperLevelCarModelL0 <: CarModel
  frame::LowLevelCarFrameL0
  currNode::FSM_Node{Int64}
end
==(m1::UpperLevelCarModelL0, m2::UpperLevelCarModelL0) = (m1.frame == m2.frame) && (m1.currNode == m2.currNode)
Base.hash(m::UpperLevelCarModelL0, h::UInt64=zero(UInt64)) = hash(m.frame, hash(m.currNode, h))
Base.copy(m::UpperLevelCarModelL0) = UpperLevelCarModelL0(m.frame, m.currNode)

type CarLocalIS{M <: CarModel}
  physicalState::CarPhysicalState
  model::M
end
==(is1::CarLocalIS, is2::CarLocalIS) = (is1.physicalState == is2.physicalState) && (is1.model == is2.model)
Base.hash(is::CarLocalIS, h::UInt64=zero(UInt64)) = hash(is.physicalState, hash(is.model, h))
Base.copy(is::CarLocalIS) = CarLocalIS(is.physicalState, is.model)

collision(s1::CarPhysicalState, is2::CarLocalIS) = (abs(s1.state[1] - is2.physicalState.state[1]) < (CAR_LENGTH + is2.model.frame.carLength)/2) && (abs(s1.state[2] - is2.physicalState.state[2]) < (CAR_WIDTH + is2.model.frame.carWidth)/2)
collision(is1::CarLocalIS, is2::CarLocalIS) = (abs(is1.physicalState.state[1] - is2.physicalState.state[1]) < (is1.model.frame.carLength + is2.model.frame.carLength)/2) && (abs(is1.physicalState.state[2] - is2.physicalState.state[2]) < (is1.model.frame.carWidth + is2.model.frame.carWidth)/2)

#ddot_x is determined by IDM, Only for dot_y
function createFSM()
  nodeSet = Array{FSM_Node{Int64},1}(5)
  for i in 1:5
    nodeSet[i] = FSM_Node{Int64}(i)
  end
  edgeLabels = [FSM_Edge{String}("Reached"),
                FSM_Edge{String}("SafeSmooth"),
                FSM_Edge{String}("UnsafeOrUnsmooth"),
                FSM_Edge{String}("Undetermined")]
  actionProb = Dict{Tuple{FSM_Node{Int64}, Float64}, Float64}()

  actionProb[(nodeSet[1], 0.0)] = 0.5  #1: Keep straight with prob 0.5
  actionProb[(nodeSet[1], 2.0)] = 0.5  #1: Move toward target with prob 0.5
  actionProb[(nodeSet[2], 0.0)] = 1.0  #2: Reached target keep straight for ever
  actionProb[(nodeSet[3],-2.0)] = 1.0  #3: Unsafe to move towards target, move to the center of the current lane
  actionProb[(nodeSet[4], 2.0)] = 1.0  #4: Move towards the target

  transitionProb = Dict{Tuple{FSM_Node{Int64}, FSM_Edge{String}, FSM_Node{Int64}}, Float64}()

  transitionProb[(nodeSet[1], edgeLabels[1], nodeSet[2])] = 1.0 #Reached target_y transition to 2
  transitionProb[(nodeSet[1], edgeLabels[2], nodeSet[4])] = 1.0 #SafeSmooth so transition to 4
  transitionProb[(nodeSet[1], edgeLabels[3], nodeSet[3])] = 1.0 #Unsafe, transition to 3
  transitionProb[(nodeSet[1], edgeLabels[4], nodeSet[3])] = 0.5 #Undeterminable move to 3 with prob 0.5
  transitionProb[(nodeSet[1], edgeLabels[4], nodeSet[4])] = 0.5 #Undeterminable move to 4 with prob 0.5

  transitionProb[(nodeSet[2], edgeLabels[1], nodeSet[2])] = 1.0 #Reached, ∀ observation keep straight
  transitionProb[(nodeSet[2], edgeLabels[2], nodeSet[2])] = 1.0
  transitionProb[(nodeSet[2], edgeLabels[3], nodeSet[2])] = 1.0
  transitionProb[(nodeSet[2], edgeLabels[4], nodeSet[2])] = 1.0

  transitionProb[(nodeSet[3], edgeLabels[1], nodeSet[3])] = 1.0 #Shouldn't recieve this observation. Just stay here.
  transitionProb[(nodeSet[3], edgeLabels[2], nodeSet[4])] = 1.0 #Safe to move, transition to 4
  transitionProb[(nodeSet[3], edgeLabels[3], nodeSet[3])] = 1.0 #Unsafe, Keep in 3
  transitionProb[(nodeSet[3], edgeLabels[4], nodeSet[3])] = 0.5 #Undeterminable, stay here with prob. 0.5
  transitionProb[(nodeSet[3], edgeLabels[4], nodeSet[4])] = 0.5 #Undeterminable, transition to 4 with prob 0.5

  transitionProb[(nodeSet[4], edgeLabels[1], nodeSet[2])] = 1.0 #Reached, transition to 2
  transitionProb[(nodeSet[4], edgeLabels[2], nodeSet[4])] = 1.0 #safe, keep moving towards target.
  transitionProb[(nodeSet[4], edgeLabels[3], nodeSet[3])] = 1.0 #unsafe, return to center of current lane
  transitionProb[(nodeSet[4], edgeLabels[4], nodeSet[4])] = 1.0 #undeterminable, keep moving

  return FSM{Int64, Float64, String}(nodeSet, edgeLabels, actionProb, transitionProb)
end

function getFrameList()
  frameList = Array{LowLevelCarFrameL0,1}(3)

    fsm = createFSM()
    #TimidFrame
    idmTimid = createIDM_timid()
    mobilTimid = createMOBIL_timid()
    frameList[1] = LowLevelCarFrameL0(idmTimid, mobilTimid, fsm, CAR_LENGTH, CAR_WIDTH)

    #NormalFrame
    idmNormal = createIDM_normal()
    mobilNormal = createMOBIL_normal()
    frameList[2] = LowLevelCarFrameL0(idmNormal, mobilNormal, fsm, CAR_LENGTH, CAR_WIDTH)

    #AggressiveFrame
    idmAggressive = createIDM_aggressive()
    mobilAggressive = createMOBIL_aggressive()
    frameList[3] = LowLevelCarFrameL0(idmAggressive, mobilAggressive, fsm, CAR_LENGTH, CAR_WIDTH)

  return frameList
end

type EgoActionSpace
  actions::Array{CarAction,1}
end
#I am not sure how these function are supposed to work but just adding this line  for good measure
EgoActionSpace() = EgoActionSpace([CarAction(0.0,0.0), CarAction(0.0,-2.0), CarAction(0.0,2.0), CarAction(-2.0,-2.0), CarAction(-2.0,0.0), CarAction(-2.0,2.0), CarAction(2.0,-2.0), CarAction(2.0,0.0), CarAction(2.0,2.0), CarAction(-6.0,0.0)])

Base.length(asp::EgoActionSpace) = length(asp.actions)
iterator(actSpace::EgoActionSpace) = 1:length(actSpace.actions)
dimensions(::EgoActionSpace) = 1

#Sample random action
Base.rand(rng::AbstractRNG, asp::EgoActionSpace) = Base.rand(rng, 1:Base.length(asp))

#Modified MOBIL parameters. Ignoring parts for ensuring smooth traffic flow. Self interested lane changes only.
function isLaneChangeSafe(mobil::MOBILParam, idm::IDMParam, selfState::CarPhysicalState, nextFollowingState::CarPhysicalState)
  tilda_xn_ddot = get_idm_accln(idm, nextFollowingState.state[3], nextFollowingState.state[3] - selfState.state[3], selfState.state[1]-nextFollowingState.state[1]-CAR_LENGTH)
  if tilda_xn_ddot < -mobil.b_safe
    #println("\tUnsafe: tilda_xn_ddot = $tilda_xn_ddot")
    return false
  end
  return true
end

function isLaneChangeSmooth(mobil::MOBILParam, idm::IDMParam, selfState::CarPhysicalState, nextLeadingState::CarPhysicalState, xc_ddot::Float64)
	tilda_xc_ddot = get_idm_accln(idm, selfState.state[3], selfState.state[3]-nextLeadingState.state[3], nextLeadingState.state[1]-selfState.state[1]-CAR_LENGTH)
  if ((tilda_xc_ddot - xc_ddot) < mobil.a_thr) #Should I add this too? || (tilda_xc_ddot < -mobil.b_safe)
    #println("\tUnsmooth: tilda_xc_ddot - xc_ddot = $tilda_xc_ddot - $xc_ddot = ", tilda_xc_ddot - xc_ddot, " a_thr = ",mobil.a_thr)
    #println("\txdot = ",selfState.state[3]," dxdot = ", selfState.state[3],"-",nextLeadingState.state[3], "=", selfState.state[3]-nextLeadingState.state[3], " g = ", nextLeadingState.state[1],"-",selfState.state[1],"-$CAR_LENGTH = ",nextLeadingState.state[1]-selfState.state[1]-CAR_LENGTH)
    return false
  end
  return true
end
