#include("FSM.jl")
#include("DrivingParams.jl")

#import Base: ==, +, *, -, copy, Random, hash
#importall POMDPs, MCVI

abstract Action
abstract State
abstract Frame
abstract Model #Associated with a Frame and contains a belief/node of the controller

type CarAction <: Action
  ddot_x::Float64
  dot_y::Float64
end
==(a1::CarAction, a2::CarAction) = (a1.ddot_x == a2.ddot_x) && (a1.dot_y == a2.dot_y)
Base.hash(a1::CarAction, h::UInt64=zero(UInt64)) = hash(a1.ddot_x, hash(a1.dot_y,h))
Base.copy(a1::CarAction) = CarAction(a1.ddot_x, a1.dot_y)


type CarPhysicalState <: State
  absent::Bool
  state::NTuple{3,Float64} #<x, y, \dot{x} >.
end
CarPhysicalState(st::CarPhysicalState) = CarPhysicalState(st.absent, st.state)
==(s1::CarPhysicalState, s2::CarPhysicalState) = (s1.absent == s2.absent) && (s1.state == s2.state)
Base.hash(s::CarPhysicalState, h::UInt64=zero(UInt64)) = hash(s.absent, hash(s.state,h))
Base.copy(s::CarPhysicalState) = CarPhysicalState(s.absent, s.state)

collision(s1::CarPhysicalState, s2::CarPhysicalState) = (!s1.absent) && (!s2.absent) && (abs(s1.state[1] - s2.state[1]) < CAR_LENGTH/2) && (abs(s1.state[2] - s2.state[2]) < CAR_WIDTH/2)


type CarFrameL0 <: Frame
  longitudinal::IDMParam
  lateral::MOBILParam
  policy::FSM{Int64, Float64, String} #Edge label is string for now
  carLength::Float64
  carWidth::Float64
end
==(f1::CarFrameL0, f2::CarFrameL0) = (f1.longitudinal == f2.longitudinal) && (f1.lateral == f2.lateral) && (f1.policy == f2.policy) && (f1.carLength == f2.carLength) && (f1.carWidth == f2.carWidth)
Base.hash(f::CarFrameL0, h::UInt64=zero(UInt64)) = hash(f.longitudinal, hash(f.lateral, hash(f.policy, hash(f.carLength, hash(f.carWidth, h)))))
Base.copy(f::CarFrameL0) = CarFrameL0(f.longitudinal, f.lateral, f.policy, f.carLength, f.carWidth)

type CarModelL0 <: Model
  targetLane::Int64
  frame::CarFrameL0
  currNode::FSM_Node{Int64} #Node in car's policy FSM, equivalent to belief
end
==(m1::CarModelL0, m2::CarModelL0) = (m1.targetLane == m2.targetLane) && (m1.frame == m2.frame) && (m1.currNode == m2.currNode)
Base.hash(m::CarModelL0, h::UInt64=zero(UInt64)) = hash(m.targetLane, hash(m.frame, hash(m.currNode, h)))
Base.copy(m::CarModelL0) = CarModelL0(m.targetLane, m.frame, m.currNode)

type CarLocalISL0
  physicalState::CarPhysicalState
  modelL0::CarModelL0
end
==(is1::CarLocalISL0, is2::CarLocalISL0) = (is1.physicalState == is2.physicalState) && (is1.modelL0 == is2.modelL0)
Base.hash(is::CarLocalISL0, h::UInt64=zero(UInt64)) = hash(physicalState, hash(modelL0, h))
Base.copy(is::CarLocalISL0) = CarLocalISL0(is.physicalState, is.modelL0)

collision(s1::CarPhysicalState, is2::CarLocalISL0) = (!s1.absent) && (!is2.physicalState.absent) && (abs(s1.state[1] - is2.physicalState.state[1]) < (CAR_LENGTH + is2.modelL0.frame.carLength)/2) && (abs(s1.state[2] - is2.physicalState.state[2]) < (CAR_WIDTH + is2.modelL0.frame.carWidth)/2)
collision(is1::CarLocalISL0, is2::CarLocalISL0) = (!is1.physicalState.absent) && (!is2.physicalState.absent) && (abs(is1.physicalState.state[1] - is2.physicalState.state[1]) < (is1.modelL0.frame.carLength + is2.modelL0.frame.carLength)/2) && (abs(s1.state[2] - is2.physicalState.state[2]) < (is1.modelL0.frame.carWidth + is2.modelL0.frame.carWidth)/2)

#ddot_x is determined by IDM, Only for dot_y
function createFSM()
  nodeSet = Array{FSM_Node{Int64},1}(4)
  for i in 1:4
    nodeSet[i] = FSM_Node{Int64}(i)
  end
  edgeLabels = [FSM_Edge{String}("Reached"), FSM_Edge{String}("SafeSmooth"), FSM_Edge{String}("UnsafeOrUnsmooth"), FSM_Edge{String}("Undetermined")]
  actionProb = Dict{Tuple{FSM_Node{Int64}, Float64}, Float64}()

  actionProb[(nodeSet[1],0.0)] = 0.5
  actionProb[(nodeSet[1],2.0)] = 0.25
  actionProb[(nodeSet[1],-2.0)] = 0.25
  actionProb[(nodeSet[2],0.0)] = 1.0
  actionProb[(nodeSet[3],0.0)] = 1.0
  actionProb[(nodeSet[4],2.0)] = 1.0

  transitionProb = Dict{Tuple{FSM_Node{Int64}, FSM_Edge{String}, FSM_Node{Int64}}, Float64}()

  transitionProb[(nodeSet[1], edgeLabels[1], nodeSet[2])] = 1.0
  transitionProb[(nodeSet[1], edgeLabels[2], nodeSet[4])] = 1.0
  transitionProb[(nodeSet[1], edgeLabels[3], nodeSet[3])] = 1.0
  transitionProb[(nodeSet[1], edgeLabels[4], nodeSet[3])] = 0.8
  transitionProb[(nodeSet[1], edgeLabels[4], nodeSet[4])] = 0.2

  transitionProb[(nodeSet[2], edgeLabels[1], nodeSet[2])] = 1.0
  transitionProb[(nodeSet[2], edgeLabels[2], nodeSet[2])] = 1.0
  transitionProb[(nodeSet[2], edgeLabels[3], nodeSet[2])] = 1.0
  transitionProb[(nodeSet[2], edgeLabels[4], nodeSet[2])] = 1.0

  transitionProb[(nodeSet[3], edgeLabels[1], nodeSet[2])] = 1.0
  transitionProb[(nodeSet[3], edgeLabels[2], nodeSet[4])] = 1.0
  transitionProb[(nodeSet[3], edgeLabels[3], nodeSet[3])] = 1.0
  transitionProb[(nodeSet[3], edgeLabels[4], nodeSet[3])] = 0.8
  transitionProb[(nodeSet[3], edgeLabels[4], nodeSet[4])] = 0.2

  transitionProb[(nodeSet[4], edgeLabels[1], nodeSet[2])] = 1.0
  transitionProb[(nodeSet[4], edgeLabels[2], nodeSet[4])] = 1.0
  transitionProb[(nodeSet[4], edgeLabels[3], nodeSet[4])] = 1.0
  transitionProb[(nodeSet[4], edgeLabels[4], nodeSet[4])] = 1.0

  return FSM{Int64, Float64, String}(nodeSet, edgeLabels, actionProb, transitionProb)
end

function getFrameList()
  frameList = Array{CarFrameL0,1}(3)

    fsm = createFSM()
    #TimidFrame
    idmTimid = createIDM_timid()
    mobilTimid = createMOBIL_timid()
    frameList[1] = CarFrameL0(idmTimid, mobilTimid, fsm, CAR_LENGTH, CAR_WIDTH)

    #NormalFrame
    idmNormal = createIDM_normal()
    mobilNormal = createMOBIL_normal()
    frameList[2] = CarFrameL0(idmNormal, mobilNormal, fsm, CAR_LENGTH, CAR_WIDTH)

    #AggressiveFrame
    idmAggressive = createIDM_aggressive()
    mobilAggressive = createMOBIL_aggressive()
    frameList[3] = CarFrameL0(idmAggressive, mobilAggressive, fsm, CAR_LENGTH, CAR_WIDTH)

  return frameList
end

function getModelList()
  modelList = Array{CarModelL0,1}()
  frameList = getFrameList()
  for targetLane in 1:4
    for fr in frameList
      for node in fr.policy.nodeSet
        model = CarModelL0(targetLane, fr, node)
        push!(modelList, model)
      end
    end
  end
  return modelList
end


type EgoActionSpace
  actions::Array{CarAction,1}
end
#I am not sure how these function are supposed to work but just adding this line  for good measure
EgoActionSpace() = EgoActionSpace([CarAction(-2.0,-2.0), CarAction(-2.0,0.0), CarAction(-2.0,2.0), CarAction(0.0,-2.0), CarAction(0.0,0.0), CarAction(0.0,2.0), CarAction(2.0,-2.0), CarAction(2.0,0.0), CarAction(2.0,2.0), CarAction(-6.0,0.0), CarAction(Inf,Inf)])

Base.length(asp::EgoActionSpace) = length(asp.actions)
iterator(actSpace::EgoActionSpace) = actSpace.actions
dimensions(::EgoActionSpace) = 1

#Sample random action
Base.rand(rng::AbstractRNG, asp::EgoActionSpace) = Base.rand(rng, 1:Base.length(asp))

#Modified MOBIL parameters. Ignoring parts for ensuring smooth traffic flow. Self interested lane changes only.
function isLaneChangeSafe(mobil::MOBILParam, idm::IDMParam, selfState::CarPhysicalState, nextFollowingState::CarPhysicalState)
  tilda_xn_ddot = get_idm_accln(idm, nextFollowingState.state[3], selfState.state[3]-nextFollowingState.state[3], selfState.state[1]-nextFollowingState.state[1])
  if tilda_xn_ddot < -mobil.b_safe
    return false
  end
  return true
end

function isLaneChangeSmooth(mobil::MOBILParam, idm::IDMParam, selfState::CarPhysicalState, nextLeadingState::CarPhysicalState, xc_ddot::Float64)
	tilda_xc_ddot = get_idm_accln(idm, selfState.state[3], nextLeadingState.state[3]-selfState.state[3], nextLeadingState.state[1]-selfState.state[1])
  if (tilda_xc_ddot - xc_ddot) < mobil.a_thr
    return false
  end
  return true
end
