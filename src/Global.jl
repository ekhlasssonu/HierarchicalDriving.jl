#include("Agent.jl")

#Noise parameters
OBS_NOISE_X = 0.1
OBS_NOISE_Y = 0.01
OBS_NOISE_XDOT = 1

TRN_NOISE_X = 0.1
TRN_NOISE_Y = 0.0
TRN_NOISE_XDOT = 0.1

#POMDP parameters
TIME_STEP = 0.2

#Sampling initial state distributions
#Origin (x -> start location of ego vehicle), (y -> dividing line), (\dot(x) -> 25 mps)
#A neighboring vehicle can be absent (too far away) with probability 0.15
#Velocities vary according to normal distribution with std. dev. of 2m/s

type NormalDist
  mean::Float64
  std::Float64
end
function sample(rng::AbstractRNG, nd::NormalDist)
  return nd.mean,randn(rng) * nd.std
end

type UniformDist
  lb::Float64
  ub::Float64
end
function sample(rng::AbstractRNG, ud::UniformDist)
  fact = ub - lb
  return lb + Base.rand(rng) * fact
end

#State of world. Should work for upper level as well.
type GlobalStateL1
  ego::CarPhysicalState
  neighborhood::Array{Array{CarLocalISL0,1},1}  #In order  of lane no. and x position
end
==(s1::GlobalStateL1,s2::GlobalStateL1) = (s1.ego == s2.ego) && (s1.neighborhood == s2.neighborhood)
Base.hash(s1::GlobalStateL1, h::UInt64 = zero(UInt64)) = hash(s1.ego, hash(s1.neighborhood, h))
Base.copy(s1::GlobalStateL1) = GlobalStateL1(s1.ego, s1.neighborhood)


type EgoObservation
  ego::CarPhysicalState
  neighborhood::Array{Array{CarPhysicalState,1},1}  #In order  of lane no. and x position
end
==(s1::EgoObservation,s2::EgoObservation) = (s1.ego == s2.ego) && (s1.neighborhood == s2.neighborhood)
Base.hash(s1::EgoObservation, h::UInt64 = zero(UInt64)) = hash(s1.ego, hash(s1.neighborhood, h))
Base.copy(s1::EgoObservation) = EgoObservation(s1.ego, s1.neighborhood)


#=
     Motion model implemented here
     Find the next state of a car given the current physical state and
     action = <\ddot{x}, \dot{y}>

=#
#=
     Motion model implemented here
     Find the next state of a car given the current physical state and
     action = <\ddot{x}, \dot{y}>

=#
function propagateCar(s::CarPhysicalState, a::CarAction, dt::Float64, rng::AbstractRNG, noise::NTuple{3, Float64}=NTuple{3, Float64}((0,0,0)))
  # If car is absent or car velocity is negative or actions is terminal
  if(s.absent || s.state[3] < 0.0) || (a.ddot_x == Inf) || (a.ddot_x == -Inf) || (a.dot_y == Inf) || (a.dot_y == -Inf)
    return s
  end
  x = s.state[1]
  y = s.state[2]
  xdot = s.state[3]

  xddot = a.ddot_x
  ydot = a.dot_y


  dx = xdot * dt + 0.5 * xddot * dt^2
  x += (dx + dx * randn(rng) * noise[1]) # x += \dotx * dt + 0.5 \ddotx dt^2 + noise, noise is proportionate to absolute displacement

  dy = ydot * dt
  y += (dy + dy * randn(rng) * noise[2]) # y += \doty * dt + noise

  dxdot = xddot * dt
  xdot += dxdot + dxdot * randn(rng) * noise[3] #\dotx += \ddotx * dt + noise
  if xdot < 0.0
    xdot = 0.0
  end #Car doesn't move in reverse direction
  sp = (x,y,xdot)

  return CarPhysicalState(false, sp)
end


#=

Generate state

=#
function randCarPhysicalState(rng::AbstractRNG, d::NTuple{3,NormalDist},  ego::Bool = false)

  #phySt = Nullable{CarPhysicalState}()
  #ego vehicle can't be absent. Otherwise absent with probability 0.25, no reason for 0.25
  absent = !ego && (Base.rand(rng) < 0.25 ? true : false)

  x = d[1].mean + randn(rng) * d[1].std
  y = d[2].mean
  y_noise = randn(rng) * d[2].std

  y_noise >  (LANE_WIDTH/2.0) ? (y_noise = (LANE_WIDTH/2.0)) : (y_noise < -LANE_WIDTH/2.0) ? (y_noise = -LANE_WIDTH/2.0) : nothing

  xdot = d[3].mean
  xdot_noise = randn(rng) * d[3].std
  xdot_noise > 2 * VEL_STD_DEV ? xdot_noise = 2 * VEL_STD_DEV : (xdot_noise < - 2 * VEL_STD_DEV ? xdot_noise = -2 * VEL_STD_DEV : nothing)

  xdot += xdot_noise

  return(CarPhysicalState(absent, (x,y,xdot)))
end

function randCarLocalISL0(rng::AbstractRNG, d::NTuple{3,NormalDist}, intentionDist::Array{Float64,1}, frameList::Array{CarFrameL0,1})
  phySt = randCarPhysicalState(rng, d, false)
  cumProbDist = cumsum(intentionDist)
  x = Base.rand(rng)
  targetLane = 1
  while (targetLane < length(cumProbDist)) && (x <= cumProbDist[targetLane])
    targetLane += 1
  end
  if (targetLane > length(intentionDist))
    println("THIS SHOULD NOT HAPPEN")
    println("THIS SHOULD NOT HAPPEN")
    println("THIS SHOULD NOT HAPPEN")
    targetLane = length(intentionDist)
  end
  frame = frameList[Base.rand(1:length(frameList))]
  node = frame.policy.nodeSet[1]

  localISL0 = CarLocalISL0(phySt, CarModelL0(targetLane, frame, node))
  return localISL0
end

function checkForCollision(gblISL1::GlobalStateL1)
  egoState = gblISL1.ego
  if egoState.absent
    return true
  end
  nbrhood = gblISL1.neighborhood
  numLanes = length(nbrhood)
  for ln in 1:numLanes
    for carIS in nbrhood[ln]
      if collision(egoState, carIS)
        return true
      end
    end
  end
  return false
end
