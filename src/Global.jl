#include("Agent.jl")

#Noise parameters
OBS_NOISE_X = 0.1
OBS_NOISE_Y = 0.3
OBS_NOISE_XDOT = 1.0

TRN_NOISE_X = 0.0
TRN_NOISE_Y = 0.0
TRN_NOISE_XDOT = 0.1

#POMDP parameters
TIME_STEP = 0.5

#Sampling initial state distributions
#Origin (x -> start location of ego vehicle), (y -> dividing line), (\dot(x) -> 25 mps)
#A neighboring vehicle can be absent (too far away) with probability 0.25
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
type GlobalStateL1{C <: CarLocalIS}
  terminal::Int64 #0 for not terminal, 1 for collision, 2 for success
  ego::CarPhysicalState
  neighborhood::Array{Array{C,1},1}  #In order  of lane no. and x position
end
GlobalStateL1(ego::CarPhysicalState, neighborhood::Array{Array{CarLocalIS,1},1}) = GlobalStateL1(0, ego::CarPhysicalState, neighborhood::Array{Array{CarLocalIS,1},1})
==(s1::GlobalStateL1,s2::GlobalStateL1) = (s1.terminal == s2.terminal) && (s1.ego == s2.ego) && (s1.neighborhood == s2.neighborhood)
Base.hash(s1::GlobalStateL1, h::UInt64 = zero(UInt64)) = hash(s1.terminal, hash(s1.ego, hash(s1.neighborhood, h)))
Base.copy(s1::GlobalStateL1) = GlobalStateL1(s1.terminal, s1.ego, s1.neighborhood)


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

function getLaneNo(phySt::CarPhysicalState, laneCenters::Array{Float64,1})
  for ln in 1:length(laneCenters)-1
    if phySt.state[2] < laneCenters[ln]+LANE_WIDTH/2.0
      return ln
    end
  end
  return length(laneCenters)
end

function propagateCar(s::CarPhysicalState, a::CarAction, dt::Float64, rng::AbstractRNG, noise::NTuple{3, Float64}=NTuple{3, Float64}((0.0,0.0,0.0)))
  # If car is absent or car velocity is negative or actions is terminal
  #=if(s.absent || s.state[3] < 0.0) || (a.ddot_x == Inf) || (a.ddot_x == -Inf) || (a.dot_y == Inf) || (a.dot_y == -Inf)
    return s
  end=#
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

  return CarPhysicalState(sp)
end


#=

Generate state

=#
function randCarPhysicalState(rng::AbstractRNG, d::NTuple{3,NormalDist})
  x = d[1].mean + randn(rng) * d[1].std

  y = d[2].mean
  y_noise = randn(rng) * d[2].std
  y_noise >  (LANE_WIDTH/2.0) ? (y_noise = (LANE_WIDTH/2.0)) : (y_noise < -LANE_WIDTH/2.0) ? (y_noise = -LANE_WIDTH/2.0) : nothing
  y += y_noise

  xdot = d[3].mean
  xdot_noise = randn(rng) * d[3].std
  xdot_noise > 2 * VEL_STD_DEV ? xdot_noise = 2 * VEL_STD_DEV : (xdot_noise < - 2 * VEL_STD_DEV ? xdot_noise = -2 * VEL_STD_DEV : nothing)
  xdot += xdot_noise

  return (CarPhysicalState((x,y,xdot)))
end

function randCarLocalISL0(rng::AbstractRNG, d::NTuple{3,NormalDist}, intentionDist::Array{Float64,1}, frameList::Array{LowLevelCarFrameL0,1})
  phySt = randCarPhysicalState(rng, d)
  cumProbDist = cumsum(intentionDist)
  #=for i in intentionDist
    print(i," ")
  end
  println()
  for i in cumProbDist
    print(i," ")
  end
  println()=#
  x = Base.rand(rng)
  #println("x = ", x)
  targetLane = 1
  while (targetLane < length(cumProbDist)) && (x > cumProbDist[targetLane])
    targetLane += 1
  end
  if (targetLane > length(intentionDist))
    targetLane = length(intentionDist)
  end
  #println(" targetLane = ", targetLane)
  frame = frameList[Base.rand(rng, 1:length(frameList))]
  node = frame.policy.nodeSet[1]

  localISL0 = CarLocalIS{LowLevelCarModelL0}(phySt, LowLevelCarModelL0(targetLane, frame, node))
  return localISL0
end


function gauss(x::Float64, sigma::Float64)
  return 1 / sqrt(2*pi) / sigma * exp(-1*x^2/(2*sigma^2))
end

#TODO: This is not the pdf by convention, this is obs_weight
function pdf(s::GlobalStateL1, o::EgoObservation)
  #First verify that the size of neighborohood is same in both s and o
  if length(s.neighborhood) != length(o.neighborhood)
    #println("Neighborhood length mismatch.",length(s.neighborhood),", ",length(o.neighborhood), " End pdf, 0.0")
    return 0.0
  end

  numLanes = length(s.neighborhood)
  for ln in 1:numLanes
    if length(s.neighborhood[ln]) != length(o.neighborhood[ln])
      #println("Lane $ln Length mismatch.", length(s.neighborhood[ln]),", ", length(o.neighborhood[ln]), " End pdf, ", (1e-8)^abs(length(s.neighborhood[ln])-length(o.neighborhood[ln])))
      return (1e-8)^abs(length(s.neighborhood[ln])-length(o.neighborhood[ln]))
    end
  end

  #Ego State
  #if !(s.ego == o.ego)
  #  println("Ego State mismatch.", #=s.ego,", ", o.ego,=# " End pdf, 0.0001")
  #  return 0.0001
  #end
  prob = 1.0
  s_ego = s.ego
  o_ego = o.ego
  p_x = gauss(abs(s_ego.state[1]-o_ego.state[1]), OBS_NOISE_X)
  prob *= p_x
  p_y = gauss(abs(s_ego.state[2]-o_ego.state[2]), OBS_NOISE_Y)
  prob *= p_y
  p_xdot = gauss(abs(s_ego.state[3]-o_ego.state[3]), OBS_NOISE_XDOT)
  prob *= p_xdot
  #println("p_x = ", p_x, " p_y = ", p_y, " p_xdot = ", p_xdot)
  #if (s.ego == o.ego)
    #println(s_ego,"\n", o_ego,"\n", prob)
  #end

  for ln in 1:numLanes
    numCars = length(s.neighborhood[ln])
    for cIdx = 1:numCars
      phySt_st = s.neighborhood[ln][cIdx].physicalState
      phySt_ob = o.neighborhood[ln][cIdx]

      prob *= gauss(abs(phySt_st.state[1]-phySt_ob.state[1]), OBS_NOISE_X)
      prob *= gauss(abs(phySt_st.state[2]-phySt_ob.state[2]), OBS_NOISE_Y)
      prob *= gauss(abs(phySt_st.state[3]-phySt_ob.state[3]), OBS_NOISE_XDOT)
    end
  end
  #println("End pdf, $prob")
  return prob
end
