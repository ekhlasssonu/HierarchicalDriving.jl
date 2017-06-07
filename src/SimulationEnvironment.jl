type SimulationMDP <: POMDPs.MDP{GlobalStateL1, Int64}
  discount_factor::Float64
  TIME_STEP::Float64
  roadSegment::RoadSegment
  egoStartState::CarPhysicalState
  egoTargetState::NTuple{2, CarPhysicalState} #Two values with lb and ub on target physical state
end

SimulationMDP() = SimulationMDP(0.9, 0.2,
                                  RoadSegment((-100.0, 500.0),[0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH]),
                                  CarPhysicalState((0.0, 1.0 * LANE_WIDTH/2.0, AVG_HWY_VELOCITY)),
                                  (CarPhysicalState((425.0, 7.0 * LANE_WIDTH/2.0 - 0.5, AVG_HWY_VELOCITY - 0.5)),
                                   CarPhysicalState((500.0, 7.0 * LANE_WIDTH/2.0 + 0.5, AVG_HWY_VELOCITY + 0.5))) )

discount(p::SimulationMDP) = p.discount_factor
function isterminal(p::SimulationMDP, st::GlobalStateL1{CarLocalIS{LowLevelCarModelL0}})
  st.terminal > 0 ? true : false
end
n_actions(p::SimulationMDP) = length(actions(p))
actions(::SimulationMDP) = EgoActionSpace()
n_lanes(p::SimulationMDP) = length(p.roadSegment.laneMarkings)-1
#Function to generate initial state of Simulation Environment
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
  if length(cars == 0)
    return [0,0]
  end
  car_x = phySt.state[1]
  lead_idx = 1
  lead_x = cars[1].physicalState.state[1]
  trail_idx = length(cars)
  trail_x = cars[end].physicalState.state[1]

  while trail_idx > lead_idx
    mid_idx = floor((lead_idx + trail_idx)/2)
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

#neighborhood is sorted. Binary search to find the current car index or next leading car index. 0 if the car is the first car
#Return next leading and next following vehicle position for all lanes
function getImmediateNeighbors(gblSt::GlobalStateL1, p::SimulationMDP, phySt::CarPhysicalState)
  carLane = getLaneNo(st, p)
  egoState = gblSt.ego
  egoLane = getLaneNo(egoState, p)
  car_x = phySt.state[1]
  imm_neighbor = Array{Array{CarPhysicalState,1},1}(numLanes)
  for ln in 1:n_lanes(p)
    imm_neighbor[ln] = Array{CarPhysicalState,1}()

    arr = gblSt.neighborhood[ln]
    if length(arr) == 0  #Empty lane: Add two cars at infinite distance from current car
      ldr = CarPhysicalState((Inf, getLaneCenter(p.roadSegment, laneNo), AVG_HWY_VELOCITY))
      trlr = CarPhysicalState((-Inf, getLaneCenter(p.roadSegment, laneNo), AVG_HWY_VELOCITY))
      push!(imm_neighbor[ln], ldr)
      push!(imm_neighbor[ln], trlr)
    else
      nbr_indices = binary_search(arr, phySt) #[lead_idx, trail_idx]
      lead_idx = nbr_indices[1]
      trail_idx = nbr_indices[2]
      #Add imm. Leading car
      if lead_idx < 1 #Probably never happens
        push!(imm_neighbor[ln], CarPhysicalState((Inf, getLaneCenter(p.roadSegment, laneNo), AVG_HWY_VELOCITY)))
      #Lead car can be the same as the current car
      else
        if (arr[lead_idx].physicalState == phySt)
          lead_idx -= 1
        end
        if lead_idx < 1
          push!(imm_neighbor[ln], CarPhysicalState((Inf, getLaneCenter(p.roadSegment, laneNo), AVG_HWY_VELOCITY)))
        else
          push!(imm_neighbor[ln], arr[lead_idx].physicalState)
        end
      end

      #Add imm. trailing car
      if length(arr) < trail_idx #Happens when all vehicles are leading (or the same)
        push!(imm_neighbor[ln], CarPhysicalState((-Inf, getLaneCenter(p.roadSegment, laneNo), AVG_HWY_VELOCITY)))
      else
        if (arr[trail_idx].physicalState == phySt)
          trail_idx += 1
        end
        if length(arr) < trail_idx
          push!(imm_neighbor[ln], CarPhysicalState((-Inf, getLaneCenter(p.roadSegment, laneNo), AVG_HWY_VELOCITY)))
        else
          push!(imm_neighbor[ln], arr[trail_idx].physicalState)
        end
      end

    end

    # Handle ego lane
    if ln == egoLane
      if imm_neighbor[ln][1].state[1] > egoState.state[1] && egoState.state[1] > phySt.state[1]
        imm_neighbor[ln][1] = egoState
      end

      if imm_neighbor[ln][2].state[1] < egoState.state[1] && egoState.state[1] < phySt.state[1]
        imm_neighbor[ln][2] = egoState
      end
    end

  end
  return imm_neighbor
end
#Code to propagate
function propagateScene(env::GlobalStateL1, a::Int64)


end


#Code to gridify
