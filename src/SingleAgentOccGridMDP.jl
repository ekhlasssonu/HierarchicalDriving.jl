function int2BoolArray(x::Int64, places::Int64)
  boolArr = Vector{Bool}()
  n = x-1
  for pl in 1:places
    n%2 == 0 ? push!(boolArr, false) : push!(boolArr, true)
    n = round(Int64, floor(n/2))
  end
  return boolArr
end

function boolArray2Int(arr::Vector{Bool})
  x = 0
  for idx in 1:length(arr)
    arr[idx] == 1 ? x += (2^(idx-1)) : nothing
  end
  return x+1
end

@auto_hash_equals immutable ImmGridOccSt
    egoGrid::AgentGridLocation
    ld_distance::UInt64 #Discretized distance of the leading vehicle, leading vehicle in [egoCell + ld_distance * cellLength, egoCell + (ld_distance + 1) cellLength)] max 3
    occ_l::NTuple{3,Bool}
    occ_r::NTuple{3,Bool}
end

immutable SingleAgentOccGridMDP <: POMDPs.MDP{ImmGridOccSt, Int}
  n_agents::Int64
  roadSegment::RoadSegment
  n_v_cells::Int64
  goal::Vector{AgentGridLocation}
  goal_reward::Float64
  egoTranProb::Array{Float64, 6}
  ego_r_s_a::Array{Float64, 4}
  discount::Float64
end

function SingleAgentOccGridMDP(n_agents::Int64, roadSegment::RoadSegment, n_v_cells::Int64, goal_reward::Float64, tranFileName::String, rwdFileName::String)
  goalCell = AgentGridLocation(n_lanes(roadSegment), n_v_cells)
  tranProb = load(tranFileName, "tranProb")
  r_s_a= load(rwdFileName,  "r_s_a")
  return SingleAgentOccGridMDP(n_agents, roadSegment, n_v_cells,
                                [AgentGridLocation(n_lanes(roadSegment), n_v_cells),
                                  AgentGridLocation(n_lanes(roadSegment), n_v_cells-1),
                                  AgentGridLocation(n_lanes(roadSegment), n_v_cells-2)],
                                  goal_reward, tranProb, r_s_a, 0.9)
end

discount(p::SingleAgentOccGridMDP) = p.discount
n_lanes(p::SingleAgentOccGridMDP) = n_lanes(p.roadSegment)
n_v_cells(p::SingleAgentOccGridMDP)= p.n_v_cells
isterminal(p::SingleAgentOccGridMDP, s::ImmGridOccSt) = s.egoGrid.distance < 1 || s.egoGrid.distance > n_v_cells(p) || s.egoGrid.lane < 1 || s.egoGrid.lane > n_lanes(p)
n_states(p::SingleAgentOccGridMDP) = n_v_cells(p)*n_lanes(p)*4*8*8 + 1

function states(p::SingleAgentOccGridMDP)
  states = Vector{ImmGridOccSt}()
  for l in 1:n_lanes(p)
    for d in 1:n_v_cells(p)
      for ldCell in 0:3
        for occLt in 1:8
          for occRt in 1:8
            st = ImmGridOccSt(AgentGridLocation(l,d), ldCell, tuple(int2BoolArray(occLt, 3)...), tuple(int2BoolArray(occRt, 3)...))
            push!(states, st)
          end
        end
      end
    end
  end
  st = ImmGridOccSt(AgentGridLocation(n_lanes(p) + 1, n_v_cells(p)+1), 1, tuple(int2BoolArray(1, 3)...), tuple(int2BoolArray(1, 3)...))
  push!(states, st)
  return states
end

function state_index(p::SingleAgentOccGridMDP, s::ImmGridOccSt)
  if isterminal(p,s)
    return n_states(p)
  else
    egoLane = s.egoGrid.lane
    egoDist = s.egoGrid.distance
    ld_distance = s.ld_distance
    ltOcc = boolArray2Int([s.occ_l...])
    rtOcc = boolArray2Int([s.occ_r...])

    return rtOcc + 8 * ((ltOcc-1) + 8 * (ld_distance + 4 * ((egoDist - 1) + n_v_cells(p) * (egoLane - 1))))
  end
end

actions(p::SingleAgentOccGridMDP) = -1:1
n_actions(p::SingleAgentOccGridMDP) = 3
action_index(p::SingleAgentOccGridMDP, a::Int) = a + 2


immutable SAOccGridDist
  p::SingleAgentOccGridMDP
  s::ImmGridOccSt
  a::Int
end

transition(p::SingleAgentOccGridMDP, s::ImmGridOccSt, a::Int) = SAOccGridDist(p, s, a)

#Iterable list of every reachable state from initial state in p
function iterator(d::SAOccGridDist)
  p = d.p
  maxDistOffset = size(d.p.egoTranProb)[6]-1
  next_states = Vector{ImmGridOccSt}()

  #Next Ego Locations
  next_ego_locations = Vector{AgentGridLocation}()
  for laneOffset in -1:1
    nextLane = clamp(d.s.egoGrid.lane + laneOffset, 1, n_lanes(d.p))
    for distOffset in 0:maxDistOffset
      nextDist = d.s.egoGrid.distance + distOffset
      next_ego_locations = union(next_ego_locations, [AgentGridLocation(nextLane, nextDist)])
    end
  end

  for next_ego in next_ego_locations
    if next_ego.distance < 1 || next_ego.distance > n_v_cells(d.p) || next_ego.lane > n_lanes(d.p) || next_ego.lane < 1
      st = ImmGridOccSt(AgentGridLocation(n_lanes(p) + 1, n_v_cells(p)+1), 1, tuple(int2BoolArray(1, 3)...), tuple(int2BoolArray(1, 3)...))
      next_states = union(next_states, [st])
    else
      for ldCell in 0:3
        for occLt in 1:8
          for occRt in 1:8
            st = ImmGridOccSt(next_ego, ldCell, tuple(int2BoolArray(occLt, 3)...), tuple(int2BoolArray(occRt, 3)...))
            push!(next_states, st)
          end
        end
      end
    end
  end
  return next_states
end

function rand(rng::AbstractRNG, d::SAOccGridDist)
  p = d.p
  a = d.a
  s = d.s

  numLanes = n_lanes(p)
  egoLane = s.egoGrid.lane
  egoDist = s.egoGrid.distance
  ld_dist = s.ld_distance
  if ld_dist > size(d.p.egoTranProb)[2]-1
    ld_dist = size(d.p.egoTranProb)[2]-1
  end
  occ_l = s.occ_l
  occ_r = s.occ_r
  occ_l_int = boolArray2Int([occ_l...])
  occ_r_int = boolArray2Int([occ_r...])
  n_agents = p.n_agents
  n_cells = n_v_cells(p) * n_lanes(p)

  pr_cell_occupied = n_agents/n_cells #Assuming a random distribution of cars, not accurate but will do
  pr_cell_occupied > 1.0 ? pr_cell_occupied = 1.0 : nothing
  #TODO: Change this part may be.
  nxtEgoLocation = s.egoGrid
  rnd = rand(rng)
  sum = 0.0
  found = false
  for d_ln in -1:1
    for d_dist in 0:size(d.p.egoTranProb)[6]-1
      sum += p.egoTranProb[action_index(p,a), ld_dist+1, occ_l_int, occ_r_int, d_ln+2, d_dist+1]
      if sum > rnd
        nxtEgoLocation = AgentGridLocation(clamp(egoLane + d_ln, 0, numLanes), egoDist+d_dist)
        found = true
        break
      end
    end
    found && break
  end

  #Leading Vehicle Distance
  #Each cell may be occupied with a probability of pr_cell_occupied, including ego cell
  ld_car_occ_prob = [pr_cell_occupied,
                      pr_cell_occupied * (1. - pr_cell_occupied),
                      pr_cell_occupied * (1. - pr_cell_occupied)^2,
                      (1. - pr_cell_occupied)^3]  #Adds to 1

  ld_car_dist_prob = cumsum(ld_car_occ_prob)

  next_ld_distance = 0
  r_ldcr = rand(rng)
  for x in 1:length(ld_car_dist_prob)
    if r_ldcr < ld_car_dist_prob[x]
      next_ld_distance = x - 1
      break
    end
  end

  occupancy_prob = Vector{Float64}()  #Will add to 1
  for i in 1:8
    boolOccArr = int2BoolArray(i,3)
    num_true = 0
    for occ in boolOccArr
      occ ? num_true += 1 : nothing
    end
    push!(occupancy_prob, (pr_cell_occupied)^num_true * (1. - pr_cell_occupied)^(3 - num_true))
  end
  cum_occ_prob = cumsum(occupancy_prob)
  #Left occupancy
  lt_occ_int = 0
  rnd_lt_occ = rand(rng)
  for i in 1:length(cum_occ_prob)
    if rnd_lt_occ < cum_occ_prob[i]
      lt_occ_int = i
      break
    end
  end

  #Right occupancy
  rnd_rt_occ = rand(rng)
  rt_occ_int = 0
  for i in 1:length(cum_occ_prob)
    if rnd_rt_occ < cum_occ_prob[i]
      rt_occ_int = i
      break
    end
  end

  return ImmGridOccSt(nxtEgoLocation, next_ld_distance, tuple(int2BoolArray(lt_occ_int, 3)...), tuple(int2BoolArray(rt_occ_int, 3)...))
end

function pdf(d::SAOccGridDist, s_fin::ImmGridOccSt)
  p = d.p
  a = d.a
  s_init = d.s

  if s_init.ld_distance < 0
    println("[pdf] Error: ld_dist < 0")
    return 0.0
  end

  n_agents = p.n_agents
  n_cells = n_v_cells(p) * n_lanes(p)

  pr_cell_occupied = n_agents/n_cells #Assuming a random distribution of cars, not accurate but will do
  pr_cell_occupied > 1.0 ? pr_cell_occupied = 1.0 : nothing

  init_ego = s_init.egoGrid
  fin_ego  = s_fin.egoGrid

  ld_dist = s_init.ld_distance
  if ld_dist > size(p.egoTranProb)[2]-1
    ld_dist = size(p.egoTranProb)[2]-1
  end
  occ_l = s_init.occ_l
  occ_r = s_init.occ_r
  occ_l_int = boolArray2Int([occ_l...])
  occ_r_int = boolArray2Int([occ_r...])

  #Prob. of ego position given d
  probEgoPos = 0.0
  d_ln = fin_ego.lane - init_ego.lane
  d_dist = fin_ego.distance - init_ego.distance

  if d_ln < -1 || d_ln > 1 || d_dist < 0 || d_dist > size(p.egoTranProb)[6]-1
    return 0.0
  else
    #TODO: Change this maybe
    probEgoPos += p.egoTranProb[action_index(p,a), ld_dist+1, occ_l_int, occ_r_int, d_ln+2, d_dist+1]
    if init_ego.lane == 1 && d_ln == 0
      probEgoPos += p.egoTranProb[action_index(p,a), ld_dist+1, occ_l_int, occ_r_int, -1+2, d_dist+1]
    elseif init_ego.lane == n_lanes(p) && d_ln == 0
      probEgoPos += p.egoTranProb[action_index(p,a), ld_dist+1, occ_l_int, occ_r_int, 1+2, d_dist+1]
    end
  end
  #Prob. of occupancy
  probOcc = 1.0
  ld_dist = s_fin.ld_distance
  if ld_dist < 0
    return 0.0
  elseif ld_dist > size(p.egoTranProb)[2]-1
    ld_dist = size(p.egoTranProb)[2]-1
  end
  occ_l = s_fin.occ_l
  occ_r = s_fin.occ_r

  ld_car_occ_prob = [pr_cell_occupied,
                      pr_cell_occupied * (1. - pr_cell_occupied),
                      pr_cell_occupied * (1. - pr_cell_occupied)^2,
                      (1. - pr_cell_occupied)^3]  #Adds to 1
  probOcc *= ld_car_occ_prob[ld_dist+1]

  for i in occ_l
    i ? probOcc *= pr_cell_occupied : probOcc *= (1-pr_cell_occupied)
  end
  for i in occ_r
    i ? probOcc *= pr_cell_occupied : probOcc *= (1-pr_cell_occupied)
  end

  return probOcc * probEgoPos
end

function initial_state(p::SingleAgentOccGridMDP, rng::AbstractRNG)
  init_egoGrid = AgentGridLocation(1, 5)

  n_agents = p.n_agents
  n_cells = n_v_cells(p) * n_lanes(p)

  pr_cell_occupied = n_agents/n_cells #Assuming a random distribution of cars, not accurate but will do
  pr_cell_occupied > 1.0 ? pr_cell_occupied = 1.0 : nothing

  ld_car_occ_prob = [pr_cell_occupied,
                      pr_cell_occupied * (1. - pr_cell_occupied),
                      pr_cell_occupied * (1. - pr_cell_occupied)^2,
                      (1. - pr_cell_occupied)^3]  #Adds to 1

  ld_car_dist_prob = cumsum(ld_car_occ_prob)

  next_ld_distance = 0
  r_ldcr = rand(rng)
  for x in 1:length(ld_car_dist_prob)
    if r_ldcr < ld_car_dist_prob[x]
      next_ld_distance = x - 1
      break
    end
  end

  occupancy_prob = Vector{Float64}()  #Will add to 1
  for i in 1:8
    boolArr = int2BoolArray(i,3)
    num_true = 0
    for j in 1:3
      boolArr[j] ? num_true += 1 : nothing
    end
    push!(occupancy_prob, (pr_cell_occupied)^num_true * (1. - pr_cell_occupied)^(3 - num_true))
  end
  cum_occ_prob = cumsum(occupancy_prob)
  #Left occupancy
  lt_occ_int = 0
  rnd_lt_occ = rand(rng)
  for i in 1:length(cum_occ_prob)
    if rnd_lt_occ < cum_occ_prob[i]
      lt_occ_int = i
      break
    end
  end

  #Right occupancy
  rnd_rt_occ = rand(rng)
  rt_occ_int = 0
  for i in 1:length(cum_occ_prob)
    if rnd_rt_occ < cum_occ_prob[i]
      rt_occ_int = i
      break
    end
  end
  return ImmGridOccSt(init_egoGrid, next_ld_distance, tuple(int2BoolArray(lt_occ_int, 3)...), tuple(int2BoolArray(rt_occ_int, 3)...))
end

function reward(p::SingleAgentOccGridMDP, s::ImmGridOccSt, a::Int, sp::ImmGridOccSt)
  egoGrid = s.egoGrid
  for finGrid in p.goal
    if egoGrid == finGrid
      return p.goal_reward
    end
  end
  return 0.0
end


function getCarGridLocation(p::SingleAgentOccGridMDP, phySt::CarPhysicalState)
  x = phySt.state[1]
  y = phySt.state[2]
  lane = getLaneNo(y, p.roadSegment)
  cellLength = length(p.roadSegment)/p.n_v_cells
  x_offset = x - p.roadSegment.x_boundary[1]
  distance = round(Int64, ceil(x_offset/cellLength))
  return AgentGridLocation(lane, distance)
end

function normalize_egoTranProb(p::SingleAgentOccGridMDP)
  tDimensions = size(p.egoTranProb)
  for a in -1:1
    for ldDist in 0:tDimensions[2]-1
      for lt_occ_int in 1:8
        for rt_occ_int in 1:8
          sum = 0.0
          for d_ln in -1:1
            for d_dist in 0:tDimensions[6]-1
              sum += p.egoTranProb[a+2, ldDist+1, lt_occ_int, rt_occ_int, d_ln+2, d_dist+1]
            end
          end
          for d_ln in -1:1
            for d_dist in 0:tDimensions[6]-1
              p.egoTranProb[a+2, ldDist+1, lt_occ_int, rt_occ_int, d_ln+2, d_dist+1] /= sum
            end
          end
        end
      end
    end
  end
end

function checkProb(p::SingleAgentOccGridMDP)
  sum = 0.0
  tDimensions = size(p.egoTranProb)
  for a in -1:1
    for ldDist in 0:tDimensions[2]-1
      for lt_occ_int in 1:8
        for rt_occ_int in 1:8
          sum = 0.0
          for d_ln in -1:1
            for d_dist in 0:tDimensions[6]-1
              sum += p.egoTranProb[a+2, ldDist+1, lt_occ_int, rt_occ_int, d_ln+2, d_dist+1]
            end
          end
          if abs(1.0-sum) > 0.00000001
            println("Error in probability. Sum = ",sum," at index $a, $ldDist, $lt_occ_int, $rt_occ_int")
          end
        end
      end
    end
  end

  return 0.0
end
