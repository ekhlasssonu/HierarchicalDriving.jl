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
    ld_distance::UInt64 #Discretized distance of the leading vehicle, leading vehicle in [egoCell + ld_distance * cell_length, egoCell + (ld_distance + 1) cell_length)] max 3
    occ_l::Vector{Bool}
    occ_r::Vector{Bool}
end

immutable SingleAgentOccGridMDP <: POMDPs.MDP{ImmGridOccSt, Int}
  n_agents::Int64           #Needed for transition probability
  #Same as SingleAgentGridMDP
  roadSegment::RoadSegment
  cell_length::Float64
  nbr_cell_length::Float64
  n_v_cells::Int64

  num_ld_nbr_blocks::Int64  #How many leading cells' occupancy to consider
  num_adj_nbr_blocks::Int64 #How many adjacent cells' occupancy to consider

  goal::Vector{AgentGridLocation}
  goal_reward::Float64

  egoTranProb::Array{Float64, 6}
  ego_r_s_a::Array{Float64, 4}
  discount::Float64
end

function SingleAgentOccGridMDP(n_agents::Int64, roadSegment::RoadSegment, cell_length::Float64, nbr_cell_length::Float64, n_ld_nbr::Int64, n_adj_nbr::Int64, goal_reward::Float64, tranFileName::String, rwdFileName::String)
  tranProb = load(tranFileName, "tranProb")
  r_s_a= load(rwdFileName,  "r_s_a")
  n_v_cells = convert(Int64,ceil(length(roadSegment)/cell_length))
  goalCells = [AgentGridLocation(n_lanes(roadSegment), n_v_cells), AgentGridLocation(n_lanes(roadSegment), n_v_cells-1), AgentGridLocation(n_lanes(roadSegment), n_v_cells-2), AgentGridLocation(n_lanes(roadSegment), n_v_cells-3), AgentGridLocation(n_lanes(roadSegment), n_v_cells-4)]
  return SingleAgentOccGridMDP(n_agents, roadSegment, cell_length, nbr_cell_length, n_v_cells, n_ld_nbr, n_adj_nbr, goalCells,
                                  goal_reward, tranProb, r_s_a, 0.9)
end

discount(p::SingleAgentOccGridMDP) = p.discount
n_lanes(p::SingleAgentOccGridMDP) = n_lanes(p.roadSegment)
n_v_cells(p::SingleAgentOccGridMDP)= p.n_v_cells
isterminal(p::SingleAgentOccGridMDP, s::ImmGridOccSt) = s.egoGrid.distance < 1 || s.egoGrid.distance > n_v_cells(p) || s.egoGrid.lane < 1 || s.egoGrid.lane > n_lanes(p)
n_states(p::SingleAgentOccGridMDP) = n_v_cells(p)*n_lanes(p)*(p.num_ld_nbr_blocks+1)*(2^p.num_adj_nbr_blocks)^2 + 1

function states(p::SingleAgentOccGridMDP)
  states = Vector{ImmGridOccSt}()
  for l in 1:n_lanes(p)
    for d in 1:n_v_cells(p)
      for ld_dist in 1:p.num_ld_nbr_blocks + 1
        for occLt in 1:2^p.num_adj_nbr_blocks
          for occRt in 1:2^p.num_adj_nbr_blocks
            st = ImmGridOccSt(AgentGridLocation(l,d), ld_dist, int2BoolArray(occLt, p.num_adj_nbr_blocks), int2BoolArray(occRt, p.num_adj_nbr_blocks))
            push!(states, st)
          end
        end
      end
    end
  end
  st = ImmGridOccSt(AgentGridLocation(n_lanes(p) + 1, n_v_cells(p)+1), 1, int2BoolArray(1, p.num_adj_nbr_blocks), int2BoolArray(1, p.num_adj_nbr_blocks))
  push!(states, st)
  return states
end

function get_terminal_state(p::SingleAgentOccGridMDP)
  return ImmGridOccSt(AgentGridLocation(n_lanes(p) + 1, n_v_cells(p)+1), 1, int2BoolArray(1, p.num_adj_nbr_blocks), int2BoolArray(1, p.num_adj_nbr_blocks))
end

function state_index(p::SingleAgentOccGridMDP, s::ImmGridOccSt)
  if isterminal(p,s)
    return n_states(p)
  else
    egoLane = s.egoGrid.lane
    egoDist = s.egoGrid.distance
    ld_distance = s.ld_distance
    ltOcc = boolArray2Int(s.occ_l)
    rtOcc = boolArray2Int(s.occ_r)

    return rtOcc + 2^p.num_adj_nbr_blocks * ((ltOcc-1) + 2^p.num_adj_nbr_blocks * ((ld_distance - 1) + (p.num_ld_nbr_blocks+1) * ((egoDist - 1) + n_v_cells(p) * (egoLane - 1))))
  end
end

#Get x bounds of a cell given the distance
function get_x_bounds(p::SingleAgentOccGridMDP, dist::Int64)
  cell_length = p.cell_length
  return (dist-1)*cell_length + p.roadSegment.x_boundary[1], dist * cell_length + p.roadSegment.x_boundary[1]
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
      #Terminal State
      st = ImmGridOccSt(AgentGridLocation(n_lanes(p) + 1, n_v_cells(p)+1), 1, int2BoolArray(1, p.num_adj_nbr_blocks), int2BoolArray(1, p.num_adj_nbr_blocks))
      next_states = union(next_states, [st])
    else
      for ld_dist in 1:p.num_ld_nbr_blocks+1
        for occLt in 1:2^p.num_adj_nbr_blocks
          for occRt in 1:2^p.num_adj_nbr_blocks
            st = ImmGridOccSt(next_ego, ld_dist, int2BoolArray(occLt, p.num_adj_nbr_blocks), int2BoolArray(occRt, p.num_adj_nbr_blocks))
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
  if ld_dist > size(d.p.egoTranProb)[2]
    ld_dist = size(d.p.egoTranProb)[2]
  end
  occ_l = s.occ_l
  occ_r = s.occ_r
  occ_l_int = boolArray2Int(occ_l)
  occ_r_int = boolArray2Int(occ_r)
  n_agents = p.n_agents

  n_cells = convert(Int64,ceil(length(p.roadSegment)/p.nbr_cell_length)) * n_lanes(p)

  pr_cell_occupied = n_agents/n_cells #Assuming a random distribution of cars, not accurate but will do
  pr_cell_occupied > 1.0 ? pr_cell_occupied = 1.0 : nothing

  nxtEgoLocation = s.egoGrid
  rnd = rand(rng)
  sum = 0.0
  found = false
  for d_ln in -1:1
    for d_dist in 0:size(d.p.egoTranProb)[6]-1
      sum += p.egoTranProb[action_index(p,a), ld_dist, occ_l_int, occ_r_int, d_ln+2, d_dist+1]
      if sum > rnd
        nxtEgoLocation = AgentGridLocation(clamp(egoLane + d_ln, 0, numLanes), egoDist+d_dist)
        found = true
        break
      end
    end
    found && break
  end

  if nxtEgoLocation.distance > n_v_cells(p)
    return get_terminal_state(p)
  end

  #Leading Vehicle Distance
  #Each cell may be occupied with a probability of pr_cell_occupied, including ego cell
  ld_car_occ_prob = [pr_cell_occupied]
  for j in 1:p.num_ld_nbr_blocks-1
    push!(ld_car_occ_prob,pr_cell_occupied * (1-pr_cell_occupied)^j)
  end
  push!(ld_car_occ_prob, (1-pr_cell_occupied)^p.num_ld_nbr_blocks)

  cum_ld_car_occ_prob = cumsum(ld_car_occ_prob)

  next_ld_distance = 1
  r_ldcr = rand(rng)
  for x in 1:length(cum_ld_car_occ_prob)
    if r_ldcr < cum_ld_car_occ_prob[x]
      next_ld_distance = x
      break
    end
  end

  occupancy_prob = Vector{Float64}()  #Will add to 1
  for i in 1:2^p.num_adj_nbr_blocks
    boolOccArr = int2BoolArray(i,p.num_adj_nbr_blocks)
    num_true = 0
    for occ in boolOccArr
      occ ? num_true += 1 : nothing
    end
    push!(occupancy_prob, (pr_cell_occupied)^num_true * (1. - pr_cell_occupied)^(p.num_adj_nbr_blocks - num_true))
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
  rt_occ_int = 0
  rnd_rt_occ = rand(rng)
  for i in 1:length(cum_occ_prob)
    if rnd_rt_occ < cum_occ_prob[i]
      rt_occ_int = i
      break
    end
  end

  return ImmGridOccSt(nxtEgoLocation, next_ld_distance, int2BoolArray(lt_occ_int, p.num_adj_nbr_blocks), int2BoolArray(rt_occ_int, p.num_adj_nbr_blocks))
end

function pdf(d::SAOccGridDist, s_fin::ImmGridOccSt)
  p = d.p
  a = d.a
  s_init = d.s

  if s_init.ld_distance < 0
    #Should never happen
    println("[pdf] Error: ld_dist < 0")
    return 0.0
  end

  init_ego = s_init.egoGrid
  fin_ego  = s_fin.egoGrid

  ld_dist = s_init.ld_distance
  if ld_dist > size(p.egoTranProb)[2]
    ld_dist = size(p.egoTranProb)[2]
  end
  occ_l_int = boolArray2Int(s_init.occ_l)
  occ_r_int = boolArray2Int(s_init.occ_r)

  #Prob. of ego position given d
  probEgoPos = 0.0
  d_ln = fin_ego.lane - init_ego.lane
  d_dist = fin_ego.distance - init_ego.distance

  if d_ln < -1 || d_ln > 1 || d_dist < 0 || d_dist > size(p.egoTranProb)[6]-1
    return 0.0
  else
    # Change this maybe
    probEgoPos += p.egoTranProb[action_index(p,a), ld_dist, occ_l_int, occ_r_int, d_ln+2, d_dist+1]
    if init_ego.lane == 1 && d_ln == 0
      #d_ln = -1 = d_ln = 0
      probEgoPos += p.egoTranProb[action_index(p,a), ld_dist, occ_l_int, occ_r_int, -1+2, d_dist+1]
    elseif init_ego.lane == n_lanes(p) && d_ln == 0
      #d_ln = +1 = d_ln = 0
      probEgoPos += p.egoTranProb[action_index(p,a), ld_dist, occ_l_int, occ_r_int, 1+2, d_dist+1]
    end
  end

  #Prob. of occupancy
  n_agents = p.n_agents
  n_cells = convert(Int64,ceil(length(p.roadSegment)/p.nbr_cell_length)) * n_lanes(p)

  pr_cell_occupied = n_agents/n_cells #Assuming a random distribution of cars, not accurate but will do
  pr_cell_occupied > 1.0 ? pr_cell_occupied = 1.0 : nothing

  probOcc = 1.0
  ld_dist = s_fin.ld_distance
  if ld_dist < 0
    return 0.0
  elseif ld_dist > size(p.egoTranProb)[2]-1
    ld_dist = size(p.egoTranProb)[2]-1
  end
  occ_l = s_fin.occ_l
  occ_r = s_fin.occ_r

  #Each cell may be occupied with a probability of pr_cell_occupied, including ego cell
  ld_car_occ_prob = [pr_cell_occupied]
  for j in 1:p.num_ld_nbr_blocks-1
    push!(ld_car_occ_prob,pr_cell_occupied * (1-pr_cell_occupied)^j)
  end
  push!(ld_car_occ_prob, (1-pr_cell_occupied)^p.num_ld_nbr_blocks)

  probOcc *= ld_car_occ_prob[ld_dist]

  for i in occ_l
    i ? probOcc *= pr_cell_occupied : probOcc *= (1-pr_cell_occupied)
  end
  for i in occ_r
    i ? probOcc *= pr_cell_occupied : probOcc *= (1-pr_cell_occupied)
  end

  return probOcc * probEgoPos
end

function initial_state(p::SingleAgentOccGridMDP, rng::AbstractRNG)
  egoState = CarPhysicalState((0.0, LANE_WIDTH/2.0, AVG_HWY_VELOCITY))
  init_egoGrid = getCarGridLocation(p, egoState)

  n_agents = p.n_agents
  n_cells = convert(Int64,ceil(length(p.roadSegment)/p.nbr_cell_length)) * n_lanes(p)

  pr_cell_occupied = n_agents/n_cells #Assuming a random distribution of cars, not accurate but will do
  pr_cell_occupied > 1.0 ? pr_cell_occupied = 1.0 : nothing

  #Each cell may be occupied with a probability of pr_cell_occupied
  ld_car_occ_prob = [pr_cell_occupied]
  for j in 1:p.num_ld_nbr_blocks-1
    push!(ld_car_occ_prob,pr_cell_occupied * (1-pr_cell_occupied)^j)
  end
  push!(ld_car_occ_prob, (1-pr_cell_occupied)^p.num_ld_nbr_blocks)

  cum_ld_car_occ_prob = cumsum(ld_car_occ_prob)

  next_ld_distance = 1
  r_ldcr = rand(rng)
  for x in 1:length(cum_ld_car_occ_prob)
    if r_ldcr < cum_ld_car_occ_prob[x]
      next_ld_distance = x
      break
    end
  end

  occupancy_prob = Vector{Float64}()  #Will add to 1
  for i in 1:2^p.num_adj_nbr_blocks
    boolOccArr = int2BoolArray(i,p.num_adj_nbr_blocks)
    num_true = 0
    for occ in boolOccArr
      occ ? num_true += 1 : nothing
    end
    push!(occupancy_prob, (pr_cell_occupied)^num_true * (1. - pr_cell_occupied)^(p.num_adj_nbr_blocks - num_true))
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
  return ImmGridOccSt(init_egoGrid, next_ld_distance, int2BoolArray(lt_occ_int, p.num_adj_nbr_blocks), int2BoolArray(rt_occ_int, p.num_adj_nbr_blocks))
end

#TODO: Fine-tune this part to get improvement if improvement at all possible
function reward(p::SingleAgentOccGridMDP, s::ImmGridOccSt, a::Int, sp::ImmGridOccSt)
  rwd = 0.0
  rwd += 1.0 * p.ego_r_s_a[a+2, s.ld_distance, boolArray2Int(s.occ_l), boolArray2Int(s.occ_r)]
  egoGrid = s.egoGrid
  for finGrid in p.goal
    if egoGrid == finGrid
      rwd += p.goal_reward
      break
    end
  end
  return rwd
end


function getCarGridLocation(p::SingleAgentOccGridMDP, phySt::CarPhysicalState)
  x = phySt.state[1]
  y = phySt.state[2]
  lane = getLaneNo(y, p.roadSegment)
  cell_length = p.cell_length
  x_offset = x - p.roadSegment.x_boundary[1]
  distance = round(Int64, ceil(x_offset/cell_length))
  return AgentGridLocation(lane, distance)
end

function extractImmGridOccSt(p::SingleAgentOccGridMDP, gblSt::GlobalStateL1)
  egoState = gblSt.ego
  egoGridLoc = getCarGridLocation(p, egoState)

  if egoGridLoc.distance > n_v_cells(p)
    return get_terminal_state(p)
  end

  egoLane = egoGridLoc.lane
  ego_x = egoState.state[1]

  ld_dist = 0
  occ_l = falses(p.num_adj_nbr_blocks)
  occ_r = falses(p.num_adj_nbr_blocks)

  #Index of the car right next to ego car for any given adj nbrhd length
  #For example, if number of adjacent cars considered were 7, the car adjacent to ego vehicle is 1,
  #the order is 6,4,2,1,3,5,7. The index of adjacent car is 4.
  #May be an overkill
  adj_idx = convert(Int64, floor(p.num_adj_nbr_blocks/2))+1

  numLanes = n_lanes(p)

  l_nbrs_idx = [0,0]
  r_nbrs_idx = [0,0]

  if egoLane-1 > 0
    r_nbrs_idx = binary_search(gblSt.neighborhood[egoLane-1], egoState)
  end

  if egoLane+1 <= numLanes
    l_nbrs_idx = binary_search(gblSt.neighborhood[egoLane+1], egoState)
  end

  if r_nbrs_idx != [0,0]
    ld_idx = r_nbrs_idx[1]
    while ld_idx > 0
      ld_state = gblSt.neighborhood[egoLane-1][ld_idx].physicalState
      ld_x = ld_state.state[1]
      x_offset = ld_x - ego_x
      ld_offset = convert(Int64, ceil(x_offset/p.nbr_cell_length))
      if (ld_offset > ceil((p.num_adj_nbr_blocks-1)/2))
        break
      end
      occ_r[adj_idx - ld_offset] = true
      ld_idx -= 1
    end
    fl_idx = r_nbrs_idx[2]
    while fl_idx <= length(gblSt.neighborhood[egoLane-1])
      fl_state = gblSt.neighborhood[egoLane-1][fl_idx].physicalState
      fl_x = fl_state.state[1]
      x_offset = ego_x - fl_x
      fl_offset = convert(Int64, ceil(x_offset/p.nbr_cell_length))
      if (fl_offset > floor((p.num_adj_nbr_blocks-1)/2))
        break
      end
      occ_r[adj_idx + fl_offset] = true
      fl_idx += 1
    end
  end

  if l_nbrs_idx != [0,0]
    ld_idx = l_nbrs_idx[1]
    while ld_idx > 0
      ld_state = gblSt.neighborhood[egoLane+1][ld_idx].physicalState
      ld_x = ld_state.state[1]
      x_offset = ld_x - ego_x
      ld_offset = convert(Int64, ceil(x_offset/p.nbr_cell_length))
      if (ld_offset > ceil((p.num_adj_nbr_blocks-1)/2))
        break
      end
      occ_l[adj_idx - ld_offset] = true
      ld_idx -= 1
    end
    fl_idx = l_nbrs_idx[2]
    while fl_idx <= length(gblSt.neighborhood[egoLane+1])
      fl_state = gblSt.neighborhood[egoLane+1][fl_idx].physicalState
      fl_x = fl_state.state[1]
      x_offset = ego_x - fl_x
      fl_offset = convert(Int64, ceil(x_offset/p.nbr_cell_length))
      if (fl_offset > floor((p.num_adj_nbr_blocks-1)/2))
        break
      end
      occ_l[adj_idx + fl_offset] = true
      fl_idx += 1
    end
  end

  ln = egoLane
  nbr_indices = binary_search(gblSt.neighborhood[ln], egoState)
  if nbr_indices[1] < 1
    ld_dist = size(p.egoTranProb)[2]
  else
    ldrSt = gblSt.neighborhood[ln][nbr_indices[1]].physicalState
    ldr_x = ldrSt.state[1]
    ego_x = egoState.state[1]
    ld_dist = convert(Int64, ceil((ldr_x - ego_x)/p.nbr_cell_length))
    if ld_dist > p.num_ld_nbr_blocks + 1
      ld_dist = p.num_ld_nbr_blocks + 1
    end
  end
  return ImmGridOccSt(egoGridLoc, ld_dist, occ_l, occ_r)
end

function normalize_egoTranProb(p::SingleAgentOccGridMDP)
  tDimensions = size(p.egoTranProb)
  for a in -1:1
    for ldDist in 1:tDimensions[2]
      for lt_occ_int in 1:tDimensions[3]
        for rt_occ_int in 1:tDimensions[4]
          sum = 0.0
          for d_ln in -1:1
            for d_dist in 0:tDimensions[6]-1
              sum += p.egoTranProb[a+2, ldDist, lt_occ_int, rt_occ_int, d_ln+2, d_dist+1]
            end
          end
          for d_ln in -1:1
            for d_dist in 0:tDimensions[6]-1
              p.egoTranProb[a+2, ldDist, lt_occ_int, rt_occ_int, d_ln+2, d_dist+1] /= sum
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
    for ldDist in 1:tDimensions[2]
      for lt_occ_int in 1:tDimensions[3]
        for rt_occ_int in 1:tDimensions[4]
          sum = 0.0
          for d_ln in -1:1
            for d_dist in 0:tDimensions[6]-1
              sum += p.egoTranProb[a+2, ldDist, lt_occ_int, rt_occ_int, d_ln+2, d_dist+1]
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
