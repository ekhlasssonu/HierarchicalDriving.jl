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
    ld_distance::UInt64 #Discretized distance of the leading vehicle, leading vehicle in [ld_distance * cellLength, (ld_distance + 1) cellLength) max 3
    occ_l::NTuple{3,Bool}
    occ_r::NTuple{3,Bool}
end

immutable SingleAgentOccGridMDP <: POMDPs.MDP{ImmGridOccSt, Int}
  n_agents::Int64
  roadSegment::RoadSegment
  n_v_cells::Int64
  goal::Vector{AgentGridLocation}
  goal_reward::Float64
  p_next_lane::Array{Float64,4}          #Pr of reaching next lane given action, ld_dist, lt_occ_int, rt_occ_int
  p_next_dist::Array{Float64,5} #Pr of reaching next cell (vertically) where next cell = current cell + idx

  discount::Float64
end

function SingleAgentOccGridMDP(n_agents::Int64, n_v_cells::Int64, goal_reward::Float64, p_next_lane::Array{Float64,4}, p_next_dist::Array{Float64,5})
  roadSegment = RoadSegment((-100.0, 500.0),[0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH])
  goalCell = AgentGridLocation(n_lanes(roadSegment), n_v_cells)
  return SingleAgentOccGridMDP(n_agents, roadSegment, n_v_cells,
                                [AgentGridLocation(n_lanes(roadSegment), n_v_cells),
                                  AgentGridLocation(n_lanes(roadSegment), n_v_cells-1),
                                  AgentGridLocation(n_lanes(roadSegment), n_v_cells-2)],
                                  goal_reward, p_next_lane, p_next_dist, 0.9)
end

discount(p::SingleAgentOccGridMDP) = p.discount
isterminal(p::SingleAgentOccGridMDP, s::ImmGridOccSt) = s.egoGrid.distance > p.n_v_cells
n_lanes(p::SingleAgentOccGridMDP) = n_lanes(p.roadSegment)
n_v_cells(p::SingleAgentOccGridMDP)= p.n_v_cells
n_states(p::SingleAgentOccGridMDP) = n_v_cells(p)*n_lanes(p)*4*8*8

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
  return states
end

function state_index(p::SingleAgentOccGridMDP, s::ImmGridOccSt)
  egoLane = s.egoGrid.lane
  egoDist = s.egoGrid.distance
  ld_distance = s.ld_distance
  ltOcc = boolArray2Int([s.occ_l...])
  rtOcc = boolArray2Int([s.occ_r...])

  return rtOcc + 8 * ((ltOcc-1) + 8 * (ld_distance + 4 * ((egoDist - 1) + n_v_cells(p) * (egoLane - 1))))
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

function iterator(d::SAOccGridDist)
  next_states = Vector{ImmGridOccSt}()
  next_ego_locations = Vector{AgentGridLocation}()
  if d.s.egoGrid.lane + d.a > n_lanes(d.p) || d.s.lane + d.a < 1
    next_ego_locations = [(AgentGridLocation(d.s.lane, d.s.distance + dd) for dd in (0,1))...]
  else
    next_ego_locations = [(AgentGridLocation(d.s.lane + l, d.s.distance + dd) for l in (0, d.a), dd in (0,1))...]
  end

  for next_ego in next_ego_locations
    for ldCell in 0:3
      for occLt in 1:8
        for occRt in 1:8
          st = ImmGridOccSt(next_ego, ldCell, tuple(int2BoolArray(occLt, 3)...), tuple(int2BoolArray(occRt, 3)...))
          push!(next_states, st)
        end
      end
    end
  end
  return next_states
end

function rand(rng::AbstractRNG, d::SAGridDist)
  p = d.p
  a = d.a
  s = d.s
  egoLane = s.egoGrid.lane
  egoDist = s.egoGrid.distance
  ld_dist = s.ld_distance
  occ_l = s.occ_l
  occ_r = s.occ_r
  occ_l_int = boolArray2Int(occ_l)
  occ_r_int = boolArray2Int(occ_r)
  n_agents = p.n_agents
  n_cells = n_v_cells(p) * n_lanes(p)

  pr_cell_occupied = n_agents/n_cells #Assuming a random distribution of cars, not accurate but will do
  pr_cell_occupied > 1.0 ? pr_cell_occupied = 1.0 : nothing

  p_next_dist = p.p_next_dist[action_index(a)][ld_dist+1][occ_l_int][occ_r_int]
  p_desired_lane = p.p_next_lane[action_index(a)][ld_dist+1][occ_l_int][occ_r_int]

  #Next Position of ego vehicle
  #next lane
  rl = rand(rng)
  next_ego_lane = egoLane
  if rl < p_desired_lane
    next_ego_lane = = clamp(egoLane + a, 1, n_lanes(p))
  end
  #next distance
  rd = rand(rng)
  next_ego_dist = egoDist
  for nxt_dist_idx in 1:length(p_next_dist)
    if rd < p_next_dist[nxt_dist_idx]
      next_ego_dist = egoDist + nxt_dist_idx
      break
    end
  end

  next_ego_grid = AgentGridLocation(next_ego_lane, next_ego_dist)

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

  return ImmGridOccSt(next_ego_grid, next_ld_distance, tuple(int2BoolArray(lt_occ_int, 3)...), tuple(int2BoolArray(rt_occ_int, 3)...))
end

function pdf(d::SAOccGridDist, s::ImmGridOccSt)


end
