@auto_hash_equals immutable AgentGridLocation
    lane::Int
    distance::Int # increases as car travels
end

immutable SingleAgentGridMDP <: POMDPs.MDP{AgentGridLocation, Int}
  roadSegment::RoadSegment
  cellLength::Float64
  n_v_cells::Int64

  goal::AgentGridLocation
  goal_reward::Float64

  p_next_cell::Float64
  p_desired_lane::Float64

  discount::Float64
end

function SingleAgentGridMDP()
  cellLength = 80 #m
  roadLength = length(road_segment)
  n_v_cells = convert(Int64, ceil(roadLength/cellLength))
  return SingleAgentGridMDP(road_segment, cellLength, n_v_cells, AgentGridLocation(n_lanes(road_segment), n_v_cells), 100.0, 0.7, 0.7, 0.9)
end
function SingleAgentGridMDP(cellLength::Float64)
  roadLength = length(road_segment)
  n_v_cells = convert(Int64, ceil(roadLength/cellLength))
  return SingleAgentGridMDP(road_segment, cellLength, n_v_cells, AgentGridLocation(n_lanes(road_segment), n_v_cells), 100.0, 0.9, 0.9, 0.9)
end
discount(p::SingleAgentGridMDP) = p.discount
n_lanes(p::SingleAgentGridMDP) = n_lanes(p.roadSegment)
isterminal(p::SingleAgentGridMDP, s::AgentGridLocation) = s.distance < 1 || s.distance  > p.goal.distance || s.lane < 1 || s.lane > n_lanes(p)
n_v_cells(p::SingleAgentGridMDP)= p.n_v_cells

n_states(p::SingleAgentGridMDP) = n_v_cells(p)*n_lanes(p)+1
states(p::SingleAgentGridMDP) = [[AgentGridLocation(l, d) for l in 1:n_lanes(p), d in 1:n_v_cells(p)]..., AgentGridLocation(n_lanes(p)+1, n_v_cells(p)+1)]
function state_index(p::SingleAgentGridMDP, s::AgentGridLocation)
  if s.lane < 1 || s.lane > n_lanes(p) || s.distance < 1 || s.distance > n_v_cells(p)
    return n_v_cells(p)*n_lanes(p)+1
  else
    return sub2ind((n_lanes(p), n_v_cells(p)), s.lane, s.distance)
  end
end

function get_x_bounds(p::SingleAgentGridMDP, dist::Int64)
  cellLength = length(p.roadSegment)/p.n_v_cells
  return (dist-1)*cellLength + p.roadSegment.x_boundary[1], dist * cellLength + p.roadSegment.x_boundary[1]
end

# actions
const MAINTAIN_LANE = 0
const MOVE_LEFT = 1
const MOVE_RIGHT = -1

actions(p::SingleAgentGridMDP) = -1:1
n_actions(p::SingleAgentGridMDP) = 3
action_index(p::SingleAgentGridMDP, a::Int) = a + 2

immutable SAGridDist
    p::SingleAgentGridMDP
    s::AgentGridLocation
    a::Int
end

transition(p::SingleAgentGridMDP, s::AgentGridLocation, a::Int) = SAGridDist(p, s, a)

function iterator(d::SAGridDist)
    if d.s.lane + d.a > n_lanes(d.p) || d.s.lane + d.a < 1
        (AgentGridLocation(d.s.lane, d.s.distance + dd) for dd in (0,1))
    else
        (AgentGridLocation(d.s.lane + l, d.s.distance + dd) for l in (0, d.a), dd in (0,1))
    end
end

function rand(rng::AbstractRNG, d::SAGridDist)
  distance = d.s.distance
  lane = d.s.lane
  rd = rand(rng)
  if rd < d.p.p_next_cell
      distance = d.s.distance + 1
  end

  rl = rand(rng)
  if rl < d.p.p_desired_lane
      lane = clamp(d.s.lane + d.a, 1, n_lanes(d.p))
  end

  return AgentGridLocation(lane, distance)
end

function pdf(d::SAGridDist, s::AgentGridLocation)
  p_lane = 0.0
  if s.lane == clamp(d.s.lane + d.a, 1, n_lanes(d.p))
    p_lane += d.p.p_desired_lane
  end
  if s.lane == d.s.lane
    p_lane += 1.0 - d.p.p_desired_lane
  end

  if s.distance == d.s.distance + 1
    p_distance = d.p.p_next_cell
  elseif s.distance == d.s.distance
    p_distance = 1.0 - d.p.p_next_cell
  else
    p_distance = 0.0
  end

  return p_lane*p_distance
end

initial_state(p::SingleAgentGridMDP, rng::AbstractRNG) = AgentGridLocation(1, 2)

reward(p::SingleAgentGridMDP, s::AgentGridLocation, a::Int, sp::AgentGridLocation) = sp == p.goal ? p.goal_reward : 0.0

function getCarGridLocation(p::SingleAgentGridMDP, phySt::CarPhysicalState)
  x = phySt.state[1]
  y = phySt.state[2]
  lane = getLaneNo(y, p.roadSegment)
  cellLength = length(p.roadSegment)/p.n_v_cells
  x_offset = x - p.roadSegment.x_boundary[1]
  distance = convert(Int64, ceil(x_offset/cellLength))
  return AgentGridLocation(lane, distance)
end
