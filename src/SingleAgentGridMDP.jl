@auto_hash_equals immutable AgentGridLocation
    lane::Int
    distance::Int # increases as car travels
end

@with_kw immutable SingleAgentGridMDP <: POMDPs.MDP{AgentGridLocation, Int}
    roadSegment::RoadSegment= RoadSegment((-100.0, 500.0),[0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH])
    n_v_cells::Int64        = 8
    timestep::Float64       = 3.0

    goal::AgentGridLocation = AgentGridLocation(n_lanes(roadSegment), n_v_cells)      # lane, distance
    goal_reward::Float64    = 100.0

    p_next_cell::Float64    = 0.9
    p_desired_lane::Float64 = 0.9

    discount::Float64       = 0.9
end

#speed(p::SingleAgentGridMDP) = p.cell_length/p.timestep
discount(p::SingleAgentGridMDP) = p.discount
isterminal(p::SingleAgentGridMDP, s::AgentGridLocation) = s.distance > p.goal.distance
n_lanes(p::SingleAgentGridMDP) = n_lanes(p.roadSegment)
n_v_cells(p::SingleAgentGridMDP)= p.n_v_cells

n_states(p::SingleAgentGridMDP) = n_v_cells(p)*n_lanes(p)
states(p::SingleAgentGridMDP) = [AgentGridLocation(l, d) for l in 1:n_lanes(p), d in 1:n_v_cells(p)]
state_index(p::SingleAgentGridMDP, s::AgentGridLocation) = sub2ind((n_lanes(p), n_v_cells(p)), s.lane, s.distance)

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
    if s.lane == d.s.lane + d.a
        p_lane = d.p.p_desired_lane
    elseif s.lane == d.s.lane
        p_lane = 1.0 - d.p.p_desired_lane
    else
        p_lane = 0.0
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
