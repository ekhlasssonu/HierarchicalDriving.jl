@auto_hash_equals immutable AgentGridLocation
    lane::Int
    distance::Int # increases as car travels
end

@with_kw immutable SingleAgentGridMDP <: MDP{AgentGridLocation, Int}
    n_lanes::Int            = 4
    length::Int             = 8
    cell_length::Float64    = 75.0
    timestep::Float64       = 3.0

    goal::AgentGridLocation = AgentGridLocation(n_lanes, 7)      # lane, distance
    goal_reward::Float64    = 100.0

    p_next_cell::Float64    = 0.9
    p_desired_lane::Float64 = 0.9

    discount::Float64       = 0.99
end

#speed(p::SingleAgentGridMDP) = p.cell_length/p.timestep
discount(p::SingleAgentGridMDP) = p.discount
isterminal(p::SingleAgentGridMDP, s::AgentGridLocation) = s.distance > p.goal.distance

n_states(p::SingleAgentGridMDP) = p.length*p.n_lanes
states(p::SingleAgentGridMDP) = (AgentGridLocation(l, d) for l in 1:p.n_lanes, d in 1:p.length)
state_index(p::SingleAgentGridMDP, s::AgentGridLocation) = sub2ind((p.n_lanes, p.length), s.lane, s.distance)

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
    if d.s.lane + d.a > d.p.n_lanes || d.s.lane + d.a < 1
        (AgentGridLocation(d.s.lane, d.s.distance + dd) for dd in (0,1))
    else
        (AgentGridLocation(d.s.lane + l, d.s.distance + dd) for l in (0, d.a), dd in (0,1))
    end
end

function rand(rng::AbstractRNG, d::SAGridDist)
    lane = d.s.lane
    rd = rand(rng)
    if rd < d.p.p_next_cell
        distance = d.s.distance + 1
    else
        distance = d.s.distance
    end

    rl = rand(rng)
    if rd < d.p.p_desired_lane
        lane = clamp(d.s.lane + d.a, 1, d.p.n_lanes)
    else
        lane = d.s.lane
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

initial_state(p::SingleAgentGridMDP, rng::AbstractRNG) = AgentGridLocation(2, 1)

reward(p::SingleAgentGridMDP, s::AgentGridLocation, a::Int, sp::AgentGridLocation) = sp == p.goal ? p.goal_reward : 0.0
