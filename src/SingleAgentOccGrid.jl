function int2BoolArray(x::Int64)
  boolArr = Array{Bool}()
  n = x
  while n > 1
    n%2 == 0 ? push!(boolArr, false) : push!(boolArr, true)
    n = round(Int64, floor(n/2))
  end
  return boolArr
end

@auto_hash_equals immutable ImmGridOccSt
    egoGrid::AgentGridLocation
    occ_s::UInt64 #Discretized distance of the leading vehicle, leading vehicle in [occ_s * cellLength, (occ_s + 1) cellLength) max 3
    occ_l::NTuple{3,Bool}
    occ_r::NTuple{3,Bool}
end
immutable SingleAgentOccGridMDP <: POMDPs.MDP{ImmGridOccSt, Int}
  n_agents::Int64
  roadSegment::RoadSegment
  n_v_cells::Int64
  goal::Vector{AgentGridLocation}
  goal_reward::Float64
  p_next_lane::Float64          #Pr of reaching next lane
  p_next_cell::Vector{Float64}  #Pr of reaching next cell (vertically) where next cell = current cell + idx

  discount::Float64       = 0.9
end

function SingleAgentOccGridMDP(n_agents::Int64, n_v_cells::Int64, goal_reward::Float64, p_next_lane::Float64, p_next_cell::Vector{Float64})
  roadSegment = RoadSegment((-100.0, 500.0),[0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH])
  cellLength = (roadSegment.x_boundary[2]-roadSegment.x_boundary[1])/n_v_cells
  goalCell = AgentGridLocation(n_lanes(roadSegment), n_v_cells)
  return SingleAgentOccGridMDP(n_agents, roadSegment, n_v_cells,
                                [AgentGridLocation(n_lanes(roadSegment), n_v_cells),
                                  AgentGridLocation(n_lanes(roadSegment), n_v_cells-1),
                                  AgentGridLocation(n_lanes(roadSegment), n_v_cells-2)],
                                  goalReward, p_next_lane, p_next_cell)
end

discount(p::SingleAgentOccGridMDP) = p.discount
isterminal(p::SingleAgentOccGridMDP, s::ImmGridOccSt) = s.egoGrid.distance > p.n_v_cells
n_lanes(p::SingleAgentOccGridMDP) = n_lanes(p.roadSegment)
n_v_cells(p::SingleAgentOccGridMDP)= p.n_v_cells
n_states(p::SingleAgentOccGridMDP) = n_v_cells(p)*n_lanes(p)
function states(p::SingleAgentOccGridMDP)
  states = Vector(ImmGridOccSt)
  for l in 1:n_lanes(p)
    for d in 1:n_v_cells(p)
      for ldCell in 0:3
        for occLt in 0:7
          for occRt in 0:7
            st = ImmGridOccSt(AgentGridLocation(l,d), ldCell, )
            push!(states, )

end
state_index(p::SingleAgentGridMDP, s::AgentGridLocation) = sub2ind((n_lanes(p), n_v_cells(p)), s.lane, s.distance)
