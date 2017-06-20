type RoadSegment
  x_boundary::NTuple{2, Float64}
  laneMarkings::Array{Float64,1}
end

n_lanes(rs::RoadSegment) = length(rs.laneMarkings)-1
length(rs::RoadSegment) = rs.x_boundary[2] = rs.x_boundary[1]

function getLaneNo(y::Float64, rs::RoadSegment)
  for j = 2:length(rs.laneMarkings)
    if y < rs.laneMarkings[j]
      return j-1
    end
  end
  return (length(rs.laneMarkings)-1)
end

function getLaneCenter(rs::RoadSegment, ln::Int64)
  if ln < 1 || ln >= length(rs.laneMarkings)
    return Inf
  end
  return mean([rs.laneMarkings[ln], rs.laneMarkings[ln+1]])
end

function getLaneCenters(rs::RoadSegment)
  laneCenters = Array{Float64,1}(length(rs.laneMarkings-1))
  for ln in 2:length(rs.laneMarkings)
      laneCenters[ln-1] = (rs.laneMarkings[ln-1] + rs.laneMarkings[ln])/2
  end
  return laneCenters
end

#=
type Roadway
  segments::Array{RoadSegment,1}
  direction::Array{Int, 1} #-1, 0, 1, 0 will be rarely used when there is a two way traffic
end
=#
