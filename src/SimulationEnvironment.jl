type SimEnvironmentState
  egoState::CarPhysicalState
  oa_list::Array{Array{CarLocalISL0,1},1}
end

#Function to generate initial state of Simulation Environment
function getLaneNo(y::Float64, p::SimEnvironmentState)
  return getLaneNo(y, p.roadSegment)
end

function getLaneNo(phySt::CarPhysicalState, p::SimEnvironmentState)
  y = phySt.state[2]
  return getLaneNo(y,p)
end

function getLaneCenter(phySt::CarPhysicalState, p::SimEnvironmentState)
  laneNo = getLaneNo(phySt, p)
  return getLaneCenter(p.roadSegment, laneNo)
end

#Code to propagate
function propagateScene(env::SimEnvironmentState, egoPolicy::Policy)


end


#Code to gridify
