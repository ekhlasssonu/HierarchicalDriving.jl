immutable PhysicalState
    x::Float64
    y::Float64
    xdot::Float64
end

immutable CarState{I}
    id::Int
    physical::PhysicalState
    internal::I
end

function step(s::CarState{}, g::GlobalState, physical::PhysicalParameter)

end

type GlobalState{C<:CarState}
    cars::Vector{C}
    neighborhoods::Nullable{Vector{Neighborhood}}
end

function get_neighborhood(s::GlobalState, i::Int)
    if isnull(s.neighborhoods)
        # calculate neighborhood
        return get(s.neighborhoods)[i]
    else
        return get(s.neighborhoods)[i]
    end
end



immutable LowLevelPOMDP <: POMDP{GlobalStateL1, Int, Vector{PhysicalState}}
    mdp::LowLevelMDP
end

generate_s(p::LowLevelPOMDP, s, a, rng) = generate_s(p.mdp, s, a, rng)
function generate_o()

end


immutable CarFilter

end

immutable NeighborhoodFilter

end

function update(f::CarFilter, b::ParticleCollection{CarLocalIS}, o)
    # see ParticleFilters.jl
end

function update(f::NeighborhoodFilter, b::Vector{ParticleCollection{CarLocalIS}}, a::Int, o::Vect{PhysicalState})

end

function rand(rng, b::Vector{ParticleCollection{CarLocalIS}})
    cars = Array(CarLocalIS, length(b))
    for i in 1:length(b)
        cars[i] = rand(rng, b[i])
    end
    return SimEnvironmentState(ego, cars)
end








function updateNeighborState(globalISL1::GlobalStateL1, p::LowLevelMDP, rng::AbstractRNG)
  laneCenters = getLaneCenters(p.roadSegment)

  egoState = globalISL1.ego
  egoLane = getLaneNo(egoState,p)
  numLanes = length(globalISL1.neighborhood)
  if numLanes != n_lanes(p)
    println("[updateNeighborState] Incorrect numLanes. ")
  end
  if egoLane < 1 || egoLane > numLanes
    println("[updateNeighborState] Incorrect egoLane index, $egoLane. $egoState, y = ", egoState.state[2])
  end
  updatedNeighborhood = Array{Array{CarLocalIS,1},1}(numLanes)

  for ln in 1:numLanes #For every lane
    numCars = length(globalISL1.neighborhood[ln])
    updatedNeighborhood[ln] = Array{CarLocalIS,1}()
    ldCarIS = Nullable{CarLocalIS}()
    for carIdx in 1:numCars #For every car in the lane
      carIS = globalISL1.neighborhood[ln][carIdx] #Initial IS of the current car under consideration
      #carAct = actions[ln][carIdx] #action performed by current car

      carPhySt = carIS.physicalState
      carModel = carIS.model
      carFrame = carModel.frame
      targetLane = carModel.targetLane
      currNode = carModel.currNode
      fsm = carFrame.policy

      updatedFrame = carFrame

      x = carPhySt.state[1]
      xdot = carPhySt.state[3]
      dxdot = 0.0
      g = AVG_GAP
      if ln != egoLane
        if !isnull(ldCarIS)
          dxdot = xdot - ldCarIS.physicalState.state[3]
          g = ldCarIS.physicalState.state[1] - x - CAR_LENGTH
        end
      else
        if isnull(ldCarIS)
          if (egoState.state[1] > x)
            g = egoState.state[1] - x - CAR_LENGTH
            dxdot = xdot - egoState.state[3]
          end
        else
          if (egoState.state[1] > x) && (ldCarIS.physicalState.state[1] > egoState.state[1])  #ego in between
            g = egoState.state[1] - x - CAR_LENGTH
            dxdot = xdot - egoState.state[3]
          elseif (egoState.state[1] > ldCarIS.physicalState.state[1])
            g = ldCarIS.physicalState.state[1] - x - CAR_LENGTH
            dxdot = xdot - ldCarIS.physicalState.state[3]
          end
        end
      end

      ddotx = get_idm_accln(carIS.model.frame.longitudinal, xdot, dxdot, g)

      y = carPhySt.state[2]

      target_y = laneCenters[targetLane]

      #Sample action from current node
      rnd = Base.rand(rng)
      ydotCumProb = [0.0,0.0,0.0]

      ydotCumProb[1] = get(fsm.actionProb, (currNode, 2.0), 0.0)
      ydotCumProb[2] = get(fsm.actionProb, (currNode, 0.0), 0.0) + ydotCumProb[1]
      ydotCumProb[3] = get(fsm.actionProb, (currNode, -2.0), 0.0)+ ydotCumProb[2]

      ydot = 0.0
      if rnd < ydotCumProb[1]
        ydot = 2.0  #Move towards desired lane
      elseif rnd < ydotCumProb[2]
        ydot = 0.0  #Keep lane
      else
        ydot = -2.0 #Abort motion to next lane and move to center of current lane
      end

      if ydot < 0.0 #Reverse direction to center of current lane
        target_y = laneCenters[ln]
        ydot = -ydot  #Sign is immaterial, target_y matters
      end

      if target_y != y
        ydot = min(ydot, abs(target_y - y)/p.TIME_STEP)
        ydot *= (target_y - y)/abs(target_y - y)  #By convention
      else
        ydot = 0.0
      end
      #println("x = $x, y = $y, target_y = $target_y, ydot = $ydot")
      updatedCarPhySt = propagateCar(carPhySt, CarAction(ddotx, ydot), p.TIME_STEP, rng, (TRN_NOISE_X, TRN_NOISE_Y, TRN_NOISE_XDOT))

      nextLane = ln
      if targetLane > ln
        nextLane = ln+1
      elseif targetLane < ln
        nextLane = ln-1
      end

      nxtFlPhySt = Nullable{CarPhysicalState}()
      nxtLdPhySt = Nullable{CarPhysicalState}()

      #Update car's model, frame remains same, target lane remains same, only currNode changes
      #Generate observation for other car's lateral motion
      edgeLabel = "Undetermined"
      #If reached target
      #println("Target y = ", target_y, " y = ", y, " target lane = ", targetLane, " lane = ", ln)
      if (abs(target_y - y) < LANE_WIDTH/16.0) && (targetLane == ln) #Second part is important
        edgeLabel = "Reached"
        #else
      elseif targetLane == ln #If already in the targetLane then might as well finish moving to center
        edgeLabel = "SafeSmooth"
      else
        #Check smooth and safe, first check if nextLeading and nextFollowing are known
        for nxtLnIS in globalISL1.neighborhood[nextLane]
            if nxtLnIS.physicalState.state[1] >= carPhySt.state[1]
              nxtLdPhySt = nxtLnIS.physicalState
            end
            if nxtLnIS.physicalState.state[1] <= carPhySt.state[1]
              nxtFlPhySt = nxtLnIS.physicalState
              break
            end
        end
        if egoLane == nextLane
            if egoState.state[1] >= carPhySt.state[1]
              if isnull(nxtLdPhySt) || (nxtLdPhySt.state[1] > egoState.state[1])
                  nxtLdPhySt = egoState
              end
            end
            if egoState.state[1] <= carPhySt.state[1]
              if isnull(nxtFlPhySt) || (nxtFlPhySt.state[1] < egoState.state[1])
                nxtFlPhySt = egoState
              end
            end
        end
        isSmooth = false
        isSafe = false
        if !isnull(nxtLdPhySt)
          isSmooth = isLaneChangeSmooth(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtLdPhySt, ddotx)
          if !isSmooth
            edgeLabel = "UnsafeOrUnsmooth"
          end
        end
        if !isnull(nxtFlPhySt)
          isSafe = isLaneChangeSafe(carFrame.lateral, carFrame.longitudinal, carPhySt, nxtFlPhySt)
          if !isSafe
            edgeLabel = "UnsafeOrUnsmooth"
          end
        end

        if (isSafe && isSmooth)
            edgeLabel = "SafeSmooth"
        end
      end
      #println("EdgeLabel = ", edgeLabel)
      updatedNode = currNode

      rnd = Base.rand(rng)
      #println("rnd = ", rnd)
      cumProb = 0.0
      for nextNode in fsm.nodeSet
        #println("NextNode = ", nextNode.nodeLabel)
        cumProb += get(fsm.transitionProb,(currNode, FSM_Edge(edgeLabel), nextNode),0.0)
        #println("CumProb = ", cumProb)
        if rnd < cumProb
          updatedNode = nextNode
          break
        end
      end
      #println("cNode = ",currNode.nodeLabel," edgeLabel = ",edgeLabel, " nNode = ",updatedNode.nodeLabel)
      updatedModel = ParamCarModelL0(targetLane, updatedFrame, updatedNode)
      updatedIS = CarLocalIS(updatedCarPhySt, updatedModel)
      push!(updatedNeighborhood[ln], updatedIS)

      ldCarIS = carIS

    end
  end

  return sortNeighborhood(updatedNeighborhood, p)

end
