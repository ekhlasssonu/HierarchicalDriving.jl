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
