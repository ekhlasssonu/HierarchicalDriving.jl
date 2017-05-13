# XXX eventually this should not go here!!
function setrng!(sim::HistoryRecorder, rng::AbstractRNG)
    sim.rng = rng
    return sim
end
# by default do nothing
setrng!(o, rng::AbstractRNG) = o #XXX this should be a generated function that throws a warning
function setrng!(p::RandomPolicy, rng::AbstractRNG)
    p.rng = rng
end

# SimResult #
#############
immutable SimResult
    d::Dict{Symbol, Any}
end

SimResult(;kwargs...) = SimResult(Dict{Symbol, Any}(kwargs...))
function SimResult(problem, policy, result ;kwargs...)
    SimResult(Dict{Symbol, Any}(:problem=>problem, :policy=>policy, :result=>result, kwargs...))
end

Base.haskey(s::SimResult, k) = haskey(s.d, k)
Base.getindex(s::SimResult, k) = s.d[k]
Base.setindex!(s::SimResult, v, k) = setindex!(s.d, v, k)

#= keys
Always Present:
    problem::Union{POMDP, MDP}
    policy::Policy
    result::Any # result from the individual_simulator
Sometimes Present
    history::SimHistory # if the individual_simulator is a HistoryRecorder
=#


# PmapSimulator #
#################

type PmapSimulator <: Simulator
    analyze::Any # function or object returns a collection of pairs given a SimResult
    individual_simulator::Simulator
    seeds::Nullable{AbstractVector}
    show_progress::Bool # eventually maybe we should allow passing in a custom Progress
end
PmapSimulator(f::Function, is::Simulator;
              seeds=nothing,
              show_progress=true) = PmapSimulator(f, is, seeds, show_progress)
simulate(sim::PmapSimulator, args...) = simulate(sim.individual_simulator, args...)

function simulate(sim::PmapSimulator, problems::AbstractVector, policies::AbstractVector, args...)
    @assert length(problems) == length(policies)
    if isnull(sim.seeds)
        seeds = rand(UInt32, length(poblems))
    else
        seeds = get(sim.seeds)
    end
    @assert length(seeds) == length(problems)

    if sim.show_progress
        progress = Progress(length(problems), desc="Simulating: ")
        results = pmap(progress, seeds, problems, policies, args...) do seed, problem, policy, args...
            result = seed_simulate(seed, sim, problem, policy, args...)
            sr = SimResult(problem, policy, result)
            if isa(result, SimHistory)
                sr[:history] = result
            end
            return analyze(sim.analyze, sr)
        end
    else
        results = pmap(seeds, problems, policies, args...) do seed, problem, policy, args...
            hist = seed_simulate(seed, sim, problem, policy, args...)
            sr = SimResult(problem, policy, result)
            if isa(result, SimHistory)
                sr[:history] = result
            end
            return analyze(sim.analyze, sr)
        end
    end
    return make_dataframe(results)
end

function seed_simulate(seed, sim, problem, policy, args...)
    setrng!(sim, MersenneTwister(seed))
    setrng!(policy, MersenneTwister(hash(seed)))
    return simulate(sim, problem, policy, args...)
end

function make_dataframe(results::AbstractVector) # results should be a vector of collections of pairs
    columns = []
    for i in 1:length(results[1])
        t = results[1][i]
        name = first(t)
        values = Array(typeof(last(t)), length(results))
        for j in 1:length(results)
            t = results[j][i]
            @assert first(t)==name
            values[j] = last(t)
        end
        push!(columns, name=>values)
    end
    return DataFrame(;columns...)
end

analyze(f::Function, r::SimResult) = f(r)



# SimSet #
##########

immutable SimSet
    d::Dict{Symbol,Any}
end

Base.haskey(s::SimSet, k) = haskey(s.d, k)
Base.getindex(s::SimSet, k) = s.d[k]
Base.setindex!(s::SimSet, v, k) = setindex!(s.d, v, k)

#=
keys:
    policy_key::String
    policies::Dict{String, Any}
    problem_key::String
    problems::Dict{String, Any}
    simulator::Simulator
    n::Int
    seeds::AbstractVector
    name::String
=#

function SimSet(base::SimSet=SimSet(Dict{Symbol,Any}()),
                policy_key::String="",
                problem_key::String="";
                kwargs...
               )
    # specify default values here
    defaults = Dict{Symbol, Any}(:n=>1)

    keys = Dict{Symbol, Any}()
    if !isempty(policy_key)
        keys[:policy_key] = policy_key
    end
    if !isempty(problem_key)
        keys[:problem_key] = problem_key
    end

    # merge all the options specified up to this point
    combined = merge(defaults, base.d, Dict(kwargs), keys)

    # construct name if not supplied
    if !haskey(combined, :name) && haskey(combined, :policy_key) && haskey(combined, :problem_key)
        combined[:name] = combined[:policy_key]*"_"*combined[:problem_key]
    end

    if !haskey(combined, :seeds)
        combined[:seeds] = rand(UInt32, combined[:n])
    end
    return SimSet(combined)
end

function Base.run(sets::AbstractVector)
    policylist = []
    problemlist = []
    seeds = []
    names = []
    policy_keys = []
    problem_keys = []
    for set in sets
        policy = set[:policies][set[:policy_key]]
        for i in 1:set[:n]
            push!(problemlist, set[:problems][set[:problem_key]])
            push!(policylist, deepcopy(policy)) # XXX should it just be copy?
            push!(names, set[:name])
            push!(policy_keys, set[:policy_key])
            push!(problem_keys, set[:problem_key])
        end
        if haskey(set, :seeds)
            append!(seeds, set[:seeds])
        end
    end
    sim = first(sets)[:simulator]
    sim.seeds = Nullable{AbstractVector}(seeds)
    df = simulate(sim, problemlist, policylist)
    df[:set] = names
    df[:policy_key] = policy_keys
    df[:problem_key] = problem_keys
    df[:seed] = seeds
    return df
end
Base.run(s::SimSet) = run([s])



function rerun(dfrow, s::SimSet; sim=s[:simulator])
    policy = s[:policies][first(dfrow[:policy_key])]
    problem = s[:problems][first(dfrow[:problem_key])]
    seed = first(dfrow[:seed])
    return seed_simulate(seed, sim, problem, policy)
end
