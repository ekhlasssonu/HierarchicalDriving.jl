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

# PmapSimulator #
#################

type PmapSimulator <: Simulator
    analyze::Any # function or object returns a collection of pairs given the problem and a history
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
            return analyze(sim.analyze, problem, result)
        end
    else
        results = pmap(seeds, problems, policies, args...) do seed, problem, policy, args...
            result = seed_simulate(seed, sim, problem, policy, args...)
            return analyze(sim.analyze, problem, result)
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

analyze(f::Function, p::Union{MDP, POMDP}, h::SimHistory) = f(p, h)



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
    solver_key::String
    solvers::Dict{String, Any}
    solve_with::String (a problem key)
    problem_key::String
    problems::Dict{String, Any}
    simulator::Simulator
    n::Int
    seeds::AbstractVector
    name::String
=#

function SimSet(base::SimSet=SimSet(Dict{Symbol,Any}()),
                solver_key::String="",
                problem_key::String="";
                kwargs...
               )
    # specify default values here
    defaults = Dict{Symbol, Any}(:n=>1)

    keys = Dict{Symbol, Any}()
    if !isempty(solver_key)
        keys[:solver_key] = solver_key
    end
    if !isempty(problem_key)
        keys[:problem_key] = problem_key
        defaults[:solve_with] = problem_key
    end

    # merge all the options specified up to this point
    combined = merge(defaults, base.d, Dict(kwargs), keys)

    # construct name if not supplied
    if !haskey(combined, :name) && haskey(combined, :solver_key) && haskey(combined, :problem_key)
        combined[:name] = combined[:solver_key]*"_"*combined[:problem_key]
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
    solver_keys = []
    problem_keys = []
    solve_withs = []
    for set in sets
        solver = set[:solvers][set[:solver_key]] # XXX need to set solver rng!! setrng!(solver, )
        if isa(solver, Solver)
            solve_with = set[:problems][set[:solve_with]]
            policy = solve(solver, solve_with)
        else
            policy = solver
        end
        for i in 1:set[:n]
            push!(problemlist, set[:problems][set[:problem_key]])
            push!(policylist, deepcopy(policy)) # XXX should it just be copy?
            push!(names, set[:name])
            push!(solver_keys, set[:solver_key])
            push!(problem_keys, set[:problem_key])
            push!(solve_withs, set[:solve_with])
        end
        if haskey(set, :seeds)
            append!(seeds, set[:seeds])
        end
    end
    sim = first(sets)[:simulator]
    sim.seeds = Nullable{AbstractVector}(seeds)
    df = simulate(sim, problemlist, policylist)
    df[:set] = names
    df[:solver_key] = solver_keys
    df[:problem_key] = problem_keys
    df[:seed] = seeds
    df[:solved_with] = solve_withs
    return df
end
Base.run(s::SimSet) = run([s])



function rerun(dfrow, s::SimSet; sim=s[:simulator])
    solver = s[:solvers][first(dfrow[:solver_key])]
    if isa(solver, Solver)
        solve_with = s[:problems][s[:solve_with]]
        policy = solve(solver, solve_with)
    else
        policy = solver
    end
    problem = s[:problems][first(dfrow[:problem_key])]
    seed = first(dfrow[:seed])
    return seed_simulate(seed, sim, problem, policy)
end
