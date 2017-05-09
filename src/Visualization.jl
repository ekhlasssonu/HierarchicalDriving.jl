# imports are here in case this is ever moved out of the module

using Vec
import AutomotiveDrivingModels: Scene, Vehicle, VehicleState, VehicleDef, gen_straight_roadway
using AutoViz
# using HierarchicalDriving
using Iterators

import Base.show

show(io::IO, mime::MIME"image/png", t::Tuple{LowLevelMDP, GlobalStateL1}) = show(io, mime, (t..., [CarVelOverlay(), CarIDOverlay()]))

function show{AT<:AbstractArray}(io::IO, mime::MIME"image/png", t::Tuple{LowLevelMDP, GlobalStateL1, AT})
    mdp = t[1]
    s = t[2]
    scene = convert(Scene, s)
    n_lanes = length(mdp.nbrLaneMarkings)-1

    es = s.ego.state
    # origin is center of rightmost lane
    roadway = gen_straight_roadway(n_lanes,
                                   es[1]+1000.0,
                                   lane_widths=diff(mdp.nbrLaneMarkings),
                                   origin=VecSE2(es[1]-500.0, mean(mdp.nbrLaneMarkings[1:2]), 0.0),
                                  )
    # cam = FitToContentCamera(0.1)
    cam = SceneFollowCamera(12.0) # second number is pixels per meter
    c = render(scene, roadway, t[3], cam=cam)
    show(io, mime, c)
end

function show(io::IO, mime::MIME"image/png", s::GlobalStateL1)
    scene = convert(Scene, s)
    cam = SceneFollowCamera(12.0) # second number is pixels per meter
    c = render(scene, nothing, [CarVelOverlay(), CarIDOverlay()], cam=cam)
    show(io, mime, c)
end

function convert(::Type{Scene}, s::GlobalStateL1)
    scene = Scene()
    es = s.ego.state
    push!(scene, Vehicle(VehicleState(VecSE2(es[1], es[2], 0.0), es[3]),
                         VehicleDef(), 0))
    for (i,cs) in enumerate(chain(s.neighborhood...))
        s = cs.physicalState.state
        push!(scene, Vehicle(VehicleState(VecSE2(s[1], s[2], 0.0), s[3]),
                             VehicleDef(), i))
    end
    return scene
end

type CarVelOverlay <: SceneOverlay end

function AutoViz.render!(rm::RenderModel, o::CarVelOverlay, scene::Scene, roadway)
    for (i,v) in enumerate(scene)
        cx = v.state.posG.x
        cy = v.state.posG.y
        vx = cx + v.def.length/2 + 1.0
        vy = cy + v.def.width/2 - 0.5
        add_instruction!(rm, render_text,
                         (@sprintf("%04.1f",v.state.v), vx, vy, 14, colorant"white"))
    end
end

type CarIDOverlay <: SceneOverlay end

function AutoViz.render!(rm::RenderModel, o::CarIDOverlay, scene::Scene, roadway)
    for (i,v) in enumerate(scene)
        cx = v.state.posG.x
        cy = v.state.posG.y
        # nx = cx - v.def.length/2
        # ny = cy + v.def.width/2 - 0.6
        # add_instruction!(rm, render_text,
        #                  (@sprintf("%02.d",i), nx, ny, 14, colorant"white"))
        idx = cx + v.def.length/2 + 1.0
        idy = cy - v.def.width/2
        add_instruction!(rm, render_text,
                         (@sprintf("%02.d",v.id), idx, idy, 14, colorant"white"))
    end
end
