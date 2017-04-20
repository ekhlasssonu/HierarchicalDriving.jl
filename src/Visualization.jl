# imports are here in case this is ever moved out of the module

using Vec
import AutomotiveDrivingModels: Scene, Vehicle, VehicleState, VehicleDef
using AutoViz
# using HierarchicalDriving
using Iterators

import Base.show

function show(io::IO, mime::MIME"image/png", s::GlobalStateL1)
    scene = Scene()
    es = s.ego.state
    push!(scene, Vehicle(VehicleState(VecSE2(es[1], es[2], 0.0), es[3]),
                         VehicleDef(), 0))
    for (i,cs) in enumerate(chain(s.neighborhood...))
        s = cs.physicalState.state
        push!(scene, Vehicle(VehicleState(VecSE2(s[1], s[2], 0.0), s[3]),
                             VehicleDef(), i))
    end
    c = render(scene, nothing, [CarVelOverlay(), CarIDOverlay()], cam=FitToContentCamera(0.1))
    show(io, mime, c)
end

type CarVelOverlay <: SceneOverlay end

function AutoViz.render!(rm::RenderModel, o::CarVelOverlay, scene::Scene, roadway)
    for (i,v) in enumerate(scene)
        cx = v.state.posG.x
        cy = v.state.posG.y
        vx = cx + v.def.length/2 - 1.4
        vy = cy + v.def.width/2 - 0.4
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
        idx = cx - v.def.length/2
        idy = cy - v.def.width/2 + 0.1
        add_instruction!(rm, render_text,
                         (@sprintf("%02.d",v.id), idx, idy, 14, colorant"white"))
    end
end

