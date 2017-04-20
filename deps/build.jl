packages = keys(Pkg.installed())
if !in("AutomotiveDrivingModels", packages)
    Pkg.clone("https://github.com/tawheeler/AutomotiveDrivingModels.jl.git")
end
Pkg.build("AutomotiveDrivingModels")
if !in("AutoViz", packages)
    Pkg.clone("https://github.com/tawheeler/AutoViz.jl.git")
end
Pkg.build("AutoViz")
