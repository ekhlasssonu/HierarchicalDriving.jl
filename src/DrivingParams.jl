LANE_WIDTH = 4.0
AVG_GAP = 75.0 #53.5 #From steady state #40.24 #Derived from 1.5 second time gap at 60mph
AVG_HWY_VELOCITY = 25.0
VEL_STD_DEV = 2.5

#Car dimensions
CAR_LENGTH = 6.0
CAR_WIDTH = 2.5

LONG_ACCLN = 2.0
LAT_VEL = 2.0

import Base: ==, hash, copy

#=
Modeling other vehicles.

Other vehicle's longitudinal motion is governed by the IDM model.
Lateral motion is governed by a modified version of the MOBIL model. MOBIL is meant for macroscopic motion of traffic. We restrict it to local interactions.
In addition to modified MOBIL model, we add intentions to other vehicle's model. A vehice's motion is determined by its intention and MOBIL parameters.

=#

#=
IDM for motion along the x axis. Output: acceleration along x-axis.
Here, the only unknown in IDM model among all parameters is the desired highway velocity xdot0.
=#
type IDMParam
	a::Float64 #max  comfy acceleration
	b::Float64 #max comfy brake speed
	T::Float64 #desired safety time headway
	xdot0::Float64 #desired speed
	g0::Float64 #minimum headway (e.g. if x is less than this then you crashed)
	del::Float64 #'accel exponent'
end #IDMParam
IDMParam(;a::Float64=1.4, b::Float64=2.0, T::Float64=1.5, xdot0::Float64=AVG_HWY_VELOCITY, g0::Float64=AVG_GAP, del::Float64=4.0) = IDMParam(a,b,T,xdot0,g0,del)
IDMParam(xdot0::Float64) = IDMParam(1.4, 2.0, 1.5, xdot0, AVG_GAP, 4.0)
==(a::IDMParam,b::IDMParam) = (a.a==b.a) && (a.b==b.b) &&(a.T == b.T)&&(a.xdot0==b.xdot0)&&(a.g0==b.g0)&&(a.del==b.del)
Base.hash(a::IDMParam,h::UInt64=zero(UInt64)) = hash(a.a,hash(a.b,hash(a.T,hash(a.xdot0,hash(a.g0,hash(a.del,h))))))

#=
When leading vehicle is not in scope, set gap (g) to AVG_GAP and dxdot to 0
=#
function get_idm_g_star(idm::IDMParam, xdot::Float64, dxdot::Float64=0.0)#Desired gap
    #return idm.g0 + max(0.,xdot*idm.T + xdot*dxdot/(2*sqrt(idm.a*idm.b))) #This is how it was implemented earlier, no idea why
		return idm.g0 + xdot*idm.T + xdot*dxdot/(2*sqrt(idm.a*idm.b))
end

function get_idm_accln(idm::IDMParam,xdot::Float64,dxdot::Float64=0.0,g::Float64=AVG_GAP)
	g_ = get_idm_g_star(idm, xdot, dxdot)
	#println("\txdot = $xdot idm.xdot0 = ", idm.xdot0," => (xdot/idm.xdot0)^idm.del = ",(xdot/idm.xdot0)^idm.del," g_ = $g_ g = $g =>  (g_/g)^2 = ", (g_/g)^2)
  xddot = idm.a*(1. - (xdot/idm.xdot0)^idm.del - (g_/g)^2)
	#dvdt = min(max(dvdt,-p.b),p.a)
	return xddot
end

#=
MOBIL is meant to guide the lateral motion of vehicle. However it is meant for macroscopic modeling.
Since at lower level, we limit the scope, we settle for only checking the safety criteria, i.e. does lane change induce unsafe braking in the new following vehicle?
To do so, we set p to 0 and a_thr to -Inf
=#

type MOBILParam
	p::Float64 #politeness factor
	b_safe::Float64 #safe braking value
	a_thr::Float64 #minimum accel
	#db::Float64 #lane bias #we follow the symmetric/USA lane change rule
end #MOBILParam
#MOBILParam(;p::Float64=0.0,b_safe::Float64=1.0,a_thr::Float64=0.0) = MOBILParam(p,b_safe,a_thr)
MOBILParam() = MOBILParam(0.0,1.0,-2.0) #Default for local problem modeling
==(a::MOBILParam,b::MOBILParam) = (a.p==b.p) && (a.b_safe==b.b_safe) && (a.a_thr == b.a_thr)
Base.hash(a::MOBILParam,h::UInt64=zero(UInt64)) = hash(a.p,hash(a.b_safe,hash(a.a_thr,h)))


function createIDM_timid()
  return IDMParam(1.4, 2.0, 1.5, AVG_HWY_VELOCITY-2.2352, 2.0, 4.0)

end

function createIDM_normal()
  return IDMParam(1.4, 2.0, 1.5, AVG_HWY_VELOCITY, 1.0, 4.0)

end

function createIDM_aggressive()
  return IDMParam(1.4, 2.0, 1.5, AVG_HWY_VELOCITY+2.2352, 0.0, 4.0)

end

function createMOBIL_timid()
  return MOBILParam(0.0, 2.0, -1.0)

end

function createMOBIL_normal()
  return MOBILParam(0.0, 2.0, -1.5)

end

function createMOBIL_aggressive()
  return MOBILParam(0.0, 2.0, -2.0)

end
