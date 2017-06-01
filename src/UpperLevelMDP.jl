#=
type UpperLevelState
  egoLane::UInt64
  egoCell::UInt64
  egoVelocity::Float64             #To compute next state, noisy.
  leadVehicleState::UInt64
  ltCellState::NTuple(3,UInt64)
  rtCellState::NTuple(3,UInt64)
end

type UpperLevelMDP <:POMDPs.MDP{UpperLevelState, Int64}
  discount_factor::Float64
  TIME_STEP::Float64
  length::Float64
  lanes::UInt64
  cellSize::Float64
  goalCell::NTuple{2, UInt64}
  goalReward::Float64
  idm_models::Array{IDMParam,1}
end

UpperLevelMDP() = UpperLevelMDP(0.9, 3.0, 600.0, 4, 75.0, (4,6), 50.0, [IDMParam(a=1.4, b=2.0, T=1.5, xdot0=AVG_HWY_VELOCITY, g0=AVG_GAP, del=4.0)])

discount(p::UpperLevelMDP) = p.discount_factor
=#
