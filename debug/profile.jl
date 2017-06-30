using HierarchicalDriving
import POMDPs:initial_state
using POMDPToolbox
using Interact
using MCTS
using AutoViz
import ParticleFilters: obs_weight
import Base: ==, +, *, -, <, >, copy, Random, hash

HORIZON = 60
TIME_STEP = 0.3
lb_x = (AVG_HWY_VELOCITY - 5.0) * TIME_STEP * HORIZON
ub_x = (AVG_HWY_VELOCITY + 5.0) * TIME_STEP * HORIZON
p2 = SimulationMDP(0.9, 0.2, 40,
                   HierarchicalDriving.RoadSegment((-100.0, 500.0),[0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH]),
                   CarPhysicalState((0.0, 1.0 * LANE_WIDTH/2.0, AVG_HWY_VELOCITY)),
                   (CarPhysicalState((425.0, 7.0 * LANE_WIDTH/2.0 - 0.5, AVG_HWY_VELOCITY - 0.5)),
                   CarPhysicalState((500.0, 7.0 * LANE_WIDTH/2.0 + 0.5, AVG_HWY_VELOCITY + 0.5))),
                   HierarchicalDriving.getFrameList() );

using ProfileView

solver = DPWSolver(depth=HORIZON,
                   exploration_constant=10.0,
                   n_iterations=500,
                   k_action=10.0,
                   alpha_action=1/10,
                   k_state=5.0,
                   alpha_state=1/10)

policy1 = solve(solver, p2)
ro = RolloutSimulator(max_steps=1, rng=MersenneTwister(4))
@time simulate(ro, p2, policy1)
@time simulate(ro, p2, policy1)
Profile.clear()
@profile simulate(ro, p2, policy1)
ProfileView.view()
