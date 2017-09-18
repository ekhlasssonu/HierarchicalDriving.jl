LANE_WIDTH = 4.0
AVG_GAP = 75.0 #53.5 #From steady state #40.24 #Derived from 1.5 second time gap at 60mph
AVG_HWY_VELOCITY = 25.0
VEL_STD_DEV = 2.5

#Car dimensions
CAR_LENGTH = 6.0
CAR_WIDTH = 2.5
COLLISION_CUSHION = 0.0

LONG_ACCLN = 2.0
LAT_VEL = 2.0

road_segment = RoadSegment((-200.0, 1000.0),[0.0, LANE_WIDTH, 2.0 * LANE_WIDTH, 3.0 * LANE_WIDTH, 4.0 * LANE_WIDTH])

#LowLevelMDP parameters
ll_discount = 0.99
ll_TIME_STEP = 0.3
ll_HORIZON = 15
ll_goalReward = 50.0
ll_collisionCost = -500.0
ll_y_dev_cost = -2.0
ll_hardbrakingCost = -5.0
ll_discomfortCost = -5.0
ll_velocityDeviationCost = -0.0

ul_HORIZON = ll_HORIZON * 7
ul_TIME_STEP = 4.0

ul_n_agents = 60

#Simulation parameters
sim_TIME_STEP = 0.1
