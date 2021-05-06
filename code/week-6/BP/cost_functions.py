from collections import namedtuple
from math import sqrt, exp

TrajectoryData = namedtuple(
    "TrajectoryData", [
        'intended_lane',
        'final_lane',
        'end_distance_to_goal',
    ]
)

'''
Provided here are templates for two possible suggestions for cost functions,
but other metrics can be used as long as they represent the cost for choosing
a specific behaviour.
The weighted cost over all cost functions is computed in calculate_cost.
Cost calculation of each cost function is based on the trajectory data
whose format is defined above, and the trajectory data values are
computed by get_helper_data function.
'''

# weights for costs

REACH_GOAL = 0.6
EFFICIENCY = 1.0

DEBUG = True

def goal_distance_cost(vehicle, trajectory, predictions, data):
    '''
    Cost increases based on distance of intended lane (for planning a
    lane change) and final lane of a trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches
    the goal distance.
    '''
    goal_lane = vehicle.goal_lane
    current_dist = vehicle.s

    weight1 = 5.0
    weight2 = 2.0
    weight3 = 1.0

    cost = weight1*(current_dist/(current_dist + data.end_distance_to_goal)) + weight2*abs(goal_lane - data.intended_lane) + weight3*abs(goal_lane - data.final_lane)

    # print("data.intended_lane : ", data.intended_lane)
    # print("data.end_distance_to_goal : ", data.end_distance_to_goal)
    # print("vehicle.goal_lane : ", vehicle.goal_lane)
    # print("vehicle.s : ", vehicle.s)

    print("cost : ", cost)

    return cost

def inefficiency_cost(vehicle, trajectory, predictions, data):
    '''
    Cost becomes higher for trajectories with intended lane and final lane
    that have slower traffic.
    '''
    current_dist = vehicle.s

    lane_num = vehicle.lanes_available
    lane = []
    lane_weights = []
    num = 0

    intended_weight, final_weight = 0, 0

    for index in range(lane_num):
        lane.append(num)
        num = num + 1

    for index in range(lane_num):
        weight = 2.0*(lane_num-index)
        lane_weights.append(weight)

    # print("lane : ", lane)
    # print("lane_weights : ", lane_weights)

    for i in range(lane_num):
        if data.intended_lane == lane[i]:
            intended_weight = lane_weights[i]

        if data.final_lane == lane[i]:
            final_weight = lane_weights[i]

    # print("lane : ", lane)
    # print("lane_weights : ", lane_weights)
    # print("intended_weight : ", intended_weight)
    # print("final_weight : ", final_weight)
    # print("current_dist : ", current_dist)
    # print("data.end_distance_to_goal : ", data.end_distance_to_goal)

    cost = (current_dist / (current_dist + data.end_distance_to_goal)) + (intended_weight + final_weight)
    if current_dist > data.end_distance_to_goal:
        cost = 0.2*cost

    return cost

def calculate_cost(vehicle, trajectory, predictions):
    '''
    Sum weighted cost functions to get total cost for trajectory.
    '''
    trajectory_data = get_helper_data(vehicle, trajectory, predictions)
    cost = 0.0
    # list of cost functions and their associated costs
    cf_list = [goal_distance_cost, inefficiency_cost]
    weight_list = [REACH_GOAL, EFFICIENCY]

    for weight, cf in zip(weight_list, cf_list):
        new_cost = weight \
                   * cf(vehicle, trajectory, predictions, trajectory_data)
        if DEBUG:
            print(
                "%s has cost %.1f for lane %d" % \
                (cf.__name__, new_cost, trajectory[-1].lane)
            )
        cost += new_cost
    return cost

def get_helper_data(vehicle, trajectory, predictions):
    '''
    Generate helper data to use in cost functions:
    intended_lane:  +/-1 from the current lane if the vehicle is planning
                    or executing a lane change.
    final_lane: The lane of the vehicle at the end of the trajectory.
                The lane is unchanged for KL and LCL/LCR trajectories.
    distance_to_goal: The s distance of the vehicle to the goal.

    Note that intended_lane and final_lane are both included to help
    differentiate between planning and executing a lane change
    in the cost functions.
    '''

    last = trajectory[1]

    if last.state == "PLCL":
        intended_lane = last.lane + 1
    elif last.state == "PLCR":
        intended_lane = last.lane - 1
    else:
        intended_lane = last.lane

    distance_to_goal = vehicle.goal_s - last.s
    final_lane = last.lane

    return TrajectoryData(
        intended_lane,
        final_lane,
        distance_to_goal
    )

def velocity(predictions, lane):
    '''
    Get the velocity (speed) flowing in the specified lane.
    All non-ego vehicles in a lane have the same speed,
    so to get the speed limit for a lane,
    we can just find one vehicle in that lane.
    '''
    for v_id, predicted_trajectory in predictions.items():
        if predicted_trajectory[0].lane == lane and v_id != -1:
            return predicted_trajectory[0].v
