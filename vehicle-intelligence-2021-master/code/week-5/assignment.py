import numpy as np
import itertools

# Given map
grid = np.array([
    [1, 1, 1, 0, 0, 0],
    [1, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0],
    [1, 1, 1, 0, 1, 1],
    [1, 1, 1, 0, 1, 1]
])

# List of possible actions defined in terms of changes in
# the coordinates (y, x)
forward = [
    (-1,  0),   # Up
    ( 0, -1),   # Left
    ( 1,  0),   # Down
    ( 0,  1),   # Right
]

# Three actions are defined:
# - right turn & move forward
# - straight forward
# - left turn & move forward
# Note that each action transforms the orientation along the
# forward array defined above.
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

init = (4, 3, 0)    # Representing (y, x, o), where
                    # o denotes the orientation as follows:
                    # 0: up
                    # 1: left
                    # 2: down
                    # 3: right
                    # Note that this order corresponds to forward above.
goal = (2, 0)
cost = (2, 1, 20)   # Cost for each action (right, straight, left)

# EXAMPLE OUTPUT:
# calling optimum_policy_2D with the given parameters should return
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]

def optimum_policy_2D(grid, init, goal, cost):
    # Initialize the value function with (infeasibly) high costs.
    value = np.full((4, ) + grid.shape, 999, dtype=np.int32)
    # Initialize the policy function with negative (unused) values.
    policy = np.full((4,) + grid.shape, -1, dtype=np.int32)
    # Final path policy will be in 2D, instead of 3D.

    policy2D = np.full(grid.shape, ' ')

    # print("x : ", x)
    # print("y : ", y)
    # print("t : ", t)
    # print("policy2D : ", policy2D)
    # print("value : ", value)
    # print("policy : ", policy)

    # print("d : ", d)
    # print("a : ", a)
    # print("y2 : ", y2)
    # print("x2 : ", x2)

    # Apply dynamic programming with the flag change.
    change = True
    while change:
        change = False
        # This will provide a useful iterator for the state space.
        p = itertools.product(
            range(grid.shape[0]),
            range(grid.shape[1]),
            range(len(forward))
        )
        # Compute the value function for each state and
        # update policy function accordingly.
        for y, x, t in p:
            # print("x : ", x)
            # print("y : ", y)
            # print("t : ", t)

            # Mark the final state with a special value that we will
            # use in generating the final path policy.
            if (y, x) == goal and value[(t, y, x)] > 0:
                value[(t, y, x)] = 0
                policy2D[(y, x)] = '*'

                change = True
                # TODO: implement code.
                pass
            # Try to use simple arithmetic to capture state transitions.
            elif grid[(y, x)] == 0:                     ## no obstacle
                for d, a in zip(action, action_name):
                    dir = (t + d) % 4
                    y2, x2 = y + forward[dir][0], x + forward[dir][1]

                    if 0 <= y2 < grid.shape[0] \
                            and 0 <= x2 < grid.shape[1] \
                            and grid[(y2, x2)] == 0:

                            v2 = value[(dir, y2, x2)] + cost[1]

                            if v2 < value[(t, y, x)]:
                                change = True
                                value[(t, y, x)] = v2
                                policy[(t, y, x)] = a

                # TODO: implement code.
                pass

    # Now navigate through the policy table to generate a
    # sequence of actions to take to follow the optimal path.
    # TODO: implement code.

    print("policy : ", policy)

    dir, y, x = init
    start_point = policy[(dir, y, x)]

    for d, a in zip(action, action_name):
        if start_point == action[d]:
            start_name = a

    # print("start_point : ", start_point)
    # print("start_name : ", start_name)

    # while not policy[(dir, y, x) != 999:
    #     if policy[(dir, y, x)] ==



    # Return the optimum policy generated above.
    return policy2D

print(optimum_policy_2D(grid, init, goal, cost))
