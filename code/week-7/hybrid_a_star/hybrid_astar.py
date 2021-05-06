import numpy as np
import math

class HybridAStar:
    # Determine how many grid cells to have for theta-axis.
    NUM_THETA_CELLS = 360

    # Define min, max, and resolution of steering angles
    omega_min = -35
    omega_max = 35
    omega_step = 5

    # A very simple bicycle model
    speed = 1.0
    length = 0.5

    # Initialize the search structure.
    def __init__(self, dim):
        self.dim = dim
        self.closed = np.zeros(self.dim, dtype=np.int)
        self.came_from = np.full(self.dim, None)

    # Expand from a given state by enumerating reachable states.
    def expand(self, current, goal):
        g = current['g']
        x, y, theta = current['x'], current['y'], current['t']


        # The g value of a newly expanded cell increases by 1 from the
        # previously expanded cell.
        g2 = g + 1
        next_states = []
        # Consider a discrete selection of steering angles.
        for delta_t in range(self.omega_min, self.omega_max+1, self.omega_step):
            # print("delta_t : ", delta_t)
            omega = (self.speed / self.length) * math.tan(math.radians(delta_t))
            x2 = x + self.speed * math.cos(theta)
            y2 = y + self.speed * math.sin(theta)

            theta2 = (theta + omega) % (2*math.pi)
            f2 = g2 + self.heuristic(x2, y2, goal)
            state = {'x': x2, 'y': y2, 't': theta2, 'g': g2, 'f': f2}

            if 0 <= x2 < self.dim[1]:
                if 0 <= y2 < self.dim[2]:
                    next_states.append(state)

                    # if self.closed[(x2, y2)] == 0:
                    # if grid[(x_index, y_index)] == 0:
            # print("delta_t :", delta_t)
            # print("state :", next_states)


            # TODO: implement the trajectory generation based on
            # a simple bicycle model.
            # Let theta2 be the vehicle's heading (in radian)
            # between 0 and 2 * PI.
            # Check validity and then add to the next_states list.


        return next_states

    # Perform a breadth-first search based on the Hybrid A* algorithm.
    def search(self, grid, start, goal):
        # Initial heading of the vehicle is given in the
        # last component of the tuple start.
        theta = start[-1]
        # Determine the cell to contain the initial state, as well as
        # the state itself.
        stack = self.theta_to_stack_num(theta)
        g = 0
        s = {
            'f': self.heuristic(start[0], start[1], goal),
            'g': g,
            'x': start[0],
            'y': start[1],
            't': theta,
        }
        self.final = s
        # Close the initial cell and record the starting state for
        # the sake of path reconstruction.
        self.closed[stack][self.idx(s['x'])][self.idx(s['y'])] = 1
        self.came_from[stack][self.idx(s['x'])][self.idx(s['y'])] = s
        total_closed = 1
        opened = [s]
        # Examine the open list, according to the order dictated by
        # the heuristic function.
        # print('opened : ', opened)

        while len(opened) > 0:
            # TODO: implement prioritized breadth-first search
            # for the hybrid A* algorithm.
            opened.sort(key=lambda s : s['f'], reverse=True)
            curr = opened.pop()
            x, y = curr['x'], curr['y']
            if (self.idx(x), self.idx(y)) == goal:
                self.final = curr
                found = True
                break

            # Compute reachable new states and process each of them.
            next_states = self.expand(curr, goal)

            for n in next_states:
                x_index, y_index = self.idx(n['x']), self.idx(n['y'])
                dist_x = abs(self.idx(x)-x_index)
                dist_y = abs(self.idx(y)-y_index)
                dist = dist_x + dist_y

                # if dist > 1:
                #     continue


                stack = self.theta_to_stack_num(n['t'])
                if 0 <= x_index < self.dim[1]:
                    if 0 <= y_index < self.dim[2]:
                        if grid[(x_index, y_index)] == 0:
                            if self.closed[stack][x_index][y_index] == 0:
                                self.closed[stack][x_index][y_index] = 1
                                self.came_from[stack][x_index][y_index] = curr
                                opened.append(n)
                                total_closed = total_closed + 1
                                    # print('total_closed : ', total_closed)

        else:
            # We weren't able to find a valid path; this does not necessarily
            # mean there is no feasible trajectory to reach the goal.
            # In other words, the hybrid A* algorithm is not complete.
            found = False

        return found, total_closed

    # Calculate the stack index of a state based on the vehicle's heading.
    def theta_to_stack_num(self, theta):
        # TODO: implement a function that calculate the stack number
        # given theta represented in radian. Note that the calculation
        # should partition 360 degrees (2 * PI rad) into different
        # cells whose number is given by NUM_THETA_CELLS.
        degree = math.degrees(theta)
        value = 360/self.NUM_THETA_CELLS
        stack = (degree/value)
        # print('theta : ', theta)
        # print('degree : ', degree)
        # print('value : ', value)
        # print('stack : ', stack)
        # print('int stack : ', int(stack))
        if stack == self.NUM_THETA_CELLS:
            stack = 0

        return int(stack)

    # Calculate the index of the grid cell based on the vehicle'as position.
    def idx(self, pos):
        # We simply assume that each of the grid cell is the size 1 X 1.
        return int(np.floor(pos))

    # Implement a heuristic function to be used in the hybrid A* algorithm.
    def heuristic(self, x, y, goal):
        # TODO: implement a heuristic function.
        #Euclidean Distance
        h_cost = np.sqrt((goal[1]-y)**2+(goal[0]-x)**2)
        return h_cost

    # Reconstruct the path taken by the hybrid A* algorithm.
    def reconstruct_path(self, start, goal):
        # Start from the final state, and follow the link to the
        # previous state using the came_from matrix.
        curr = self.final
        x, y = curr['x'], curr['y']
        path = []
        while x != start[0] and y != start[1]:
            path.append(curr)
            stack = self.theta_to_stack_num(curr['t'])
            x, y = curr['x'], curr['y']
            curr = self.came_from[stack][self.idx(x)][self.idx(y)]
        # Reverse the path so that it begins at the starting state
        # and ends at the final state.
        path.reverse()
        return path
