## Report

# Hybrid Astar

Hybrid Astar는 기존 Astar와 다르게 complete와 optimal하지 않은 알고리즘이다.

이동하는 물체는 장애물이 있는 grid 상에서 회전할 수 있는 조향 각도에 대한 제한을 가지며 이동한다.

### * expand 
    def expand(self, current, goal):
        g = current['g']
        x, y, theta = current['x'], current['y'], current['t']

        g2 = g + 1
        next_states = []

        for delta_t in range(self.omega_min, self.omega_max+1, self.omega_step):
            omega = (self.speed / self.length) * math.tan(math.radians(delta_t))
            x2 = x + self.speed * math.cos(theta)
            y2 = y + self.speed * math.sin(theta)

            theta2 = (theta + omega) % (2*math.pi)
            f2 = g2 + self.heuristic(x2, y2, goal)
            state = {'x': x2, 'y': y2, 't': theta2, 'g': g2, 'f': f2}

            if 0 <= x2 < self.dim[1]:
                if 0 <= y2 < self.dim[2]:
                    next_states.append(state)


            # TODO: implement the trajectory generation based on
            # a simple bicycle model.
            # Let theta2 be the vehicle's heading (in radian)
            # between 0 and 2 * PI.
            # Check validity and then add to the next_states list.


        return next_states

* expand 함수는 현재 state에서 이동할 수 있는 next state들에 대한 값을 반환한다.
  고정된 속도와 길이가 주어진 조건에서 motion_model를 통해 이동할 수 있는 다음 x, y에 대한 위치와 조향각들을
  계산한다.
  이때 이동하는 좌표가 grid 상에서 벗어났는지 확인하고 시작점으로부터 현재까지 이동한 g cost와 
  현재 위치에서 목적지까지에 대한 heuristic 함수를 계산하여 f cost를 구한다.
  expand 함수로부터 반환되는 state들은 x와 y위치, 조향각, g cost와 f cost를 포함한다.
  
### * search 

    def search(self, grid, start, goal):
        theta = start[-1]
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
        self.closed[stack][self.idx(s['x'])][self.idx(s['y'])] = 1
        self.came_from[stack][self.idx(s['x'])][self.idx(s['y'])] = s
        total_closed = 1
        opened = [s]

        while len(opened) > 0:
            opened.sort(key=lambda s : s['f'], reverse=True)
            curr = opened.pop()
            x, y = curr['x'], curr['y']
            if (self.idx(x), self.idx(y)) == goal:
                self.final = curr
                found = True
                break

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

        else:
            found = False

        return found, total_closed

* search 함수는 open list에 state들이 없거나 혹은 목적지에 도달할 때까지 계속 반복하여 목적지까지의 경로를 찾는다.
  시작점으로부터 시작하여 grid 상의 cell을 이동하면서 grid 상에서 벗어났는지, 장애물이 있는 cell인지, 이미 탐색한 cell인지 체크한다.
  3가지 조건에 해당되는 cell이 아니라면 해당 cell을 closed list에 추가하고 시작점에서 현재 cell까지 이동했을 때 지나간 cell들의 기록들을
  저장한 후 open list에 추가한다.
  
### * theta_to_stack_num 

    # Calculate the stack index of a state based on the vehicle's heading.
    def theta_to_stack_num(self, theta):
        degree = math.degrees(theta)
        value = 360/self.NUM_THETA_CELLS
        stack = (degree/value)

        if stack == self.NUM_THETA_CELLS:
            stack = 0

        return int(stack)

* theta_to_stack_num 함수는 자동차의 방향에 기반하여 stack의 index를 부여하는 함수이다.
  0~360도 사이에 있는 stack들 중 입력받은 각도들에 따른 stack의 index를 반환한다.
  
### * heuristic 

    def heuristic(self, x, y, goal):
        #Euclidean Distance
        h_cost = np.sqrt((goal[1]-y)**2+(goal[0]-x)**2)
        return h_cost

* heuristic 비용의 경우, 현재 위치와 목적지까지의 Euclidean Distance를 이용하였다.


## Result
자동차의 speed와 NUM_THETA_CELLS을 변경하여 나타난 결과는 아래와 같다.

* speed : 0.5 & NUM_THETA_CELLS : 90

![0 5speed_90cell](https://user-images.githubusercontent.com/48784519/117338903-13030280-aeda-11eb-96f7-44ddbf550ba9.JPG)

* speed : 1.0 & NUM_THETA_CELLS : 90

![1 0speed_90cell](https://user-images.githubusercontent.com/48784519/117339119-565d7100-aeda-11eb-83b4-c972f51b2c58.JPG)



* speed : 0.5 & NUM_THETA_CELLS : 180

![0 5speed_180cell](https://user-images.githubusercontent.com/48784519/117338933-1ac2a700-aeda-11eb-9157-8c75f8a5c704.JPG)

* speed : 1.0 & NUM_THETA_CELLS : 180

![1 0speed_180cell](https://user-images.githubusercontent.com/48784519/117339126-58bfcb00-aeda-11eb-9120-28953d863465.JPG)



* speed : 0.5 & NUM_THETA_CELLS : 360

![0 5speed_360cell](https://user-images.githubusercontent.com/48784519/117338984-2c0bb380-aeda-11eb-8884-5cf2257250d1.JPG)

* speed : 1.0 & NUM_THETA_CELLS : 360

![1 0speed_360cell](https://user-images.githubusercontent.com/48784519/117339137-5e1d1580-aeda-11eb-869f-c9997b1145fb.JPG)



