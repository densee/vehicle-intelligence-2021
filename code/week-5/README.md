# Week 5 - Path Planning & the A* Algorithm

---

## Report
simple path planning algorithm based on the dynamic programming technique

* Cost Function에 따라 경로 계획이 달라지는 예제이다.
  State space와 Cost function의 내용을 아래와 같이 보완하였다.
  
  
### * optimum_policy_2D

    def optimum_policy_2D(grid, init, goal, cost):
        value = np.full((4, ) + grid.shape, 999, dtype=np.int32)
        policy = np.full((4,) + grid.shape, -1, dtype=np.int32)
        policy2D = np.full(grid.shape, ' ')

        change = True
        while change:
            change = False
            p = itertools.product(
                range(grid.shape[0]),
                range(grid.shape[1]),
                range(len(forward))
            )   

            for y, x, t in p:
                if (y, x) == goal and value[(t, y, x)] > 0:
                    value[(t, y, x)] = 0
                    policy2D[(y, x)] = '*'

                    change = True
                    pass
                elif grid[(y, x)] == 0:                     ## no obstacle
                    for d, a in zip(action, action_name):
                        direction = (t + d) % len(forward)
                        y2, x2 = y + forward[direction][0], x + forward[direction][1]

                        if 0 <= y2 < grid.shape[0] \
                            and 0 <= x2 < grid.shape[1] \
                            and grid[(y2, x2)] == 0:

                            for i in action:
                                if d == action[i]:
                                    cost_value = cost[i]

                            v2 = value[(direction, y2, x2)] + cost_value

                            if v2 < value[(t, y, x)]:
                                change = True
                                value[(t, y, x)] = v2
                                policy[(t, y, x)] = d

        y, x, dir = init
        start = policy[(dir, y, x)]

        for i in action:
            if start == action[i]:
                start_name = action_name[i]
                policy2D[(y, x)] = start_name

        while policy2D[(y, x)] != '*':

            for t in range(len(forward)):
                if t == 0:
                    min_value = value[t, y, x]

                elif value[t, y, x] < min_value:
                    min_value = value[t, y, x]

            for i in action:
                if (policy[(dir, y, x)] == action[i]):
                    policy2D[(y, x)] = action_name[i]

            for i in action:
                if policy[(dir, y, x)] == action[i]:
                    move_dir = (dir + action[i]) % len(forward)

            move_y, move_x = y + forward[move_dir][0], x + forward[move_dir][1]

            y = move_y
            x = move_x
            dir = move_dir

        return policy2D


* t는 방향의 정보(Up, Left, Down, Right)를 나타내며
  각 방향들에 대한 2차원 grid(x,y)가 존재한다.
  2차원 grid에서 장애물가 있을 경우 해당 좌표의 값은 1이므로 
  값이 0인 좌표들에 대해서 search를 시작한다.

* d는 Turn_Right, Straight, Turn_Left의 action 정보를 나타내며
  direction = (t + d) % len(forward) 를 통해
  현재 방향 상태(Up, Left, Down, Right)에서 action을 통해 이동된 방향의 정보를 선택한다.
  
* 이후 이동하려는 cell에 장애물이 있는지와 주어진 grid로부터 벗어났는지 체크하고
  action에 따라 다른 cost 값을 부여하여 계산한다.
  만일 계산한 cost 값이 이전에 저장된 값보다 작다면 action과 cost를 업데이트한다.
  
* 방향 문자열이 표시된 Map인 policy2D에서 초기 start지점에 action_name을 갱신한다.
  policy(dir, y, x)에 저장된 action 정보에 따라
  policy2D에서 해당 action name을 저장한다.
  이 작업은 Goal 지점을 찾을 때까지 반복하여 수행한다.   
