# Week 6 - Prediction & Behaviour Planning

---

## Report

# 1. Gaussian Naive Bayes

GNB 과제는 수집된 데이터를 이용하여 
각 주행(왼쪽차선 이동, 직진 이동, 오른쪽차선 이동)의 평균들과 표준편차들을 계산하고
Observation으로 취득된 데이터들과 평균들 및 표준편차들을 이용해 가능성이 높은 주행 경로를 예측하는 것이다.
코드는 아래와 같이 작성하였다.

### * Train 
    def train(self, X, Y):
        state_arr = []
        label_arr = []

        left_s, left_d, left_sdot, left_ddot = [], [], [], []
        keep_s, keep_d, keep_sdot, keep_ddot = [], [], [], []
        right_s, right_d, right_sdot, right_ddot = [], [], [], []

        # print("X : ", X)
        for x_component in X:
            [s, d, s_dot, d_dot] = self.process_vars(x_component)
            var = [s, d, s_dot, d_dot]
            state_arr.append(var)
            # print("state : ", state_arr)

        for y_component in Y:
            label = y_component
            label_arr.append(label)
            # print("label_arr : ", label_arr)


        for label_arr, state_arr in list(zip(label_arr, state_arr)):
            if label_arr == "left":
                left_s.append(state_arr[0])
                left_d.append(state_arr[1])
                left_sdot.append(state_arr[2])
                left_ddot.append(state_arr[3])
            elif label_arr == "keep":
                keep_s.append(state_arr[0])
                keep_d.append(state_arr[1])
                keep_sdot.append(state_arr[2])
                keep_ddot.append(state_arr[3])
            elif label_arr == "right":
                right_s.append(state_arr[0])
                right_d.append(state_arr[1])
                right_sdot.append(state_arr[2])
                right_ddot.append(state_arr[3])

        self.left_mean = [np.mean(left_s), np.mean(left_d), np.mean(left_sdot), np.mean(left_ddot)]
        self.left_std = [np.std(left_s), np.std(left_d), np.std(left_sdot), np.std(left_ddot)]

        self.keep_mean = [np.mean(keep_s), np.mean(keep_d), np.mean(keep_sdot), np.mean(keep_ddot)]
        self.keep_std = [np.std(keep_s), np.std(keep_d), np.std(keep_sdot), np.std(keep_ddot)]

        self.right_mean = [np.mean(right_s), np.mean(right_d), np.mean(right_sdot), np.mean(right_ddot)]
        self.right_std = [np.std(right_s), np.std(right_d), np.std(right_sdot), np.std(right_ddot)]

* train 함수에서는 데이터에 포함된 라벨에 따라 분류하여
  각각의 주행에서 s, d, sdot, ddot에 대한 평균과 표준편차들을 계산하였다.

### * Prediction
  

    def predict(self, observation):
        index = 0
        left_prob, keep_prob, right_prob = 1.0, 1.0, 1.0
        left_normalize, keep_normalize, right_normalize = 0.0, 0.0, 0.0
        for data in observation:

            left_prob *= gaussian_prob(data, self.left_mean[index], self.left_std[index])
            keep_prob *= gaussian_prob(data, self.keep_mean[index], self.keep_std[index])
            right_prob *= gaussian_prob(data, self.right_mean[index], self.right_std[index])

            left_normalize += gaussian_prob(data, self.left_mean[index], self.left_std[index])
            keep_normalize += gaussian_prob(data, self.keep_mean[index], self.keep_std[index])
            right_normalize += gaussian_prob(data, self.right_mean[index], self.right_std[index])

            index += 1

        left_normalize = 1/left_normalize
        keep_normalize = 1/keep_normalize
        right_normalize = 1/right_normalize

        left_prob = left_prob*left_normalize
        keep_prob = keep_prob*keep_normalize
        right_prob = right_prob * right_normalize

        prob_label = np.argmax([left_prob, keep_prob, right_prob])
  
        return self.classes[prob_label]


* train에서 계산한 각 주행에 대한 평균과 표준편차 및 Observation data의 데이터들을 이용하여
  Observation에서 가장 높은 확률을 가진 주행 경로를 예측하는 것이다.
  계산된 정확도는 100점 중에 84.40 퍼센트를 보였다.
 ![percent](https://user-images.githubusercontent.com/48784519/117338544-ac7de480-aed9-11eb-92d3-54d9a07efefd.JPG)

# 2. Behaviour Planning
BP 과제는 cost(차선에 따른 속도, goal lane과의 거리)를 고려하여 현재 자동차에 대한 
Trajectory을 구하는 과제이다. 코드는 아래와 같이 작성하였다.

### * choose_next_state

    def choose_next_state(self, predictions):

        v_states = self.successor_states()

        print("state : ", v_states)  # state : ['KL', 'PLCL', 'PLCR']
        for i in range(len(v_states)):
            traj = self.generate_trajectory(v_states[i], predictions)
            cost = calculate_cost(self, traj, predictions)

            if i == 0:
                minimum_cost = cost
                minimum_cost_trajectory = traj

            else:
                if cost < minimum_cost:
                    minimum_cost = cost
                    minimum_cost_trajectory = traj

        return minimum_cost_trajectory

* choose_next_state 함수는 현재 자동차의 위치에 대해서 다음 state(KL, PLCL, PLCR, LCL, LCR)을 
  어디로 정할 것인지에 대해 결정한다.
  예상되는 state들에 대해서 갈 수 있는 state들인지 체크한후 
  현재 state에서 갈 수 있는 state로 이동하는 궤적을 계산한다.
  이후 각 궤적에 대한 cost를 계산하여 가장 비용이 최소가 되는 궤적을 선택한다.


### * goal_distance_cost  

    def goal_distance_cost(vehicle, trajectory, predictions, data):
    
      goal_lane = vehicle.goal_lane
      current_dist = vehicle.s

      weight1 = 5.0
      weight2 = 2.0
      weight3 = 1.0

      cost = weight1*(current_dist/(current_dist + data.end_distance_to_goal)) + weight2*abs(goal_lane - data.intended_lane) + weight3*abs(goal_lane - data.final_lane)

      return cost



* goal_distance의 경우에는 goal_lane과 현재 자동차가 주행중인 lane에 대한 cost를 계산한다. 총 3개의 가중치를 이용하여
  비용을 계산하였으며 가중치1(weight1)을 이용한 부분은 목표 지점에 가까울수록 증가시키게 하였다.
  이를 통해 목적지에 가까워지고 현재 주행하는 차선이 목표지점에 멀어질수록, cost를 더 부과하였다. 
* 가중치2(weight2)에 대한 부분은 목표 lane과 intended_lane 사이에 대한 거리 비용이고
  가중치3(weight3)에 대한 부분은 목표 lane과 final lane에 대한 거리 비용을 나타낸다.

### * inefficiency_cost 

    def inefficiency_cost(vehicle, trajectory, predictions, data):
    
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

      for i in range(lane_num):
          if data.intended_lane == lane[i]:
              intended_weight = lane_weights[i]

          if data.final_lane == lane[i]:
              final_weight = lane_weights[i]

      cost = (current_dist / (current_dist + data.end_distance_to_goal)) + (intended_weight + final_weight)
      if current_dist > data.end_distance_to_goal:
          cost = 0.2*cost

      return cost


* inefficient_cost는 각 lane에 대해서 일정한 속도로 주행해야한다는 조건하에, 속도가 낮을 경우 이를 비효율적이라고 판단하여 
  생성된 cost이다.
  속도가 느릴수록 시간이 더 많이 소요되기에 cost를 더 증가시키게 하였다.
* 자동차 입장에서, 최소의 비용을 가진 궤적을 찾는다는 것은 가장 빠른 차선을 통해 빠른 속도로 주행한 뒤, 목적지에 가까워 졌을 때
  목표 lane쪽으로 이동하는 것을 의미한다.
  inefficient_cost는 각 차선[1번 lane, 2번 lane, 3번 lane, 4번 lane]에 대해 [8 6 4 2] 로 cost를 설정하였다.
  즉 1번 lane은 가장 느린 속도를 가지고 있으므로 가장 큰 cost를 지닌다.
* 현재 거리가 목적지로부터 남은 거리보다 클 때, inefficient_cost보다 goal_distance_cost에 대한 비중을 더 높여서
  목표 lane쪽으로 이동하도록 하였다.
  
* 시뮬레이션 결과, Vehicle이 목표지점에 도달하기까지 33초가 소요되었다.
![BP_goal](https://user-images.githubusercontent.com/48784519/117338574-b3a4f280-aed9-11eb-9fe8-844dc3cb235c.JPG)
  

