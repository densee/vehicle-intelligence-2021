# Week 4 - Motion Model & Particle Filters

---

[//]: # (Image References)
[empty-update]: ./empty-update.gif
[example]: ./example.gif

## Assignment

1. I put the observed data calculated by map frame into global_observation and the data observed  by vehicle into local_observation.
2. When I calculated multivariate normal pdf, I put observation data as input state, distance between landmark and particle as mean. 
3. After that, we calculated the weight of the particles and updated them through resampling.
The contents below are the contents of the code file I wrote.
   
The contents of the "particle_filter.py" are as follows:

    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,observations, map_landmarks):    
        for p in self.particles:
            visible_landmarks = []
            global_observations = []
            local_observations = []

            for map_landmark in map_landmarks:
                if distance(p, map_landmarks[map_landmark]) <= sensor_range:
                    map_landmark_x = map_landmarks[map_landmark]['x']
                    map_landmark_y = map_landmarks[map_landmark]['y']
                    map_landmark_id = map_landmark

                    visible_landmark = {
                        'id': map_landmark_id,
                        'x': map_landmark_x,
                        'y': map_landmark_y,
                    }
                    visible_landmarks.append(visible_landmark)

            for observation in observations:
                global_observation_x = p['x'] + observation['x']*np.cos(p['t']) - observation['y']*np.sin(p['t'])
                global_observation_y = p['y'] + observation['x']*np.sin(p['t']) + observation['y']*np.cos(p['t'])
                local_observation_x = observation['x']
                local_observation_y = observation['y']

                global_observation = {
                    'x': global_observation_x,
                    'y': global_observation_y,
                }

                local_observation = {
                    'x': local_observation_x,
                    'y': local_observation_y,
                }

                local_observations.append(local_observation)
                global_observations.append(global_observation)

            if len(visible_landmarks) == 0:
                continue

            p['assoc'] = []
            p['w'] = 1.0

            association_landmarks = self.associate(visible_landmarks, global_observations)

            for i in range(len(association_landmarks)):
                cov = [[std_landmark_x**2, 0],
                       [0, std_landmark_y**2]]
                arr_cov = np.array(cov)

                k = 2

                tmp_obs = [local_observations[i]['x'],
                           local_observations[i]['y']]
                arr_obs = np.array(tmp_obs)

                tmp_assoc = [association_landmarks[i]['x']-p['x'],
                             association_landmarks[i]['y']-p['y']]
                arr_assoc = np.array(tmp_assoc)

                p['w'] *= self.multivariate_normal_pdf(arr_obs, arr_assoc, cov, k)
                p['assoc'].append(association_landmarks[i]['id'])


    def multivariate_normal_pdf(self, x, mean, cov, k):
        x_m = x - mean
        cov_det = abs(np.linalg.det(cov))
        one_over_sqrt_2pi = 1 / (np.sqrt(((2 * np.pi) ** k) * cov_det))
        return one_over_sqrt_2pi * np.exp(-0.5 * np.dot(np.dot(np.transpose(x_m), np.linalg.inv(cov)), x_m))


    def resample(self):
        copied_particle = []
        weights = [p['w'] for p in self.particles]

        N = len(weights)
        positions = (np.arange(len(weights)) + np.random.random()) / N

        indexes = np.zeros(N, 'i')
        cumulative_sum = np.cumsum(weights)
        i, j = 0, 0

        while i < N and j < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1

        copied_particle.append(self.particles[indexes[i]])
        self.particles = copied_particle