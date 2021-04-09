import numpy as np
from math import exp
from numpy.random import uniform
from helpers import distance

class ParticleFilter:
    def __init__(self, num_particles):
        self.initialized = False
        self.num_particles = num_particles

    # Set the number of particles.
    # Initialize all the particles to the initial position
    #   (based on esimates of x, y, theta and their uncertainties from GPS)
    #   and all weights to 1.0.
    # Add Gaussian noise to each particle.
    def initialize(self, x, y, theta, std_x, std_y, std_theta):
        self.particles = []
        for i in range(self.num_particles):
            self.particles.append({
                'x': np.random.normal(x, std_x),
                'y': np.random.normal(y, std_y),
                't': np.random.normal(theta, std_theta),
                'w': 1.0,
                'assoc': [],
            })
        self.initialized = True

    # Add measurements to each particle and add random Gaussian noise.
    def predict(self, dt, velocity, yawrate, std_x, std_y, std_theta):
        # Be careful not to divide by zero.
        cnt = 0
        v_yr = velocity / yawrate if yawrate else 0
        yr_dt = yawrate * dt
        for p in self.particles:
            # We have to take care of very small yaw rates;
            #   apply formula for constant yaw.
            if np.fabs(yawrate) < 0.0001:
                xf = p['x'] + velocity * dt * np.cos(p['t'])
                yf = p['y'] + velocity * dt * np.sin(p['t'])
                tf = p['t']
            # Nonzero yaw rate - apply integrated formula.
            else:
                xf = p['x'] + v_yr * (np.sin(p['t'] + yr_dt) - np.sin(p['t']))
                yf = p['y'] + v_yr * (np.cos(p['t']) - np.cos(p['t'] + yr_dt))
                tf = p['t'] + yr_dt
            p['x'] = np.random.normal(xf, std_x)
            p['y'] = np.random.normal(yf, std_y)
            p['t'] = np.random.normal(tf, std_theta)

    # Find the predicted measurement that is closest to each observed
    #   measurement and assign the observed measurement to this
    #   particular landmark.
    def associate(self, predicted, observations):
        associations = []
        # For each observation, find the nearest landmark and associate it.
        #   You might want to devise and implement a more efficient algorithm.
        for o in observations:
            min_dist = -1.0
            for p in predicted:
                dist = distance(o, p)
                if min_dist < 0.0 or dist < min_dist:
                    min_dist = dist
                    min_id = p['id']
                    min_x = p['x']
                    min_y = p['y']
            association = {
                'id': min_id,
                'x': min_x,
                'y': min_y,
            }
            associations.append(association)
        # Return a list of associated landmarks that corresponds to
        #   the list of (coordinates transformed) predictions.
        return associations

    def norm_pdf(self, x, m, s):
        one_over_sqrt_2pi = 1 / np.sqrt(2 * np.pi)
        return (one_over_sqrt_2pi / s) * exp(-0.5 * ((x - m) / s) ** 2)

    def bivariate_pdf(self, x1, x2, y1, y2, s1, s2):
        #Bivariate Normal Distribution
        cor = 0.0
        one_over_sqrt_2pi = 1 / np.sqrt(2 * np.pi*s1*s2)
        tmp = ((x1-x2)**2/s1**2) + ((y1-y2)**2/s2**2) -2*cor*((x1-x2)*(y1-y2)/s1*s2)
        return one_over_sqrt_2pi * exp(-0.5 * tmp)

    def calculate_dist(self, value1, value2):
        dist = (value1**2 + value2**2)**0.5
        return dist

    def multivariate_normal_pdf(self, x, mean, cov, k):
        x_m = x - mean
        cov_det = abs(np.linalg.det(cov))
        one_over_sqrt_2pi = 1 / (np.sqrt(((2 * np.pi) ** k) * cov_det))
        return one_over_sqrt_2pi * np.exp(-0.5 * np.dot(np.dot(np.transpose(x_m), np.linalg.inv(cov)), x_m))

    # Update the weights of each particle using a multi-variate
    #   Gaussian distribution.
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y, observations, map_landmarks):


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

    # Resample particles with replacement with probability proportional to
    #   their weights.

    def resample(self):
        # TODO: Select (possibly with duplicates) the set of particles
        #       that captures the posteior belief distribution, by
        # 1. Drawing particle samples according to their weights.
        # 2. Make a copy of the particle; otherwise the duplicate particles
        #    will not behave independently from each other - they are
        #    references to mutable objects in Python.
        # Finally, self.particles shall contain the newly drawn set of
        #   particles.

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


    # Choose the particle with the highest weight (probability)
    def get_best_particle(self):
        highest_weight = -1.0
        for p in self.particles:
            if p['w'] > highest_weight:
                highest_weight = p['w']
                best_particle = p
        return best_particle
