import numpy as np
import random
from math import sqrt, pi, exp
import numpy as np

def gaussian_prob(obs, mu, sig):
    # Calculate Gaussian probability given
    # - observation
    # - mean
    # - standard deviation
    num = (obs - mu) ** 2
    denum = 2 * sig ** 2
    norm = 1 / sqrt(2 * pi * sig ** 2)
    return norm * exp(-num / denum)

# Gaussian Naive Bayes class
class GNB():
    # Initialize classification categories

    left_mean, left_std = [], []
    keep_mean, keep_std = [], []
    right_mean, right_std = [], []

    def __init__(self):
        self.classes = ['left', 'keep', 'right']

    # Given a set of variables, preprocess them for feature engineering.
    def process_vars(self, vars):
        # The following implementation simply extracts the four raw values
        # given by the input data, i.e. s, d, s_dot, and d_dot.
        s, d, s_dot, d_dot = vars
        return s, d, s_dot, d_dot

    # Train the GNB using a combination of X and Y, where
    # X denotes the observations (here we have four variables for each) and
    # Y denotes the corresponding labels ("left", "keep", "right").
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



        '''
        Collect the data and calculate mean and standard variation
        for each class. Record them for later use in prediction.
        '''


        # TODO: implement code.

    # Given an observation (s, s_dot, d, d_dot), predict which behaviour
    # the vehicle is going to take using GNB.
    def predict(self, observation):
        index = 0
        left_prob, keep_prob, right_prob = 1.0, 1.0, 1.0
        left_normalize, keep_normalize, right_normalize = 0.0, 0.0, 0.0
        for data in observation:
            # print("data : ", data)
            # print("index : ", index)

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

        # print("left_prob : ", left_prob)
        # print("keep_prob : ", keep_prob)
        # print("right_prob : ", right_prob)

        prob_label = np.argmax([left_prob, keep_prob, right_prob])
        '''
        Calculate Gaussian probability for each variable based on the
        mean and standard deviation calculated in the training process.
        Multiply all the probabilities for variables, and then
        normalize them to get conditional probabilities.
        Return the label for the highest conditional probability.
        '''
        # TODO: implement code.
        return self.classes[prob_label]

