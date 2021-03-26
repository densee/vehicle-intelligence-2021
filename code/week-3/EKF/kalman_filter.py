import numpy as np
import math
from math import sqrt
from math import atan2
from tools import Jacobian

class KalmanFilter:
    def __init__(self, x_in, P_in, F_in, H_in, R_in, Q_in):
        self.x = x_in
        self.P = P_in
        self.F = F_in
        self.H = H_in
        self.R = R_in
        self.Q = Q_in

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # Calculate new estimates
        self.x = self.x + np.dot(K, z - np.dot(self.H, self.x))
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)

    def update_ekf(self, z):
        # TODO: Implement EKF update for radar measurements
        # 1. Compute Jacobian Matrix H_j
        # 2. Calculate S = H_j * P' * H_j^T + R
        # 3. Calculate Kalman gain K = H_j * P' * Hj^T + R
        # 4. Estimate y = z - h(x')
        # 5. Normalize phi so that it is between -PI and +PI
        # 6. Calculate new estimates
        #    x = x' + K * y
        #    P = (I - K * H_j) * P

        px, py, vx, vy = self.x

        H_j = Jacobian(self.x)
        S = np.dot(np.dot(H_j, self.P), H_j.T) + self.R
        K = np.dot(np.dot(self.P, H_j.T), np.linalg.inv(S))

        c1 = px * px + py * py
        c2 = sqrt(c1)
        c3 = atan2(py, px)
        c4 = (px*vx+py*vy)/c2

        y = z - [c2, c3, c4]

        if y[1] > math.pi:
            y[1] = y[1] - 2*math.pi
        elif y[1] < -math.pi:
            y[1] = y[1] + 2*math.pi

        self.x = self.x + np.dot(K, y)

        self.P = self.P - np.dot((np.dot(K, H_j)), self.P)

        print("x = ")
        print(self.x)
        print("P = ")
        print(self.P)
        print("R = ")
        print(self.R)
        print("S = ")
        print(S)
