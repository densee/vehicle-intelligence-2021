# Week 3 - Kalman Filters, EKF and Sensor Fusion

---

[//]: # (Image References)
[EKF-results]: ./EKF/EKF_ysj.png

## EKF-results
The contents of the function "def update_ekf" are as follows:

    def update_ekf(self, z):
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

The result of running this program with test input data is illustrated below:

![Testing of Kalman Filter Example][EKF-results]

