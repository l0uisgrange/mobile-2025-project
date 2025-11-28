# -----------------------------------------------------------
# EKF CLASS
# -----------------------------------------------------------
import numpy as np


"""
-----------------------------------
# Create EKF with initial pose (x0, y0, theta0)
ekf = ThymioEKF(init_pose=[0.0, 0.0, 0.0])


inputs:
#   vL        → measured left wheel speed
#   vR        → measured right wheel speed
#   cam_pose  → (if available) [x_cam, y_cam, theta_cam]
#

# If camera data is available:
state, covariance = ekf.step(vL, vR, cam_pose=[x_cam, y_cam, theta_cam])

# If NO camera:
state, covariance = ekf.step(vL, vR)

returns:

state (np.array shape (5,)):
    state[0] = x position (mm)
    state[1] = y position (mm)
    state[2] = heading theta (rad)
    state[3] = left wheel speed vL
    state[4] = right wheel speed vR

covariance (np.array 5x5):
    Full covariance matrix P describing the
    uncertainty of the state estimate.

"""


class ThymioEKF:
    def __init__(self, init_pose, L=95.0, dt=0.1):
        """
        Initializes the EKF with:
        - initial pose [x, y, theta]
        - wheel separation L (change)
        - time step dt (change maybe)
        - state vector x = [x, y, theta, vL, vR]
        - covariance matrix P
        - process noise Q (uncertainty of motion model)
        - measurement noise R_vision (camera noise)
        """
        self.dt = dt
        self.L = L

        #assumed to be at given position and with velocities 0
        self.x = np.array([init_pose[0], init_pose[1], init_pose[2], 0.0, 0.0])
        
        # P is the covariance matrix (initial uncertainty) covariance matrix
        self.P = np.eye(5) * 1.0

        # Process noise (how uncertain the motion is) noise encoder
        self.Q = np.diag([1e-2, 1e-2, 1e-3, 3.0, 3.0])

        # Measurement noise (camera uncertainty) noise camera
        self.R_vision = np.diag([5.0, 5.0, 0.05])

        self.w = []

    # ---------------------------------------------
    # PREDICTION (wheel encoders only)
    # ---------------------------------------------
    def predict(self, vL, vR):
        """
        Predicts the next state using only encoders

        Steps:
        1. Update wheel speeds inside the state vector.
        2. Compute linear and angular velocity.
        3. Apply nonlinear motion model to get new (x,y,theta).
        4. Compute Jacobian F of the motion model.
        5. Update covariance P to reflect increased uncertainty.
        """
        x, y, th, _, _ = self.x
        dt = self.dt

        # update wheel speeds in the state
        self.x[3] = vL
        self.x[4] = vR

        v = (vL + vR) / 2          # linear velocity
        w = (vR - vL) / self.L     # angular velocity

        # Nonlinear motion model (predict new pose)
        self.x[0] += v * np.cos(th) * dt
        self.x[1] += v * np.sin(th) * dt
        self.x[2] += w * dt

        if len(self.w) == 0:
            self.w = [self.x[0], self.x[1], self.x[2]]
        else:
            self.w[0] = self.x[0]
            self.w[1] = self.x[1]
            self.w[2] = self.x[2]

        # Jacobian F (linearization of f(x))
        F = np.eye(5)
        F[0,2] = -v * np.sin(th) * dt
        F[1,2] =  v * np.cos(th) * dt
        F[0,3] = 0.5 * np.cos(th) * dt
        F[0,4] = 0.5 * np.cos(th) * dt
        F[1,3] = 0.5 * np.sin(th) * dt
        F[1,4] = 0.5 * np.sin(th) * dt
        F[2,3] = -dt / self.L
        F[2,4] =  dt / self.L

        # Covariance prediction: P_(t+1) = (F * P_(t) * F^T) + Q
        self.P = F @ self.P @ F.T + self.Q

    # ---------------------------------------------
    # UPDATE (vision only)
    # ---------------------------------------------
    def update_vision(self, cam_pose):
        """
        Corrects the predicted state using the camera measurement.

        Steps:
        1. Build measurement vector z = [x_cam, y_cam, theta_cam].
        2. Build measurement model h(x) = [x, y, theta].
        3. Build measurement Jacobian H.
        4. Compute innovation y = z - h(x).
        5. Compute innovation covariance S.
        6. Compute Kalman gain K.
        7. Update state and covariance.
        """
        z = np.array([cam_pose[0], cam_pose[1], cam_pose[2]])

        # h(x) = expected measurement from predicted state
        h = self.x[:3]

        # Jacobian H (we only measure x, y, theta)
        H = np.zeros((3,5))
        H[0,0] = 1
        H[1,1] = 1
        H[2,2] = 1

        # Innovation (measurement error)
        y = z - h

        # Innovation covariance
        S = H @ self.P @ H.T + self.R_vision

        # Kalman gain (sensor vs odometry confidence)
        K = self.P @ H.T @ np.linalg.inv(S)

        # Posterior update (state correction)
        self.x = self.x + K @ y

        # Covariance update (uncertainty reduction)
        self.P = (np.eye(5) - K @ H) @ self.P

    # ---------------------------------------------
    # Main step
    # ---------------------------------------------
    def step(self, vL, vR, cam_pose=None):
        """
        Performs one full EKF iteration:
        1. Prediction with odometry (always).
        2. Correction with camera (only if available).

        Returns:
            - updated state estimate
            - updated covariance
        """
        self.predict(vL, vR)

        if cam_pose is not None:
            self.update_vision(cam_pose)

        return self.x.copy(), self.P.copy()
