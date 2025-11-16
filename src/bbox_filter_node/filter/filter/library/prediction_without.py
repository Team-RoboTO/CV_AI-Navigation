import numpy as np
from scipy.optimize import minimize
from geometry_msgs.msg import PoseStamped

class PredictionWithout():
    def __init__(self, v_p, lag=0.02, refs=5):
        self.v_p = v_p
        self.lag_time = lag
        self.num_refs = refs
        self.g = 9.81
        self.x_prev = self.y_prev = self.z_prev = None
        self.ts_prev = None
        self.wth_avg_t = None
        self.wth_avg_p = None

        self.vax = self.vay = self.vaz = 0.0

        # Weights for the average of the angles
        self.nanmeandim = 5
        self.weig_avg : np.array = np.array([[np.nan for _ in range(self.nanmeandim)], [np.nan for _ in range(self.nanmeandim)], [np.nan for _ in range(self.nanmeandim)]])
        self.weig_ind = 0
        self.weig_avg_t : np.array = np.array([np.nan for _ in range(5)])
        self.weig_avg_p : np.array = np.array([np.nan for _ in range(5)])
        self.weig_ind_angle = 0
        
    

    # Function to compute the position of the target
    def target_linear_position(self, x0, y0, z0, vx, vy, vz, t):
        xt = x0 + vx * t
        yt = y0 + vy * t
        zt = z0 + vz * t
        return np.array([xt, yt, zt])

    # Function to compute the position of the projectile
    def projectile_position_without(self, theta, psi, t, theta_0=None):
        vpx = self.v_p * np.cos(theta) * np.cos(psi)
        vpy = self.v_p * np.cos(theta) * np.sin(psi)
        vpz = self.v_p * np.sin(theta)
        
        x = vpx * t
        y = vpy * t
        z = vpz * t - 0.5 * self.g * t**2
        position = np.array([x, y, z])

        # Just for the visualization of the projectile in the camera frame
        # In the optimization process, the rotation is computed before
        if theta_0 is not None:
            position[0] = position[0] * np.cos(theta_0) + position[2] * np.sin(theta_0)
            position[2] = - position[0] * np.sin(theta_0) + position[2] * np.cos(theta_0)
        
        return position

    # Objective Function to minimize (distance between projectile and target at t_hit)
    def objective_function_without(self, x0, y0, z0, vx, vy, vz, lag_time, params):
        theta, psi, t_hit = params
        # Ensure t_hit is positive
        t_hit = np.abs(t_hit)        
        
        projectile_pos = self.projectile_position_without(theta, psi, t_hit)
        target_pos = self.target_linear_position(x0, y0, z0, vx, vy, vz, t_hit + lag_time)
        # Compute the distance between the projectile and the target at t_hit
        distance = np.linalg.norm(projectile_pos - target_pos)
        return distance

    def prediction_without(self, x0, y0, z0, wz, ts):

        # FIRST ITERATION - NONE VALUES
        if self.ts_prev is None:
            self.ts_prev = ts
            self.x_prev = x0
            self.y_prev = y0
            self.z_prev = z0
            return (0, 0, 0, 0)
        
        ## FOLLOWING ITERATIONS
        # Time difference between the current and velocities
        dt = ts - self.ts_prev
        if dt == 0: 
            return (0, 0, 0, 0)
        

        vx = (x0 - self.x_prev) * 1e9 / dt #- self.vax
        vy = (y0 - self.y_prev) * 1e9 / dt #- self.vay
        vz = (z0 - self.z_prev) * 1e9 / dt #+ self.vaz

        d = np.sqrt(x0**2 + y0**2)
        
        yaw_pos = np.arctan2( - y0/d, x0/d)
        
        vy_final = vy - wz * d * np.cos(yaw_pos)

        # update self variables
        self.x_prev, self.y_prev, self.z_prev, self.ts_prev = x0, y0, z0, ts

        # Velocity normalization
        v_abs = np.sqrt(vx**2 + vy**2 + vz**2)
        if v_abs > 5: 
            div_factor = v_abs / 5
            vx /= div_factor
            vy /= div_factor
            vz /= div_factor

        # Update the average of the velocities and compute the mean
        self.weig_avg[0, self.weig_ind] = vx
        self.weig_avg[1, self.weig_ind] = vy_final
        self.weig_avg[2, self.weig_ind] = vz
        
        self.weig_ind = (self.weig_ind + 1) % self.nanmeandim

        mu = np.nanmean(self.weig_avg, axis=1)
        vx = mu[0]
        vy = mu[1]
        vz = mu[2]

        # Initial guess for theta, psi, and t_hit
        init_t = np.sqrt(x0**2 + y0**2 + z0**2) / self.v_p
        x_fut = x0 + vx * (init_t + self.lag_time)
        y_fut = y0 + vy * (init_t + self.lag_time)
        z_fut = z0 + vz * (init_t + self.lag_time)
        init_theta = np.arctan2(z_fut, np.sqrt(x_fut**2 + y_fut**2))
        init_psi = np.arctan2(y_fut, x_fut)
        initial_guess = [init_theta, init_psi, init_t]

        # Optimization bounds to ensure realistic values
        bounds = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2), (0, None)]  # Angle bounds and positive hit time

        # Use 'SLSQP' optimization for bounded problem
        objective = lambda params: self.objective_function_without(x0, y0, z0, vx, vy, vz, self.lag_time, params)
        result = minimize(objective, initial_guess, method='SLSQP', bounds=bounds)

        # VALUES
        theta_opt, psi_opt, t_hit_opt = result.x

        # Update the average of the angles and compute the mean
        self.weig_avg_t[self.weig_ind_angle] = theta_opt
        self.weig_avg_p[self.weig_ind_angle] = psi_opt
        self.weig_ind_angle = (self.weig_ind_angle + 1) % 5
        
        self.wth_avg_t = np.nanmean(self.weig_avg_t)
        self.wth_avg_p = np.nanmean(self.weig_avg_p)

        # Compute the reference angles for theta and psi during the lag
        # theta_ref = np.linspace(0, self.wth_avg_t, self.num_refs)
        # psi_ref = np.linspace(0, self.wth_avg_p, self.num_refs)

        # Outputs
        return (self.wth_avg_t, self.wth_avg_p, t_hit_opt, result)