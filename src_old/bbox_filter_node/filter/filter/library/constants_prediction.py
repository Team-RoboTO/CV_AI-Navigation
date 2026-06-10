import numpy as np

g = 9.81  # gravitational acceleration (m/s^2)
m = 3.2 * 1e-3   # mass of the projectile (kg)
h = 0.5 # initial height (m)
r = 17 * 1e-3  # radius of the projectile (m)
mu = 1.85 * 1e-5 # Dynamic viscosity of air (kg/m/s)
k_s = 6 * np.pi * r * mu # Stokes' drag coefficient