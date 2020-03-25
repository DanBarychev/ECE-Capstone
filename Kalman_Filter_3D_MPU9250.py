import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

x = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T
#x = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T

P = 100.0*np.eye(9)
#P = 10.0*np.eye(6)

dt = 0.5 # Time Step between Filter Steps
A = np.matrix([[1.0, 0.0, 0.0, dt, 0.0, 0.0, 1/2.0*dt**2, 0.0, 0.0],
              [0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0, 1/2.0*dt**2, 0.0],
              [0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0, 1/2.0*dt**2],
              [0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

#dt = 0.5
# A = np.matrix([[1.0, 0.0, dt, 0.0, 1/2.0*dt**2, 0.0],
#               [0.0, 1.0, 0.0, dt, 0.0, 1/2.0*dt**2],
#               [0.0, 0.0, 1.0, 0.0, dt, 0.0],
#               [0.0, 0.0, 0.0, 1.0, 0.0, dt],
#               [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
#               [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

# Measure x'', y'', z''
H = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

# H = np.matrix([[0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
#               [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

ra = 10.0**2
R = np.matrix([[ra, 0.0, 0.0],
               [0.0, ra, 0.0],
               [0.0, 0.0, ra]])

# R = np.matrix([[ra, 0.0],
#                [0.0, ra]])

sa = 0.1
G = np.matrix([[1/2.0*dt**2],
               [1/2.0*dt**2],
               [1/2.0*dt**2],
               [dt],
               [dt],
               [dt],
               [1.0],
               [1.0],
               [1.0]])

# G = np.matrix([[1/2.0*dt**2],
#                [1/2.0*dt**2],
#                [dt],
#                [dt],
#                [1.0],
#                [1.0]])

Q = G*G.T*sa**2

I = np.eye(9)

file = open('IMUCapture_NoGrav1.txt', 'r') # IMU data file
Lines = file.readlines()

m = len(Lines)

# Acceleration
sa= 0.1 # Sigma for acceleration
ax= 0.0 # in X
ay= 0.0 # in Y
az= 0.0 # in Z

ax_values = np.array([])
ay_values = np.array([])
az_values = np.array([])

for line in Lines:
    data = line.split(",")
    accel_x = float(data[0])
    accel_y = float(data[1])
    accel_z = float(data[2])

    ax_values = np.append(ax_values, accel_x)
    ay_values = np.append(ay_values, accel_y)
    az_values = np.append(az_values, accel_z)

mx = np.array(ax+sa*ax_values)
my = np.array(ay+sa*ay_values)
mz = np.array(az+sa*az_values)

measurements = np.vstack((mx,my,mz))


# Preallocation for Plotting
xt = []
yt = []
zt = []
dxt = []
dyt = []
dzt = []
ddxt = []
ddyt = []
ddzt = []
Zx = []
Zy = []
Zz = []
Px = []
Py = []
Pz = []
Pdx = []
Pdy = []
Pdz = []
Pddx = []
Pddy = []
Pddz = []
Kx = []
Ky = []
Kz = []
Kdx = []
Kdy = []
Kdz = []
Kddx = []
Kddy = []
Kddz = []

for n in range(m):      
    # Time Update (Prediction)
    # ========================
    # Project the state ahead
    x = A*x
    
    # Project the error covariance ahead
    P = A*P*A.T + Q    
    
    
    # Measurement Update (Correction)
    # ===============================
    # Compute the Kalman Gain
    S = H*P*H.T + R
    K = (P*H.T) * np.linalg.pinv(S)

  	# Update the estimate via z
    Z = measurements[:,n].reshape(H.shape[0],1)
    y = Z - (H*x)                            # Innovation or Residual
    x = x + (K*y)
    
    # Update the error covariance
    P = (I - (K*H))*P

   
    
    # Save states for Plotting
    # xt.append(float(x[0]))
    # yt.append(float(x[1]))
    # dxt.append(float(x[2]))
    # dyt.append(float(x[3]))
    # ddxt.append(float(x[4]))
    # ddyt.append(float(x[5]))
    # Zx.append(float(Z[0]))
    # Zy.append(float(Z[1]))
    # Px.append(float(P[0,0]))
    # Py.append(float(P[1,1]))
    # Pdx.append(float(P[2,2]))
    # Pdy.append(float(P[3,3]))
    # Pddx.append(float(P[4,4]))
    # Pddy.append(float(P[5,5]))
    # Kx.append(float(K[0,0]))
    # Ky.append(float(K[1,0]))
    # Kdx.append(float(K[2,0]))
    # Kdy.append(float(K[3,0]))
    # Kddx.append(float(K[4,0]))
    # Kddy.append(float(K[5,0]))

    xt.append(float(x[0]))
    yt.append(float(x[1]))
    zt.append(float(x[2]))
    dxt.append(float(x[3]))
    dyt.append(float(x[4]))
    dzt.append(float(x[5]))
    ddxt.append(float(x[6]))
    ddyt.append(float(x[7]))
    ddzt.append(float(x[8]))
    Zx.append(float(Z[0]))
    Zy.append(float(Z[1]))
    Zz.append(float(Z[2]))
    Px.append(float(P[0,0]))
    Py.append(float(P[1,1]))
    Pz.append(float(P[2,2]))
    Pdx.append(float(P[3,3]))
    Pdy.append(float(P[4,4]))
    Pdz.append(float(P[5,5]))
    Pddx.append(float(P[6,6]))
    Pddy.append(float(P[7,7]))
    Pddz.append(float(P[8,8]))
    Kx.append(float(K[0,0]))
    Ky.append(float(K[1,0]))
    Kz.append(float(K[2,0]))
    Kdx.append(float(K[3,0]))
    Kdy.append(float(K[4,0]))
    Kdz.append(float(K[5,0]))
    Kddx.append(float(K[6,0]))
    Kddy.append(float(K[7,0]))
    Kddz.append(float(K[7,0]))



fig = plt.figure(figsize=(16,4))
#plt.plot(range(len(measurements[0])),Px, label='$x$')
#plt.plot(range(len(measurements[0])),Py, label='$y$')
plt.plot(range(len(measurements[0])),Pddx, label='$\ddot x$')
plt.plot(range(len(measurements[0])),Pddy, label='$\ddot y$')

plt.xlabel('Filter Step')
plt.ylabel('')
plt.title('Uncertainty (Elements from Matrix $P$)')
plt.legend(loc='best',prop={'size':22})

plt.show()

