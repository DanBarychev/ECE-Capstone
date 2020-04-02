import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

x = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T

P = 100*np.eye(9)

dt = 0.02 # Time Step between Filter Steps
A = np.matrix([[1.0, 0.0, 0.0, dt, 0.0, 0.0, 1/2.0*dt**2, 0.0, 0.0],
              [0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0, 1/2.0*dt**2, 0.0],
              [0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0, 1/2.0*dt**2],
              [0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  dt],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

# Measure x'', y'', z''
H = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

ra = 10**2
R = np.matrix([[ra, 0.0, 0.0],
               [0.0, ra, 0.0],
               [0.0, 0.0, ra]])

sa = .1
G = np.matrix([[1/2.0*dt**2],
               [1/2.0*dt**2],
               [1/2.0*dt**2],
               [dt],
               [dt],
               [dt],
               [1.0],
               [1.0],
               [1.0]])

Q = G*G.T*sa**2

I = np.eye(9)

file = open('IMUCapture_Circle1.txt', 'r') # IMU data file
Lines = file.readlines()

m = len(Lines)

# Acceleration
sa= .1 # Sigma for acceleration
ax= 0.0 # in X
ay= 0.0 # in Y
az= 0.0 # in Z

ax_values = np.array([])
ay_values = np.array([])
az_values = np.array([])

ax_calib_sum = 0
ay_calib_sum = 0
az_calib_sum = 0

#The first 300 samples is the first 6 seconds under 50Hz
for a in range(300):
    line = Lines[a]
    data = line.split(",")
    accel_x = float(data[0])
    accel_y = float(data[1])
    accel_z = float(data[2])

    ax_calib_sum += accel_x
    ay_calib_sum += accel_y
    az_calib_sum += accel_z

ax_calib = ax_calib_sum / 300
ay_calib = ay_calib_sum / 300
az_calib = az_calib_sum / 300

print("Average Estimates")
print(ax_calib)
print(ay_calib)
print(az_calib)

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

for line in Lines:
    data = line.split(",")
    accel_x = float(data[0])  
    accel_x = accel_x - ax_calib

    accel_y = float(data[1])
    accel_y = accel_y - ay_calib
    
    accel_z = float(data[2])
    accel_z = accel_z - az_calib

    accel_z = 0;

    ax_values = np.append(ax_values, accel_x)
    ay_values = np.append(ay_values, accel_y)
    az_values = np.append(az_values, accel_z)

mx = np.array(ax+sa*ax_values)
my = np.array(ay+sa*ay_values)
mz = np.array(az+sa*az_values)

measurements = np.vstack((mx,my,mz))

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

    # if (n == 1000):
    #   print("\nData at 1000\n")
    #   print(x[6])
    #   print(x[7])
    #   print(x[8])

    # if (n == 1500):
    #   print("\nData at 1500\n")
    #   print(x[6])
    #   print(x[7])
    #   print(x[8])

    # if (n == 2000):
    #   print("\nData at 2000\n")
    #   print(x[6])
    #   print(x[7])
    #   print(x[8])
   
    
    # Save states for Plotting
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
    Kddz.append(float(K[8,0]))



# fig = plt.figure(figsize=(16,4))
# #plt.plot(range(len(measurements[0])),Px, label='$x$')
# #plt.plot(range(len(measurements[0])),Py, label='$y$')
# plt.plot(range(len(measurements[0])),Pddx, label='$\ddot x$')
# plt.plot(range(len(measurements[0])),Pddy, label='$\ddot y$')
# plt.plot(range(len(measurements[0])),Pddz, label='$\ddot z$')

# plt.xlabel('Filter Step')
# plt.ylabel('')
# plt.title('Uncertainty (Elements from Matrix $P$)')
# plt.legend(loc='best',prop={'size':22})

# plt.show()

fig = plt.figure(figsize=(16,9))

plt.subplot(311)
plt.step(range(len(measurements[0])),ddxt, label='$\ddot x$')
plt.step(range(len(measurements[0])),ddyt, label='$\ddot y$')
plt.step(range(len(measurements[0])),ddzt, label='$\ddot z$')

plt.title('Estimate (Elements from State Vector $x$)')
plt.legend(loc='best',prop={'size':22})
plt.ylabel('Acceleration')
plt.ylim([-1,1])

plt.subplot(312)
plt.step(range(len(measurements[0])),dxt, label='$\dot x$')
plt.step(range(len(measurements[0])),dyt, label='$\dot y$')
plt.step(range(len(measurements[0])),dzt, label='$\dot z$')

plt.ylabel('')
plt.legend(loc='best',prop={'size':22})
plt.ylabel('Velocity')
           
plt.subplot(313)
plt.step(range(len(measurements[0])),xt, label='$x$')
plt.step(range(len(measurements[0])),yt, label='$y$')
plt.step(range(len(measurements[0])),zt, label='$z$')

plt.xlabel('Filter Step')
plt.ylabel('')
plt.legend(loc='best',prop={'size':22})
plt.ylabel('Position')

plt.show()

fig = plt.figure(figsize=(16,16))
plt.scatter(xt[0],yt[0], s=100, label='Start', c='g')
plt.scatter(xt[-1],yt[-1], s=100, label='Goal', c='r')
plt.plot(xt,yt, label='State',alpha=0.5)

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Position')
plt.legend(loc='best')
plt.xlim([-100, 100])
plt.ylim([-100, 100])

plt.show()

