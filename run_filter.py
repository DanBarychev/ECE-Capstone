import numpy as np
import matplotlib.pyplot as plt
from madgwickahrs import MadgwickAHRS
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
from scipy.stats import norm
from scipy import signal

dt = 1/50 # Time Step between Filter Steps

file = open('IMUCapture_ThrowNormal.txt', 'r') # IMU data file
Lines = file.readlines()

m = len(Lines)

ax_values = np.array([])
ay_values = np.array([])
az_values = np.array([])

gx_values = np.array([])
gy_values = np.array([])
gz_values = np.array([])

ax_calib_sum = 0
ay_calib_sum = 0
az_calib_sum = 0

gx_calib_sum = 0
gy_calib_sum = 0
gz_calib_sum = 0

#The first 100 samples is the first 2 seconds under 50Hz

# for a in range(100):
#     line = Lines[a]
#     data = line.split(",")
#     accel_x = float(data[0])
#     accel_y = float(data[1])
#     accel_z = float(data[2])

#     # gyro_x = float(data[3])
#     # gyro_y = float(data[4])
#     # gyro_z = float(data[5])

#     ax_calib_sum += accel_x
#     ay_calib_sum += accel_y
#     az_calib_sum += accel_z

#     # gx_calib_sum += gyro_x
#     # gy_calib_sum += gyro_y
#     # gz_calib_sum += gyro_z


# ax_calib = ax_calib_sum / 100
# ay_calib = ay_calib_sum / 100
# az_calib = az_calib_sum / 100

# gx_calib = gx_calib_sum / 100
# gy_calib = gy_calib_sum / 100
# gz_calib = gz_calib_sum / 100

# print("Average Estimates")
# print(ax_calib)
# print(ay_calib)
# print(az_calib)


for line in Lines:
    data = line.split(",")

    throwState = int(data[0])

    timestamp = float(data[1])

    accel_x = float(data[2])  

    accel_y = float(data[3])
    
    accel_z = float(data[4])

    # !!! Flip x and y to use Pendulum1.txt

    gyro_x = float(data[5])
    gyro_y = float(data[6])
    gyro_z = float(data[7])

    # accel_x = float(data[0])  

    # accel_y = float(data[1])
    
    # accel_z = float(data[2])

    # # !!! Flip x and y to use Pendulum1.txt

    # gyro_x = float(data[3])
    # gyro_y = float(data[4])
    # gyro_z = float(data[5])
    
    ax_values = np.append(ax_values, accel_x)
    ay_values = np.append(ay_values, accel_y)
    az_values = np.append(az_values, accel_z)

    gx_values = np.append(gx_values, gyro_x)
    gy_values = np.append(gy_values, gyro_y)
    gz_values = np.append(gz_values, gyro_z)

a_values = np.vstack((ax_values, ay_values, az_values))
g_values = np.vstack((gx_values, gy_values, gz_values))

# Use the Madgwick AHRS filter

ahrs = MadgwickAHRS()
R = np.zeros((m,3,3))

for i in range(m):
	ahrs.update_imu(g_values[:, i], a_values[:, i])
	R[i,:,:] = Rotation.from_quat(ahrs.quaternion._q).as_matrix()

# Compute the tilt compensated acceleration and subtract gravity

tc_a_values = np.zeros((3,m))

for i in range(m):
	tc_a_values[:,i] = np.dot(R[i,:,:], a_values[:,i])

a_values_final = tc_a_values - np.vstack((np.zeros(m), np.zeros(m), np.ones(m) * 9.81))

# Integrate acceleration to get velocity, then high pass filter

v_values = np.zeros((3,m))

for i in range(2, len(Lines)):
	v_values[:,i] = v_values[:,i-1] + (a_values_final[:,i] * dt)

order = 1;
filtCutOff = .5;
b, a = signal.butter(order, (2*filtCutOff)/(1/dt), 'high')

vx_hp = signal.filtfilt(b, a, v_values[0,:])
vy_hp = signal.filtfilt(b, a, v_values[1,:])
vz_hp = signal.filtfilt(b, a, v_values[2,:])

# Integrate velocity to get position, then high pass filter

p_values = np.zeros((3,m))

for i in range(2, len(Lines)):
	p_values[0,i] = p_values[0,i-1] + (vx_hp[i] * dt)
	p_values[1,i] = p_values[1,i-1] + (vy_hp[i] * dt)
	p_values[2,i] = p_values[2,i-1] + (vz_hp[i] * dt)

order = 1;
filtCutOff = .5
b, a = signal.butter(order, (2*filtCutOff)/(1/dt), 'high')

px_hp = signal.filtfilt(b, a, p_values[0,:])
py_hp = signal.filtfilt(b, a, p_values[1,:])
pz_hp = signal.filtfilt(b, a, p_values[2,:])

# Allocation for Plotting
xt = px_hp
yt = py_hp
zt = pz_hp
dxt = vx_hp
dyt = vy_hp
dzt = vz_hp
ddxt = a_values_final[0,:]
ddyt = a_values_final[1,:]
ddzt = a_values_final[2,:]

#Plotting

fig = plt.figure(figsize=(16,9))

plt.subplot(311)
plt.step(range(m),ddxt, label='$\ddot x$')
plt.step(range(m),ddyt, label='$\ddot y$')
plt.step(range(m),ddzt, label='$\ddot z$')

plt.title('Estimate (Elements from State Vector $x$)')
plt.legend(loc='best',prop={'size':22})
plt.ylabel('Acceleration')
plt.ylim([-1,1])

plt.subplot(312)
plt.step(range(m),dxt, label='$\dot x$')
plt.step(range(m),dyt, label='$\dot y$')
plt.step(range(m),dzt, label='$\dot z$')

plt.ylabel('')
plt.legend(loc='best',prop={'size':22})
plt.ylabel('Velocity')
           
plt.subplot(313)
plt.step(range(m),xt, label='$x$')
plt.step(range(m),yt, label='$y$')
plt.step(range(m),zt, label='$z$')

plt.xlabel('Filter Step')
plt.ylabel('')
plt.legend(loc='best',prop={'size':22})
plt.ylabel('Position')

plt.show()

# 2D Position Plot

# fig = plt.figure(figsize=(16,16))
# plt.scatter(xt[0],yt[0], s=100, label='Start', c='g')
# plt.scatter(xt[-1],yt[-1], s=100, label='Goal', c='r')
# plt.plot(xt,yt, label='State',alpha=0.5)

# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Position')
# plt.legend(loc='best')
# plt.xlim([-100, 100])
# plt.ylim([-100, 100])

# plt.show()

# 3D Position Plot

fig = plt.figure(figsize=(16,9))
ax = fig.add_subplot(111, projection='3d')
ax.plot(xt,yt,zt, label='Position Estimate')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()

