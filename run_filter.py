import numpy as np
import matplotlib.pyplot as plt
from madgwickahrs import MadgwickAHRS
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
from scipy.stats import norm
from scipy import signal
import math

dt = 1/50 # Time Step between Filter Steps

file = open('IMUCapture_Run7.txt', 'r') # IMU data file

Lines = file.readlines()

m = len(Lines)

throw_states = np.array([]) 
time_stamps = np.array([])

ax_values_h = np.array([]) # h for hand IMU
ay_values_h = np.array([])
az_values_h = np.array([])

gx_values_h = np.array([])
gy_values_h = np.array([])
gz_values_h = np.array([])

mx_values_h = np.array([])
my_values_h = np.array([])
mz_values_h = np.array([])

for line in Lines:
    data = line.split(",")

    throw_state = int(data[0])

    time_stamp = float(data[1])

    accel_x_h = float(data[2])  
    accel_y_h = float(data[3])
    accel_z_h = float(data[4])

    gyro_x_h = float(data[5])
    gyro_y_h = float(data[6])
    gyro_z_h = float(data[7])

    if (len(data) > 8):
        mag_x_h = float(data[8])
        mag_y_h = float(data[9])
        mag_z_h = float(data[10])
    
    throw_states = np.append(throw_states, throw_state)
    time_stamps = np.append(time_stamps, time_stamp)
    
    ax_values_h = np.append(ax_values_h, accel_x_h)
    ay_values_h = np.append(ay_values_h, accel_y_h)
    az_values_h = np.append(az_values_h, accel_z_h)

    gx_values_h = np.append(gx_values_h, gyro_x_h)
    gy_values_h = np.append(gy_values_h, gyro_y_h)
    gz_values_h = np.append(gz_values_h, gyro_z_h)

    if (len(data) > 8):
        mx_values_h = np.append(mx_values_h, mag_x_h)
        my_values_h = np.append(my_values_h, mag_y_h)
        mz_values_h = np.append(mz_values_h, mag_z_h)

try:
    throw_ind = (np.where(throw_states == 1))[0][0]
except:
  print("No throw detected")


a_values = np.vstack((ax_values_h, ay_values_h, az_values_h))
g_values = np.vstack((gx_values_h, gy_values_h, gz_values_h))
m_values = np.vstack((mx_values_h, my_values_h, mz_values_h))

# Use the Madgwick AHRS filter

ahrs = MadgwickAHRS()
R = np.zeros((m,3,3))
Z_flip = [[1,0,0], 
          [0,1,0],
          [0,0,-1]]

for i in range(m):
    ahrs.update_imu(g_values[:, i], a_values[:, i])
    #ahrs.update(g_values[:, i], a_values[:, i], m_values[:, i])
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
filtCutOff = .3;
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
filtCutOff = .3
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


#R_throw_mean = (R[throw_ind] + R[throw_ind+1] + R[throw_ind+2]) / 3 
R_throw_mean = R[throw_ind]

v_x = sum(dxt[throw_ind:(throw_ind + 3)]) / 3
v_y = sum(dyt[throw_ind:(throw_ind + 3)]) / 3
v_z = sum(dzt[throw_ind:(throw_ind + 3)]) / 3

theta_x = math.atan2(R_throw_mean[2][1], R_throw_mean[2][2])
theta_y = math.atan2(-R_throw_mean[2][0], math.sqrt((R_throw_mean[2][1])**2 + R_throw_mean[2][2]**2))
theta_z = math.atan2(R_throw_mean[1][0], R_throw_mean[0][0])

print(v_x)
print(v_y)
print(-1 * v_z)

print(180 - ((theta_x / math.pi) * 180))
print((theta_y / math.pi) * 180)
print((theta_z / math.pi) * 180)


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


# 3D Position Plot

fig = plt.figure(figsize=(16,9))
ax = fig.add_subplot(111, projection='3d')
ax.plot(xt,yt,-1 * zt, label='Position Estimate')

ax.plot(xt[(throw_ind):(throw_ind+3)],yt[(throw_ind):(throw_ind+3)],
    -1 * zt[(throw_ind):(throw_ind+3)], color='red')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()