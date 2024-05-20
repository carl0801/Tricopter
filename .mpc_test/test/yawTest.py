import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

# Load data
# [rotation[0], rotation[1], rotation[2], rotation[3], translation[0], translation[1], translation[2], qw, qx, qy, qz, timecode]
data = np.genfromtxt('test/yawTest4.csv', delimiter=',', skip_header=1)
time = data[:, -1]
qVicon = data[:, 0:4]
qVicon = np.array([qVicon[:, 1], qVicon[:, 2], qVicon[:, 3], qVicon[:, 0]]).T
qDrone = data[:, 7:11]

# Quaternion to euler
rV = R.from_quat(qVicon)
rD = R.from_quat(qDrone)

# Get euler angles
eulerV = rV.as_euler('zyx', degrees=True)
eulerD = rD.as_euler('zyx', degrees=True)

# Compute offset
eulerOffset = R.from_euler('zyx', eulerV - eulerD, degrees=True)
offset = eulerOffset.as_euler('zyx', degrees=True)
# Compute mean yaw offset
yawOffset = np.mean(offset[:, 2])
eulerV[:, 2] = eulerV[:, 2] - yawOffset
# Normalize yaw between -60 and 300
eulerV[:, 2] = np.mod(eulerV[:, 2] + 60, 360) - 60
eulerD[:, 2] = np.mod(eulerD[:, 2] + 60, 360) - 60

# Normalize time
time = time - time[0]

# Compute mean sampling time
samplingTime = np.mean(np.diff(time))

print(f"Mean yaw offset: {yawOffset:.2f} deg")
print(f"Mean sampling time: {samplingTime:.6f} s")

# Plot
plt.figure(figsize=(10, 5))
plt.plot(time, eulerV[:, 2], label='Vicon')
plt.plot(time, eulerD[:, 2], label='IMU')
plt.xlabel('Time [s]')
plt.ylabel('Yaw [deg]')
plt.grid()
plt.legend()
plt.savefig('test/yawTest4.png')
plt.show()
