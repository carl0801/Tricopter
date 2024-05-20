import pybullet as p
import numpy as np
import time
import pybullet_data
import subprocess

# Connect to PyBullet physics server
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Define simulation parameters
time_step = 0.01   # Simulation time step in seconds
gravity = -9.81  # Gravity in m/s^2

# Set up PyBullet simulation environment
p.setGravity(0, 0, gravity)

# Load URDF file for the drone
drone_id = p.loadURDF('simulation\drone.urdf', [0, 0, 0], p.getQuaternionFromEuler([0, 0, -np.pi]))

# Get the joint indices for the motors and wings
num_joints = p.getNumJoints(drone_id)
motor_joints = []
wing_joints = []
for jointIndex in range(num_joints):
    joint_info = p.getJointInfo(drone_id, jointIndex)
    joint_name = joint_info[1].decode("utf-8")  # Convert bytes to string

    if "motor" in joint_name:
        motor_joints.append(jointIndex)
    elif "wing" in joint_name:  
        wing_joints.append(jointIndex)

print("Motor joints:", motor_joints)
print("Wing joint index:", wing_joints)

# create data array
data = []

# Run c++
# Start the C++ program as a subprocess
cpp_process = subprocess.Popen(["./cpp/main.exe"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

# Send a message to the C++ program
def send(msg):
    cpp_process.stdin.write(msg)
    cpp_process.stdin.flush()

# Read the response from the C++ program
def read():
    response_from_cpp = cpp_process.stdout.readline()
    #print(response_from_cpp)
    return response_from_cpp.strip()


# Make the loop as a try, if the user stops the simulation, the data will be saved
# Run simulation continuously
try:
    pt = [0,0,0]
    tilt_angle = 180
    while True:
        start = time.time()
        # Extract position and orientation
        pos, orn = p.getBasePositionAndOrientation(drone_id)
        # rotation matrix
        R = p.getMatrixFromQuaternion(orn)
        # Velocity and angular velocity
        vel, ang_vel = p.getBaseVelocity(drone_id)

        # Send the position and orientation to the C++ program
        msg = f"{pos[0]},{pos[1]},{pos[2]},{vel[0]},{vel[1]},{vel[2]},{ang_vel[0]},{ang_vel[1]},{ang_vel[2]},{R[0]},{R[1]},{R[2]},{R[3]},{R[4]},{R[5]},{R[6]},{R[7]},{R[8]},{pt[0]},{pt[1]},{pt[2]}\n"
        send(msg)
        response = read()
        # Convert each string element to a float
        motor_forces = [float(x) for x in response.split(',')]

        # Apply external force to each motor
        for jointIndex, force in zip(motor_joints, motor_forces):
            p.applyExternalForce(drone_id, jointIndex, [0, 0, force], [0, 0, 0], p.LINK_FRAME)

        # Set tilt angle for the wing
        tilt_angle +=10
        for jointIndex in wing_joints:
            p.resetJointState(drone_id, jointIndex, targetValue=tilt_angle)

        p.resetDebugVisualizerCamera(1.0, 45, -30, pos)

        p.stepSimulation()
        execution_time = time.time() - start
        time.sleep(max(0, time_step - execution_time))
except KeyboardInterrupt:
    print("Simulation stopped by the user")
    p.disconnect()
    # Close the subprocess
    cpp_process.stdin.close()
    cpp_process.stdout.close()
    cpp_process.stderr.close()
    cpp_process.wait()
