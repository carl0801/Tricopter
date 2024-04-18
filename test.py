import pybullet as p
import time
import pybullet_data
import numpy as np

# Connect to PyBullet physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally
p.setGravity(0, 0, -9.82)

# Load the URDF file
planeId = p.loadURDF("plane.urdf")

# Load the drone URDF file
droneId = p.loadURDF("Simulation\drone.urdf", [0, 0, 1], p.getQuaternionFromEuler([0, 0, 0]))

# Get the joint indices for the motors and wings
num_joints = p.getNumJoints(droneId)
motor_joints = []
wing_joint_index = -1
for jointIndex in range(num_joints):
    joint_info = p.getJointInfo(droneId, jointIndex)
    joint_name = joint_info[1].decode("utf-8")  # Convert bytes to string
    print(joint_name)  # Print joint names for inspection
    if "motor" in joint_name:
        motor_joints.append(jointIndex)
    elif "Revolute_5" in joint_name:
        wing_joint_index = jointIndex

print("Motor joints:", motor_joints)
print("Wing joint index:", wing_joint_index)

# Start with zero thrust
thrust = 0.07

# Define torques for each motor (assuming all motors rotate in the same direction)
torque_direction = 1  # 1 for clockwise, -1 for counterclockwise
torques = [thrust * torque_direction, thrust * torque_direction, thrust * torque_direction]

# Define wing tilt angle
tilt_angle = np.radians(-10.0)  # Initial angle in radians

# Simulation loop
for _ in range(10000):
    # Increase thrust gradually
    thrust += 0.0001
    
    # Calculate net torque
    net_torque = np.sum(torques)
    
    # Calculate required tilt angle adjustment based on net torque
    tilt_angle_adjustment = net_torque * 0.0001  # Adjust this factor as needed
    
    # Update wing tilt angle
    tilt_angle += tilt_angle_adjustment
    
    # Ensure the wing tilt angle stays within a valid range
    tilt_angle = np.clip(tilt_angle, np.radians(-180.0), np.radians(180.0))  # Adjust range as needed
    
    # Set the motor torques (compensating for torques)
    for jointIndex, torque in zip(motor_joints, torques):
        # Apply torque to each motor
        p.setJointMotorControl2(droneId, jointIndex, p.TORQUE_CONTROL, force=torque)
    
    # Set tilt angle for the wing
    p.setJointMotorControl2(droneId, wing_joint_index, p.POSITION_CONTROL, targetPosition=tilt_angle)

    # Step the simulation
    p.stepSimulation()

    # reset view to follow drone
    position, orientation = p.getBasePositionAndOrientation(droneId)
    p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=position)

    # Slow down the simulation to make it visible
    time.sleep(1./240.)

# Disconnect from the physics server
p.disconnect()
