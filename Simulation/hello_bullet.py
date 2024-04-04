import pybullet as p
import time
import pybullet_data

# Connect to PyBullet physics server
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Optionally
p.setGravity(0, 0, -9.82)

# Load the URDF file
planeId = p.loadURDF("plane.urdf")

# Load the drone URDF file
droneId = p.loadURDF("Simulation/drone.urdf", [0, 0, 1], p.getQuaternionFromEuler([0, 0, 0]))

# Get the joint indices for the motors
num_joints = p.getNumJoints(droneId)
motor_joints = []
for jointIndex in range(num_joints):
    joint_info = p.getJointInfo(droneId, jointIndex)
    joint_name = joint_info[1].decode("utf-8")  # Convert bytes to string
    print(joint_name)  # Print joint names for inspection
    if "motor" in joint_name:
        motor_joints.append(jointIndex)

print("Motor joints:", motor_joints)

# Start with zero thrust
thrust = 5.0

# Simulation loop
for _ in range(10000):
    # Increase thrust gradually
    thrust += 0.1
    
    # Apply the thrust force to the drone body
    for jointIndex in motor_joints:
        p.applyExternalForce(objectUniqueId=droneId, linkIndex=jointIndex, forceObj=[0, 0, thrust], posObj=[0, 0, 0], flags=p.LINK_FRAME)


    # Step the simulation
    p.stepSimulation()

    # Slow down the simulation to make it visible
    time.sleep(1./240.)

# Disconnect from the physics server
p.disconnect()
