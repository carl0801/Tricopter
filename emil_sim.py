import pybullet as p
import numpy as np
import time
import pandas as pd

# Connect to PyBullet physics server
p.connect(p.GUI)


# Load errors from csv file with layout below:
"""error,time
"[0, 0, 0, 0.0, 0.0, 0]",0.0
"[0.0, -0.00011772000000000118, 0.0, -0.0014133463200000143, 0.0, 0.0]",0.01"""
errors_pd = pd.read_csv("sim.csv")
errors = errors_pd['pos'].values
time_steps = errors_pd['time'].values

print(errors[0])
#make sure the errors are floats
errors = [np.array(eval(error)) for error in errors]
time_steps = [float(time) for time in time_steps]

# Load rotational errors from NPZ files
#rotational_errors = np.load("points_data_2024-03-15_14-12-16.npz")
#roll_pitch_errors = rotational_errors['points']

# Generate time steps with a 10 ms interval
#num_points = len(errors)
#time_steps = np.arange(num_points) * 0.01  # Assuming 10 ms between each measurement
print(f"Number of points: {len(time_steps)}")
# Define simulation parameters
time_step = 0.001   # Simulation time step in seconds
gravity = -9.81  # Gravity in m/s^2

# Set up PyBullet simulation environment
p.setGravity(0, 0, gravity)

# Initialize drone position and orientation
start_position = [0, 0, 1]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])

# Create visual shape for the drone
drone_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                         fileName="Tricopter.stl",
                                         meshScale=[0.01, 0.01, 0.01])

# Create multi-body with visual shape only
drone_id = p.createMultiBody(baseMass=1,
                              basePosition=start_position,
                              baseOrientation=start_orientation,
                              baseVisualShapeIndex=drone_visual_shape)

# Set up camera position and orientation
camera_target_position = [0, 0, 0]
camera_distance = 10
camera_yaw = 90
camera_pitch = -45#-90
camera_roll = 0
up_axis_index = 2
camera_position = [camera_distance, 0, 0]
p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                             cameraYaw=camera_yaw,
                             cameraPitch=camera_pitch,
                             cameraTargetPosition=camera_target_position)


#roll_pitch_erros is equal two the 3 last elements of the error array




# Run simulation continuously
while True:
    for i, roll_pitch_error in enumerate(errors):
        # Assuming yaw is zero
        #yaw_error = 0
        
        # rotation_error the last 3 elements of the error array
        #rotation_error = np.array([roll_pitch_error[3], roll_pitch_error[4], roll_pitch_error[5]])
        rotation_error = [-roll_pitch_error[3], roll_pitch_error[4], roll_pitch_error[5]]
        
        #print(f"rotation_error: {rotation_error}")
        #print(f"i: {i * 0.01}")
        start_position = [roll_pitch_error[0], roll_pitch_error[1], - roll_pitch_error[2]]

        #update the camera target position to follow the drone
        camera_target_position = start_position
        p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                             cameraYaw=camera_yaw,
                             cameraPitch=camera_pitch,
                             cameraTargetPosition=camera_target_position)

        #print(f"start_position: {start_position}")
        # Calculate actual rotation
        actual_rotation = rotation_error  # Assuming errors are calculated as 0 - actual rotation

        # Apply rotation to the drone
        p.resetBasePositionAndOrientation(drone_id, start_position, p.getQuaternionFromEuler(actual_rotation))

        # Step simulation
        p.stepSimulation()
        
        #p.realTimeSimulation(100)


        # Optionally, you can add a delay to visualize the simulation
        time.sleep(time_step)

# Disconnect from the simulation (this won't be reached in an infinite loop)
#p.disconnect()