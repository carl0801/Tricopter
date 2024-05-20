import pybullet as p
import numpy as np
import time
import pybullet_data

def calculate_lift(pwm_output):
    return 1.985 * pwm_output - 176.27

print("Starting simulation...")
# Connect to PyBullet physics server
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Define simulation parameters
time_step = 0.01   # Simulation time step in seconds
gravity = -9.81  # Gravity in m/s^2

# Set up PyBullet simulation environment
p.setGravity(0, 0, gravity)

# Load the URDF file
planeId = p.loadURDF("plane.urdf")
# Load URDF file
drone_id = p.loadURDF("C:/Users/emilr/OneDrive/Documents/GitHub/Tricopter/Simulation/drone.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, -np.pi]))

# Initialize variables for regulation
integral_x = 0.0
integral_y = 0.0
integral_alti = 0.00  # Integral term for altitude control
previous_error_x = 0.0
previous_error_y = 0.0
previous_error_alti = 0.0  # Previous error for altitude control


kp_2 = 0.08 #roll and pitch
ki = 0.00   #roll and pitch
kd = 0.04

kp = 0.08
ki_alti = 0.02#0.01  # Ki for altitude control
kd_alti = 1  # Kd for altitude control
target_roll = 0.0
target_pitch = 0.0
target_yaw = 0.1
target_altitude = 3.0
output1 = 0.0
output2 = 0.0
output3 = 0.0
tilt_angle = 0.0

yaw_control = 0.0
yaw_error = 0.0
kp_yaw = 0.05
ki_yaw = 0.0001
kd_yaw = 0.04
integral_yaw = 0.0
previous_error_yaw = 0.0
derivative_yaw = 0.0

# Get the joint indices for the motors and wings
num_joints = p.getNumJoints(drone_id)
motor_joints = []
wing_joint_index = -1
for jointIndex in range(num_joints):
    joint_info = p.getJointInfo(drone_id, jointIndex)
    joint_name = joint_info[1].decode("utf-8")  # Convert bytes to string
    print(joint_name)  # Print joint names for inspection
    if "motor" in joint_name:
        motor_joints.append(jointIndex)
    elif "Revolute_5" in joint_name:
        wing_joint_index = jointIndex

#print("Motor joints:", motor_joints)
print("Wing joint index:", wing_joint_index)

# create data array
data = []

# make the loop as a try, if the user stops the simulation, the data will be saved

torque_scale = 0.01

# Run simulation continuously
try:
    while True:
        # Extract position and orientation
        pos, orn = p.getBasePositionAndOrientation(drone_id)
        roll, pitch, yaw = p.getEulerFromQuaternion(orn)
        


        #print("roll:", roll, "pitch:", pitch, "yaw:", yaw)
        # Calculate errors
        error_orientation_x = target_roll - roll
        error_orientation_y = target_pitch - pitch
        error_altitude = target_altitude - pos[2]  # Altitude error
        yaw_error = target_yaw - orn[2]
        print("target_yaw: ", target_yaw, "yaw: ", orn[2])
        print("yaw_error: ", yaw_error)
        tilt_angle = (-kp_yaw * yaw_error - ki_yaw * integral_yaw - kd_yaw * derivative_yaw)
        
        

        # Update integral terms
        integral_x += error_orientation_x * time_step
        integral_y += error_orientation_y * time_step
        integral_alti += error_altitude * time_step  # Altitude integral term
        integral_yaw += yaw_error * time_step

        # Update derivative terms
        derivative_x = (error_orientation_x - previous_error_x) / time_step
        derivative_y = (error_orientation_y - previous_error_y) / time_step
        derivative_alti = (error_altitude - previous_error_alti) / time_step  # Altitude derivative term
        derivative_yaw = (yaw_error - previous_error_yaw) / time_step

        # Calculate control inputs
        
        U_alti = kp * error_altitude + ki_alti * integral_alti + kd_alti * derivative_alti  # Altitude control

        U_r1 = kp_2 * error_orientation_x + ki * integral_x + kd * derivative_x
        U_r2 = -kp_2 * error_orientation_x + (-ki * integral_x) + (-kd * derivative_x)
        U_p1 = -kp_2 * error_orientation_y + (-ki * integral_y) + (-kd * derivative_y)
        U_p2 = -kp_2 * error_orientation_y + (-ki * integral_y) + (-kd * derivative_y)
        U_p3 = kp_2 * error_orientation_y  + ki * integral_y + kd * derivative_y

        # Combine control inputs
        offset = 88
        pwm1 = offset+U_alti + U_r1 + U_p1
        pwm2 = offset+U_alti + U_r2 + U_p2
        pwm3 = offset+U_alti + U_p3
        print("PWM1: ", pwm1, "PWM2: ", pwm2, "PWM3: ", pwm3)
        output1 = calculate_lift(offset+U_alti + U_r1 + U_p1)+1
        output3 = calculate_lift(offset+U_alti + U_r2 + U_p2)+1
        output2 = calculate_lift(offset+U_alti + U_p3 )+1

        thrust = 0.07
        # Define torques for each motor (assuming all motors rotate in the same direction)
        clockwise = 1
        
        counterclockwise = -1
        torques = [output1*torque_scale*clockwise, output2*torque_scale*counterclockwise, output3*torque_scale*clockwise]
        projected_force_up = np.sin(tilt_angle)*output2
        projected_force_horisontally = np.cos(tilt_angle)*output2
        
        # Apply external force based on control inputs
        motor_forces = [output1, output2, output3]  # Assuming 3 motors
        
        print("Motor forces:")
        print(motor_forces)
        # Apply external force to each motor
        for jointIndex, force in zip(motor_joints, motor_forces):
            p.applyExternalForce(drone_id, jointIndex, [0, 0, force], [0, 0, 0], p.LINK_FRAME)
        
        for jointIndex, torque in zip(motor_joints, torques):
        # Apply torque to each motor
            p.setJointMotorControl2(drone_id, jointIndex, p.TORQUE_CONTROL, force=torque)
    
        #tilt_angle = np.clip(tilt_angle, np.radians(-180.0), np.radians(180.0))  # Adjust range as needed
        # Set tilt angle for the wing
        print("tilt_angle: ", tilt_angle)
        p.setJointMotorControl2(drone_id, wing_joint_index, p.POSITION_CONTROL, targetPosition=tilt_angle)

        # save all errors and motor output to data
        
        data.append([pos[0], pos[1], pos[2], np.rad2deg(error_orientation_x), np.rad2deg(error_orientation_y), error_altitude, output1, output2, output3])

        # Update previous errors for derivative term
        previous_error_x = error_orientation_x
        previous_error_y = error_orientation_y
        previous_error_alti = error_altitude
        previous_error_yaw = yaw_error
        p.resetDebugVisualizerCamera(2.0, 90.0, -10, pos)

        # Optionally, you can add a delay to visualize the simulation
        p.stepSimulation()
        time.sleep(time_step)
except KeyboardInterrupt:
    # Save data when simulation is stopped by the user
    np.savetxt('simulation_data.csv', data, delimiter=',')
