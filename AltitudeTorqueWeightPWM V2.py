import pybullet as p
import numpy as np
import time
import pybullet_data

def calculate_lift(pwm_output):
    return 1.985 * pwm_output - 176.27

print("Starting simulation...")
# Connect to PyBullet physics server
Hover = True
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
startOrientation = np.pi/2+np.deg2rad(-120)
drone_id = p.loadURDF("./Simulation/drone.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, startOrientation]))

# Initialize variables for regulation
integral_x = 0.0
integral_y = 0.0
integral_alti = 0.00  # Integral term for altitude control
previous_error_x = 0.0
previous_error_y = 0.0
previous_error_alti = 0.0  # Previous error for altitude control


kp_2 = 0.08 #roll and pitch
ki = 0.00   #roll and pitch
kd = 0.12

kp = 1.5
ki_alti = 0.3#0.01  # Ki for altitude control
kd_alti = 1.5  # Kd for altitude control
target_roll = 0.0
target_pitch = 0.0
target_yaw = -0.25
target_altitude = 3.0
target_x = 0.0
target_y = 0.0
output1 = 0.0
output2 = 0.0
output3 = 0.0
tilt_angle = 0.0

yaw = 0.0
yaw_control = 0.0
yaw_error = 0.0
kp_yaw = 0.8
ki_yaw = 0.0
kd_yaw = 0.08
integral_yaw = 0.0
previous_error_yaw = 0.0
derivative_yaw = 0.0
yaw_offset = 0.005

# Get the joint indices for the motors and wings
num_joints = p.getNumJoints(drone_id)
motor_joints = []
wing_joints = []
for jointIndex in range(num_joints):
    joint_info = p.getJointInfo(drone_id, jointIndex)
    joint_name = joint_info[1].decode("utf-8")  # Convert bytes to string
    print(joint_name)  # Print joint names for inspection
    if "motor" in joint_name:
        motor_joints.append(jointIndex)
    elif "Revolute" in joint_name:
        wing_joints.append(jointIndex)

# Color the wings for better visualization diffent colors
colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
for jointIndex, color in zip(wing_joints, colors):
    p.changeVisualShape(drone_id, jointIndex, rgbaColor=color)
    
print("First wing is red, second wing is green, third wing is blue")
print("first wing is m1, second wing is m3, third wing is m2")


#print("Motor joints:", motor_joints)
print("Wing joints", wing_joints)

# create data array
data = []

# make the loop as a try, if the user stops the simulation, the data will be saved

torque_scale = 0.01

# add kp_yaw, ki_yaw, kd_yaw as parameters to the gui
p.addUserDebugParameter("kp_yaw", 0, 1, kp_yaw)
p.addUserDebugParameter("ki_yaw", 0, 1, ki_yaw)
p.addUserDebugParameter("kd_yaw", 0, 1, kd_yaw)


kp_position_x = 0.5 # 0.08
ki_position_x = 0.000
kd_position_x = 0.00
kp_position_y = 0.5
ki_position_y = 0.000
kd_position_y = 0.0000

# Initialize variables for position control
integral_position_x = 0.0
integral_position_y = 0.0
previous_error_position_x = 0.0
previous_error_position_y = 0.0

offset_pitch = 0.0

def invert_quaternion(quat):
    return [-quat[0], -quat[1], -quat[2], quat[3]]

# Run simulation continuously
try:
    while Hover == True:
        # Extract position and orientation
        pos, orn = p.getBasePositionAndOrientation(drone_id)
        global_orientation = orn
        orn = invert_quaternion(orn)
        roll = -orn[0]
        pitch = -orn[1]
        yaw = -orn[2]
        #print("global orientation: ", global_orientation, "local orientation: ", roll, pitch, yaw)
        # Position control
        error_x = (target_x - pos[0])
        error_y = -(target_y - pos[1])

        # Update integral terms
        integral_position_x += error_x * time_step
        integral_position_y += error_y * time_step

        # Update derivative terms
        derivative_position_x = (error_x - previous_error_position_x) / time_step
        derivative_position_y = (error_y - previous_error_position_y) / time_step

        pitch_error = kp_position_x * error_x + ki_position_x * integral_position_x + kd_position_x * derivative_position_x
        roll_error = kp_position_y * error_y + ki_position_y * integral_position_y + kd_position_y * derivative_position_y

        # define dynamics for the position control
        additional_tilt_m1 = pitch_error + roll_error*0.5
        additional_tilt_m2 = -pitch_error - roll_error*0.5
        additional_tilt_m3 = 0 - roll_error
        #print("additional_tilt_m1: ", additional_tilt_m1, "additional_tilt_m2: ", additional_tilt_m2, "additional_tilt_m3: ", additional_tilt_m3)

        

        error_orientation_x = target_roll - roll
        error_orientation_y = target_pitch - pitch
        error_altitude = target_altitude - pos[2]  # Altitude error
        #print("error roll", error_orientation_x, "error pitch", error_orientation_y)
        #print("roll:", roll, "pitch:", pitch, "yaw:", yaw)
        # Calculate errors
        error_orientation_x = target_roll - roll
        error_orientation_y = target_pitch - pitch
        error_altitude = target_altitude - pos[2]  # Altitude error
        
        # calculate yaw error in quaternions

        yaw_error = -(target_yaw - yaw)
        #print("target_yaw: ", target_yaw, "yaw: ", yaw)
        #print("yaw_error: ", yaw_error)
        tilt_angle = (kp_yaw * yaw_error + ki_yaw * integral_yaw + kd_yaw * derivative_yaw)
        tilt_angles = [tilt_angle+additional_tilt_m1, tilt_angle+additional_tilt_m3, tilt_angle+additional_tilt_m2]
        
        # calculate projected force
        ver_force_1 = np.sin(tilt_angles[0])*output1
        ver_force_2 = np.sin(tilt_angles[1])*output2
        ver_force_3 = np.sin(tilt_angles[2])*output3

        hor_force_1 = np.cos(tilt_angles[0])*output1
        hor_force_2 = np.cos(tilt_angles[1])*output2
        hor_force_3 = np.cos(tilt_angles[2])*output3




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
        
        U_r1 = kp_2 * error_orientation_x + (ki * integral_x) + (kd * derivative_x)
        U_r2 = -kp_2 * error_orientation_x + (-ki * integral_x) + (-kd * derivative_x)
        U_p1 = -kp_2 * error_orientation_y + (-ki * integral_y) + (-kd * derivative_y)
        U_p2 = -kp_2 * error_orientation_y + (-ki * integral_y) + (-kd * derivative_y)
        U_p3 = kp_2 * error_orientation_y  + ki * integral_y + kd * derivative_y

        # Combine control inputs
        offset = 88
        pwm1 = offset+U_alti + U_r1 + U_p1
        pwm2 = offset+U_alti + U_r2 + U_p2
        pwm3 = offset+U_alti + U_p3
        # constraning the pwm values 0 to 180
        pwm1 = np.clip(pwm1, 0, 180)
        pwm2 = np.clip(pwm2, 0, 180)
        pwm3 = np.clip(pwm3, 0, 180)
        
        #print("PWM1: ", pwm1, "PWM2: ", pwm2, "PWM3: ", pwm3)
        output1 = calculate_lift(pwm1)+2
        output3 = calculate_lift(pwm2)+2
        output2 = calculate_lift(pwm3)+2

        thrust = 0.07
        # Define torques for each motor (assuming all motors rotate in the same direction)
        clockwise = 1
        
        counterclockwise = -1
        torques = [output1*torque_scale*clockwise, output2*torque_scale*counterclockwise, output3*torque_scale*clockwise]
        projected_force_up = np.sin(tilt_angle)*output2
        projected_force_horisontally = np.cos(tilt_angle)*output2
        
        # Apply external force based on control inputs
        motor_forces = [output1+hor_force_1, output2+hor_force_2, output3+hor_force_3]  # Assuming 3 motors
        
        #print("Motor forces:")
        #print(motor_forces)
        # Apply external force to each motor
        for jointIndex, force in zip(motor_joints, motor_forces):
            p.applyExternalForce(drone_id, jointIndex, [0, 0, force], [0, 0, 0], p.LINK_FRAME)
        
        for jointIndex, torque in zip(motor_joints, torques):
        # Apply torque to each motor
            p.setJointMotorControl2(drone_id, jointIndex, p.TORQUE_CONTROL, force=torque)
    
        #tilt_angle = np.clip(tilt_angle, np.radians(-180.0), np.radians(180.0))  # Adjust range as needed
        # Set tilt angle for the wing
        #print("tilt_angle: ", tilt_angle)

        
        #print("tilt_angles: ", tilt_angles)
        print(tilt_angles[0])
        # Apply the tilt angles to all the wings
        for jointIndex, tilt_angle in zip(wing_joints, tilt_angles):
            p.setJointMotorControl2(drone_id, jointIndex, p.POSITION_CONTROL, targetPosition=tilt_angle)

        # save all errors and motor output to data
        
        data.append([pos[0], pos[1], pos[2], np.rad2deg(error_orientation_x), np.rad2deg(error_orientation_y), error_altitude, output1, output2, output3])

        # Update previous errors for derivative term
        previous_error_x = error_orientation_x
        previous_error_y = error_orientation_y
        previous_error_alti = error_altitude
        previous_error_yaw = yaw_error
        p.resetDebugVisualizerCamera(1, 40.0, -40, pos)
        
        #show current yaw in the gui
         
        p.addUserDebugText("Yaw: "+str(yaw), [pos[0], pos[1], pos[2]+0.5], [0,0,0], lifeTime=0.2, textSize=2.0)

        # Optionally, you can add a delay to visualize the simulation
        p.stepSimulation()
        time.sleep(time_step)
except KeyboardInterrupt:
    # Save data when simulation is stopped by the user
    np.savetxt('simulation_data.csv', data, delimiter=',')
