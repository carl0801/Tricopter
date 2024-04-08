import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import minimize
from spatialmath import SE3, SO3, UnitQuaternion
from spatialmath.base.quaternions import *
import pybullet as p
import pybullet_data
import time
from multiprocessing import Pool


dt = 0.01
class PIDController:
    def __init__(self, kp, ki, kd, dt=0.01, max_integral=None, derivative_filter=0.1):
        """
        Initialize PID controller.

        Parameters:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        dt (float, optional): Time step. Default is 0.01.
        max_integral (float, optional): Maximum integral value (for anti-windup). Default is None.
        derivative_filter (float, optional): Derivative filter coefficient (0 to 1). Default is None (no filter).
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.prev_derivative = 0
        self.dt = dt
        self.max_integral = max_integral
        self.derivative_filter = derivative_filter

    def calculate(self, error):
        """
        Calculate PID output.

        Parameters:
        error (float): Error signal.

        Returns:
        float: PID output.
        """
        self.integral += error * self.dt
        
        # Anti-windup
        if self.max_integral is not None:
            self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
            
        derivative = (error - self.prev_error) / self.dt
        
        # Apply derivative filter
        if self.derivative_filter is not None:
            derivative = self.prev_derivative * (1 - self.derivative_filter) + derivative * self.derivative_filter
        
        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error
        self.prev_derivative = derivative
        
        return output

class Quadcopter:
    def __init__(self, mass, l_0, gravity, drag, j_x = 0.05, j_y = 0.05, j_z = 0.02, k_t =0.01 , k_d = 0.0000003 ):  #k_d = 0.0003
        self.mass = mass
        self.l_0 = l_0
        self.gravity = gravity
        self.drag = drag
        self.j_x = j_x
        self.j_y = j_y
        self.j_z = j_z
        self.k_t = k_t
        self.k_d = k_d
        self.l_1 = np.sin(60 * np.pi / 180) * l_0
        self.l_2 = np.sin(-60 * np.pi / 180) * l_0

def opposite_angle(angle_deg):
    if angle_deg < 0:
        return angle_deg + 360
    else:
        return angle_deg - 360

def quaternion_rotation(current_angle, target_angle):
    # Convert angles to radians
    current_angle_rad = np.radians(current_angle)
    target_angle_rad = np.radians(target_angle)

    # Define the rotation axis (unit vector)
    rotation_axis = [0, 0, 1]

    # Calculate the angle of rotation
    angle = target_angle_rad - current_angle_rad

    # Calculate the quaternion components
    q0 = np.cos(angle / 2)
    q1 = np.sin(angle / 2) * rotation_axis[0]
    q2 = np.sin(angle / 2) * rotation_axis[1]
    q3 = np.sin(angle / 2) * rotation_axis[2]

    # Create the quaternion
    q = np.array([q0, q1, q2, q3])

    # Normalize the quaternion
    q /= np.linalg.norm(q)

    # Convert quaternion to rotation matrix
    rotation_matrix = np.array([
        [1 - 2 * (q2**2 + q3**2), 2 * (q1*q2 - q0*q3), 2 * (q0*q2 + q1*q3)],
        [2 * (q1*q2 + q0*q3), 1 - 2 * (q1**2 + q3**2), 2 * (q2*q3 - q0*q1)],
        [2 * (q1*q3 - q0*q2), 2 * (q0*q1 + q2*q3), 1 - 2 * (q1**2 + q2**2)]
    ])

    output_angle_rad = np.arccos((np.trace(rotation_matrix) - 1) / 2)
    output_angle_deg = np.degrees(output_angle_rad)

    opposite_target_angle = opposite_angle(target_angle)
    
    if (current_angle < -178 and current_angle >= -180) and (target_angle > 178 and target_angle <= 180):
        #print("here")
        output_angle_deg = -output_angle_deg
    # elif current angle is 179+-1 and target angle is -179+-1 then outputangle is positive
    elif (current_angle > 178 and current_angle <= 180) and (target_angle < -178 and target_angle >= -180):
        output_angle_deg = output_angle_deg
    # elif current angle is -179+-1 and target angle is 179+-1 then outputangle is negative
        
    elif (output_angle_deg + current_angle < target_angle + 0.01 and output_angle_deg + current_angle > target_angle - 0.01) or (output_angle_deg + current_angle < opposite_target_angle + 0.01 and output_angle_deg + current_angle > opposite_target_angle - 0.01):
        output_angle_deg = output_angle_deg
        
    else:
        output_angle_deg = -output_angle_deg

    return output_angle_deg


""" 

def tricopter_flight_controller():
    
    trans_control_x = PIDController(0.1965757725903785, 0.010342614229023009, 0.2051560043445344, dt)
    rot_control_pitch = PIDController(0.30164029571075623, 0.000128899544963653, 1.1998256078947662, dt)


    trans_control_y = PIDController(0.21248410411380658, 0.12152492457843977, 0.32629781318334894)
    rot_control_roll = PIDController(0.27778911515638605, -0.017256089514188208, 0.8259220175587367)

    trans_control_z = PIDController(0.9945873259566484, 0.04333792067287307, 4.150268773767612)


    rot_control_yaw = PIDController(0.08935726607591638, 0.009453101478065287, 0.41156854675899196, dt)




    drone = Quadcopter(0.25, 0.33, 9.81, 0.02)



    o_target_x = 0
    o_target_y = 0
    target_z = -1
    target_roll = 0
    target_pitch = 0
    target_yaw = 0
  



    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0
 
    x_vel = 0
    y_vel = 0
    z_vel = 0

    roll_vel = 0
    pitch_vel = 0
    yaw_vel = 0


    omega_1 = 0
    omega_2 = 0
    omega_3 = 0
    alpha = 0

    omega_1_result = []
    omega_2_result = []
    omega_3_result = []
    alpha_result = []

    x_result = []
    y_result = []
    z_result = []
    roll_result = []
    pitch_result = []
    yaw_result = []


    U_x_result = []
    U_y_result = []
    U_pitch_result = []
    U_roll_result = []
    U_yaw_result = [] 

    error_result = []
    pos_result = []

    stopped_spinning = False
    yawStopper = 0
    prev_alpha = 0
    servo_alpha = 0
    i = 0
    time = 10000*dt
    while i < time:
   



        #target_z = -i
        #o_target_x = np.sin(4*np.pi/30 * i)
        #o_target_y = 2*np.sin(8*np.pi/30 * i)
        #target_yaw = np.sin(np.pi/30 * i)/4

        z_error = (target_z - z) + np.random.normal(0, 0.01)
        U_z = trans_control_z.calculate((z_error)) #- z_vel * 3
        #U_z -= z_vel * 3

        U_z -= drone.mass * drone.gravity  

    
        #target_x = np.cos(yaw) * o_target_x - np.sin(yaw) * o_target_y
        #target_y = np.sin(yaw) * o_target_x + np.cos(yaw) * o_target_y

        #x_error = (np.cos(yaw) * (o_target_x - x) - np.sin(yaw) * (o_target_y - y)) + np.random.normal(0, 0.01)
        #y_error = (np.sin(yaw) * (o_target_x - x) + np.cos(yaw) * (o_target_y - y)) + np.random.normal(0, 0.01)

   
        global_x_error = (o_target_x - x) + np.random.normal(0, 0.01)
        global_y_error = (o_target_y - y) + np.random.normal(0, 0.01)

        #local_x_error = np.cos(yaw) * global_x_error + np.sin(yaw) * global_y_error
        #local_y_error = -np.sin(yaw) * global_x_error + np.cos(yaw) * global_y_error

        #print(f"yaw_vel: {yaw_vel}")
        #print(f"target_z: {target_z}")
        #print(stopped_spinning)
        #add some noise to the errors
        #x_error = (o_target_x - x) + np.random.normal(0, 0.01)
        x_error = np.cos(yaw) * global_x_error + np.sin(yaw) * global_y_error
        #print(f"x_error: {x_error}")
        U_x = trans_control_x.calculate(x_error) #- x_vel * 0.1

        pitch_error = U_x - pitch + np.random.normal(0, 0.001)*np.pi/180
        U_pitch = rot_control_pitch.calculate(pitch_error) #- pitch_vel * 0.1


        #y_error = (target_y - y) + np.random.normal(0, 0.01)
        y_error = -np.sin(yaw) * global_x_error + np.cos(yaw) * global_y_error
        #print(f"y_error: {y_error}")
        U_y = trans_control_y.calculate(y_error) #- y_vel * 0.1

        roll_error = U_y - roll + np.random.normal(0, 0.001)*np.pi/180
        U_roll = rot_control_roll.calculate(roll_error) #- roll_vel * 0.1






        yaw_error = target_yaw - yaw #+ np.random.normal(0, 0.001)*np.pi/180
        U_yaw = rot_control_yaw.calculate(yaw_error)

    




        #print(f"U_z: {U_z}")
  


        #print(f"U_z: {U_z}")
        #print(f"Z_error: {z_error}")
        #print(f"hover force = {-drone.mass * drone.gravity}")
        term1_12 = (2 * drone.l_0 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2))
        term2_12 = (2 * U_roll) / (drone.l_1 * drone.k_t)
        term3_12 = (2 * U_pitch) / (drone.k_t * (drone.l_0 + drone.l_2))
        omega_1_mid = -term1_12 - term2_12 + term3_12
        omega_2_mid = -term1_12 + term2_12 + term3_12
        omega_1 = 0 if omega_1_mid < 0 else np.sqrt(omega_1_mid) / 2
        omega_2 = 0 if omega_2_mid < 0 else np.sqrt(omega_2_mid) / 2

        

        term1_3p1 = (drone.l_2 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2))
        term1_3p2 = U_pitch / (drone.k_t * (drone.l_0 + drone.l_2))
        term1_3 = (-term1_3p1 - term1_3p2) ** 2

        term2_3p1 = (drone.k_d * U_z) / (drone.k_t * drone.k_t * drone.l_0)
        term2_3p2 = U_yaw / (drone.l_0 * drone.k_t)
        term2_3 = (-term2_3p1 + term2_3p2) ** 2

        omega_3 = (term1_3 + term2_3) ** (0.25)



        alpha_term1 = -(drone.k_d * U_z) / (drone.k_t * drone.k_t * drone.l_0) + U_yaw / (drone.l_0 * drone.k_t)
        alpha_term2 = -(drone.l_2 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2)) - U_pitch / (drone.k_t * (drone.l_0 + drone.l_2))
        alpha = np.arctan2(alpha_term1, alpha_term2)

        

        

        roll_acc = (1 / drone.j_x) * ((drone.j_y - drone.j_z) * pitch_vel * yaw_vel + U_roll)
        pitch_acc = (1 / drone.j_y) * ((drone.j_z - drone.j_x) * roll_vel * yaw_vel + U_pitch)
        yaw_acc = (1 / drone.j_z) * ((drone.j_x - drone.j_y) * roll_vel * pitch_vel + U_yaw)

        #roll_acc =  (1/drone.j_x)*U_roll #(drone.k_t * (omega_1 ** 2 - omega_2 ** 2)) / drone.j_x #(1/drone.j_x)*U_roll
        #pitch_acc = (1/drone.j_y)*U_pitch #(drone.k_t * (omega_1 ** 2 + omega_2 ** 2 - omega_3 ** 2) ) / drone.j_y 
        #yaw_acc =   (1/drone.j_z)*U_yaw #(drone.k_t * (omega_1 ** 2 - omega_2 ** 2)) / drone.j_z 

        #roll_acc = drone.l_1 * drone.k_t * (-omega_1 ** 2 + omega_2 ** 2) / drone.j_x
        #pitch_acc = (drone.k_t * (drone.l_2 * omega_1 ** 2 + drone.l_2 * omega_2 ** 2 - drone.l_0 * omega_3 ** 2 * np.cos(alpha))) / drone.j_y
        #yaw_acc = (-drone.k_d * (omega_1 ** 2 - omega_2 ** 2 - omega_3 **2 * np.cos(alpha)) + drone.l_0 * drone.k_t * omega_3**2 * np.sin(alpha)) / drone.j_z

        roll_vel += roll_acc * dt
        pitch_vel += pitch_acc * dt
        yaw_vel += yaw_acc * dt

        roll += roll_vel * dt
        pitch += pitch_vel * dt
        yaw += yaw_vel * dt


        f_y = drone.k_t * omega_3**2 * np.sin(alpha)

        #x_acc = (f_y * (np.sin(roll) * np.sin(pitch) * np.cos(yaw) - np.cos(roll) * np.sin(yaw)) + U_z * (np.sin(pitch) * np.cos(roll) * np.cos(yaw) + np.sin(roll) * np.sin(yaw)))/drone.mass
        x_acc = -(f_y * (np.sin(roll) * np.sin(pitch) * np.cos(yaw) - np.cos(roll) * np.sin(yaw)) +
                U_z * (np.sin(pitch) * np.cos(roll) * np.cos(yaw) + np.sin(roll) * np.sin(yaw))) / drone.mass

        y_acc = (f_y * (np.sin(roll) * np.sin(pitch) * np.sin(yaw) + np.cos(roll) * np.cos(yaw)) +
                U_z * (np.sin(pitch) * np.cos(roll) * np.sin(yaw) - np.sin(roll) * np.cos(yaw))) / drone.mass
        

        #transform the acceleration to earth coordinates using yaw
        x_acc = np.cos(yaw) * x_acc - np.sin(yaw) * y_acc
        y_acc = -np.sin(yaw) * x_acc + np.cos(yaw) * y_acc    


       
        z_acc = (f_y * np.sin(roll) * np.cos(pitch) + U_z * np.cos(roll) * np.cos(pitch) + drone.mass * drone.gravity)/drone.mass
        



        #x_vel = 0
        #y_vel = 0
        x_vel += x_acc * dt
        y_vel += y_acc * dt
        z_vel += z_acc * dt

        x += x_vel * dt
        y += y_vel * dt
        z += z_vel * dt

  
        #print(f"omega_1: {omega_1}, omega_2: {omega_2}, omega_3: {omega_3}, alpha: {alpha*180/np.pi -180 }")
        omega_1_result.append([omega_1,i])
        omega_2_result.append([omega_2,i])
        omega_3_result.append([omega_3,i])
        #alpha = alpha*-1 - np.pi

        

        alpha *= 180/np.pi
        #alpha -= 180



        #alpha_result.append([quaternion_rotation(alpha, servo_alpha),i])
        alpha_result.append([alpha,i])

        prev_alpha = alpha

        x_result.append([x,i])
        y_result.append([y,i])
        z_result.append([z,i])
        #print(f"z: {z}")

        roll_result.append([roll*180/np.pi,i])
        pitch_result.append([pitch*180/np.pi,i])
        yaw_result.append([yaw*180/np.pi,i])


        U_x_result.append([U_x*180/np.pi,i])
        U_pitch_result.append([U_pitch*180/np.pi,i])

        U_y_result.append([U_y*180/np.pi,i])
        U_roll_result.append([U_roll*180/np.pi,i])

        U_yaw_result.append([U_yaw*180/np.pi,i])

        #error_result.append([[x_error, y_error, z_error, roll_error, pitch_error, yaw_error],i])
        pos_result.append([[x, y, z, roll, pitch, yaw],i])


        #print(f"U_y: {U_y} U_roll: {U_roll}")
        
        
        i += dt


    



    df = pd.DataFrame(pos_result, columns = ['pos', 'time'])
    df.to_csv('sim.csv', index=False)


    # Create a figure and a set of subplots
    fig, axs = plt.subplots(6, 2, figsize=(12, 8))

    # Plot x_result on the first subplot
    axs[0, 0].plot([data[1] for data in x_result], [data[0] for data in x_result])
    axs[0, 0].set_title('x')

    # Plot y_result on the second subplot
    axs[1, 0].plot([data[1] for data in y_result], [data[0] for data in y_result])
    axs[1, 0].set_title('y')

    # Plot z_result on the third subplot
    axs[2, 0].plot([data[1] for data in z_result], [data[0] for data in z_result])
    axs[2, 0].set_title('z')

    # Plot roll_result on the fourth subplot
    axs[3, 0].plot([data[1] for data in roll_result], [data[0] for data in roll_result])
    axs[3, 0].set_title('roll')

    # Plot pitch_result on the fifth subplot
    axs[4, 0].plot([data[1] for data in pitch_result], [data[0] for data in pitch_result])
    axs[4, 0].set_title('pitch')

    # Plot yaw_result on the sixth subplot
    axs[5, 0].plot([data[1] for data in yaw_result], [data[0] for data in yaw_result])
    axs[5, 0].set_title('yaw')


    # Plot omega_1_result on the first subplot
    axs[0, 1].plot([data[1] for data in omega_1_result], [data[0] for data in omega_1_result])
    axs[0, 1].set_title('omega_1')

    # Plot omega_2_result on the second subplot
    axs[1, 1].plot([data[1] for data in omega_2_result], [data[0] for data in omega_2_result])
    axs[1, 1].set_title('omega_2')

    # Plot omega_3_result on the third subplot
    axs[2, 1].plot([data[1] for data in omega_3_result], [data[0] for data in omega_3_result])
    axs[2, 1].set_title('omega_3')

    axs[3, 1].plot([data[1] for data in alpha_result], [data[0] for data in alpha_result])
    axs[3, 1].set_title('alpha')



    axs[4, 1].plot([data[1] for data in U_y_result], [data[0] for data in U_y_result])
    axs[4, 1].set_title('U_y')

    axs[5, 1].plot([data[1] for data in U_roll_result], [data[0] for data in U_roll_result])
    axs[5, 1].set_title('U_roll')





    # Set a common label for all subplots
    for ax in axs.flat:
        ax.set(xlabel='Time')

    # Display the figure with subplots
    plt.tight_layout()
    plt.show()

    #run the emil_sim.py
    #import emil_sim
    
    return

 """

def plant(u1,u2,t):
    


    drone = Quadcopter(0.25, 0.25, 9.81, 0.02)

    #trans_control_x = PIDController(0.2, 0.01, 0.2, dt)
    trans_control_x = PIDController(0.1965757725903785, 0.010342614229023009, 0.2051560043445344, dt)

    #rot_control_pitch = PIDController(0.3, 0, 1.2, dt)
    rot_control_pitch = PIDController(0.30164029571075623, 0.000128899544963653, 1.1998256078947662, dt)


    #trans_control_y = PIDController(0.2, 0.01, 0.2)
    trans_control_y = u1

    #rot_control_roll = PIDController(0.5, 0, 1.2)
    rot_control_roll = u2

    #trans_control_z = PIDController(0.9545651916204778,0.04441356161831689,4.0843239598328)
    trans_control_z = PIDController(0.9945873259566484, 0.04333792067287307, 4.150268773767612)

    #rot_control_yaw = PIDController(0.08508958604791192, 0.00923672770996732, 0.4286804507510326)
    rot_control_yaw = PIDController(0.08935726607591638, 0.009453101478065287, 0.41156854675899196, dt)


 
    
    target_x = 1
    target_y = 1
    target_z = -1
    target_roll = 0
    target_pitch = 0
    target_yaw = 0#10*np.pi/180
    

    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0
 
    x_vel = 0
    y_vel = 0
    z_vel = 0

    roll_vel = 0
    pitch_vel = 0
    yaw_vel = 0


    omega_1 = 0
    omega_2 = 0
    omega_3 = 0
    alpha = 0



    x_result = []
    y_result = []
    z_result = []
    roll_result = []
    pitch_result = []
    yaw_result = []




    for i in t:
        #Position error

        z_error = (target_z - z) + np.random.normal(0, 0.01)
        U_z = trans_control_z.calculate((z_error)) #- z_vel * 3
        #U_z -= z_vel * 3

        U_z -= drone.mass * drone.gravity  

    
        #target_x = np.cos(yaw) * o_target_x - np.sin(yaw) * o_target_y
        #target_y = np.sin(yaw) * o_target_x + np.cos(yaw) * o_target_y

        #x_error = (np.cos(yaw) * (o_target_x - x) - np.sin(yaw) * (o_target_y - y)) + np.random.normal(0, 0.01)
        #y_error = (np.sin(yaw) * (o_target_x - x) + np.cos(yaw) * (o_target_y - y)) + np.random.normal(0, 0.01)

   
        global_x_error = (target_x - x) + np.random.normal(0, 0.01)
        global_y_error = (target_y - y) + np.random.normal(0, 0.01)

        #local_x_error = np.cos(yaw) * global_x_error + np.sin(yaw) * global_y_error
        #local_y_error = -np.sin(yaw) * global_x_error + np.cos(yaw) * global_y_error

        #print(f"yaw_vel: {yaw_vel}")
        #print(f"target_z: {target_z}")
        #print(stopped_spinning)
        #add some noise to the errors
        #x_error = (o_target_x - x) + np.random.normal(0, 0.01)
        x_error = np.cos(yaw) * global_x_error + np.sin(yaw) * global_y_error
        #print(f"x_error: {x_error}")
        U_x = trans_control_x.calculate(x_error) #- x_vel * 0.1

        pitch_error = U_x - pitch + np.random.normal(0, 0.01)*np.pi/180
        U_pitch = rot_control_pitch.calculate(pitch_error) #- pitch_vel * 0.1


        #y_error = (target_y - y) + np.random.normal(0, 0.01)
        y_error = -np.sin(yaw) * global_x_error + np.cos(yaw) * global_y_error
        #print(f"y_error: {y_error}")
        U_y = trans_control_y.calculate(y_error) #- y_vel * 0.1

        roll_error = U_y - roll + np.random.normal(0, 0.01)*np.pi/180
        U_roll = rot_control_roll.calculate(roll_error) #- roll_vel * 0.1


        yaw_error = target_yaw - yaw + np.random.normal(0, 0.01)*np.pi/180
        U_yaw = rot_control_yaw.calculate(yaw_error)

   

        term1_12 = (2 * drone.l_0 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2))
        term2_12 = (2 * U_roll) / (drone.l_1 * drone.k_t)
        term3_12 = (2 * U_pitch) / (drone.k_t * (drone.l_0 + drone.l_2))
        omega_1_mid = -term1_12 - term2_12 + term3_12
        omega_2_mid = -term1_12 + term2_12 + term3_12
        omega_1 = 0 if omega_1_mid < 0 else np.sqrt(omega_1_mid) / 2
        omega_2 = 0 if omega_2_mid < 0 else np.sqrt(omega_2_mid) / 2

        

        term1_3p1 = (drone.l_2 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2))
        term1_3p2 = U_pitch / (drone.k_t * (drone.l_0 + drone.l_2))
        term1_3 = (-term1_3p1 - term1_3p2) ** 2

        term2_3p1 = (drone.k_d * U_z) / (drone.k_t * drone.k_t * drone.l_0)
        term2_3p2 = U_yaw / (drone.l_0 * drone.k_t)
        term2_3 = (-term2_3p1 + term2_3p2) ** 2

        omega_3 = (term1_3 + term2_3) ** (0.25)


        alpha_term1 = -(drone.k_d * U_z) / (drone.k_t * drone.k_t * drone.l_0) + U_yaw / (drone.l_0 * drone.k_t)
        alpha_term2 = -(drone.l_2 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2)) - U_pitch / (drone.k_t * (drone.l_0 + drone.l_2))
        alpha = np.arctan2(alpha_term1, alpha_term2)
 

        roll_acc = (1 / drone.j_x) * ((drone.j_y - drone.j_z) * pitch_vel * yaw_vel + U_roll)
        pitch_acc = (1 / drone.j_y) * ((drone.j_z - drone.j_x) * roll_vel * yaw_vel + U_pitch)
        yaw_acc = (1 / drone.j_z) * ((drone.j_x - drone.j_y) * roll_vel * pitch_vel + U_yaw)

 
        #roll_acc = 0
        #pitch_acc = 0
        #yaw_acc = 0
        
        roll_vel += roll_acc * dt
        pitch_vel += pitch_acc * dt
        yaw_vel += yaw_acc * dt

        roll += roll_vel * dt
        pitch += pitch_vel * dt
        yaw += yaw_vel * dt


        f_y = drone.k_t * omega_3**2 * np.sin(alpha)

        #x_acc = (f_y * (np.sin(roll) * np.sin(pitch) * np.cos(yaw) - np.cos(roll) * np.sin(yaw)) + U_z * (np.sin(pitch) * np.cos(roll) * np.cos(yaw) + np.sin(roll) * np.sin(yaw)))/drone.mass
        x_acc = -(f_y * (np.sin(roll) * np.sin(pitch) * np.cos(yaw) - np.cos(roll) * np.sin(yaw)) +
                U_z * (np.sin(pitch) * np.cos(roll) * np.cos(yaw) + np.sin(roll) * np.sin(yaw))) / drone.mass

        y_acc = (f_y * (np.sin(roll) * np.sin(pitch) * np.sin(yaw) + np.cos(roll) * np.cos(yaw)) +
                U_z * (np.sin(pitch) * np.cos(roll) * np.sin(yaw) - np.sin(roll) * np.cos(yaw))) / drone.mass


        z_acc = (f_y * np.sin(roll) * np.cos(pitch) + U_z * np.cos(roll) * np.cos(pitch) + drone.mass * drone.gravity)/drone.mass
        

        #z_acc = 0
        #y_acc = 0
        #x_acc = 0


        #x_vel = 0
        #y_vel = 0
        x_vel += x_acc * dt
        y_vel += y_acc * dt
        z_vel += z_acc * dt

        x += x_vel * dt
        y += y_vel * dt
        z += z_vel * dt

  
 

        x_result.append([x_error])
        y_result.append([y_error])
        z_result.append([z_error])

        roll_result.append([roll_error*180/np.pi])
        pitch_result.append([pitch_error*180/np.pi])
        yaw_result.append([yaw_error*180/np.pi])


    #return roll_result as vector
    return np.array(y_result)

# Define a separate function that calls minimize
def minimize_cost_function(guess):
    return minimize(cost_function, guess, method='Nelder-Mead')

def parallel_minimize(cost_function, initial_guesses):
    with Pool() as pool:
        results = pool.map(minimize_cost_function, initial_guesses)
    return results

def pid_autotune(simulate_system, initial_guess, time):
    # Simulate system
    t = np.linspace(0, time, num=1000)  # Adjust as needed
    #y_desired = np.ones_like(t)  # Desired response, assuming step input
    y_desired = np.zeros_like(t)  # Desired response, assuming step input
    
    # Define cost function
    def cost_function(params):
        kp, ki, kd = params
        controller = PIDController(kp, ki, kd, dt)
        y_actual = simulate_system(controller, t)
        #print(f"y_actual: {y_actual[10]}, y_desired: {y_desired[10]}")
        return np.sum(np.square(y_actual - y_desired))

    # Perform optimization
    result = minimize(cost_function, initial_guess, method='Nelder-Mead')

    return result.x
    
def pid_autotune2(simulate_system, initial_guess, time):
    # Simulate system
    t = np.linspace(0, time, num=int(time/0.01))  # Adjust as needed
    y_desired = np.zeros_like(t)  # Desired response, assuming step input
    
    # Define cost function
    def cost_function(params):
        kp1, ki1, kd1, kp2, ki2, kd2 = params
        
        # Create PID controllers with the given parameters
        controller1 = PIDController(kp1, ki1, kd1)
        controller2 = PIDController(kp2, ki2, kd2)
        
        # Simulate system with both controllers
        y_actual = simulate_system(controller1, controller2, t)
        
        # Compute the sum of squared errors
        return np.sum(np.square(y_actual - y_desired))

    # Perform optimization
    result = minimize(cost_function, initial_guess, method='Nelder-Mead')

    # Extract tuned parameters
    return result.x
    tuned_kp1, tuned_ki1, tuned_kd1, tuned_kp2, tuned_ki2, tuned_kd2 = tuned_params


# Define cost function
def cost_function(params):
    time = 1.0
    t = np.linspace(0, time, num=int(time/0.01))  # Adjust as needed
    y_desired = np.zeros((len(t),1,6))  # Desired response, assuming step input
    kp1, ki1, kd1, kp2, ki2, kd2, kp3, ki3, kd3, kp4, ki4, kd4, kp5, ki5, kd5, kp6, ki6, kd6 = params
    
    # Create PID controllers with the given parameters
    controller1 = PIDController(kp1, ki1, kd1)
    controller2 = PIDController(kp2, ki2, kd2)
    controller3 = PIDController(kp3, ki3, kd3)
    controller4 = PIDController(kp4, ki4, kd4)
    controller5 = PIDController(kp5, ki5, kd5)
    controller6 = PIDController(kp6, ki6, kd6)
    
    # Simulate system with both controllers
    y_actual = tricopterSimPID(controller1, controller2, controller3, controller4, controller5, controller6, t)
    
    # Compute the sum of squared errors
    return np.sum(np.square(y_actual - y_desired))
#write a function like pid_autotune2 but with 6 controllers
def pid_autotune3(simulate_system, initial_guess, time):

    # Perform optimization
    #result = minimize(cost_function, initial_guess, method='Nelder-Mead')

    #generate more initial guesses using np.random.uniform with each guess the same size as the initial_guess
    initial_guesses = np.random.uniform(-1, 1, size=(18, len(initial_guess)))
    

    result = parallel_minimize(cost_function, initial_guesses)

    # Extract tuned parameters
    return result
    tuned_kp1, tuned_ki1, tuned_kd1, tuned_kp2, tuned_ki2, tuned_kd2, tuned_kp3, tuned_ki3, tuned_kd3, tuned_kp4, tuned_ki4, tuned_kd4, tuned_kp5, tuned_ki5, tuned_kd5, tuned_kp6, tuned_ki6, tuned_kd6 = tuned_params

def test(u1):
    controller = PIDController(*u1)
    t = np.linspace(0, 10, num=1000)
    y_actual = plant(controller,t)
    #plot the result
    plt.plot(t, y_actual)
    plt.show()
    
def test2(params):
    controller1 = PIDController(*params[:3])
    controller2 = PIDController(*params[3:])
    t = np.linspace(0, 10, num=1000)
    y_actual = plant(controller1,controller2,t)
    #plot the result
    plt.plot(t, y_actual)
    plt.show()

def tricopterSimPID(u1,u2,t):

    trans_control_x = PIDController(4.127457134181349, -4.398200938676669e-06, 0.103571025731448, dt)
    rot_control_pitch = PIDController(6.188328298297597, 1.0298904898311492, -0.00015437334853366663, dt)


    trans_control_y = PIDController(0.21248410411380658, 0.12152492457843977, 0.32629781318334894)
    rot_control_roll = PIDController(0.27778911515638605, -0.017256089514188208, 0.8259220175587367)

    #trans_control_y = u1
    #rot_control_roll = u2

    #trans_control_z = PIDController(0.9945873259566484, 0.04333792067287307, 4.150268773767612)
    trans_control_z = u1


    rot_control_yaw = PIDController(0.08935726607591638, 0.009453101478065287, 0.41156854675899196, dt)








    mass = 0.15021955564014492 + 0.04060892541828371 + 0.04060892541828371

    drone = Quadcopter(0.015021955564014492, 0.33, 9.81, 0.02)

    # grafik
    #p.connect(p.GUI)

    # ingen grafik
    p.connect(p.DIRECT)


    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set up PyBullet simulation environment
    p.setGravity(0, 0, -drone.gravity)

    #Load the URDF file
    planeId = p.loadURDF("plane.urdf")




    

    o_target_x = 0
    o_target_y = 0
    target_z = 10
    target_roll = 0
    target_pitch = 0
    target_yaw = 0
  



    x = 0
    y = 0
    z = 10
    roll = 0
    pitch = 0
    yaw = 0#-np.pi/2
 

    # Load the drone URDF file
    droneId = p.loadURDF("Simulation/drone.urdf", [x, y, z], p.getQuaternionFromEuler([roll, pitch, yaw]))

    # Get the joint indices for the motors
    num_joints = p.getNumJoints(droneId)
    motor_joints = []

    wing_joint_index = -1
    for jointIndex in range(num_joints):
        joint_info = p.getJointInfo(droneId, jointIndex)
        joint_name = joint_info[1].decode("utf-8")  # Convert bytes to string
        #print(joint_name)  # Print joint names for inspection
        if "motor" in joint_name:
            motor_joints.append(jointIndex)
        elif "Revolute_3" in joint_name:
            wing_joint_index = jointIndex

    #revolute 3 = tail wing
    #revolute 4 = right wing
    #revolute 5 = left wing


    omega_1 = 0
    omega_2 = 0
    omega_3 = 0
    alpha = 0

    omega_1_result = []
    omega_2_result = []
    omega_3_result = []
    alpha_result = []

    x_result = []
    y_result = []
    z_result = []
    roll_result = []
    pitch_result = []
    yaw_result = []


    U_x_result = []
    U_y_result = []
    U_z_result = []
    U_pitch_result = []
    U_roll_result = []
    U_yaw_result = [] 


    """ # Create visual shape for the drone
    drone_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                            fileName="Tricopter.stl",
                                            meshScale=[0.01, 0.01, 0.01])

    # Create multi-body with visual shape only
    drone_id = p.createMultiBody(baseMass=1,
                                basePosition=start_position,
                                baseOrientation=start_orientation,
                                baseVisualShapeIndex=drone_visual_shape) """

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


    error_result = []
    forces = [0,0,0]
    test_rot = 0
    i = 0
    sim_time = 100*dt
    for i in t:



        z_error = (target_z - z) #+ np.random.normal(0, 0.01)
        U_z = trans_control_z.calculate(-(z_error)) 


        U_z -= drone.mass * drone.gravity  

    


   
        global_x_error = (o_target_x - x) #+ np.random.normal(0, 0.01)
        global_y_error = (o_target_y - y) #+ np.random.normal(0, 0.01)


        x_error = np.cos(yaw) * global_x_error + np.sin(yaw) * global_y_error
        #print(f"x_error: {x_error}")
        U_x = trans_control_x.calculate(x_error) 
        

        pitch_error = U_x - pitch #+ np.random.normal(0, 0.001)*np.pi/180
        U_pitch = rot_control_pitch.calculate(pitch_error) 


        #y_error = (target_y - y) + np.random.normal(0, 0.01)
        y_error = -np.sin(yaw) * global_x_error + np.cos(yaw) * global_y_error
        #print(f"y_error: {y_error}")
        U_y = trans_control_y.calculate(y_error) #- y_vel * 0.1

        U_y = 0

        roll_error = U_y - roll #+ np.random.normal(0, 0.001)*np.pi/180
        U_roll = rot_control_roll.calculate(-roll_error) #- roll_vel * 0.1

        U_roll = 0

        yaw_error = target_yaw - yaw #+ np.random.normal(0, 0.001)*np.pi/180
        U_yaw = rot_control_yaw.calculate(yaw_error)

    

        """ U_z = 0
        U_x = 0
        U_y = 0
        U_pitch = 0
        U_roll = 0
        U_yaw = 0 """




        term1_12 = (drone.l_0 * U_z) / (2*drone.k_t * (drone.l_0 + drone.l_2))
        term2_12 = (U_roll) / (2*drone.l_1 * drone.k_t)
        term3_12 = (U_pitch) / (2*drone.k_t * (drone.l_0 + drone.l_2))

        omega_1_mid = -term1_12 - term2_12 + term3_12
        omega_2_mid = -term1_12 + term2_12 + term3_12

        omega_1 = 0 if omega_1_mid < 0 else np.sqrt(omega_1_mid) 
        omega_2 = 0 if omega_2_mid < 0 else np.sqrt(omega_2_mid) 

        

        #omega_1 = np.sqrt(omega_1_mid) / 2
        #omega_2 = np.sqrt(omega_2_mid) / 2

        term1_3p1 = -(drone.l_2 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2))
        term1_3p2 = U_pitch / (drone.k_t * (drone.l_0 + drone.l_2))
        term1_3 = (term1_3p1 + term1_3p2) ** 2

        term2_3p1 = -(drone.k_d * U_z) / (drone.k_t * drone.k_t * drone.l_0)
        term2_3p2 = U_yaw / (drone.l_0 * drone.k_t)
        term2_3 = (term2_3p1 + term2_3p2) ** 2

        omega_3 = (term1_3 + term2_3) ** (0.25)



        alpha_term1 = -(drone.k_d * U_z) / (drone.k_t * drone.k_t * drone.l_0) + U_yaw / (drone.l_0 * drone.k_t)
        alpha_term2 = -(drone.l_2 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2)) - U_pitch / (drone.k_t * (drone.l_0 + drone.l_2))
        #alpha = np.arctan2(alpha_term1, alpha_term2) /2 -np.pi/2 #- np.pi #+ 55*np.pi/180
        alpha = np.arctan(alpha_term1 / alpha_term2)

        forces[0] = drone.k_t * omega_1 ** 2 
        forces[1] = drone.k_t * omega_2 ** 2 
        forces[2] = drone.k_t * omega_3 ** 2


        

        # Set tilt angle for the wing
        p.setJointMotorControl2(droneId, wing_joint_index, p.POSITION_CONTROL, targetPosition=(alpha))

        
        #left motor
        p.applyExternalForce(objectUniqueId=droneId, linkIndex=motor_joints[2], forceObj=[0, 0, forces[0]], posObj=[0, 0, 0], flags=p.LINK_FRAME)

        #tail motor
        p.applyExternalForce(objectUniqueId=droneId, linkIndex=motor_joints[0], forceObj=[0, 0, forces[2]], posObj=[0, 0, 0], flags=p.LINK_FRAME)

        #right motor
        p.applyExternalForce(objectUniqueId=droneId, linkIndex=motor_joints[1], forceObj=[0, 0, forces[1]], posObj=[0, 0, 0], flags=p.LINK_FRAME)

            # Step the simulation
        #p.setRealTimeSimulation(1)
        p.stepSimulation()


        #error_result.append([[x_error, y_error, z_error, roll_error, pitch_error, yaw_error]])

        x_result.append(x_error)
        y_result.append([y_error])
        z_result.append([z_error])
        roll_result.append([roll_error])
        pitch_result.append([pitch_error])
        yaw_result.append([yaw_error])

        U_x_result.append([U_x])
        U_y_result.append([U_y])
        U_z_result.append([U_z])
        U_pitch_result.append([U_pitch])
        U_roll_result.append([U_roll])
        U_yaw_result.append([U_yaw])


        omega_1_result.append([omega_1,i])
        omega_2_result.append([omega_2,i])
        omega_3_result.append([omega_3,i])
        alpha_result.append([alpha,i])


        #get the position and orientation of the drone
        position, orientation = p.getBasePositionAndOrientation(droneId)
        y, x, z = position
        pitch, roll, yaw = p.getEulerFromQuaternion(orientation)

        x *= -1
        y *= -1
        roll *= -1
        yaw *= -1

        camera_target_position = position
        p.resetDebugVisualizerCamera(cameraDistance=1,
                             cameraYaw=camera_yaw,
                             cameraPitch=camera_pitch,
                             cameraTargetPosition=camera_target_position)


        
        #print(f"i: {i}")
       #i+= dt
        # Slow down the simulation to make it visible
        #time.sleep(dt)
 
    p.disconnect()







    
    return z_result

def tricopterSim():

    #trans_control_x = PIDController(0.19816292661139326, 0.01066284301894934, 0.2069431760457202, dt)
    #rot_control_pitch = PIDController(0.30194848389591544, 0.0001296937711999009, 1.20871843369099, dt)

    #trans_control_x = PIDController(4.127457134181349, -4.398200938676669e-06, 0.103571025731448, dt)
    #rot_control_pitch = PIDController(6.188328298297597, 1.0298904898311492, -0.00015437334853366663, dt)

    trans_control_x = PIDController(4.127457134181349, 4.398200938676669e-06, 0.103571025731448, dt)
    #rot_control_pitch = PIDController(7.188328298297597, 0.0298904898311492, -0.00015437334853366663, dt)

    rot_control_pitch = PIDController(0.08,0,0.04 , dt)

    #trans_control_y = PIDController(0.21427750964023945, 0.12552874150003993, 0.3236196057877744)
    #rot_control_roll = PIDController(0.2794641446035615, -0.017086307242385333, 0.801853093493018)

    trans_control_y = PIDController(0.1,0,1, dt)
    #rot_control_roll = PIDController(0.818049163458644, 0, 2.00011980882849824745, dt)

    rot_control_roll = PIDController(0.08,0,0.04, dt)

    #trans_control_z = PIDController(1.0021720967289238, 0.04369073411746757, 4.1877918692437355)
    #trans_control_z = PIDController(0.01,0,0.1, dt)

    trans_control_z = PIDController(0.08,0.02,1, dt)




    rot_control_yaw = PIDController(0.08954740703540659, 0.009507839547302766, 0.411545168858215, dt)





    mass = 0.15021955564014492 + 0.04060892541828371 + 0.04060892541828371

    drone = Quadcopter(0.023021955564014493, 0.33, 9.81, 0.02, k_d = 0.0) #0.3119363164318645, 0.33, 9.81, 0.02

    # grafik
    p.connect(p.GUI)

    # ingen grafik
    #p.connect(p.DIRECT)


    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set up PyBullet simulation environment
    p.setGravity(0, 0, -drone.gravity)

    #Load the URDF file
    planeId = p.loadURDF("plane.urdf")




    

    o_target_x = 0
    o_target_y = 0
    target_z = 1
    target_roll = 0
    target_pitch = 0
    target_yaw = 0
  



    x = 0
    y = 0
    z = 0
    roll = 0
    pitch = 0
    yaw = 0#-np.pi/2
 

    # Load the drone URDF file
    droneId = p.loadURDF("Simulation/drone.urdf", [x, y, z], p.getQuaternionFromEuler([roll, pitch, yaw]))

    # Get the joint indices for the motors
    num_joints = p.getNumJoints(droneId)
    motor_joints = []

    wing_joint_index = -1
    for jointIndex in range(num_joints):
        joint_info = p.getJointInfo(droneId, jointIndex)
        joint_name = joint_info[1].decode("utf-8")  # Convert bytes to string
        #print(joint_name)  # Print joint names for inspection
        if "motor" in joint_name:
            motor_joints.append(jointIndex)
        elif "Revolute_3" in joint_name:
            wing_joint_index = jointIndex

    #revolute 3 = tail wing
    #revolute 4 = right wing
    #revolute 5 = left wing


    omega_1 = 0
    omega_2 = 0
    omega_3 = 0
    alpha = 0

    omega_1_result = []
    omega_2_result = []
    omega_3_result = []
    alpha_result = []

    x_result = []
    y_result = []
    z_result = []
    roll_result = []
    pitch_result = []
    yaw_result = []


    U_x_result = []
    U_y_result = []
    U_z_result = []
    U_pitch_result = []
    U_roll_result = []
    U_yaw_result = [] 


    """ # Create visual shape for the drone
    drone_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                            fileName="Tricopter.stl",
                                            meshScale=[0.01, 0.01, 0.01])

    # Create multi-body with visual shape only
    drone_id = p.createMultiBody(baseMass=1,
                                basePosition=start_position,
                                baseOrientation=start_orientation,
                                baseVisualShapeIndex=drone_visual_shape) """

    # Set up camera position and orientation
    camera_target_position = [0, 0, 0]
    camera_distance = 10
    camera_yaw = 90
    camera_pitch = -45#-90
    camera_roll = 0
    up_axis_index = 2
    pitch_p = pitch
    roll_p = roll
    yaw_p = yaw
    camera_position = [camera_distance, 0, 0]
    p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                cameraYaw=camera_yaw,
                                cameraPitch=camera_pitch,
                                cameraTargetPosition=camera_target_position)


    error_result = []
    forces = [0,0,0]
    test_rot = 0
    i = 0
    sim_time = 1000*dt
    while i < sim_time:



        z_error = (target_z - z) #+ np.random.normal(0, 0.01)
        U_z = trans_control_z.calculate(-(z_error)) 

        

        U_z -= drone.mass * drone.gravity  

        if U_z > 0:
            U_z = 0

        print(f"z_error: {z_error}, U_z: {U_z}")
   
        global_x_error = (o_target_x - x) #+ np.random.normal(0, 0.01)
        global_y_error = (o_target_y - y) #+ np.random.normal(0, 0.01)


        x_error = np.cos(yaw) * global_x_error + np.sin(yaw) * global_y_error
        #print(f"x_error: {x_error}")
        U_x = trans_control_x.calculate(-x_error) 
        
        U_x = 0

        pitch_error = U_x - pitch #+ np.random.normal(0, 0.001)*np.pi/180
        U_pitch = rot_control_pitch.calculate(pitch_error) 


        #y_error = (target_y - y) + np.random.normal(0, 0.01)
        y_error = -np.sin(yaw) * global_x_error + np.cos(yaw) * global_y_error
        #print(f"y_error: {y_error}")
        U_y = trans_control_y.calculate(y_error) #- y_vel * 0.1

        U_y = 0

        roll_error = U_y - roll #+ np.random.normal(0, 0.001)*np.pi/180
        U_roll = rot_control_roll.calculate(-roll_error) #- roll_vel * 0.1


        yaw_error = target_yaw - yaw #+ np.random.normal(0, 0.001)*np.pi/180
        U_yaw = rot_control_yaw.calculate(yaw_error)

    

        #U_z = -drone.mass * drone.gravity
        #U_z = -drone.mass * drone.gravity
        #U_x = 0
        #U_y = 0
        #U_pitch = 0
        #U_roll = 0
        U_yaw = 0


        

        """ term1_12 = (2 * drone.l_0 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2))
        term2_12 = (2 * U_roll) / (drone.l_1 * drone.k_t)
        term3_12 = (2 * U_pitch) / (drone.k_t * (drone.l_0 + drone.l_2)) """

        term1_12 = (drone.l_0 * U_z) / (2*drone.k_t * (drone.l_0 + drone.l_2))
        term2_12 = (U_roll) / (2*drone.l_1 * drone.k_t)
        term3_12 = (U_pitch) / (2*drone.k_t * (drone.l_0 + drone.l_2))

        omega_1_mid = -term1_12 - term2_12 + term3_12
        omega_2_mid = -term1_12 + term2_12 + term3_12

        omega_1 = 0 if omega_1_mid < 0 else np.sqrt(omega_1_mid) 
        omega_2 = 0 if omega_2_mid < 0 else np.sqrt(omega_2_mid) 

      

        #omega_1 = np.sqrt(omega_1_mid) / 2
        #omega_2 = np.sqrt(omega_2_mid) / 2

        term1_3p1 = -(drone.l_2 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2))
        term1_3p2 = -U_pitch / (drone.k_t * (drone.l_0 + drone.l_2))
        term1_3 = (term1_3p1 + term1_3p2) ** 2


        print(f"term1_3p1: {term1_3p1}")
        print(f"term1_3p2: {term1_3p2}")
        print(f"term1_3: {term1_3}")


        term2_3p1 = -(drone.k_d * U_z) / (drone.k_t * drone.k_t * drone.l_0)
        term2_3p2 = U_yaw / (drone.l_0 * drone.k_t)
        term2_3 = (term2_3p1 + term2_3p2) ** 2

        print(f"term2_3p1: {term2_3p1}")
        print(f"term2_3p2: {term2_3p2}")
        print(f"term2_3: {term2_3}")

        omega_3 = (term1_3 + term2_3) ** (0.25)



        alpha_term1 = -(drone.k_d * U_z) / (drone.k_t * drone.k_t * drone.l_0) + U_yaw / (drone.l_0 * drone.k_t)
        alpha_term2 = (drone.l_2 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2)) - U_pitch / (drone.k_t * (drone.l_0 + drone.l_2))
        alpha = np.arctan(alpha_term1 / alpha_term2)

        print(f"alpha: {alpha}")


        forces[0] = drone.k_t * omega_1 ** 2 
        forces[1] = drone.k_t * omega_2 ** 2 
        forces[2] = drone.k_t * omega_3 ** 2  #*np.cos(alpha) 

        print(f"forces: {forces}")

        torque1 = drone.k_d * (omega_1 ** 2 + omega_2 ** 2 + omega_3 ** 2 * np.cos(alpha))  #+ 4

 


        # Set tilt angle for the wing
        p.setJointMotorControl2(droneId, wing_joint_index, p.POSITION_CONTROL, targetPosition=(alpha))

        
        #left motor
        p.applyExternalForce(objectUniqueId=droneId, linkIndex=motor_joints[2], forceObj=[0, 0, forces[0]], posObj=[0, 0, 0], flags=p.LINK_FRAME)

        #tail motor
        p.applyExternalForce(objectUniqueId=droneId, linkIndex=motor_joints[0], forceObj=[0, 0, forces[2]], posObj=[0, 0, 0], flags=p.LINK_FRAME)

        #right motor
        p.applyExternalForce(objectUniqueId=droneId, linkIndex=motor_joints[1], forceObj=[0, 0, forces[1]], posObj=[0, 0, 0], flags=p.LINK_FRAME)


        # add the torque generated by the motors to the center of mass
        p.applyExternalTorque(objectUniqueId=droneId, linkIndex=-1, torqueObj=[0, 0, torque1], flags=p.LINK_FRAME)




            # Step the simulation
        #p.setRealTimeSimulation(1)
        p.stepSimulation()


        error_result.append([[x_error, y_error, z_error, roll_error, pitch_error, yaw_error]])

        x_result.append([x_error,i])
        y_result.append([y_error,i])
        z_result.append([z_error,i])
        roll_result.append([roll_error,i])
        pitch_result.append([pitch_error,i])
        yaw_result.append([yaw_error,i])

        U_x_result.append([U_x,i])
        U_y_result.append([U_y,i])
        U_z_result.append([U_z,i])
        U_pitch_result.append([U_pitch,i])
        U_roll_result.append([U_roll,i])
        U_yaw_result.append([U_yaw,i])


        omega_1_result.append([omega_1,i])
        omega_2_result.append([omega_2,i])
        omega_3_result.append([omega_3,i])
        alpha_result.append([alpha,i])


        """ #get the position and orientation of the drone
        position, orientation = p.getBasePositionAndOrientation(droneId)
        y, x, z = position
        pitch, roll, yaw = p.getEulerFromQuaternion(orientation) """


        position, orientation = p.getBasePositionAndOrientation(droneId)
        y, x, z = position
        #local_orientation = (-orientation[0], -orientation[1], -orientation[2], orientation[3])
        #pitch, roll, yaw = p.getEulerFromQuaternion(local_orientation)  

        # Initialize total_pitch, total_roll, total_yaw to 0 outside your main loop


        # Then, inside your main loop:
        old_pitch, old_roll, old_yaw = pitch_p, roll_p, yaw_p
        local_orientation = (-orientation[0], -orientation[1], -orientation[2], orientation[3])
        pitch_p, roll_p, yaw_p = p.getEulerFromQuaternion(local_orientation)

        # Calculate the differences in the angles
        delta_pitch = pitch_p - old_pitch
        delta_roll = roll_p - old_roll
        delta_yaw = yaw_p - old_yaw

        # If the change is greater than pi, it means we've wrapped around, so subtract 2*pi
        if delta_pitch > np.pi:
            delta_pitch -= 2 * np.pi
        elif delta_pitch < -np.pi:
            delta_pitch += 2 * np.pi

        if delta_roll > np.pi:
            delta_roll -= 2 * np.pi
        elif delta_roll < -np.pi:
            delta_roll += 2 * np.pi

        if delta_yaw > np.pi:
            delta_yaw -= 2 * np.pi
        elif delta_yaw < -np.pi:
            delta_yaw += 2 * np.pi

        # Add the changes to the total
        pitch += delta_pitch
        roll += delta_roll
        yaw += delta_yaw


        x *= -1
        y *= -1
        roll *= -1
        yaw *= -1

        camera_target_position = position
        p.resetDebugVisualizerCamera(cameraDistance=1,
                             cameraYaw=camera_yaw,
                             cameraPitch=camera_pitch,
                             cameraTargetPosition=camera_target_position)


        
        #print(f"i: {i}")
        i+= dt
        # Slow down the simulation to make it visible
        time.sleep(dt)
 
    p.disconnect()







    
    return error_result

if __name__ == "__main__":

    tricopterSim()
    exit()

    initial_guess = [0.1,0,0.0,0,0,0]
    #controller = PIDController(*initial_guess)
    tuned_params = pid_autotune2(tricopterSimPID,initial_guess, time=1.0)
    tuned_kp1, tuned_ki1, tuned_kd1, tuned_kp2, tuned_ki2, tuned_kd2 = tuned_params
    #tuned_kp1, tuned_ki1, tuned_kd1 = tuned_params
    print(f"Tuned PID parameters1: {tuned_kp1}, {tuned_ki1}, {tuned_kd1}")
    print(f"Tuned PID parameters2: {tuned_kp2}, {tuned_ki2}, {tuned_kd2}")
 

    #save the parameters to a .txt file
    file = open("pid_params.txt", "w")
    file.write(f"Tuned PID parameters1: {tuned_kp1}, {tuned_ki1}, {tuned_kd1}")
    file.write(f"Tuned PID parameters2: {tuned_kp2}, {tuned_ki2}, {tuned_kd2}")


    """ values = [0.08935726607591638, 0.009453101478065287, 0.41156854675899196]
    test(values) """

    #test2([0.1965757725903785, 0.010342614229023009, 0.2051560043445344, 0.5, 0, 1.2])


    #tricopter_flight_controller()

    #test3()

# z-axis: Tuned PID parameters: Kp=2.331951430168105, Ki=-0.03521780216430419, Kd=0.022501130417111208
#1.331951430168105,0.03521780216430419,1.02250113041711120
#0.9545651916204778, Ki=0.04441356161831689, Kd=4.0843239598328
         
# roll-axis: Tuned PID parameters: Kp=-1.0433619536316832, Ki=0.04656576063946878, Kd=3.8059764298859196