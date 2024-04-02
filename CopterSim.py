import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import minimize
from spatialmath import SE3, SO3, UnitQuaternion
from spatialmath.base.quaternions import *

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
    def __init__(self, mass, l_0, gravity, drag, j_x = 0.05, j_y = 0.05, j_z = 0.02, k_t =0.00001 , k_d = 0.0000003 ):  #k_d = 0.0003
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


def tricopter_flight_controller():
    
    trans_control_x = PIDController(0.2, 0.01, 0.2, dt)
    rot_control_pitch = PIDController(0.3, 0, 1.2, dt)


    trans_control_y = PIDController(0.2, 0.01, 0.2)
    rot_control_roll = PIDController(0.5, 0, 1.2)

    trans_control_z = PIDController(0.9545651916204778,0.04441356161831689,4.0843239598328)


    rot_control_yaw = PIDController(0.08508958604791192, 0.00923672770996732, 0.4286804507510326)




    drone = Quadcopter(0.25, 0.25, 9.81, 0.02)



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
    i = 0
    time = 10000*dt
    while i < time:
        #Position error


        #calculate target x and y to move the drone using yaw angle
        #At yaw = 0, x is forward and y is right
        #target_x = np.cos(target_yaw) * o_target_x - np.sin(target_yaw) * o_target_y
        #target_y = np.sin(target_yaw) * o_target_x + np.cos(target_yaw) * o_target_y

        target_x = np.cos(yaw) * o_target_x - np.sin(yaw) * o_target_y
        target_y = np.sin(yaw) * o_target_x + np.cos(yaw) * o_target_y

    
        
        #target_x += 0.001
        #target_z -= 0.001

        #target_z = -1

        """ if i < 100*dt:
            U_z =  -drone.mass * drone.gravity * 1.01
        elif i < 200*dt:
            U_z =  -drone.mass * drone.gravity * 1.08
        else:
            #Altitude error
            target_z = i
            target_x = 2*np.sin(np.pi/30 * i)
            target_y = 2*np.sin(2*np.pi/30 * i - np.pi/2)
            target_yaw = np.sin(2*np.pi/30 * i)

            z_error = (target_z - z) + np.random.normal(0, 0.01)
            U_z = trans_control_z.calculate((z_error)) #- z_vel * 3
            #U_z -= z_vel * 3

            U_z -= drone.mass * drone.gravity  """



        target_z = -i
        target_x = np.sin(4*np.pi/30 * i)
        target_y = 2*np.sin(8*np.pi/30 * i)
        target_yaw = np.sin(8*np.pi/30 * i)

        z_error = (target_z - z) + np.random.normal(0, 0.01)
        U_z = trans_control_z.calculate((z_error)) #- z_vel * 3
        #U_z -= z_vel * 3

        U_z -= drone.mass * drone.gravity  

    
        


        #print(f"yaw_vel: {yaw_vel}")
        #print(f"target_z: {target_z}")
        #print(stopped_spinning)
        #add some noise to the errors
        x_error = (target_x - x) + np.random.normal(0, 0.01)
        #print(f"x_error: {x_error}")
        U_x = trans_control_x.calculate(x_error) #- x_vel * 0.1

        pitch_error = U_x - pitch + np.random.normal(0, 0.01)
        U_pitch = rot_control_pitch.calculate(pitch_error) #- pitch_vel * 0.1


        y_error = (target_y - y) + np.random.normal(0, 0.01)
        #print(f"y_error: {y_error}")
        U_y = trans_control_y.calculate(y_error) #- y_vel * 0.1

        roll_error = U_y - roll + np.random.normal(0, 0.01)
        U_roll = rot_control_roll.calculate(roll_error) #- roll_vel * 0.1






        yaw_error = target_yaw - yaw + np.random.normal(0, 0.01)*np.pi/180
        U_yaw = rot_control_yaw.calculate(yaw_error)

    




        #print(f"U_z: {U_z}")
        """ if i < 100*dt:
            U_z =  -drone.mass * drone.gravity * 1.0
        else:
            target_x = 0.0
            target_z = 0
            #print(f"U_z = {U_z} - {drone.mass * drone.gravity}")
            U_z = U_z - drone.mass * drone.gravity
            #print(f"U_z = {U_z} + {drone.mass} * {drone.gravity}") """


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


        """ print(f"roll: {roll*180/np.pi} pitch: {pitch*180/np.pi} yaw: {yaw*180/np.pi}")
        print(f"y_acc1: {f_y * (np.sin(roll) * np.sin(pitch) * np.sin(yaw) + np.cos(roll) * np.cos(yaw))} y_acc2: {U_z * (np.sin(pitch) * np.cos(roll) * np.sin(yaw) - np.sin(roll) * np.cos(yaw))}")
        print(f"y_acc: {y_acc}") """
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
        alpha_result.append([alpha*180/np.pi -180,i])

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

    #save the errors to sim.csv
    df = pd.DataFrame(pos_result, columns = ['pos', 'time'])
    df.to_csv('sim.csv', index=False)
    """ #plot u_roll_result
    fig, axs = plt.subplots(4)

    axs[0].plot([data[1] for data in x_result], [data[0] for data in x_result])
    axs[0].set_title('x')

    axs[1].plot([data[1] for data in pitch_result], [data[0] for data in pitch_result])
    axs[1].set_title('pitch')

    axs[2].plot([data[1] for data in U_x_result], [data[0] for data in U_x_result])
    axs[2].set_title('U_x')

    axs[3].plot([data[1] for data in U_pitch_result], [data[0] for data in U_pitch_result])
    axs[3].set_title('U_pitch')



    for ax in axs:
        ax.set(xlabel='Time')
    plt.show() """

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

    """ axs[4, 1].plot([data[1] for data in U_x_result], [data[0] for data in U_x_result])
    axs[4, 1].set_title('U_x')

    axs[5, 1].plot([data[1] for data in U_pitch_result], [data[0] for data in U_pitch_result])
    axs[5, 1].set_title('U_pitch') """

    axs[4, 1].plot([data[1] for data in U_y_result], [data[0] for data in U_y_result])
    axs[4, 1].set_title('U_y')

    axs[5, 1].plot([data[1] for data in U_roll_result], [data[0] for data in U_roll_result])
    axs[5, 1].set_title('U_roll')

    """ axs[4, 1].plot([data[1] for data in U_yaw_result], [data[0] for data in U_yaw_result])
    axs[4, 1].set_title('U_yaw') """




    # Set a common label for all subplots
    for ax in axs.flat:
        ax.set(xlabel='Time')

    # Display the figure with subplots
    plt.tight_layout()
    plt.show()

    #run the emil_sim.py
    import emil_sim
    
    return


def plant(u1,u2,t):
    


    drone = Quadcopter(0.25, 0.25, 9.81, 0.02)



    control_z = PIDController(0.9545651916204778,0.04441356161831689,4.0843239598328)


    trans_control_y = PIDController(0.2, 0.01, 0.2)
    rot_control_roll = PIDController(0.5, 0, 1.2)

    trans_control_x = PIDController(0.2, 0.01, 0.2)
    rot_control_pitch = PIDController(0.3, 0, 1.2)
    
    target_x = 1
    target_y = 1
    target_z = -1
    target_roll = 0
    target_pitch = 0
    target_yaw = 1*np.pi/180
    

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

        x_error = (target_x - x)
        U_x = trans_control_x.calculate(x_error)

        pitch_error = U_x - pitch
        U_pitch = rot_control_pitch.calculate(pitch_error)


        y_error = (target_y - y)
        U_y = trans_control_y.calculate(y_error)
        #U_y = 0

        roll_error = U_y - roll
        U_roll = rot_control_roll.calculate(roll_error)

        #U_roll = 0

        yaw_error = target_yaw - yaw
        U_yaw = u1.calculate(yaw_error)

        z_error = (target_z - z)
        U_z = control_z.calculate((z_error)) 
        U_z -= drone.mass * drone.gravity


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

  
 

        x_result.append([x])
        y_result.append([y])
        z_result.append([z])

        roll_result.append([roll*180/np.pi])
        pitch_result.append([pitch*180/np.pi])
        yaw_result.append([yaw*180/np.pi])


    #return roll_result as vector
    return np.array(yaw_result)

        

def pid_autotune(simulate_system, controller, initial_guess, time):
    # Simulate system
    t = np.linspace(0, time, num=1000)  # Adjust as needed
    #y_desired = np.ones_like(t)  # Desired response, assuming step input
    y_desired = np.zeros_like(t)  # Desired response, assuming step input
    
    # Define cost function
    def cost_function(params):
        controller.kp, controller.ki, controller.kd = params
        y_actual = simulate_system(controller, t)
        #print(f"y_actual: {y_actual[10]}, y_desired: {y_desired[10]}")
        return np.sum(np.square(y_actual - y_desired))

    # Perform optimization
    result = minimize(cost_function, initial_guess, method='Nelder-Mead')

    return result.x
    
def pid_autotune2(simulate_system, initial_guess, time):
    # Simulate system
    t = np.linspace(0, time, num=int(time/0.01))  # Adjust as needed
    y_desired = np.ones_like(t)  # Desired response, assuming step input
    
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

    

def test(u1):

    t = np.linspace(0, 10, num=1000)
    y_actual = plant(u1,t)
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



def test3():
    target_roll_rad = 1
    actual_roll_rad = 0

    target_pitch_rad = 1
    actual_pitch_rad = 0

    target_yaw_rad = 1
    actual_yaw_rad = 0

    # Convert Euler angles to rotation matrices
    target_rotation = SO3.RPY(target_roll_rad, target_pitch_rad, target_yaw_rad)
    actual_rotation = SO3.RPY(actual_roll_rad, actual_pitch_rad, actual_yaw_rad) 

    # Convert rotation matrices to quaternions
    target_quat = UnitQuaternion(target_rotation)
    actual_quat = UnitQuaternion(actual_rotation)
    
    # Compute the error quaternion
    error_quat = target_quat.inv() * actual_quat

    # convert error_quat to radians
    error_rpy = error_quat.rpy()

    print(f"error_rpy: {error_rpy}")

   

    print(f"error_rpy: {error_quat}")

    
    



    
 


if __name__ == "__main__":

    """ initial_guess = [0.00011056855858392918, 0, 0.00127068547473603857, 0, 0, 0]
    controller = PIDController(*initial_guess)
    tuned_params = pid_autotune2(plant,initial_guess, time=100.0)
    tuned_kp1, tuned_ki1, tuned_kd1, tuned_kp2, tuned_ki2, tuned_kd2 = tuned_params
    #tuned_kp1, tuned_ki1, tuned_kd1 = tuned_params
    print(f"Tuned PID parameters1: Kp={tuned_kp1}, Ki={tuned_ki1}, Kd={tuned_kd1}")
    print(f"Tuned PID parameters2: Kp={tuned_kp2}, Ki={tuned_ki2}, Kd={tuned_kd2}")
    test2(tuned_params)
    #save the parameters to a .txt file
    file = open("pid_params.txt", "w")
    file.write(f"Tuned PID parameters1: {tuned_kp1}, {tuned_ki1}, {tuned_kd1}") """
    #file.write(f"Tuned PID parameters2: {tuned_kp2}, {tuned_ki2}, {tuned_kd2}")

    #test(PIDController(0.09117824183681283, 0.0109722222222222219, 0.11132686783049586))

    #test2([0.2, 0.01, 0.2, 0.5, 0, 1.2])


    tricopter_flight_controller()

    #test3()

# z-axis: Tuned PID parameters: Kp=2.331951430168105, Ki=-0.03521780216430419, Kd=0.022501130417111208
#1.331951430168105,0.03521780216430419,1.02250113041711120
#0.9545651916204778, Ki=0.04441356161831689, Kd=4.0843239598328
         
# roll-axis: Tuned PID parameters: Kp=-1.0433619536316832, Ki=0.04656576063946878, Kd=3.8059764298859196