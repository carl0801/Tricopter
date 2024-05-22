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
dt = 0.005

def rpyToQuat(r,p,y):
    # Convert roll, pitch, yaw to quaternion. quat is in the form [x,y,z,w]
    cy = np.cos(y * 0.5)
    sy = np.sin(y * 0.5)
    cp = np.cos(p * 0.5)
    sp = np.sin(p * 0.5)
    cr = np.cos(r * 0.5)
    sr = np.sin(r * 0.5)

    q = np.zeros(4)
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def quatError(q1, q2):
    """
    Calculate the error quaternion between two quaternions.

    Parameters:
    q1 (tuple): A tuple representing the first quaternion in the format (x, y, z, w).
    q2 (tuple): A tuple representing the second quaternion in the format (x, y, z, w).

    Returns:
    tuple: The error quaternion in the format (x, y, z, w).
    """
    # Error quaternion = q2 * inverse(q1)
    inv_q1 = (-q1[0], -q1[1], -q1[2], q1[3])  # Inverse of q1
    error_q = (q2[3]*inv_q1[0] + q2[0]*inv_q1[3] + q2[1]*inv_q1[2] - q2[2]*inv_q1[1],  # x
               q2[3]*inv_q1[1] - q2[0]*inv_q1[2] + q2[1]*inv_q1[3] + q2[2]*inv_q1[0],  # y
               q2[3]*inv_q1[2] + q2[0]*inv_q1[1] - q2[1]*inv_q1[0] + q2[2]*inv_q1[3],  # z
               q2[3]*inv_q1[3] - q2[0]*inv_q1[0] - q2[1]*inv_q1[1] - q2[2]*inv_q1[2])  # w

    return error_q


    
    return q_error


def multiply_quaternions(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return [x, y, z, w]

def invertQuaternion(q):
    return [-q[0], -q[1], -q[2], q[3]]

def multiplyQuaternions(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return [x, y, z, w]

def global_to_local(global_x, global_y, yaw):
    # Rotate clockwise by yaw
    yaw_rad = math.radians(yaw)
    local_x = global_y * math.cos(yaw_rad) - global_x * math.sin(yaw_rad)
    local_y = global_y * math.sin(yaw_rad) + global_x * math.cos(yaw_rad)
    
    # Invert signs to match local coordinate system
    return local_x, local_y


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
        self.l_1 = abs(np.sin(60 * np.pi / 180) * l_0)
        #self.l_2 = abs(np.sin(-60 * np.pi / 180) * l_0)
        self.l_2 = np.cos(60 * np.pi / 180) * l_0
        #print(f"l_1: {self.l_1}, l_2: {self.l_2}")

def tricopterSim():

    trans_control_x = PIDController(0.1,0,1, dt)

    trans_control_y =  PIDController(0.1,0,1, dt)

    trans_control_z = PIDController(0.1,0,0.1, dt)




    mass = 0.15021955564014492 + 0.04060892541828371 + 0.04060892541828371

    drone = Quadcopter(0.3332610324554788, 0.33, 9.81, 0.02, k_d = 0.0000001) #0.3119363164318645, 0.33, 9.81, 0.02
    

    

    p.connect(p.GUI)
    print("simulating with graphics")


    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set up PyBullet simulation environment
    p.setGravity(0, 0, -drone.gravity)


    #load the plane
    #planeId = p.loadURDF("plane.urdf")






    

    o_target_x = 0
    o_target_y = 0
    target_z = 1
    #make a target orientation as a quaternion
    qd = rpyToQuat(0,0,0)
    qd = [qd[1], qd[2], qd[3], qd[0]]
  
    x = 0
    y = 0
    z = 0
    local_quat = p.getQuaternionFromEuler([0, 0, 0])
 
    # Load the drone URDF file
    droneId = p.loadURDF("Simulation/drone.urdf", [x, y, z], local_quat)

   
    #calculate the inertia of the drone
    """ total_inertia = [0, 0, 0]
    for i in range(-1, p.getNumJoints(droneId)):
        mass, lateral_friction, inertia, local_inertial_pos, local_inertial_orn, restitution, rolling_friction, spinning_friction, contact_damping, contact_stiffness, body_type, collision_margin = p.getDynamicsInfo(droneId, i)
        # Calculate the distance from the center of mass of the drone to the center of mass of the part
        distance = np.linalg.norm(local_inertial_pos)
        # Use the parallel axis theorem to calculate the moment of inertia of the part about the center of mass of the drone
        inertia_about_drone_com = [ii + mass * distance**2 for ii in inertia]
        # Add this to the total moment of inertia
        total_inertia = [ii + jj for ii, jj in zip(total_inertia, inertia_about_drone_com)]
    print(f"total_inertia {total_inertia}") """
    #exit()


    #calculate the total mass of the drone
    total_mass = 0
    for i in range(p.getNumJoints(droneId)):
        total_mass += p.getDynamicsInfo(droneId, i)[0]
 
    print(f"total_mass: {total_mass}")

    # Get the joint indices for the motors
    num_joints = p.getNumJoints(droneId)
    motor_joints = []

    tailWing = 0
    rightWing = 0
    leftWing = 0
    for jointIndex in range(num_joints):
        joint_info = p.getJointInfo(droneId, jointIndex)
        joint_name = joint_info[1].decode("utf-8")  # Convert bytes to string
        #print(joint_name)  # Print joint names for inspection
        if "motor" in joint_name:
            motor_joints.append(jointIndex)
        elif "Revolute_3" in joint_name:
            tailWing = jointIndex
        elif "Revolute_4" in joint_name:
            rightWing = jointIndex
        elif "Revolute_5" in joint_name:
            leftWing = jointIndex

    #revolute 3 = tail wing
    #revolute 4 = right wing
    #revolute 5 = left wing


    omega_1 = 0
    omega_2 = 0
    omega_3 = 0
    alpha = 0


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
    p.setTimeStep(dt)

    error_result = []
    forces = [0,0,0]
    test_rot = 0
    i = 0
    sim_time = 10000*dt

    # create a M matrix which is a 6x6 matrix

    l_0 = 0.33
    l_1 = abs(np.sin(60 * np.pi / 180) * l_0)
    l_2 = np.cos(60 * np.pi / 180) * l_0
    k_t1 = 5.369e-7 
    #k_t2 = 5.115e-7 
    #k_t3 = 5.485e-7 
    k_t2 = k_t1
    k_t3 = k_t1
    k_d = 5.295e-8

    M = np.array([
    [-1/2*np.sqrt(3)*k_t1, 1/2*np.sqrt(3)*k_t2, 0, 0, 0, 0],
    [1/2*k_t1, 1/2*k_t2, -k_t3, 0, 0, 0],
    [0, 0, 0, -k_t1, -k_t2, -k_t3],
    [-1/2*np.sqrt(3)*k_d, 1/2*np.sqrt(3)*k_d, 0, -l_1*k_t1, l_1*k_t2, 0],
    [1/2*k_d, 1/2*k_d, -k_d, 1/2*l_0*k_t1, 1/2*l_0*k_t2, -l_0*k_t3],
    [l_0*k_t1, l_0*k_t2, l_0*k_t3, -k_d, -k_d, -k_d]
    ])

    M_inverse = np.linalg.inv(M)

    U = np.array([0,0,0,0,0,0])
    

    U_z = 0
    while i < sim_time:
        target_z += 0.001
        position,q = p.getBasePositionAndOrientation(droneId)
        x,y,z = position        

        local_quat = invertQuaternion(q)

        #print(f"local rpy: {p.getEulerFromQuaternion(local_quat)}")

        yaw = p.getEulerFromQuaternion(local_quat)[2]
        #print(f"local_quat: {local_quat}")
        # Convert the current orientation to the drone's local frame

        #print(f"q: {q}")

        z_error = (target_z - z) #+ np.random.normal(0, 0.01)
 
        U[2] = trans_control_z.calculate(-(z_error)) 

        

        U[2] -= drone.mass * drone.gravity


        global_x_error = (o_target_x - x) #+ np.random.normal(0, 0.01)
        global_y_error = (o_target_y - y) #+ np.random.normal(0, 0.01)

        """ x_error = np.cos(yaw) * global_x_error + np.sin(yaw) * global_y_error
        y_error = -np.sin(yaw) * global_x_error + np.cos(yaw) * global_y_error """
        y_error = np.cos(yaw) * global_x_error + np.sin(yaw) * global_y_error
        x_error = -np.sin(yaw) * global_x_error + np.cos(yaw) * global_y_error

        x_error, y_error = global_to_local(global_x_error, global_y_error, yaw)

        #print(f"x_error: {x_error}")
        U[0] = trans_control_x.calculate(-x_error) 
        
        #U_x = 0




        #y_error = (target_y - y) + np.random.normal(0, 0.01)
        
        
        #print(f"y_error: {y_error}")
        U[1] = trans_control_y.calculate(-y_error) #- y_vel * 0.1

        #print(f"x_error: {x_error}, y_error: {y_error}")


        
    




        
        #q = (-q[0], -q[1], -q[2], q[3])
        #get the conjugate of the current orientation
        #q = [-q[0], -q[1], -q[2], q[3]]
        _, curr_angular_velocity = p.getBaseVelocity(droneId)

        # Convert the current orientation quaternion to a rotation matrix
        rotation_matrix = np.array(p.getMatrixFromQuaternion(local_quat)).reshape(3, 3)

        # Convert the angular velocity to the local frame
        local_angular_velocity = np.dot(rotation_matrix.T, curr_angular_velocity)
        
        #print(f"roll, pitch, yaw: {p.getEulerFromQuaternion(q)}")
        q_error = quatError(qd, local_quat)

        curr_angular_velocity = [curr_angular_velocity[1], curr_angular_velocity[2], curr_angular_velocity[0]]

        values = [1, 1, 0, 2,2,0]
        quat_p_outer = np.array([[values[0],0,0],
                                [0, values[1], 0],
                                [0,0,values[2]]]) 

        quat_p_inner = np.array([[values[3],0,0],
                                [0, values[4], 0],
                                [0,0,values[5]]])

        print(f"x_error: {x_error}, y_error: {y_error}, z_error: {z_error}")
        print(f"q_error: {q_error}")
        
        # Compute the control input
        pid = -quat_p_outer @ q_error[0:3] * np.sign(q_error[3]) - quat_p_inner @ local_angular_velocity
        

        U[3] = pid[0]
        U[4] = pid[1]
        U[5] = pid[2]

        #set the last 3 values to 0
        U[0] = 0.0
        U[1] = 0.0
        #U[2] = 0.0 
        """ U[3] = 0.0
        U[4] = 0.0
        U[5] = 0.0 """

        

        print(f"U: {U}")

        print(f" yaw: {yaw}")
        
        #U_yaw += torque

        #print(f"U_z: {U_z}")
        #U_z = -drone.mass * drone.gravity
        #print(f"U_z: {U_z}")
        #U_x = 0
        #U_y = 0
        #U_roll = 0
        #U_pitch = 0
        #U_yaw = 0
        


        Omega = M_inverse @ U
        print(f"Omega: {Omega}")


        omega_1 = (Omega[0]**2 + Omega[3]**2)**(1/4)
        omega_2 = (Omega[1]**2 + Omega[4]**2)**(1/4)
        omega_3 = (Omega[2]**2 + Omega[5]**2)**(1/4)

        
        if Omega[3] == 0:
            alpha_1 = 0
        else:   
            alpha_1 = -np.arctan(Omega[0] / (Omega[3]))
        
        if Omega[4] == 0:
            alpha_2 = 0
        else:
            alpha_2 = -np.arctan(Omega[1] / (Omega[4]))
        
        if Omega[5] == 0:
            alpha_3 = 0
        else:
            alpha_3 = -np.arctan(Omega[2] / (Omega[5]))



        #print(f"omega_1: {omega_1}, omega_2: {omega_2}, omega_3: {omega_3}")
        #print(f"alpha_1: {alpha_1}, alpha_2: {alpha_2}, alpha_3: {alpha_3}")


        forces[0] = -k_t1 * omega_1 ** 2 
        forces[1] = -k_t2 * omega_2 ** 2 
        forces[2] = -k_t3 * omega_3 ** 2  #*np.cos(alpha) 

        

        

        



        # Set tilt angle for the wings
        p.setJointMotorControl2(droneId, rightWing, p.POSITION_CONTROL, targetPosition=(alpha_1))
        p.setJointMotorControl2(droneId, leftWing, p.POSITION_CONTROL, targetPosition=(alpha_2))
        p.setJointMotorControl2(droneId, tailWing, p.POSITION_CONTROL, targetPosition=(alpha_3))

        
        #left motor
        p.applyExternalForce(objectUniqueId=droneId, linkIndex=motor_joints[2], forceObj=[0, 0, forces[1]], posObj=[0, 0, 0], flags=p.LINK_FRAME)

        #tail motor
        p.applyExternalForce(objectUniqueId=droneId, linkIndex=motor_joints[0], forceObj=[0, 0, forces[2]], posObj=[0, 0, 0], flags=p.LINK_FRAME)

        #right motor
        p.applyExternalForce(objectUniqueId=droneId, linkIndex=motor_joints[1], forceObj=[0, 0, forces[0]], posObj=[0, 0, 0], flags=p.LINK_FRAME)

        torque = -k_d * (omega_1**2 + omega_2**2 + omega_3**2)
        
        p.applyExternalTorque(objectUniqueId=droneId, linkIndex=-1, torqueObj=[0, 0, torque], flags=p.LINK_FRAME)

        



            # Step the simulation
        #p.setRealTimeSimulation(1)
        p.stepSimulation()


                

        camera_target_position = position
        p.resetDebugVisualizerCamera(cameraDistance=1,
                            cameraYaw=camera_yaw,
                            cameraPitch=camera_pitch,
                            cameraTargetPosition=camera_target_position)


        error_result.append([q_error[0:3]]) 
        #print(f"i: {i}")

        i+= dt
        # Slow down the simulation to make it visible
        time.sleep(dt/10)

    p.disconnect()



    
    return error_result

if __name__ == "__main__":
    tricopterSim()