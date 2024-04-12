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
        self.l_1 = abs(np.sin(60 * np.pi / 180) * l_0)
        #self.l_2 = abs(np.sin(-60 * np.pi / 180) * l_0)
        self.l_2 = np.cos(60 * np.pi / 180) * l_0
        #print(f"l_1: {self.l_1}, l_2: {self.l_2}")
""" 
def quatError(qd,q):
    # Compute the error quaternion as a quarternion
    # the quat is in the form [x,y,z,w]
    q_error = np.zeros(4)
    q_error[0] = qd[3]*q[0] - qd[0]*q[3] - qd[1]*q[2] + qd[2]*q[1]
    q_error[1] = qd[3]*q[1] + qd[0]*q[2] - qd[1]*q[3] - qd[2]*q[0]
    q_error[2] = qd[3]*q[2] - qd[0]*q[1] + qd[1]*q[0] - qd[2]*q[3]
    q_error[3] = qd[3]*q[3] + qd[0]*q[0] + qd[1]*q[1] + qd[2]*q[2]
  """   
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

def tricopterSim(values=None,sim=False):

    if values is not None:
        sim = True

    trans_control_x = PIDController(6,1,1, dt)

    trans_control_y =  PIDController(3,0,1, dt)

    trans_control_z = PIDController(2,0,0, dt)




    mass = 0.15021955564014492 + 0.04060892541828371 + 0.04060892541828371

    drone = Quadcopter(0.3119363164318645, 0.33, 9.81, 0.02, k_d = 0.0000001) #0.3119363164318645, 0.33, 9.81, 0.02
    

    
    if not sim:
        # grafik
        p.connect(p.GUI)
        print("simulating with graphics")
    else:
        # ingen grafik
        p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set up PyBullet simulation environment
    p.setGravity(0, 0, -drone.gravity)

    #Load the URDF file
    if not sim:
        #planeId = p.loadURDF("plane.urdf")
        pass





    

    o_target_x = 0
    o_target_y = 0
    target_z = 0
    #make a target orientation as a quaternion
    qd = rpyToQuat(0,0,0)
  
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
    """ total_mass = 0
    for i in range(p.getNumJoints(droneId)):
        total_mass += p.getDynamicsInfo(droneId, i)[0] """
 


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


    error_result = []
    forces = [0,0,0]
    test_rot = 0
    i = 0
    sim_time = 1000*dt

    

    U_z = 0
    while i < sim_time:
        o_target_y += 0.001
        position,q = p.getBasePositionAndOrientation(droneId)

        x,y,z = position        

        local_quat = invertQuaternion(q)

        #print(f"local rpy: {p.getEulerFromQuaternion(local_quat)}")

        yaw = p.getEulerFromQuaternion(local_quat)[2]
        #print(f"local_quat: {local_quat}")
        # Convert the current orientation to the drone's local frame

        #print(f"q: {q}")

        """ z_error = (target_z - z) #+ np.random.normal(0, 0.01)
        U_z = trans_control_z.calculate(-(z_error)) 

        

        U_z -= drone.mass * drone.gravity  """ 


        global_x_error = (o_target_x - x) #+ np.random.normal(0, 0.01)
        global_y_error = (o_target_y - y) #+ np.random.normal(0, 0.01)

        """ x_error = np.cos(yaw) * global_x_error + np.sin(yaw) * global_y_error
        y_error = -np.sin(yaw) * global_x_error + np.cos(yaw) * global_y_error """
        y_error = np.cos(yaw) * global_x_error + np.sin(yaw) * global_y_error
        x_error = -np.sin(yaw) * global_x_error + np.cos(yaw) * global_y_error

        x_error, y_error = global_to_local(global_x_error, global_y_error, yaw)

        #print(f"x_error: {x_error}")
        U_x = trans_control_x.calculate(-x_error) 
        
        #U_x = 0




        #y_error = (target_y - y) + np.random.normal(0, 0.01)
        
        
        #print(f"y_error: {y_error}")
        U_y = trans_control_y.calculate(-y_error) #- y_vel * 0.1

        #print(f"x_error: {x_error}, y_error: {y_error}")


        
    




        
        #q = (-q[0], -q[1], -q[2], q[3])
        #get the conjugate of the current orientation
        #q = [-q[0], -q[1], -q[2], q[3]]
        _, curr_angular_velocity = p.getBaseVelocity(droneId)

        # Convert the current orientation quaternion to a rotation matrix
        rotation_matrix = np.array(p.getMatrixFromQuaternion(q)).reshape(3, 3)

        # Convert the angular velocity to the local frame
        local_angular_velocity = np.dot(rotation_matrix.T, curr_angular_velocity)
        
        #print(f"roll, pitch, yaw: {p.getEulerFromQuaternion(q)}")
        q_error = quatError(qd, local_quat)


        #values = 0.12666346, 0.08184413, 0.02654659, 0.09935305, 0.14897474, 0.11669337
        if not sim:
            values = [0.09999804543891046,0.0999958690692744,0.10000005600889045,-8.698480525232753e-07,0.0002501605829050988,5.723843841086013e-07]
            values = [0.10005955877587307,0.10052783472445909,1.10000975770710871,-8.701081305169817e-07,0.00026104502776980794,0]
        quat_p = np.array([[values[0],0,0],
                            [0, values[1], 0],
                            [0,0,values[2]]]) 

        quat_d = np.array([[values[3],0,0],
                            [0, values[4], 0],
                            [0,0,values[5]]])


        # Compute the control input
        pid = quat_p * np.sign(q_error[3])* q_error[0:3] #+ quat_d * local_angular_velocity
        #print(f"pid: {pid}")
        U_pitch = pid[0][0]
        U_roll = pid[1][1]
        U_yaw = pid[2][2]
        print(f" yaw: {yaw}, U_yaw: {U_yaw}, q_error: {q_error[2]}")
        
        #U_yaw += torque

        #print(f"U_z: {U_z}")
        U_z = -drone.mass * drone.gravity
        #print(f"U_z: {U_z}")
        #U_x = 0
        #U_y = 0
        #U_roll = 0
        #U_pitch = 0
        #U_yaw = 0
        


        Omega_1 = -np.sqrt(3)*U_x/(3*drone.k_t) + U_y/(3*drone.k_t) - drone.k_d*U_z/(3*drone.l_0*drone.k_t**2) + U_yaw/(3*drone.l_0*drone.k_t)
        Omega_2 = np.sqrt(3)*U_x/(3*drone.k_t) + U_y/(3*drone.k_t) - drone.k_d*U_z/(3*drone.l_0*drone.k_t**2) + U_yaw/(3*drone.l_0*drone.k_t)
        Omega_3 = -(2*U_y)/(3*drone.k_t) - drone.k_d*U_z/(3*drone.l_0*drone.k_t**2) + U_yaw/(3*drone.l_0*drone.k_t)
        Omega_4 = drone.k_d*np.sqrt(3)*U_x/(3*drone.l_0*drone.k_t**2) - drone.k_d*U_y/(3*drone.l_0*drone.k_t**2) - U_z/(3*drone.k_t) - np.sqrt(3)*U_roll/(3*drone.l_0*drone.k_t) + U_pitch/(3*drone.l_0*drone.k_t)
        Omega_5 = -drone.k_d*np.sqrt(3)*U_x/(3*drone.l_0*drone.k_t**2) - drone.k_d*U_y/(3*drone.l_0*drone.k_t**2) - U_z/(3*drone.k_t) + np.sqrt(3)*U_roll/(3*drone.l_0*drone.k_t) + U_pitch/(3*drone.l_0*drone.k_t)
        Omega_6 = (2*drone.k_d*U_y)/(3*drone.l_0*drone.k_t**2) - U_z/(3*drone.k_t) - (2*U_pitch)/(3*drone.l_0*drone.k_t)

        omega_1 = (Omega_1**2 + Omega_4**2)**(1/4)
        omega_2 = (Omega_2**2 + Omega_5**2)**(1/4)
        omega_3 = (Omega_3**2 + Omega_6**2)**(1/4)

        alpha_1 = np.arctan(Omega_1/Omega_4)
        alpha_2 = np.arctan(Omega_2/Omega_5)
        alpha_3 = np.arctan(Omega_3/Omega_6)



        #print(f"omega_1: {omega_1}, omega_2: {omega_2}, omega_3: {omega_3}")
        #print(f"alpha_1: {alpha_1}, alpha_2: {alpha_2}, alpha_3: {alpha_3}")


        forces[0] = drone.k_t * omega_1 ** 2 
        forces[1] = drone.k_t * omega_2 ** 2 
        forces[2] = drone.k_t * omega_3 ** 2  #*np.cos(alpha) 

        

        

        



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

        torque = drone.l_0 * (forces[0]*np.sin(alpha_1) + forces[1]*np.sin(alpha_2) + forces[2]*np.sin(alpha_3)) - drone.k_d * (omega_1**2 * np.cos(alpha_1) + omega_2**2 * np.cos(alpha_2) + omega_3**2 * np.cos(alpha_3))
        # add the torque generated by the motors to the center of mass
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
        if not sim:
            time.sleep(dt/10)

    p.disconnect()



    
    return error_result

def PID_tuner():
    y_desired = np.zeros((1000, 3))  # Desired response, assuming step input 
    initial_guess = [0.09999804543891046,0.0999958690692744,0.10000005600889045,-8.698480525232753e-07,0.0002501605829050988,5.723843841086013e-07]  # Initial guess for gains
    # Define cost function
    def cost_function(params):
        values = params
        
        # Create PID controllers with the given parameters
        #controller1 = PIDController(kp1, ki1, kd1)
        #controller2 = PIDController(kp2, ki2, kd2)
        
        # Simulate system with both controllers
        y_actual = tricopterSim(values = values)
        #print mean squared error
        #print(f"length of y_actual: {len(y_actual)}")
        #print(f"length of y_desired: {len(y_desired)}")
        print(np.sum(np.square(y_actual - y_desired)))
        # Compute the sum of squared errors of the vectors
        return np.sum(np.square(y_actual - y_desired))
        

    # Perform optimization
    result = minimize(cost_function, initial_guess, method='Nelder-Mead')
    print("Optimal gains:")
    #print optimal gains with comma separated
    print(f"{result.x[0]},{result.x[1]},{result.x[2]},{result.x[3]},{result.x[4]},{result.x[5]}")
    # Extract tuned parameters
    return result.x

if __name__ == "__main__":
    tricopterSim()
    #PID_tuner()


    """
    ##### how to transform a global vector to a local vector #####
    # Get the current position and orientation of the object
    position, orientation = p.getBasePositionAndOrientation(droneId)

    # Define the vector in the global coordinate system
    vector_global = [x, y, z]

    # Translate the vector to the object's local frame
    vector_local = [vector_global[0] - position[0], vector_global[1] - position[1], vector_global[2] - position[2]]

    # Get the inverse of the object's orientation
    orientation_inverse = p.invertQuaternion(orientation)

    # Rotate the translated vector to the object's local frame
    vector_local = p.rotateVector(orientation_inverse, vector_local)
    
    
    """