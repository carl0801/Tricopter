import numpy as np
from scipy import signal, optimize
import matplotlib.pyplot as plt


def PID_tuningx():
    # Define the coefficients
    Kp_pitch = 1
    Kd_x = 1
    Kd_pitch = 1
    Ki_pitch = 0
    Kp_x = 1
    Ki_x = 0
    J_yy = 0.004836582647424344

    # Define the coefficients of the transfer function
    numerator_coeffs = [
        -9.81 * Kp_pitch / (-9.81 * Kd_x * Kd_pitch + J_yy),
        Ki_pitch / (-9.81 * Kd_x * Kd_pitch + J_yy),
        Kd_pitch * (-9.81 * Kd_x * Kd_pitch + J_yy),
        Kp_x / (-9.81 * Kd_x * Kd_pitch + J_yy),
        Ki_x / (-9.81 * Kd_x * Kd_pitch + J_yy),
        Kd_x * (-9.81 * Kd_x * Kd_pitch + J_yy)
    ]

    denominator_coeffs = [
        1,
        -9.81 * Kd_x / (-9.81 * Kd_x * Kd_pitch + J_yy),
        -9.81 * Kd_pitch / (-9.81 * Kd_x * Kd_pitch + J_yy),
        -9.81 * Ki_pitch / (-9.81 * Kd_x * Kd_pitch + J_yy),
        -9.81 * Ki_x / (-9.81 * Kd_x * Kd_pitch + J_yy),
        -9.81 * Kp_x / (-9.81 * Kd_x * Kd_pitch + J_yy),
        -9.81 * Kp_pitch / (-9.81 * Kd_x * Kd_pitch + J_yy),
        -9.81 * Ki_x / (-9.81 * Kd_x * Kd_pitch + J_yy),
        -9.81 * Kp_x / (-9.81 * Kd_x * Kd_pitch + J_yy)
    ]

    # Create the transfer function
    sys_tf = signal.TransferFunction(numerator_coeffs, denominator_coeffs)

    # Target B
    B = 1.0

    # Cost function
    def cost_function(K):
        # Update the transfer function with new gains
        Kp_x, Ki_x, Kd_x, Kp_pitch, Ki_pitch, Kd_pitch = K
        num = [
        -9.81 * Kp_pitch / (-9.81 * Kd_x * Kd_pitch + J_yy),
        Ki_pitch / (-9.81 * Kd_x * Kd_pitch + J_yy),
        Kd_pitch * (-9.81 * Kd_x * Kd_pitch + J_yy),
        Kp_x / (-9.81 * Kd_x * Kd_pitch + J_yy),
        Ki_x / (-9.81 * Kd_x * Kd_pitch + J_yy),
        Kd_x * (-9.81 * Kd_x * Kd_pitch + J_yy)
        ]
        sys_tf.num = num

        time_range = np.linspace(0, 100, 1000)  # Extend the time range to 10 seconds with 1000 points
        _, response = signal.step(sys_tf, T=time_range)

        # Compute error between A and B
        error = np.sum(np.abs(response - B))

        return error

    # Initial guess for gains
    initial_guess = [Kp_x, Ki_x, Kd_x, Kp_pitch, Ki_pitch, Kd_pitch]

    # Minimize the cost function
    result = optimize.minimize(cost_function, initial_guess, method='Nelder-Mead')

    # Extract the optimal gains
    optimal_gains = result.x

    print("Optimal gains:")
    print("Kp_x:", optimal_gains[0])
    print("Ki_x:", optimal_gains[1])
    print("Kd_x:", optimal_gains[2])

    # Update the transfer function with optimal gains
    num = [3.205782551 * optimal_gains[0], 3.205782551 * optimal_gains[1], 3.205782551 * optimal_gains[2]]
    sys_tf.num = num
    time_range = np.linspace(0, 100, 1000)  # Extend the time range to 10 seconds with 1000 points
    _, response = signal.step(sys_tf, T=time_range)
    plt.plot(time_range, response)
    plt.xlabel('Time')
    plt.ylabel('Response')
    plt.title('Step Response of System with Optimal Gains')
    plt.grid(True)
    plt.show()

def PID_tuningz():
    # Transfer function parameters
    initial_Kp_z = 1.0  # Initial guess for proportional gain
    initial_Ki_z = 0.0  # Initial guess for integral gain
    initial_Kd_z = 1.0  # Initial guess for derivative gain

    # Transfer function numerator and denominator coefficients
    numerator = [3.205782551 * initial_Kp_z, 3.205782551 * initial_Ki_z, 3.205782551 * initial_Kd_z]
    denominator = [1.0, 6.411565102 * initial_Kd_z, 6.411565102 * initial_Kp_z, 6.411565102 * initial_Ki_z]

    # Transfer function
    sys_tf = signal.TransferFunction(numerator, denominator)

    # Target B
    B = 1.0

    # Cost function
    def cost_function(K):
        # Update the transfer function with new gains
        Kp_z, Ki_z, Kd_z = K
        num = [3.205782551 * Kp_z, 3.205782551 * Ki_z, 3.205782551 * Kd_z]
        sys_tf.num = num

        time_range = np.linspace(0, 100, 1000)  # Extend the time range to 10 seconds with 1000 points
        _, response = signal.step(sys_tf, T=time_range)

        # Compute error between A and B
        error = np.sum(np.abs(response - B))

        return error

    # Initial guess for gains
    initial_guess = [initial_Kp_z, initial_Ki_z, initial_Kd_z]

    # Minimize the cost function
    result = optimize.minimize(cost_function, initial_guess, method='Nelder-Mead')

    # Extract the optimal gains
    optimal_gains = result.x

    print("Optimal gains:")
    print("Kp_z:", optimal_gains[0])
    print("Ki_z:", optimal_gains[1])
    print("Kd_z:", optimal_gains[2])

    # Update the transfer function with optimal gains
    num = [3.205782551 * optimal_gains[0], 3.205782551 * optimal_gains[1], 3.205782551 * optimal_gains[2]]
    sys_tf.num = num
    time_range = np.linspace(0, 100, 1000)  # Extend the time range to 10 seconds with 1000 points
    _, response = signal.step(sys_tf, T=time_range)
    plt.plot(time_range, response)
    plt.xlabel('Time')
    plt.ylabel('Response')
    plt.title('Step Response of System with Optimal Gains')
    plt.grid(True)
    plt.show()




if __name__ == '__main__':
    PID_tuningx()