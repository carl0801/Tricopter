import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def visualize_data(data_file):
    # Load data from CSV without column names
    data = pd.read_csv(data_file, header=None)

    # Extract data into separate variables
    pos_x = data.iloc[:, 0]
    pos_y = data.iloc[:, 1]
    pos_z = data.iloc[:, 2]
    error_orientation_x = data.iloc[:, 3]
    error_orientation_y = data.iloc[:, 4]
    error_altitude = data.iloc[:, 5]
    output1 = data.iloc[:, 6]
    output2 = data.iloc[:, 7]
    output3 = data.iloc[:, 8]
    target_position_z = 2
    # Calculate variances of the data
    var_pos_x = np.var(pos_x)
    var_pos_y = np.var(pos_y)
    var_pos_z = np.var(pos_z)
    var_error_orientation_x = np.var(error_orientation_x)
    var_error_orientation_y = np.var(error_orientation_y)
    var_error_altitude = np.var(error_altitude)
    var_output1 = np.var(output1)
    var_output2 = np.var(output2)
    var_output3 = np.var(output3)

    # Print variances
    print("Variance of Position X:", var_pos_x)
    print("Variance of Position Y:", var_pos_y)
    print("Variance of Position Z:", var_pos_z)
    print("Variance of Error Orientation X:", var_error_orientation_x)
    print("Variance of Error Orientation Y:", var_error_orientation_y)
    print("Variance of Error Altitude:", var_error_altitude)
    print("Variance of Output1:", var_output1)
    print("Variance of Output2:", var_output2)
    print("Variance of Output3:", var_output3)

    # Plot data
    plt.figure(figsize=(10, 9))

    plt.subplot(3, 1, 1)
    plt.plot(pos_x, label='Position X')
    plt.plot(pos_y, label='Position Y')
    plt.plot(pos_z, label='Position Z')
    # plot the target position for z
    plt.axhline(y=target_position_z, color='r', linestyle='--', label='Target Position Z')

    plt.title('Position Data')
    plt.xlabel('Time')
    plt.ylabel('Position')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(error_orientation_x, label='Error Orientation X')
    plt.plot(error_orientation_y, label='Error Orientation Y')
    plt.plot(error_altitude, label='Error Altitude')
    plt.title('Error Data')
    plt.xlabel('Time')
    plt.ylabel('Error')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(output1, label='Output1')
    plt.plot(output2, label='Output2')
    plt.plot(output3, label='Output3')
    plt.title('Output Data')
    plt.xlabel('Time')
    plt.ylabel('Output')
    plt.legend()

    # Show plots
    plt.tight_layout()
    plt.show()
    #print the value of the outputs when z is 0.5
    #find the index of the value of z when it is 0.5
    for i in range(len(pos_z)):
        if pos_z[i] == 0.5:
            print("Output1:", output1[i])
            print("Output2:", output2[i])
            print("Output3:", output3[i])
            break

# Example usage
visualize_data('simulation_data.csv')
