import socket
import sys
import keyboard
import numpy as np
import datetime
import matplotlib.pyplot as plt

visualize = False
HOST = '192.168.1.52'  # ESP32's IP address
PORT = 80  # Port used by the ESP32 server
rotation_errors = []
motor_speeds = []
current_npz_name_points = ""
current_npz_name_speeds = ""

def save_points_to_npz(points):
    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    np.savez(f"points_data_{current_time}.npz", points=np.array(points))
    global current_npz_name_points
    current_npz_name_points = f"points_data_{current_time}.npz"


def save_speeds_to_npz(speeds):
    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    np.savez(f"speeds_data_{current_time}.npz", speeds=np.array(speeds))
    global current_npz_name_speeds
    current_npz_name_speeds = f"speeds_data_{current_time}.npz"

def main():
    buffer = ''  # Initialize an empty buffer
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))
            print("Connected to ESP32")
            
            
            num_messages = 0
            # while connected print messages from the server
            while True:
                msg = s.recv(1524).decode()
                buffer += msg
                
                # Split the buffer into lines using 'n' as a delimiter
                lines = buffer.split('\n')
                # The last element of 'lines' might be an incomplete message, so we store it back in buffer
                buffer = lines[-1]
                # Process complete lines
                for line in lines[:-1]:
                    print(line)  # Print the received line for debugging
                    with open('data.txt', 'a') as file:
                        file.write(buffer + '\n')
                        num_msgs += 1
                        if num_msgs % 100 == 0:
                            print("Number of messages received: ", num_msgs)
                #     # Split the received message using colon (":") as a delimiter
                #     coordinates = line.strip().split(":")
                    
                #     # Convert the string coordinates to floats
                #     # Rotation Errors
                #     x_rot_error = float(coordinates[0])
                #     y_rot_error = float(coordinates[1])
                #     # motor speeds
                #     motor_1_speed = float(coordinates[2])
                #     motor_2_speed = float(coordinates[3])
                #     motor_3_speed = float(coordinates[4])

                #     # Append the coordinates to the points list
                #     rotation_errors.append([x_rot_error, y_rot_error])
                #     motor_speeds.append([motor_1_speed, motor_2_speed, motor_3_speed])

                
                # Check if the user has pressed a key (for interrupting the loop)
                if keyboard.is_pressed('q'):
                    #save_points_to_npz(rotation_errors)
                    #save_speeds_to_npz(motor_speeds)
                    #global visualize
                    #visualize = True
                    break

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


def visualize_data(rotational_error_file, motor_output_file):
    # Load rotational error data
    rotational_error_data = np.load(rotational_error_file)
    # Load npz file
    data = np.load(motor_output_file)
    # print(data.files)
    print(rotational_error_data.files)
    # Access arrays
    rot_errors = rotational_error_data['points']
    #print(rot_errors)
    motor_speeds = data['speeds']
    # Split the arrays into individual components
    rot_x = rot_errors[5:, 0]
    rot_y = rot_errors[5:, 1]
    output1_speed = motor_speeds[5:, 0]
    output2_speed = motor_speeds[5:, 1]
    output3_speed = motor_speeds[5:, 2]
    cutoff = 3000
    rot_x = rot_x[:cutoff]
    rot_y = rot_y[:cutoff]
    output1_speed = output1_speed[:cutoff]
    output2_speed = output2_speed[:cutoff]
    output3_speed = output3_speed[:cutoff]
    #calculate the variance of the rotational error
    var_rot_x = np.var(rot_x)
    var_rot_y = np.var(rot_y)
    print("Variance of Rotational Error X: ", var_rot_x)
    print("Variance of Rotational Error Y: ", var_rot_y)
    #calculate the variance of the motor output
    var_output1_speed = np.var(output1_speed)
    var_output2_speed = np.var(output2_speed)
    var_output3_speed = np.var(output3_speed)
    print("var O1:", var_output1_speed)
    print("var O2:", var_output2_speed)
    print("var O3:", var_output3_speed)
    # Plot rotational error data
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(rot_x, label='Rotation X Error')
    plt.plot(rot_y, label='Rotation Y Error')
    plt.title('Rotational Error Data')
    plt.xlabel('Time')
    plt.ylabel('Error')
    plt.legend()

    # Plot motor output data
    plt.subplot(2, 1, 2)
    plt.plot(output1_speed, label='Motor Output 1 Speed')
    plt.plot(output2_speed, label='Motor Output 2 Speed')
    plt.plot(output3_speed, label='Motor Output 3 Speed')
    plt.title('Motor Output Data')
    plt.xlabel('Time')
    plt.ylabel('Speed')
    plt.legend()

    # Show plots
    plt.tight_layout()
    plt.show()
    


if __name__ == "__main__":
    while True:
        if visualize:
            print("Visualizing data...")
            print(current_npz_name_points)
            print(current_npz_name_speeds)
            visualize_data(current_npz_name_points, current_npz_name_speeds)
            break
        else:
            main()
            
