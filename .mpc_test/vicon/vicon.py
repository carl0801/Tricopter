from vicon_dssdk import ViconDataStream
import socket
import csv
import numpy as np
import time
import re

IP = '192.168.1.55'
PORT = 5000
IP = '192.168.1.51'  # ESP32's IP address
PORT = 80  # Port used by the ESP32 server
socketCom = True

if socketCom:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((IP, PORT))

buffer = ""
def reciveDroneData(s):
    global buffer
    # Recive data
    message = s.recv(2024).decode('utf-8')
    buffer += message
    # Regular expression pattern to match key: value pairs
    pattern = r'(\w+):\s*([-+]?\d*\.\d+|\d+)[Vv]?'
    # Find all matches in the input string along with their positions
    matches = list(re.finditer(pattern, buffer))
    # Create the dictionary from the matches
    data = {match.group(1): float(match.group(2)) for match in matches}
    # Find the end position of the last match
    if matches:
        last_match_end = matches[-1].end()
    else:
        last_match_end = 0
    # Extract the remaining text starting from the end of the last match
    remaining_text = buffer[last_match_end:].strip()
    buffer = remaining_text
    return data

client = ViconDataStream.Client()

try:
    client.Connect( "192.168.1.12:801" )
    client.EnableSegmentData()
    # Check the version
    print( 'Version', client.GetVersion() )

    client.SetAxisMapping( ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft, ViconDataStream.Client.AxisMapping.EUp )
    xAxis, yAxis, zAxis = client.GetAxisMapping()
    print( 'X Axis', xAxis, 'Y Axis', yAxis, 'Z Axis', zAxis )

    while( True ):
        try:
            client.GetFrame()
            framerate = client.GetFrameRate()
            framenumber = client.GetFrameNumber()
            timecode = framenumber/framerate

            """subjectNames = client.GetSubjectNames()
            for subjectName in subjectNames:
                 segmentNames = client.GetSegmentNames( subjectName )
                 for segmentName in segmentNames:
                    segmentChildren = client.GetSegmentChildren( subjectName, segmentName )"""
            
            subjectName = "Tricopter"
            segmentName = "Tricopter"

            # Get position and rotation
            translation = client.GetSegmentGlobalTranslation( subjectName, segmentName )
            rotation = client.GetSegmentGlobalRotationQuaternion( subjectName, segmentName )  
            # To numpy array
            translation = np.array(translation[0])
            rotation = np.array(rotation[0])

            if np.round(timecode % 0.05, 3) == 0 and socketCom:
                if socketCom:
                    translation_str = ','.join(map(str, translation))
                    rotation_str = ','.join(map(str, rotation))
                    transmission_str = f"{translation_str},{rotation_str}\n"
                    #s.sendall(transmission_str.encode())
            
            if np.round(timecode % 0.5, 3) == 0:
                euler = client.GetSegmentGlobalRotationEulerXYZ( subjectName, segmentName )
                euler = np.array(euler[0])
                euler = np.rad2deg(euler)
                print(f"P:{np.round(translation)} \tR:{np.round(euler, 1)} \tT:{timecode}")

            # Save translation to csv file
            if np.linalg.norm(translation) != 0.0:

                data = reciveDroneData(s)
                qw = data.get('qw', "Error")
                qx = data.get('qx', "Error")
                qy = data.get('qy', "Error")
                qz = data.get('qz', "Error")

                if qw != "Error" and qx != "Error" and qy != "Error" and qz != "Error":
                    with open('test/yawTest.csv', mode='a', newline="") as file:
                        writer = csv.writer(file)
                        data = [rotation[0], rotation[1], rotation[2], rotation[3], translation[0], translation[1], translation[2], qw, qx, qy, qz, timecode]
                        writer.writerow(data)

        except ViconDataStream.DataStreamException as e:
            print( 'Handled data stream error', e )
            time.sleep(1)
 
except ViconDataStream.DataStreamException as e:
    print( 'Handled data stream error', e )

