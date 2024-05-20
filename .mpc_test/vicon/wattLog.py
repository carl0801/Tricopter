import socket
import time
import re

IP = '192.168.1.51'  # ESP32's IP address
PORT = 80  # Port used by the ESP32 server


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP, PORT))

buffer = ""
i = 0
while True:
    try:
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

        print(data)
        print(":::")

    except Exception as e:
        print(f"Error receiving message: {e}")
