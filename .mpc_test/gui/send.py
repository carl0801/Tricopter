import tkinter as tk
from tkinter import scrolledtext
import socket
import threading
import queue
import time

HOST = '192.168.1.51'  # ESP32's IP address
PORT = 80  # Port used by the ESP32 server
RECEIVER_WIDTH = 600
RECEIVER_HEIGHT = 300
SENDER_WIDTH = 400
SENDER_HEIGHT = 300

class SenderApplication(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Message Sender")
        self.geometry(f"{SENDER_WIDTH}x{SENDER_HEIGHT}")
        
        # Entry field for sending messages
        self.message_entry = tk.Entry(self, width=40)
        self.message_entry.pack(pady=10)

        # Create socket connection to the receiver
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((HOST, PORT))

        # Bind key press events to send_message method
        self.bind("<KeyPress>", self.send_key)

        # Create boxes for displaying keys
        self.create_key_box("W", 50, 50)
        self.create_key_box("A", 0, 110)
        self.create_key_box("S", 50, 110)
        self.create_key_box("D", 100, 110)
        self.create_key_box("R", 200, 50)
        self.create_key_box("F", 200, 110)
        self.create_key_box("L", 300, 50)
        self.create_key_box("M", 300, 110)
        
        #create button that starts subprocess for Record_&_visualize.py
        self.record_button = tk.Button(self, text="Record", command=self.start_recording)
        #place button in the buttom center of the window
        self.record_button.place(x=175, y=200)


    #create method that starts subprocess for Record_&_visualize.py
    def start_recording(self):
        import subprocess
        subprocess.Popen(["python", "Chill/GUI/Record_&_visualize.py"])


    def create_key_box(self, key, x, y):
        box = tk.Canvas(self, width=50, height=50, bg="white")
        box.place(x=x, y=y)
        text = box.create_text(25, 25, text=key, font=("Arial", 20))
        setattr(self, f"{key.lower()}_box", box)  # Set attribute for the box
        return box

    def send_key(self, event):
        key = event.char  # Get the pressed key
        if self.client_socket:
            self.client_socket.send(key.encode('utf-8'))
            self.message_entry.delete(0, tk.END)  # Clear the input field after sending the message

        # Check if the pressed key is one of "w", "a", "s", "d", "r", "f", "l", "m"
        if key.lower() in ["w", "a", "s", "d", "r", "f", "l", "m"]:
            # Change the color of the corresponding box to red for half a second
            box = getattr(self, f"{key.lower()}_box")
            box.config(bg="red")
            self.after(100, lambda: box.config(bg="white"))  # Change back to white after 500ms

sender_app = SenderApplication()
sender_app.geometry(f"{SENDER_WIDTH}x{SENDER_HEIGHT}+{RECEIVER_WIDTH}+0")  # Position sender window next to the receiver
sender_app.mainloop()