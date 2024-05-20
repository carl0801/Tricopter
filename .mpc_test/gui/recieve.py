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

class ChatReceiver(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Chat Receiver")
       
        # Box to display connection status
        self.connection_label = tk.Label(self, text="Listening for messages...")
        self.connection_label.pack(pady=10)

        # Box to display messages
        self.text_area = scrolledtext.ScrolledText(self, wrap=tk.WORD, width=RECEIVER_WIDTH, height=RECEIVER_HEIGHT)
        self.text_area.pack(pady=10)
        
        # Queue to communicate between threads
        self.message_queue = queue.Queue()

        # Start receiving messages
        self.receive_thread = threading.Thread(target=self.receive_messages)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        # send a message to the server
        #self.send_message('<r><sID>STPLC_13</sID><cID>4</cID><DT>DT#2024-04-22-14:57:31</DT></r>')

    def send_message(self, message):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                client_socket.connect((HOST, PORT))
                client_socket.sendall(message.encode('utf-8'))
        except Exception as e:
            print(f"Error sending message: {e}")

    def receive_messages(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            try:
                client_socket.connect((HOST, PORT))
                self.connection_label.config(text="Connected")
                while True:
                    message = client_socket.recv(2024).decode('utf-8')
                    current_time = time.strftime("%H:%M:%S")  # Get the current time
                    self.message_queue.put(f"[{current_time}] {message}")  # Put received message in the queue
                    self.process_received_messages()
            except Exception as e:
                print(f"Error receiving message: {e}")
                self.connection_label.config(text="Connection failed")

    def process_received_messages(self):
        while not self.message_queue.empty():
            message = self.message_queue.get()
            self.text_area.insert(tk.END, message + '\n')
            self.text_area.see(tk.END)

receiver_app = ChatReceiver()
receiver_app.geometry(f"{RECEIVER_WIDTH}x{RECEIVER_HEIGHT}+0+0")  # Position receiver window on the left side of the screen
receiver_app.mainloop()