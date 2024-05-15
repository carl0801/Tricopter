import socket
import random
import select

maxClients = 5

def processCommand(c):
    print(f"Received command: {c}")

def handle_client(client_socket, client_address):
    print(f"New client connected: {client_address}")
    client_socket.sendall(b"Welcome to the server!\n")  

    while True:
        ready_to_read, _, _ = select.select([client_socket], [], [], 0.1)  # Check for incoming data

        if ready_to_read:
            data = client_socket.recv(1024)
            if not data:  
                print(f"Client disconnected: {client_address}")
                client_socket.close()
                return  # Stop handling this client

            message = data.decode().strip('\n')
            if message: 
                processCommand(message[0])

        # Generate simulated position data
        pos = [random.randint(0, 100) for _ in range(3)]
        dataStr = f"{pos[0]}:{pos[1]}:{pos[2]}\n".encode()
        client_socket.sendall(dataStr)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.bind(("localhost", 5000))  
    server_socket.listen(maxClients)
    server_socket.setblocking(False)  # Set the server socket to non-blocking mode

    clients = []  # List to keep track of connected clients

    while True:
        try:
            client_socket, client_address = server_socket.accept()
            clients.append(client_socket)
            handle_client(client_socket, client_address)  

        except BlockingIOError:
            pass  # No pending connections, continue the loop
        except Exception as e:
            print(f"Error accepting connection: {e}") 

