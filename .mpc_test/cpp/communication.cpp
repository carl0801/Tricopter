/**
 * Sample for UNIX domain socket
 * Two-way communication between process server and client:
 * 1) Client connect and say hello to server
 * 2) Server receive client message and display
 * 3) Server say hello back to client
 * 4) Client receive server message and display
 * 5) Done.
 */

#include <unistd.h> // for fork()
#include <stdio.h> // for printf
#include <stdlib.h> // for exit()
#include <sys/socket.h>
#include <sys/un.h> // socket in Unix

// for print error message
#include <string.h>
#include <errno.h>

#define SERVER_SOCK_PATH "unix_sock.server"
#define CLIENT_SOCK_PATH "unix_sock.client"
#define SERVER_MSG "HELLO FROM SERVER"
#define CLIENT_MSG "HELLO FROM CLIENT"


int main(int argc, char **argv)
{
    struct sockaddr_un server_addr;
    struct sockaddr_un client_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    memset(&client_addr, 0, sizeof(client_addr));
    int rc;
    // for simplicity, we will assign a fixed size for buffer of the message
    char buf[256];

    // create two processes of client and server
    pid_t pid = fork();


    //---------- SERVER PROCESS
    if (pid != 0)
    {
        // maximum number of client connections in queue
        int backlog = 10;

        /****************************************************/
        /* Open the server socket with the SOCK_STREAM type */
        /****************************************************/
        int server_sock = socket(AF_UNIX, SOCK_STREAM, 0);
        if (server_sock == -1)
        {
            printf("SERVER: Error when opening server socket.\n");
            exit(1);
        }

        /*************************************/
        /* Bind to an address on file system */
        /*************************************/
        // similar to other IPC methods, domain socket needs to bind to a file system
        // so that client know the address of the server to connect to
        server_addr.sun_family = AF_UNIX;
        strcpy(server_addr.sun_path, SERVER_SOCK_PATH);
        int len = sizeof(server_addr);

        // unlink the file before bind, unless it can't bind
        unlink(SERVER_SOCK_PATH);

        rc = bind(server_sock, (struct sockaddr *)&server_addr, len);
        if (rc == -1)
        {
            printf("SERVER: Server bind error: %s\n", strerror(errno));
            close(server_sock);
            exit(1);
        }

        /***************************************/
        /* Listen and accept client connection */
        /***************************************/
        // set the server in the "listen" mode and maximum pending connected clients in queue
        rc = listen(server_sock, backlog);
        if (rc == -1)
        {
            printf("SERVER: Listen error: %s\n", strerror(errno));
            close(server_sock);
            exit(1);
        }

        printf("SERVER: Socket listening...\n");
        int client_fd = accept(server_sock, (struct sockaddr *) &client_addr, (socklen_t*)&len);
        if (client_fd == -1)
        {
            printf("SERVER: Accept error: %s\n", strerror(errno));
            close(server_sock);
            close(client_fd);
            exit(1);
        }
        printf("SERVER: Connected to client at: %s\n", client_addr.sun_path);
        printf("SERVER: Wating for message...\n");

        /********************/
        /* Listen to client */
        /********************/
        memset(buf, 0, 256);
        int byte_recv = recv(client_fd, buf, sizeof(buf), 0);
        if (byte_recv == -1)
        {
            printf("SERVER: Error when receiving message: %s\n", strerror(errno));
            close(server_sock);
            close(client_fd);
            exit(1);
        }
        else
            printf("SERVER: Server received message: %s.\n", buf);

        /**********************/
        /* Response to client */
        /**********************/
        printf("SERVER: Respond to the client...\n");
        memset(buf, 0, 256);
        strcpy(buf, SERVER_MSG);
        rc = send(client_fd, buf, strlen(buf), 0);
        if (rc == -1)
        {
            printf("SERVER: Error when sending message to client.\n");
            close(server_sock);
            close(client_fd);
            exit(1);
        }
        printf("SERVER: Done!\n");

        close(server_sock);
        close(client_fd);
        remove(SERVER_SOCK_PATH);
    }

    //---------- CLIENT PROCESS
    else
    {
        /**************************************************/
        /* Open a client socket (same type as the server) */
        /**************************************************/
        int client_sock = socket(AF_UNIX, SOCK_STREAM, 0);
        if (client_sock == -1)
        {
            printf("CLIENT: Socket error: %s\n", strerror(errno));
            exit(1);
        }

        /********************************************/
        /* Bind client to an address on file system */
        /********************************************/
        // Note: this binding could be skip if we want only send data to server without receiving
        client_addr.sun_family = AF_UNIX;
        strcpy(client_addr.sun_path, CLIENT_SOCK_PATH);
        int len = sizeof(client_addr);

        unlink (CLIENT_SOCK_PATH);
        rc = bind(client_sock, (struct sockaddr *)&client_addr, len);
        if (rc == -1)
        {
            printf("CLIENT: Client binding error. %s\n", strerror(errno));
            close(client_sock);
            exit(1);
        }

        /****************************************/
        /* Set server address and connect to it */
        /****************************************/
        server_addr.sun_family = AF_UNIX;
        strcpy(server_addr.sun_path, SERVER_SOCK_PATH);
        rc = connect(client_sock, (struct sockaddr*)&server_addr, len);
        if(rc == -1)
        {
            printf("CLIENT: Connect error. %s\n", strerror(errno));
            close(client_sock);
            exit(1);
        }
        printf("CLIENT: Connected to server.\n");

        /**************************/
        /* Send message to server */
        /**************************/
        memset(buf, 0, sizeof(buf));
        strcpy(buf, CLIENT_MSG);
        rc = send(client_sock, buf, sizeof(buf), 0);
        if (rc == -1)
        {
            printf("CLIENT: Send error. %s\n", strerror(errno));
            close(client_sock);
            exit(1);
        }
        printf("CLIENT: Sent a message to server.\n");

        /**************************************/
        /* Listen to the response from server */
        /**************************************/
        printf("CLIENT: Wait for respond from server...\n");
        memset(buf, 0, sizeof(buf));
        rc = recv(client_sock, buf, sizeof(buf), 0);
        if (rc == -1)
        {
            printf("CLIENT: Recv Error. %s\n", strerror(errno));
            close(client_sock);
            exit(1);
        }
        else
            printf("CLIENT: Message received: %s\n", buf);

        printf("CLIENT: Done!\n");

        close(client_sock);
        remove(CLIENT_SOCK_PATH);
    }

    return 0;
}