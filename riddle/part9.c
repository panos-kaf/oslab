// Server side C program to demonstrate Socket
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PORT 49842

    
char* itoa(int val, int base){
    
    static char buf[32] = {0};
    
    int i = 30;

    for(; val && i ; --i, val /= base)
    
        buf[i] = "0123456789abcdef"[val % base];
    
    return &buf[i+1];
    
}

int main(int argc, char const* argv[])
{
    int server_fd, new_socket;
    ssize_t valread;
    struct sockaddr_in address;
    int opt = 1;
    socklen_t addrlen = sizeof(address);
    char buffer[1024] = { 0 };

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET,
                   SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr*)&address,
             sizeof(address))
        < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket
         = accept(server_fd, (struct sockaddr*)&address,
                  &addrlen))
        < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    valread = read(new_socket, buffer, 1024 - 1); // subtract 1 for the null
					          // terminator at the end
    printf("%s\n", buffer);
    char* garbage = strtok(buffer," ");
    garbage = strtok(NULL," ");
    garbage = strtok(NULL," ");
    char* str1 = strtok(NULL," ");
    garbage = strtok(NULL," ");
    char* str2 = strtok(NULL," ");
    int lhs = atoi(str1);
    int rhs = atoi(str2);
    int hello1 = lhs + rhs;
    char* hello = itoa(hello1,10);
    printf("%d %d \n",lhs,rhs);
    send(new_socket, hello, strlen(hello), 0);
    printf("Hello message sent\n");

    // closing the connected socket
    close(new_socket);
    // closing the listening socket
    close(server_fd);
    return 0;
}


