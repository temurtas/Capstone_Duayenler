#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <stdlib.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>

#define CATCH "00(CATCH)"
#define ACK   "01(ACK)"
#define REJ   "11(REJ)"
#define STOP  "10(STOP)"

#define PORT 5000

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int server_funct()
{
    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[256];
    printf("%ld\n",sizeof(buffer));
    memset (buffer, 0x00, 256);

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address,
                                sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                    (socklen_t*)&addrlen))<0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    int reading;
    struct timeval timeout;
    fd_set rfd;
    while (1) {
      //Check front sensor data
      printf ("Did I catch? 1/0: \n");
      scanf("%d",&reading);
      if (reading) {
        send(new_socket , CATCH , strlen(CATCH) , 0 );
      }
      else {
        printf ("I am in the else statement. \n");

        FD_ZERO(&rfd);
        FD_SET(new_socket, &rfd);
        timeout.tv_sec =  5;
        timeout.tv_usec = 0;

        int ret = select(new_socket+1, &rfd, NULL, NULL, &timeout);
        if (ret == 0) {  // time out
          printf("Timeout ...\n");
        } else {          // data available, read it
          recv(new_socket, buffer, sizeof(buffer), 0);
          printf("%s is received.\n", buffer );
          memset (buffer, 0x00, 256);
        }
      }
    }
    return 0;
}


int client_funct(char* server_ip)
{
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;

    char buffer[256];
    printf("%ld\n",sizeof(buffer));
    memset (buffer, 0x00, 256);
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, server_ip, &serv_addr.sin_addr)<=0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\n Connection Failed \n");
        return -1;
    }


    int reading;
    struct timeval timeout;
    fd_set rfd;
    while (1) {
      //Check front sensor data
      printf ("Did I catch? 1/0: \n");
      scanf("%d",&reading);
      if (reading) {
        send(sock , CATCH , strlen(CATCH) , 0 );
      }
      else {
      printf ("I am in the else statement. \n");

      FD_ZERO(&rfd);
      FD_SET(sock, &rfd);
      timeout.tv_sec =  5;
      timeout.tv_usec = 0;

      int ret = select(sock+1, &rfd, NULL, NULL, &timeout);
      if (ret == 0) {  // time out
        printf("Timeout ...\n");
      } else {          // data available, read it
        recv(sock, buffer, sizeof(buffer), 0);
        printf("%s is received.\n", buffer );
        memset (buffer, 0x00, 256);
        }
      }
    }
    return 0;
}

int main(int argc, char *argv[])
{
    char* server_ip = "127.0.0.1";
    if (argc == 1)
    {
        server_funct();
    }
    else if (argc == 2)
    {
        client_funct(server_ip);
    }
    else
    {
    	printf("ERROR: You gave wrong arguments !!!");
		  exit(-1);
    }

    return 0;
}
