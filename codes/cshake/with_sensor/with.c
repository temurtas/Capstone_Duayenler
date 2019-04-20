#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include "vl6180_pi/vl6180_pi.h"
#include <wiringPi.h>

#define CATCH "00"
#define ACK   "01"
#define REJ   "11"
#define STOP  "10"

#define SCATCH "0700"
#define SACK   "0701"
#define SREJ   "0711"
#define SSTOP  "0710"

#define PORT 5000

#define sensor1Pin 0
#define sensor2Pin 1

void print_usage()
{
    printf("Usage: -t <HostType(s/c)> -id <OpponentId> -ip <OpponentIp> \n");
    exit (1);
}

void print_arguments(char* type ,char* opp_id ,char* opp_ip)
{
	printf("HOST TYPE: %s\n",type);
	printf("OPPONENT ID: %s\n",opp_id);
	printf("OPPONENT IP: %s\n",opp_ip);
}

void sensor_setup(vl6180* fhandle, vl6180* bhandle)
{
  if(wiringPiSetup() == -1)
  {
		printf("setup wiringPi failed !\n");
		exit(-1);
  }
  pinMode(sensor1Pin, OUTPUT);
	pinMode(sensor2Pin, OUTPUT);
	digitalWrite(sensor1Pin,LOW);
	digitalWrite(sensor2Pin,LOW);

	digitalWrite(sensor1Pin,HIGH);
	delay(50);
	*fhandle = vl6180_initialise(1);
	if(*fhandle<=0)
  {
		printf("ERROR FOR FRONT HANDLE !\n");
		exit(-1);
	}
	vl6180_change_addr(*fhandle,0x54);

	digitalWrite(sensor2Pin,HIGH);
	delay(50);
	*bhandle = vl6180_initialise(1);
	if(*bhandle<=0)
  {
		printf("ERROR FOR BACK HANDLE !\n");
		exit(-1);
	}
	vl6180_change_addr(*bhandle,0x56);
}

int server_funct(char* opp_id)
{
    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[5];
    memset (buffer, 0x00, 5);

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

    // Forcefully attaching socket to the port 5000
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

    // Define message strings to be sent
    char catch[5];
    char ack[5];
    char rej[5];
    char stop[5];

    strcpy(catch, opp_id);
    strcpy(ack, opp_id);
    strcpy(rej, opp_id);
    strcpy(stop, opp_id);

    strcat(catch, CATCH);
    strcat(ack, ACK);
    strcat(rej, REJ);
    strcat(stop, STOP);

    printf("CATCH = %s\n", catch);
    printf("ACK = %s\n", ack);
    printf("REJ = %s\n", rej);
    printf("STOP = %s\n", stop);

    vl6180 fhandle,bhandle;
    sensor_setup(&fhandle, &bhandle);
    int freading,breading;
    struct timeval timeout;
    fd_set rfd;
    while (1)
    {
      // Check front sensor data
      freading = get_distance(fhandle);
      printf ("Front sensor value is: %d\n", freading);
      if (freading) {
        send(new_socket , SCATCH , strlen(SCATCH) , 0 );
        FD_ZERO(&rfd);
        FD_SET(new_socket, &rfd);
        timeout.tv_sec =  5;
        timeout.tv_usec = 0;
        printf("Waiting...\n");
        int ret = select(new_socket+1, &rfd, NULL, NULL, &timeout);
        if (ret != 0)
        {  // No Time out, i.e. data available, read it
          recv(new_socket, buffer, sizeof(buffer), 0);
          printf("%s is received.\n", buffer );
          if (strcmp(buffer,ack) == 0)
          {
            memset (buffer, 0x00, 5);
            send ( new_socket , SSTOP , strlen(SSTOP) , 0 );
            break;
          } else if (strcmp(buffer,rej) == 0)
          {
            printf ("I was rejected, checking again... \n");
            memset (buffer, 0x00, 5);
          }
          else memset (buffer, 0x00, 5);
        }
      } // winning case


      else {


        FD_ZERO(&rfd);
        FD_SET(new_socket, &rfd);
        timeout.tv_sec =  5;
        timeout.tv_usec = 0;

        printf("Waiting...\n");
        int ret = select(new_socket+1, &rfd, NULL, NULL, &timeout);

        if (ret != 0)
        {  // No Time out, i.e. data available, read it
          recv(new_socket, buffer, sizeof(buffer), 0);
          printf("%s is received.\n", buffer );

          if (strcmp(buffer,catch) == 0) {
            memset (buffer, 0x00, 5);
            breading = get_distance(bhandle);
            printf ("Back sensor value is: %d\n", breading);
            if (breading)
            {
              send(new_socket , SACK , strlen(SACK) , 0 );
              FD_ZERO(&rfd);
              FD_SET(new_socket, &rfd);
              timeout.tv_sec = 5;
              timeout.tv_usec = 0;
              printf("Waiting...\n");
              ret = select(new_socket+1, &rfd, NULL, NULL, &timeout);
              if (ret != 0)
              {
                recv(new_socket, buffer, sizeof(buffer), 0);
                printf("%s is received.\n", buffer );
                if (strcmp(buffer, stop)==0)
                {
                  memset (buffer, 0x00, 5);
                  break;
                }
                else memset (buffer, 0x00, 5);
              }
            }
            else
            {
              send(new_socket , SREJ , strlen(SREJ) , 0 );
            }
          }
          else memset (buffer, 0x00, 5);
        }
      } // defeat case
    } // while loop
    return 0;
}


int client_funct(char* opp_id, char* server_ip)
{
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;

    char buffer[5];
    memset (buffer, 0x00, 5);
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

    // Define message strings to be sent
    char catch[5];
    char ack[5];
    char rej[5];
    char stop[5];

    strcpy(catch, opp_id);
    strcpy(ack, opp_id);
    strcpy(rej, opp_id);
    strcpy(stop, opp_id);

    strcat(catch, CATCH);
    strcat(ack, ACK);
    strcat(rej, REJ);
    strcat(stop, STOP);

    printf("CATCH = %s\n", catch);
    printf("ACK = %s\n", ack);
    printf("REJ = %s\n", rej);
    printf("STOP = %s\n", stop);

    vl6180 fhandle,bhandle;
    sensor_setup(&fhandle, &bhandle);
    int freading,breading;
    struct timeval timeout;
    fd_set rfd;
    while (1)
    {
      // Check front sensor data

      freading = get_distance(fhandle);
      printf ("Front sensor value is: %d\n", freading);
      if (freading<=50)
      {
        send(sock , SCATCH , strlen(SCATCH) , 0 );
        FD_ZERO(&rfd);
        FD_SET(sock, &rfd);
        timeout.tv_sec =  5;
        timeout.tv_usec = 0;
        printf("Waiting...\n");
        int ret = select(sock+1, &rfd, NULL, NULL, &timeout);
        if (ret != 0)
        {  // No Time out, i.e. data available, read it
          recv(sock, buffer, sizeof(buffer), 0);
          printf("%s is received.\n", buffer );
          if (strcmp(buffer,ack) == 0)
          {
            memset (buffer, 0x00, 5);
            send(sock , SSTOP , strlen(SSTOP) , 0 );
            break;
          }
          else if (strcmp(buffer,rej) == 0)
          {
            printf ("I was rejected, checking again... \n");
            memset (buffer, 0x00, 5);
          }
          else memset (buffer, 0x00, 5);
        }
      } // winning case


      else
      {

        FD_ZERO(&rfd);
        FD_SET(sock, &rfd);
        timeout.tv_sec =  5;
        timeout.tv_usec = 0;
        printf("Waiting...\n");
        int ret = select(sock+1, &rfd, NULL, NULL, &timeout);

        if (ret != 0)
        {  // No Time out, i.e. data available, read it
          recv(sock, buffer, sizeof(buffer), 0);
          printf("%s is received.\n", buffer );

          if (strcmp(buffer,catch) == 0)
          {
            memset (buffer, 0x00, 5);
            breading = get_distance(bhandle);
            printf ("Back sensor value is: %d\n", breading);
            if (breading<=50)
            {
              send(sock , SACK , strlen(SACK) , 0 );
              FD_ZERO(&rfd);
              FD_SET(sock, &rfd);
              timeout.tv_sec = 5;
              timeout.tv_usec = 0;
              printf("Waiting...\n");
              ret = select(sock+1, &rfd, NULL, NULL, &timeout);
              if (ret != 0)
              {
                recv(sock, buffer, sizeof(buffer), 0);
                printf("%s is received.\n", buffer );
                if (strcmp(buffer,stop) == 0)
                {
                  memset (buffer, 0x00, 5);
                  break;
                }
                else memset (buffer, 0x00, 5);
              }
            }
            else
            {
              send(sock , SREJ , strlen(SREJ) , 0 );
            }
          }
          else memset (buffer, 0x00, 5);
        }
      } // defeat case
    } // while loop
    return 0;
}



int main(int argc, char *argv[])
{
    char* opp_ip = "127.0.0.1";
    char* opp_id = "00";
    char* type;
    int option_index = 0;
    while (( option_index = getopt(argc, argv, "t:")) != -1){
  		switch (option_index) {
  		  case 't':
               type = malloc(strlen(optarg));
  		         strcpy (type, optarg);
  						 break;
  		  default:
  		         print_usage();
  		}
  	}

    print_arguments(type ,opp_id ,opp_ip);

    if (strcmp(type,"s")==0)
    {
      printf("I am server.\n");
      server_funct(opp_id);
      free(type);
    }
    else if (strcmp(type,"c")==0)
    {
      printf("I am client.\n");
      client_funct(opp_id, opp_ip);
      free(type);
    }
    else
    {
      printf("I don't know who I am.\n");
      free(type);
    	print_usage();
    }

    return 0;
}
