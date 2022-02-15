#include <stdio.h>
#include <iostream>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <cassert>

#define MAXBUFSIZE 1024
#define BCAST_PORT 5800

class UdpClient {
    // public constants and variables
    public: 
    int socketID = 0;
    char receiveData[MAXBUFSIZE] = {0};

    // constructor
    UdpClient(int port);

    // public methods
    public: 
    int ReceivePacket();
    int ReceiveInt();
};

