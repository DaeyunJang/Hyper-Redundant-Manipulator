#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#pragma comment(lib, "ws2_32")
#include <WinSock2.h>
#include <WS2tcpip.h>

#include <windows.h>
#include <math.h>
#include <string.h>
#include <string>
#include <time.h>
#include <chrono>
#include <thread>

// #define IP_ADDRESS "127.0.0.1"
#define IP_ADDRESS "172.16.1.5"
#define PORT_NUMBER "1918"

#define TCP_BUFFER_SIZE 256
#define QUEUE_FREQUENCY 1000    // Hz
#define QUEUE_TIME      1       // ms
#define TEST_RECV       1
#define TEST_SEND       1

#define ACTIVE_UR_ROBOT_PROTOCOL 0
void ErrorHandling(const char* _Message);
int main(int argc, const char* argv[])
{
    //argc = 2;
    argv[0] = IP_ADDRESS;
    //argv[1] = PORT_NUMBER;
    if (argc != 2) {
        printf("Usage : %s <port>\n", argv[0]);
        exit(1);
    }

    // Initialize Winsock before using any socket operations.
    // This initialization is required for Winsock applications.
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
        ErrorHandling("WSAStartup() error!");

    // Socket creation is similar to Linux socket programming.
    // Return values and error reporting differ on Windows.
    SOCKET hServSock = socket(PF_INET, SOCK_STREAM, 0);
    if (hServSock == INVALID_SOCKET)
        ErrorHandling("socket() error!");

    // Configure the address as in Linux socket programming.
    SOCKADDR_IN servAddr;
    ZeroMemory(&servAddr, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
    servAddr.sin_port = htons(atoi(argv[1]));

    // Bind the socket to the configured address.
    if (bind(hServSock, (const SOCKADDR*)&servAddr, sizeof(servAddr)) == SOCKET_ERROR)
        ErrorHandling("bind() error!");

    if (listen(hServSock, 5) == SOCKET_ERROR)
        ErrorHandling("listen() error!");

    SOCKADDR_IN clntAddr;
    ZeroMemory(&clntAddr, sizeof(clntAddr));
    int szClntAddr = sizeof(clntAddr);
    std::cout << "accecpting..." << std::endl;
    SOCKET hClntSock = accept(hServSock, (SOCKADDR*)&clntAddr, &szClntAddr);
    if (hClntSock == INVALID_SOCKET)
        ErrorHandling("accept() error!");
	
    char message[] = "Hello World!";

    static char recvMsg[TCP_BUFFER_SIZE] = {0};
    std::string sendMsg;
    int strlen;
    static double joints_vel[7];
    static uint64_t counter = 0;
    static long n = 0;
    while(true)
    {   
        #if TEST_RECV
        strlen = recv(hClntSock, recvMsg, TCP_BUFFER_SIZE, 0);
        if(strlen == -1)
        {
            std::cout << "TCP/IP disconnected... read() error" << std::endl;
            break;
        }
        printf("from server : %s, [%d]\n", recvMsg, strlen);
        std::cout << atol(recvMsg) << std::endl;;
        std::cout << recvMsg << std::endl;
        
        #endif

        #if TEST_SEND
        // std::cout << "[Command] Enter the Send message : ";
        // std::getline(std::cin, sendMsg);
        // uint32_t size_of_sendMsg = send(hClntSock, sendMsg.c_str(), TCP_BUFFER_SIZE, 0);
        // uint32_t size_ofsenMsg = send(hClntSock, message, TCP_BUFFER_SIZE, 0);
        std::string smsg = std::to_string(n++);
        uint32_t size_ofsenMsg = send(hClntSock, smsg.c_str(), TCP_BUFFER_SIZE, 0);




        #endif

        #if ACTIVE_UR_ROBOT_PROTOCOL
        sendMsg = "speedj";
        sendMsg += "([";

        for(int i=0; i<7; i++)
        {
            joints_vel[i] = 100*sin(counter * 0.001);
            sendMsg += std::to_string(joints_vel[i]);
            sendMsg += ",";
        }
        sendMsg.pop_back();
        sendMsg += "])";
        sendMsg = send(hServSock, sendMsg.c_str(), TCP_BUFFER_SIZE, 0);
        #endif

        counter++;
        //std::this_thread::sleep_for(std::chrono::milliseconds(QUEUE_TIME));
    }


    send(hClntSock, message, sizeof(message), 0);
    closesocket(hClntSock);
    closesocket(hServSock);
    // Release Winsock resources when no further socket operations are needed.
    WSACleanup();
    return 0;
}

void ErrorHandling(const char* _Message)
{
    fputs(_Message, stderr);
    fputc('\n', stderr);
    exit(1);
}
