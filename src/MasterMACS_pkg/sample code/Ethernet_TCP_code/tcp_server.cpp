#include <stdio.h>
#include <stdlib.h>
#include <winsock2.h>
#include <iostream>
#include <string>

void ErrorHandling( char* message );
 
int main( int argc, char* argv[] )
{
    WSADATA     wsaData;
    SOCKET      hServSock, hClntSock;
    SOCKADDR_IN servAddr, clntAddr;
 
    short   port = 5001;
    int     szClntAddr;
    char    message[] = "Hello World!";
 
    // Initialize Winsock before using sockets.
    if( WSAStartup( MAKEWORD( 2, 2 ), &wsaData ) != 0 )
        ErrorHandling( "WSAStartup() error!" );
 
    // Create a TCP socket.
    hServSock = socket( PF_INET, SOCK_STREAM, 0 );
 
    if( hServSock == INVALID_SOCKET )
        ErrorHandling( "socket() error!" );
 
    // Socket address information
    memset( &servAddr, 0, sizeof( servAddr ) );
 
    servAddr.sin_family         = AF_INET;
    servAddr.sin_addr.s_addr    = htonl( INADDR_ANY );  // Accept connections on local interfaces.
    servAddr.sin_port           = htons( port );        // Port number
 
    // Bind the address.
    if( bind( hServSock, (SOCKADDR*)&servAddr, sizeof( servAddr ) ) == SOCKET_ERROR )
        ErrorHandling( "bind() error!" );
 
    // Start listening for connections.
    if( listen( hServSock, 5 ) == SOCKET_ERROR )    // Pending connection backlog: 5
        ErrorHandling( "listen() error!" );
 
    // Accept a connection.
    szClntAddr  = sizeof( clntAddr );
    hClntSock   = accept( hServSock, (SOCKADDR*)&clntAddr, &szClntAddr );   // Socket for client communication
 
    if( hClntSock == INVALID_SOCKET )
        ErrorHandling( "accept() error!" );
 
    send( hClntSock, message, sizeof( message ), 0 );
 
    closesocket( hClntSock );
    closesocket( hServSock );
    WSACleanup();
 
    return 0;
}
 
void ErrorHandling( char* message )
{
    fputs( message, stderr );
    fputc( '\n', stderr );
    exit( 1 );
}
