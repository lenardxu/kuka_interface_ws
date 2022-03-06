//
// Created by Lenovo on 2022/1/24.
// UDP Server
//

// Server side implementation of UDP client-server model
#include "ros/ros.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <chrono>
#include "parser.hpp"
#include <winsock2.h>
#include <boost/interprocess/windows_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

// Link with ws2_32.lib
#pragma comment(lib, "ws2_32.lib")  //Winsock Library

#define PORT     49152  // The port on which to listen for incoming data (non-privileged ports are > 1023)
#define BUFLEN   4096  // Max length of buffer
#define HOST     "0.0.0.0"  // ip address for external connection
#define LOCALHOST "127.0.0.1"  // the local host via the loopback device

using namespace xercesc;
using namespace boost::interprocess;

/*
 * helpful guide for Boost.interprocess: https://www.boost.org/doc/libs/1_78_0/doc/html/interprocess.html
 * helpful guide for beginner of using winsock2:
 *   https://docs.microsoft.com/en-us/windows/win32/winsock/getting-started-with-winsock
 * helpful guide for choosing xml library implemented in c++:
 *   https://stackoverflow.com/questions/9387610/what-xml-parser-should-i-use-in-c  --> xerces
 * if xerces is not present, then the following should be helpful in terms of building from source.
 *   Downloading resources for xerces: https://archive.apache.org/dist/xerces/c/3/sources/
 *   Official instruction for xerces building:
 *     https://xerces.apache.org/xerces-c/build-3.html  or,
 *   another useful instruction for xerces building:
 *     http://www.yolinux.com/TUTORIALS/XML-Xerces-C.html
 */

/*
 * \brief Processing message from robot which is stored in shared memory
 *
 * \param argc An integer argument count of the command line arguments
 * \param argv An argument vector of the command line arguments
 * \return Status of the main program
 */
int main(int argc, char *argv[]) {
    // declare var param to be then filled with the given value in command line for running the node
    std::string param;
    // initialize ROS with specifying the name of node constructed in this process
    ros::init(argc, argv, "kuka_interface_server_node");
    // create a handle to this process' node - "~" is needed for fetching ros parameters
    ros::NodeHandle nh("~");
    // fetch the value of param existing in the command line and assign it to var param
    nh.getParam("param", param);
    ROS_INFO("Got parameter : %s", param.c_str());

    printf("\nServer main function starts...");
    // set the error Code as 1 initially to indicate the following error when creating DOM string
    int errCode = 1;
    SOCKET s;
    int slen , recv_len;
    char buf[BUFLEN];
    // declare the variable holding the addresses of server and client
    struct sockaddr_in server, si_other;
    WSADATA wsa;

    slen = sizeof(si_other);

    //Initialize winsock
    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        // printf("Failed. Error Code : %d",WSAGetLastError());  // should not be used to retrieve error code
        std::cout<<"WSAStartup error"<<std::endl;
        exit(EXIT_FAILURE);
    }
    printf("Initialised.\n");

    //Create a socket
    if((s = socket(AF_INET , SOCK_DGRAM , IPPROTO_UDP )) == INVALID_SOCKET)
    {
        printf("Could not create socket : %d" , WSAGetLastError());
    }
    printf("Socket created.\n");

    // https://stackoverflow.com/questions/16508685/understanding-inaddr-any-for-socket-programming for reference
    /*
     * Prepare the sockaddr_in structure: (Note: Except for the sin*_family parameter, sockaddr contents are expressed
     * in network byte order.)
     * - specify the IPv4 address family
     * - assign INADDR_ANY or "0.0.0.0" (network byte order) to s_addr to then bind the socket to all available interfaces
     * - assign a specific port number (network byte order) to sin_port
     */
    server.sin_family = AF_INET;
    //server.sin_addr.s_addr = inet_addr(HOST);  // an alternative way of defining s_addr for external connection
    server.sin_addr.s_addr = htonl(INADDR_ANY);
    //server.sin_addr.s_addr = inet_addr(LOCALHOST);  // way of defining s_addr for internal connection (localhost)
    server.sin_port = htons(PORT);

    // Bind the socket with the server address
    if(bind(s, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR)
    {
        printf("Bind failed with error code : %d" , WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    puts("Bind done");

    printf("Parent process of shared memory starts... \n");
    // TODO check the difference between managed_shared_memory and windows_shared_memory
    //No need to explicitly remove the share memory segments due to use of native windows shared memory mechanism
    //Create a native windows shared memory object.
    windows_shared_memory shm (create_only, "FirstSharedMemory", read_write, 4096);
    // managed_shared_memory managed_shm{open_or_create, "shm", 1024};

    //Map the whole shared memory in this process
    mapped_region region(shm, read_write);

    //keep listening for incoming data from robot and operating on them until robot stops sending msg or server itself
    // explicitly terminates
    while(true)
    {
        printf("Server waiting for data...");
        fflush(stdout);

        //clear the buffer by filling null, since it might have previously received data
        memset(buf,'\0', BUFLEN);

        //try to receive some data, this is a blocking call
        if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == SOCKET_ERROR)
        {
            printf("recvfrom() failed with error code : %d" , WSAGetLastError());
            exit(EXIT_FAILURE);
        }
        //print out details of the client_ip_address/port and the data received
        //set the after-tailor element of buf as null
        buf[recv_len] = '\0';
        printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        printf("Server receives:\n%s\n\n" , buf);

        // Task 1: Write message to the shared memory
        // copy buf to mapped region of this process
        /*
         * When a C++ class instance placed in a mapped region has a pointer pointing to another object also placed in
         * the mapped region, since the pointer stores an absolute address, offset pointer should be used. But it is not
         * necessary in this case, since buf is a char array.
         */
        std::memcpy(region.get_address(), buf, strlen(buf));


        // Task 2: Extract IPOC and current actual position and then send them back
        // TODO to include the control signal

        // Parse buf (char array of xml config) using Xerces-C++ XML Parser
        // record initial time for parsing
        auto start = std::chrono::high_resolution_clock::now();
        // convert buf from char array to string
        std::string s_b(buf);
        GetConfig appConfig;
        appConfig.readConfigOnFly(s_b);
        errCode = appConfig.createDOMString();

        /* Three ways of outputting the resulting DOM document
         * 1. Output to on console [debug](for debugging) or a file[not implemented](for debugging)
         * 2. Output to memory [run]
         */
        //std::string action(argv[1]);  // ros parameter is fetched instead of argument being simply fetched when using ros
        // compare the param assigned with the value provided from command line with mode "debug" / "run"
        if ( param.compare("debug") == 0 ){
            printf("debug mode is activated");
            printf("Server sends after processing: \n");
            appConfig.DoOutput2Stream(appConfig.doc, true);
            //appConfig.DoOutput2Stream(appConfig.doc, "../result.xml");  // if saving to a file is desired
            //now reply the client
            if (sendto(s, (const char *)appConfig.output, strlen((const char *)appConfig.output),
                       0, (struct sockaddr*) &si_other, slen) == SOCKET_ERROR)
            {
                printf("sendto() failed with error code : %d" , WSAGetLastError());
                exit(EXIT_FAILURE);
            }
        } else if ( param.compare("run") == 0 ){
            appConfig.DoOutput2Stream(appConfig.doc, false);
            // TODO to check the memory de-allocation of class appConfig!
            printf("the length of c_ptr is: %zu \n", strlen(appConfig.c_ptr));
            //now reply the client
            if (sendto(s, appConfig.c_ptr, strlen(appConfig.c_ptr),
                       0, (struct sockaddr*) &si_other, slen) == SOCKET_ERROR)
            {
                printf("sendto() failed with error code : %d" , WSAGetLastError());
                exit(EXIT_FAILURE);
            }
        }
        printf("\nMessage to robot is sent.\n");
        auto stop = std::chrono::high_resolution_clock::now();
        // Subtract stop and start time points and
        // cast it to required unit. Predefined units
        // are nanoseconds, microseconds, milliseconds,
        // seconds, minutes, hours.
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "Time taken from server receiving msg to sending response: "
             << duration.count() << " milliseconds" << std::endl;
    }

    closesocket(s);
    WSACleanup();

    return errCode;
}
