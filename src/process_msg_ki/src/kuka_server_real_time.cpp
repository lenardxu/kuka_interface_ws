//
// Created by Lenovo on 3/5/2022.
// UDP Server
//
/*
 * Helpful guide for beginner of using winsock2:
 *   https://docs.microsoft.com/en-us/windows/win32/winsock/getting-started-with-winsock
 * =====================================================================================================================
 * Helpful guide for choosing xml library implemented in c++:
 *   https://stackoverflow.com/questions/9387610/what-xml-parser-should-i-use-in-c, options at disposal:
 *   --> xerces (does not satisfy the real-time requirement)
 *   [
 *   If xerces is not present in your OS, then the following should be helpful in terms of building from source.
 *      Downloading resources for xerces: https://archive.apache.org/dist/xerces/c/3/sources/
 *      Official instruction for xerces building:
 *      https://xerces.apache.org/xerces-c/build-3.html  or,
 *      another useful instruction for xerces building:
 *      http://www.yolinux.com/TUTORIALS/XML-Xerces-C.html
 *   ]
 *   --> pugixml (a ver high-performing xml parser)[Ref.: https://www.aosabook.org/en/posa/parsing-xml-at-the-speed-of-light.html]
 *   --> STL: regex (less flexible and powerful to deal with complex and dynamic xml data but fast enough to simple and fixed xml data)
 * =====================================================================================================================
 * Helpful guide for synchronization mechanisms in Boost.Interprocess:
 * synchronization mechanisms: https://www.boost.org/doc/libs/1_78_0/doc/html/interprocess/synchronization_mechanisms.html
 * --> synchronization objects in question: mutex, condition variables, semaphores and file locks
 *     --> respective occasions of applying the first three synchronization objects:
 *         https://stackoverflow.com/questions/3513045/conditional-variable-vs-semaphore
 * Use managed_shared_memory to access shared memory and construct vector (as an example):
 * https://stackoverflow.com/questions/64204744/c-server-client-boostinterprocess-array-access-in-shared-memory
 */


// Server side implementation of UDP client-server model
#include "ros/ros.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <chrono>
#include <winsock2.h>
#include <regex>
#include <boost/interprocess/windows_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>


// tell linker to add "ws2_32.lib" library (Winsock2 Library) to the list of lib dependencies
// This is specific to MSVC
#pragma comment(lib, "ws2_32.lib")

#define PORT     49152  // The port on which to listen for incoming data
#define BUFLEN   4096  // Max length of buffer
#define HOST     "0.0.0.0"  // any address for binding
#define LOCALHOST "127.0.0.1"  // the local host via the loopback device

using namespace boost::interprocess;

/*
 * \brief extract the ipoc from the given tags of the input sequence - xml message
 * \param input sequence - xml message
 * \param pattern specifying ipoc tag
 * \param result storing the resulting info after regex search from the input sequence
 * \return none
 */
void extract_ipoc(const std::string& input,
                  const std::regex& r_ipoc, std::smatch& result_ipoc){
    if (std::regex_search(input, result_ipoc, r_ipoc)){
        //std::cout << "Text for IPOC:\n" << result_ipoc.str() << std::endl;
    }
}


/*
 * \brief First, it implements the function of udp network programming based on windows-specific library - winsock2
 * Secondly, it saves the received robot message into shared memory so that another process can access and process it
 * efficiently. Meanwhile, it extracts the tag <IPOC> part of the same robot message using regex (STL) which will be then
 * combined with the incoming control signal and sent back together to robot
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

    //TODO for testing the time cost of extracting ipoc, writing msg into shm and inserting control signal for msg return
    unsigned round = 0;
    float time_consumed = 0.0;

    printf("\nServer main function starts...");
    fflush(stdout);

    /*
     * Note for using regular expression:
     * The "program" that a regular expression represents is compiled at run time, not at compile time. (Compiling a
     * regular expression can be a surprisingly slow operation, especially when you're using the extended regex grammar
     * or using complicated expressions). So you should try to avoid creating more *regex* objects than needed and using
     * a regular expression in a loop.
     */
    // define pattern and regular expression for finding IPOC and capture the error of expression during running if any
    std::string pattern_ipoc("(<IPOC>)([[:digit:]]+)(</IPOC>)");
    std::regex r_ipoc;
    try{
        r_ipoc.assign(pattern_ipoc, std::regex::icase);
    } catch (std::regex_error& e) {
        std::cout << "When using regular expression for finding IPOC:\n" <<
                  e.what() << "\ncode: " << e.code() << std::endl;
    }


    // assign the suffix (fixed normally) of the xml message to be sent back to robot after processing
    std::string msg_suffix = R"(</Sen>)";
    // define the string elements standing for RKorr nodes consisting of tags and texts
    std::string msg_rkorry_pre = R"(<RKorrY>)";
    std::string msg_rkorry_val = R"(0.0)";
    std::string msg_rkorry_suf = R"(</RKorrY>)";
    std::string msg_rkorrx_pre = R"(<RKorrX>)";
    std::string msg_rkorrx_val = R"(0.0)";
    std::string msg_rkorrx_suf = R"(</RKorrX>)";
    std::string msg_rkorrz_pre = R"(<RKorrZ>)";
    std::string msg_rkorrz_val = R"(0.0)";
    std::string msg_rkorrz_suf = R"(</RKorrZ>)";

    // create a SOCKET object called s for the server to listen for client connections
    SOCKET s;
    // declare the size of buffer of data sent by client, in bytes
    // declare the size of buffer of actually received from client, in bytes
    int slen , recv_len;
    // create a buffer for the incoming data with given max. length BUFLEN, in bytes
    char buf[BUFLEN];
    // declare the variable holding the addresses of server and client
    struct sockaddr_in server, si_other;
    // create a WSADATA object called wsa
    WSADATA wsa;
    // get the size of si_other in bytes
    slen = sizeof(si_other);

    printf("\nInitialising Winsock...");
    fflush(stdout);
    /*
     * Initialize winsock by calling WSAStartup which is used for initiating use of WS2_32.dll and return its value as an
     * integer and check for errors.
     *   Arg: The MAKEWORD(2,2) parameter of WSAStartup makes a request for version 2.2 of Winsock on the system, and sets
     *   the passed version as the highest version of Windows Sockets support that the caller can use.
     *   Arg: The WSADATA structure contains information about the Windows Sockets implementation.
     */
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        // printf("Failed. Error Code : %d",WSAGetLastError());  // should not be used to retrieve error code
        std::cout<<"WSAStartup error"<<std::endl;
        exit(EXIT_FAILURE);
    }
    printf("Initialised.\n");

    /*
     * Create a socket for connecting to server and return its value and check for errors.
     * If no error occurs, socket returns a descriptor referencing the new socket. Otherwise, a value of INVALID_SOCKET
     * is returned, and a specific error code can be retrieved by calling WSAGetLastError.
     */
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
    //server.sin_addr.s_addr = htonl(INADDR_ANY);
    server.sin_addr.s_addr = inet_addr(LOCALHOST);  // way of defining s_addr for internal connection (localhost)
    server.sin_port = htons(PORT);

    /* Bind the socket with the server address and use its returned value for checking error
     * Arg: A descriptor identifying an unbound socket.
     * Arg: A pointer to a sockaddr structure of the server address to assign to the bound socket.
     * Arg: The length, in bytes, of the value pointed to by the parameter.
     */
    // (struct sockaddr *)&server
    if( bind(s ,(struct sockaddr *)&server , sizeof(server)) == SOCKET_ERROR)
    {
        printf("Bind failed with error code : %d" , WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    puts("Bind done");


    // Create a native windows shared memory object.
    //Note: No need to explicitly remove the share memory segments due to use of native windows shared memory mechanism
    printf("Parent process of first shared memory starts... \n");
    windows_shared_memory shm(create_only, "FirstSharedMemory", read_write, 4096);
    // managed_shared_memory managed_shm{open_or_create, "shm", 1024};
    // Map the whole shared memory in this process
    mapped_region region(shm, read_write);
    printf("Waiting for data...");
    fflush(stdout);


    // Keep listening for incoming data from robot and operating on them until robot stops sending msg or server itself
    // explicitly terminates. In addition, since this node subscribes to nothing, there is no need to apply spinning.
    while(true)
    {
//        printf("Waiting for data...");
//        fflush(stdout);

        //clear the buffer by filling null, it might have previously received data
        memset(buf,'\0', BUFLEN);

        //receive a datagram and stores the source address into si_other (this is a blocking call)
        if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == SOCKET_ERROR)
        {
            printf("recvfrom() failed with error code : %d" , WSAGetLastError());
            exit(EXIT_FAILURE);
        }

        // start counting time when receiving msg from robot
        auto start = std::chrono::high_resolution_clock::now();
//        //print out details of the client/peer and the data received
//        buf[recv_len] = '\0';
//        printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
//        printf("Server receives:\n%s\n" , buf);


        /*
         * Note of offset pointer:
         * When a C++ class instance placed in a mapped region has a pointer pointing to another object also placed in
         * the mapped region, since the pointer stores an absolute address, offset pointer should be used. But it is not
         * necessary in this case, since bug is a char array.
         */
//      // start counting time consumed for updating the mapped region with buf (message received from robot)
//        auto start_at_shm_write = std::chrono::high_resolution_clock::now();
        // Task 1: update the mapped region of this process with buf (message received from robot)
        std::memcpy(region.get_address(), buf, strlen(buf));
//      // stop counting time consumed for updating the mapped region with buf (message received from robot)
//        auto stop_at_shm_write = std::chrono::high_resolution_clock::now();


        // Task 2: Extract ipoc, combine it with control signal read in the second shared memory and send them back together
        // assign the prefix (fixed normally) of the xml message to be sent back to robot after processing
        std::string msg = R"(<?xml version="1.0"?><Sen Type="ImFree">)";
//        // The following commented lines serves only for testing
//        float rkorrx = 0.999;
//        float rkorry = 0.998;
//        float rkorrz = 0.997;
//        // convert the control signal in float to string in order to be sent back
//        msg_rkorrx_val = std::to_string(rkorrx);
//        msg_rkorry_val = std::to_string(rkorry);
//        msg_rkorrz_val = std::to_string(rkorrz);
        // create an "empty" mapped region (Address will be 0 (nullptr). Size will be 0.) in this process
        //mapped_region region_2;
        // there should be no shared memory object named "SecondSharedMemory" at the beginning when running this process
        try
        {
            // Open already created shared memory object.
            printf("Child process of second shared memory starts... \n");
            windows_shared_memory shm_2(open_only, "SecondSharedMemory", read_only);
            // Map the whole shared memory in this process
            mapped_region region_2(shm_2, read_only);
            // Swap the region_2 with region_temp
            //region_2.swap(region_temp);
            // obtain the address of mapped region
            // TODO the current code block below does not guarantee that the rkorrx, rkorry and rkorrz remain the last
            //  values when the corresponding shm (created by another process) is closed. Instead it sets them again as zeros.
            //  Please consult Niklas for its validity.
            double *rkorr_pbeg = static_cast<double*>(region_2.get_address());
            // convert the control signal in double to string without precision loss and then assign it to the predefined
            // rkorr values (0.0, 0.0, 0.0)
            // https://thispointer.com/convert-double-to-string-in-c-3-ways/
            msg_rkorrx_val = std::to_string(*rkorr_pbeg);
            msg_rkorry_val = std::to_string(*(rkorr_pbeg + 1));
            msg_rkorrz_val = std::to_string(*(rkorr_pbeg + 2));
            //printf("the rkorrs are: %.3f, %.3f, %.3f", *rkorr_pbeg, *(rkorr_pbeg + 1), *(rkorr_pbeg + 2));
        }
        catch (const interprocess_exception &ex) {  //std::exception
            std::cout << "When opening the second windows_shared_memory ex: "  << ex.what()
                      << "\ncode: " << ex.get_error_code() << "\n";
        }

        // convert the buffer containing the char array of xml config to string and then parse it using regex
        std::string s_b(buf);

        // declare the result for ipoc after using regex_search method later
        std::smatch result_ipoc;
        // extract the ipoc from the given tags of the input sequence - xml message from robot
        extract_ipoc(s_b, r_ipoc, result_ipoc);
        // arrange the ipoc and control signal into the msg of regulated format
        msg.append(msg_rkorry_pre);
        msg.append(msg_rkorry_val);
        msg.append(msg_rkorry_suf);
        msg.append(msg_rkorrx_pre);
        msg.append(msg_rkorrx_val);
        msg.append(msg_rkorrx_suf);
        msg.append(msg_rkorrz_pre);
        msg.append(msg_rkorrz_val);
        msg.append(msg_rkorrz_suf);
        msg.append(result_ipoc.str());
        msg.append(msg_suffix);
//        std::cout << msg << std::endl;

        //now reply the client
        if (sendto(s, msg.c_str(), strlen(msg.c_str()),
                   0, (struct sockaddr*) &si_other, slen) == SOCKET_ERROR)
        {
            printf("sendto() failed with error code : %d" , WSAGetLastError());
            exit(EXIT_FAILURE);
        }
//        printf("\nMessage to robot is sent.\n");


        // stop counting time when sending msg back to robot
        auto stop = std::chrono::high_resolution_clock::now();
        /*
         * Note of std::chrono::duration:
         * Subtract stop and start timepoints and cast it to required unit. Predefined units are nanoseconds, microseconds,
         * milliseconds, seconds, minutes, hours.
         */
//        auto duration_shm_write = std::chrono::duration_cast<std::chrono::milliseconds>(stop_at_shm_write - start_at_shm_write);
//        std::cout << "Time taken for writing into shm: "
//                  << duration_shm_write.count() << " milliseconds" << std::endl;
        /*
         * compute the time consumed for processing the robot message from reception to feedback which contains:
         * 1. writing robot msg to the first shared memory
         * 2. extracting ipoc, reading control signal from the second shared memory and then combining it with ipoc and
         * finally sending them back together
         */
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "Time taken from server receiving msg to sending response: "
                  << duration.count() << " milliseconds" << std::endl;

        // compute the duration on average
        time_consumed += duration.count();
        ++round;
        printf("the time consumed from server receiving msg to sending response after %d rounds on average: %fms\n",
               round, time_consumed/float(round));
    }

    closesocket(s);
    WSACleanup();

    return 0;
}
