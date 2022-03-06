//
// Created by rukangxu on 15.12.21.
//


#include "ros/ros.h"
#include <boost/interprocess/windows_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <cstring>
#include <cstdlib>
#include <string>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <fstream>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <chrono>
#include "ProcessMsgKIConfig.h" // Add an automatically generated configuration file


namespace fs = std::filesystem;

std::string readFileToString(const std::string& path) {
    std::ifstream input_file(path);
    if (!input_file.is_open()) {
        std::cerr << "Could not open the file - '"
             << path << "'" << std::endl;
        exit(EXIT_FAILURE);
    }
    return std::string((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
}

/*
 * Ref. to Sharing memory between processes: https://www.boost.org/doc/libs/1_44_0/doc/html/interprocess/sharedmemorybetweenprocesses.html
 * Ref. to boost::interprocess::offset_ptr: https://www.boost.org/doc/libs/1_78_0/doc/html/interprocess/offset_ptr.html
 */

// TODO To figure out how to deal with mapped_region constructor w.r.t. mapping offset and address limitations and smart pointers
/*
 * \brief Processing message from robot which is stored in shared memory
 *
 * \param argc An integer argument count of the command line arguments
 * \param argv An argument vector of the command line arguments
 * \return Status of the main program
 */
int main(int argc, char *argv[])
{
    using namespace boost::interprocess;
    ros::init(argc, argv, "generate_robot_msg_test");
    ros::NodeHandle nh;

    //Parent process
    printf("Parent process starts...\n");
    fs::path current_pth = fs::current_path();
    fs::path configFile;
    // specify the file path saving the testing xml data
    #ifdef _WIN32
        // on Windows platform
        configFile = fs::current_path() / "install" / "share" / NAMEOFPROJECT / "sample_kuka.xml";
    #else
        // on Linux platform
        // TODO to be tested
        configFile = fs::current_path() / ".." / "share" / NAMEOFPROJECT / "sample_kuka.xml";
    #endif
//    if ((current_pth.string().find("Debug"))==std::string::npos)
//        // POSIX platform
//        configFile = fs::current_path() / ".." / "sample_kuka.xml";
//    else
//        // Windows platform
//        configFile = fs::current_path() / ".." / ".." / "sample_kuka.xml";
    // read the data in the given file into string
    std::string configString = readFileToString( std::string(configFile.u8string()) );
    std::cout << "the current testing config file's path is:\n" << std::string(configFile.u8string()) << "\n\n"
         << "the xml data in string format is:\n" << configString << "\n" << std::endl;
    const char* buf = configString.c_str();

    //No need to explicitly remove the share memory segments due to use of native windows shared memory mechanism
    //Create a native windows shared memory object.
    windows_shared_memory shm (create_only, "FirstSharedMemory", read_write, 4096);

    //Map the whole shared memory in this process's address space by creating a mapped_region object
    mapped_region region(shm, read_write);

    ros::Rate loop_rate(1);
    unsigned int count = 0;
    while (ros::ok()){
        // When a C++ class instance placed in a mapped region has a pointer pointing to another object also placed in
        // the mapped region, since the pointer stores an absolute address, offset pointer should be used. But it is not
        // necessary in this case, since bug is a char array.
        std::memcpy(region.get_address(), buf, strlen(buf));
        ROS_INFO("%s_%d", "refresh", count);
        ++count;
        loop_rate.sleep();
    }

//    auto start = std::chrono::high_resolution_clock::now();
//    // to force the thread of client sending and receiving message sleep for 3 seconds
//    Sleep(3000);
//    auto stop = std::chrono::high_resolution_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
//    std::cout << "The thread actually sleeps for: "
//         << duration.count() << " milliseconds" << std::endl;

    return 0;
}
