//
// Created by Lenovo on 2/10/2022.
//

#include <boost/interprocess/windows_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <cstring>
#include <cstdlib>
#include <string>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <fstream>
#include "parser.hpp"
#include <chrono>

#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>


using namespace std;

int main(int argc, char *argv[])
{
    using namespace boost::interprocess;

    ros::init(argc, argv, "process_robot_msg_node");
    ros::NodeHandle nh;
    //TODO to set the queue size reasonably
    ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("r_ist", 10);

    //Open already created shared memory object.
    printf("Child process of shared memory starts... \n");
    windows_shared_memory shm (open_only, "FirstSharedMemory", read_only);

    //Map the whole shared memory in this process
    mapped_region region(shm, read_only);

    // error Code indicating whether there is a failure when creating DOM string from the extracted info of xml messgae
    // here initially set as 1 to indicate error state
    int errCode = 1;
    // specify a frequency that you would like to loop at, i.e., 0.5[hz]
    //TODO to set the loop rate reasonably
    ros::Rate loop_rate(0.5);
    // By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause ros::ok() to
    // return false if that happens.
    while (ros::ok()){
        //obtain the address of mapped region
        char *mem = static_cast<char*>(region.get_address());
        //printf("the msg wrote in shared memory can be read as:\n");
        //puts(mem);
        ROS_INFO("the msg wrote in shared memory can be read as:\n%s", mem);
//        for(std::size_t i = 0; i < region.get_size(); ++i)
//            if(*mem++ != 1)
//                return 1;   //Error checking memory
//        printf("shm test succeeded");

        auto start = chrono::high_resolution_clock::now();
        // convert the buffer containing the char array of xml config to string and then parse it using xercesc
        string s_b(mem);
        GetConfig appConfigTest;
        appConfigTest.readConfigOnFly(s_b);
        errCode = appConfigTest.createDOMString();
        // TODO to check if it is right to use ros::shutdown() within the loop
        if (errCode!=0){
            ROS_INFO("%s", "the node is to be shut down manually");
            ros::shutdown();
        }
        appConfigTest.DoOutput2Stream(appConfigTest.doc, false);
        printf("\nthe length of the parsed xml data is: %zu ", strlen(appConfigTest.c_ptr));
        cout << endl << "STL map contents:" << endl;

        std_msgs::Float32MultiArray msg;
        for ( auto iter = appConfigTest.extract_data.begin();
              iter != appConfigTest.extract_data.end(); ++iter ) {
            cout << "Tag: " << iter->first << ", Extracted value: ";
            cout << "the size of the vector is: " << iter->second.size() << endl;
            // convert to float (std::stof) (<-> float32 in ros built-in types)
            // or double (std::stod) (<-> float64 in ros built-in types)
            for (auto it=iter->second.begin(); it!=iter->second.end();++it) {
                cout << stof(*it) << "(float), ";  // for test
                msg.data.emplace_back(stof(*it));
            }
        }
        cout << endl;
        auto stop = chrono::high_resolution_clock::now();
        // TODO to check why the resulting time cost is zero millisecond
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        ROS_INFO("xml message decomposed.\nAnd time takes: %.2f milliseconds", duration.count());

        ROS_INFO("position: X=%.2f, Y=%.2f, Z=%.2f; orientation: A=%.2f, B=%.2f, C=%.2f",
                 msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]);
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return errCode;
}