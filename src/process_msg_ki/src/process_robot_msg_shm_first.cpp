//
// Created by Lenovo on 2/10/2022.
//
/*
 * References:
 *   - Visualizing quaternions: https://eater.net/quaternions
 */

#include <boost/interprocess/windows_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <cstring>
#include <cstdlib>
#include <string>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <chrono>
#include <regex>

#include "ros/ros.h"
//#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/PoseStamped.h>


/*
 * \brief extract the r_ist from the given tags of the input sequence - xml message
 * \param input sequence - xml message
 * \param r_ist pattern specifying r_ist tag
 * \param result_rist result storing the resulting info after regex search from the input sequence
 * \return none
 */
void extract_rist(const std::string& input,
                  const std::regex& r_ist, std::smatch& result_rist){
    if (std::regex_search(input, result_rist, r_ist))
        std::cout << "Attribute for Robot Ist:\n" << result_rist.str() << "\n";
    std::cout << std::endl;
}

/*
 * \brief convert combination of roll angle, pitch angle and yaw angle to quaternion representation
 * \param roll rotated angle (X-axis)
 * \param pitch rotated angle (Ý-axis)
 * \param yaw rotated angle (Z-axis)
 * \return q quaternion
 */
geometry_msgs::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) {
    // yaw (Z), pitch (Y), roll (X)
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
    // http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html
    geometry_msgs::Quaternion q;
    double t0 = cos(yaw * 0.5);
    double t1 = sin(yaw * 0.5);
    double t2 = cos(roll * 0.5);
    double t3 = sin(roll * 0.5);
    double t4 = cos(pitch * 0.5);
    double t5 = sin(pitch * 0.5);
    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q;
}

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

    // initialize ROS with specifying the name of node constructed in this process
    ros::init(argc, argv, "process_robot_msg_node");
    // create a handle to this process' node
    ros::NodeHandle nh;
    //TODO to set the queue size reasonably
    //ros::Publisher pose_pub = nh.advertise<std_msgs::Float32MultiArray>("r_ist", 100);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("r_ist", 100);

    //Open already created shared memory object.
    printf("Child process of shared memory starts... \n");
    windows_shared_memory shm (open_only, "FirstSharedMemory", read_only);

    //Map the whole shared memory in this process
    mapped_region region(shm, read_only);

    // define pattern and regular expression for finding R_Ist and capture the error of expression during running if any
    std::string pattern_rist(R"((<RIst)([ X="]+)([0-9.]+)([ Y="]+)([0-9.]+)([ Z="]+)([0-9.]+))"
                             R"(([ A="]+)([0-9.]+)([ B="]+)([0-9.]+)([ C="]+)([0-9.]+)("/>))");
    std::regex r_rist;
    try{
        r_rist.assign(pattern_rist, std::regex::icase);
    } catch (std::regex_error& e) {
        std::cout << "When using regular expression for finding R_Ist:\n" <<
                  e.what() << "\ncode: " << e.code() << std::endl;
    }

    // specify a frequency that you would like to loop at, i.e., 0.5[hz]
    //TODO to set the loop rate reasonably
    ros::Rate loop_rate(5);

    // By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause ros::ok() to
    // return false if that happens.
    while (ros::ok()){
        // start counting time consumed for processing robot msg read from shared memory
        auto start = std::chrono::high_resolution_clock::now();
        //obtain the address of mapped region
        char *mem = static_cast<char*>(region.get_address());
        //printf("the msg wrote in shared memory can be read as:\n");
        //puts(mem);
//        // check the shared memory object read from mapped region
//        char *mem_copy;
//        int size_check = 0;
//        std::string min_msg = R"(<Rob Type="KUKA"> </Rob>)";
//        for(mem_copy = mem; mem_copy != '\0'; mem_copy++){
//            size_check++;
//        }
//        if (size_check <= min_msg.size())
//            return 1;   //Error checking memory
//        printf("shm test succeeded");

        if ( *mem != '\0' ) {
            ROS_INFO("the msg wrote in shared memory can be read as:\n%s", mem);
            // convert the buffer containing the char array of xml config to string and then parse it using xercesc
            std::string s_b(mem);
            // declare the result for ipoc after using regex_search method later
            std::smatch result_rist;
            // extract the ipoc from the given tags of the input sequence - xml message from robot
            extract_rist(s_b, r_rist, result_rist);
            // declare the ros msg of format std_msgs::Float32MultiArray to be sent to the controller
//        std_msgs::Float32MultiArray msg;
//        unsigned int target_id_arr[] = {3,5,7,9,11,13};
//        for ( auto id : target_id_arr ){
//            //std::cout << stof(result_rist[id].str()) << "(float), ";
//            msg.data.emplace_back(stof(result_rist[id].str()));
//        }
//        ROS_INFO("position: X=%.2f, Y=%.2f, Z=%.2f; orientation: A=%.2f, B=%.2f, C=%.2f",
//                 msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5]);

            // declare the full 3d pose ros msg to be sent to the controller
            geometry_msgs::PoseStamped msg;
            // assign the extracted r_ist values to position and orientation quaternions
            msg.pose.position.x = std::stod(result_rist[3].str());
            msg.pose.position.y = std::stod(result_rist[5].str());
            msg.pose.position.z = std::stod(result_rist[7].str());
            msg.pose.orientation = createQuaternionFromRPY(std::stod(result_rist[9].str()),
                                                           std::stod(result_rist[11].str()),
                                                           std::stod(result_rist[13].str()));
            // TODO check whether A=roll angle, B=pitch angle, C=yaw angle?
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            ROS_INFO("position: X=%.2f, Y=%.2f, Z=%.2f; orientation: A=%.2f, B=%.2f, C=%.2f",
                     std::stod(result_rist[3].str()), std::stod(result_rist[5].str()), std::stod(result_rist[7].str()),
                     std::stod(result_rist[9].str()), std::stod(result_rist[11].str()), std::stod(result_rist[13].str()));
            ROS_INFO("xml message decomposed.\nAnd time takes: %.2f milliseconds", duration.count());
            ROS_INFO("position: X=%.2f, Y=%.2f, Z=%.2f; orientation quaternion: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                     msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                     msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
            // publish the full 3d pose ros msg
            pose_pub.publish(msg);
            // this spinning is removable since the corresponding node subscribes to nothing
            ros::spinOnce();
            loop_rate.sleep();
        } else {
            ROS_INFO("waiting for msg in shared memory...");
            loop_rate.sleep();
        }
    }

    return 0;
}