//
// Created by Lenovo on 2/16/2022.
//
/*
 * Helpful guide for using boost::bind to pass multiple arguments to subscriber callback function
 * --> Official instruction on boost::bind: https://www.boost.org/doc/libs/1_78_0/libs/bind/doc/html/bind.html#bind.examples.using_bind_with_standard_algorit
 * --> Useful post: https://answers.ros.org/question/11810/how-to-pass-arguments-tofrom-subscriber-callback-functions/
 */

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <boost/interprocess/windows_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/bind/bind.hpp>

using namespace boost::interprocess;


/*
 * \brief callback method should be called when receiving the full 3d pose, which then publishes the dummy control
 * signal, which consists of the corrected positions
 * \param ps3d shared Pointer to the incoming full 3d pose
 * \param m_region reference to the mapped region in this process
 * \return None
 */
void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& ps3d, const mapped_region& m_region){
    ROS_INFO("Received corrected position: X=%.3f, Y=%.3f, Z=%.3f; orientation quaternion: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
             ps3d->pose.position.x, ps3d->pose.position.y, ps3d->pose.position.z,
             ps3d->pose.orientation.x, ps3d->pose.orientation.y, ps3d->pose.orientation.z, ps3d->pose.orientation.w);
    // define and initiate rkorrs array
    double rkorrs[3]{0.0, 0.0, 0.0};
    // assign the positions to rkorrs
    rkorrs[0] = ps3d->pose.position.x;
    rkorrs[1] = ps3d->pose.position.y;
    rkorrs[2] = ps3d->pose.position.z;
    // update the mapped region of this process with rkorrs (corrected positions)
    std::memcpy(m_region.get_address(), rkorrs, 3*sizeof(double));
//    ROS_INFO("the first item of rkorrs is: %.3f", *(static_cast<double*>(m_region.get_address()))); // TODO to be removed
}


/*
 * \brief Dummy controller for receiving pose and sending control signal - corrected positions
 *
 * \param argc An integer argument count of the command line arguments
 * \param argv An argument vector of the command line arguments
 * \return Status of the main program
 */
int main(int argc, char **argv){
    //Create a native windows shared memory object.
    //Note: No need to explicitly remove the share memory segments due to use of native windows shared memory mechanism
    windows_shared_memory shm (create_only, "SecondSharedMemory", read_write, 1024);
    //Map the whole shared memory in this process
    mapped_region region(shm, read_write);

    // initialize ROS with specifying the name of node constructed in this process
    ros::init(argc, argv, "subscribe_ctrl_signal_node");
    // create a handle to this process's node
    ros::NodeHandle n;
    // create a function obj based on positionCallback, but with its arguments extended with mapped region
    auto boundPositionCallback = boost::bind(positionCallback, boost::placeholders::_1, boost::ref(region));
    // TODO to set the queue size reasonably
    // let the node subscribe the ros msg - 3d pose without explicit rotation - over topic "control_signal"
    ros::Subscriber delta_positions_sub = n.subscribe<geometry_msgs::PoseStamped>("control_signal",
                                                                  100,
                                                                  boundPositionCallback);
    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    // Use spin other spinOnce in a loop due to the fact that we only need to respond to the callback
    ros::spin();
    //ros::shutdown();
    return 0;
}