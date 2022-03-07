//
// Created by Lenovo on 3/6/2022.
//

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

class ControlNode
{
    // create a handle to this process's node
    ros::NodeHandle n;
    // create a subscriber
    ros::Subscriber positions_sub;
    // create a publisher
    ros::Publisher delta_positions_pub;
    // declare the ros msg to be sent back to input device
    geometry_msgs::PoseStamped output_msg;

    /*
     * \brief callback method should be called when receiving the full 3d pose, which then publishes the dummy control
     * signal, which consists of the corrected positions
     * \param ps3d const shared Pointer to the incoming full 3d pose
     * \return None
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& ps3d){
        ROS_INFO("Received position: X=%.2f, Y=%.2f, Z=%.2f; orientation quaternion: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                 ps3d->pose.position.x, ps3d->pose.position.y, ps3d->pose.position.z,
                 ps3d->pose.orientation.x, ps3d->pose.orientation.y, ps3d->pose.orientation.z, ps3d->pose.orientation.w);
        output_msg.pose.position.x = 0.001;
        output_msg.pose.position.y = 0.001;
        output_msg.pose.position.z = 0.001;
        delta_positions_pub.publish(output_msg);
    }

public:
    // default constructor
    ControlNode(){
        // let the node subscribe the ros msg - 3d pose - over topic "r_ist"
        positions_sub = n.subscribe<geometry_msgs::PoseStamped>("r_ist", 1000, &ControlNode::poseCallback, this);
        // let the node publish the ros msg - corrected position - over topic "control_signal"
        delta_positions_pub = n.advertise<geometry_msgs::PoseStamped>("control_signal", 1000);
    }
    // destructor
    ~ControlNode(){
    }

};


/*
 * \brief Dummy controller for receiving pose and sending control signal - corrected positions
 *
 * \param argc An integer argument count of the command line arguments
 * \param argv An argument vector of the command line arguments
 * \return Status of the main program
 */
int main(int argc, char **argv){
    // initialize ROS with specifying the name of node constructed in this process
    ros::init(argc, argv, "controller_node");
    // construct the controller node for subscribing and publishing ros msg
    ControlNode control_node;
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