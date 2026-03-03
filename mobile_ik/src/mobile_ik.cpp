#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#define WHEEL_SEPARATION        (3.0)
#define WHEEL_RADIUS            (0.5)

std_msgs::Float64 left_vel;
std_msgs::Float64 right_vel;
geometry_msgs::Vector3 vel_in;
ros::Publisher right_vel_publisher;
ros::Publisher left_vel_publisher;
ros::Subscriber local_vel_subscriber;

void velocity_recv_callback(const geometry_msgs::Vector3& msg)
{
    vel_in = msg;
    left_vel.data = (vel_in.x - (WHEEL_SEPARATION/2.0)*vel_in.z) / WHEEL_RADIUS;
    right_vel.data = (vel_in.x + (WHEEL_SEPARATION/2.0)*vel_in.z) / WHEEL_RADIUS;
    right_vel_publisher.publish(right_vel);
    left_vel_publisher.publish(left_vel);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile_ik"); /* This is how you create a node. The only thing that
    changes for this function is the string argument which indicates the name of the node.*/
    ros::NodeHandle n; /* This creates something called an object that is used in object-
    oriented programming. This object is responsible for handling the publications and 
    subscriptions that this node talks to.*/

    right_vel_publisher = n.advertise<std_msgs::Float64>("right_wheel_ang_vel", 1);
    left_vel_publisher = n.advertise<std_msgs::Float64>("left_wheel_ang_vel", 1);

    /* This creates a publisher object. This is done by calling the function n.advertise which
    is a function tied to the object that we created earlier. The <> brackets indicate what
    message type is going to be used for this topic. The first argument is the name of the 
    topic. The "1" argument says to use a buffer size of 1; could make larger, if expect network 
    backups */

    local_vel_subscriber = n.subscribe("/local_velocities", 1, velocity_recv_callback);
    /* This creates a subscriber object. This is done by calling the function n.subscribe which
    is also a function tied to the object of class ros::NodeHandle. The first argument is the 
    name of the topic that you want to subscribe to. The second argument is the buffer size which
    we will just set to 1. The third argument is the callback function that gets called when data
    is published to this topic. This works like an interupt.*/

    while (ros::ok())
    {
        
        ros::spinOnce();
    }
}