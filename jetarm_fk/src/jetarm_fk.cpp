#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

typedef float q_t;
ros::Subscriber joint_state_sub;


ros::Publisher endaffector_pub;
geometry_msgs::Vector3 endaffector_pos;

/*
void joint_state_recv_callback()
{
    
}
*/

geometry_msgs::Vector3 calc_endaffector_position(q_t q1, q_t q2, q_t q3, q_t q4, q_t q5)
{
    geometry_msgs::Vector3 endaff_p;
    q_t xpos = 0.06471*sin(q1 - q2) - 0.05743*sin(q2 - q1 + q3 + q4) - 0.06471*sin(q1 + q2 + q3) - 0.05743*sin(q1 + q2 + q3 + q4) - 0.06471*sin(q1 + q2) - 0.06471*sin(q2 - q1 + q3);
    q_t ypos = 0.06471*cos(q1 + q2 + q3) - 0.05743*cos(q2 - q1 + q3 + q4) - 0.06471*cos(q1 - q2) + 0.05743*cos(q1 + q2 + q3 + q4) + 0.06471*cos(q1 + q2) - 0.06471*cos(q2 - q1 + q3);
    q_t zpos = 0.11486*cos(q2 + q3 + q4) + 0.12942*cos(q2 + q3) + 0.12942*cos(q2) + 0.10432;
    endaff_p.x = xpos;
    endaff_p.y = ypos;
    endaff_p.z = zpos;
    return endaff_p;
}

int main (int argc, char **argv)
{
    ros::NodeHandle nodeHandle;

    // joint_state_sub = nodeHandle.subscribe("/joint_states", 1, joint_state_recv_callback);

    endaffector_pub = nodeHandle.advertise<geometry_msgs::Vector3>("arm_task_space", 1);

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}