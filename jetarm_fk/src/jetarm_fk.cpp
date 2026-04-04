#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

#define JOINT1          (0)
#define JOINT2          (1)
#define JOINT3          (2)
#define JOINT4          (3)
#define JOINT5          (4)


typedef float q_t;
ros::Subscriber joint_state_sub;
float q1val, q2val, q3val, q4val, q5val;

ros::Publisher endaffector_pub;
geometry_msgs::Vector3 endaffector_pos;

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

void joint_state_recv_callback(const sensor_msgs::JointState& msg)
{
    q1val = msg.position[JOINT1];
    q2val = msg.position[JOINT2];
    q3val = msg.position[JOINT3];
    q4val = msg.position[JOINT4];
    q5val = msg.position[JOINT5];
    endaffector_pub.publish(calc_endaffector_position(q1val, q2val, q3val, q4val, q5val));

}



int main (int argc, char **argv)
{
    ros::init(argc, argv, "jetarm_fk");
    ros::NodeHandle nodeHandle;
    q1val = q2val = q3val = q4val = q5val = 0.f;
    joint_state_sub = nodeHandle.subscribe("/joint_states", 1, joint_state_recv_callback);

    endaffector_pub = nodeHandle.advertise<geometry_msgs::Vector3>("arm_task_space", 1);

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}