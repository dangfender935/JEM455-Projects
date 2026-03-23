#include <ros/ros.h>
#include <pose_est/pose_est_msg.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <math.h>

#define K1      (1)
#define K2      (1)
#define K3      (1)
#define MAX_X_VEL   (100)
#define MAX_TH_VEL  (0.5)
#define DISTANCE_ERROR_THRESHOLD    (1.0)
#define ANGLE_ERROR_THRESHOLD       (0.07)

ros::Subscriber desired_pose_sub;
ros::Subscriber curr_pose_sub;
ros::Publisher local_vel_pub;

geometry_msgs::Vector3 desired_pose{};
geometry_msgs::Vector3 msg_out{};
geometry_msgs::Point curr_pose{};
float desired_global_x = 0.f;
float desired_global_y = 0.f;
float desired_global_th = 0.f;
bool active_state = false;

void desired_pose_recv_callback(const geometry_msgs::Vector3& msg)
{
    ROS_INFO("pose received!!!");
    active_state = true;
    desired_pose = msg;
    desired_global_x = desired_pose.x * cos(curr_pose.z) - desired_pose.y * sin(curr_pose.z) + curr_pose.x;
    desired_global_y = desired_pose.x * sin(curr_pose.z) + desired_pose.y * cos(curr_pose.z) + curr_pose.y;
    desired_global_th = desired_pose.z + curr_pose.z;
    ROS_INFO("Desired global pose: %f %f %f", desired_global_x, desired_global_y, desired_global_th);
    active_state = true;
    return;
}

void curr_pose_recv_callback(const pose_est::pose_est_msg& msg)
{
    if (!active_state) return;
    curr_pose = msg.point;
    float dx = desired_global_x - curr_pose.x;
    float dy = desired_global_y - curr_pose.y;
    float rho = sqrt(pow(dx, 2) + pow(dy, 2));
    float alpha = atan2(dy, dx) - curr_pose.z;
    float eta = desired_global_th - curr_pose.z;
    ROS_INFO("desired global x - curr pose x: %f - %f = %f", desired_global_x, curr_pose.x, dx);
    ROS_INFO("rho = %f", rho);


    if (rho < DISTANCE_ERROR_THRESHOLD) 
    {
        ROS_INFO("Arrived!");
        rho = 0.f;
        alpha = 0.f;
    }
    else if (abs(alpha) < ANGLE_ERROR_THRESHOLD) alpha = 0.f;
    if (abs(eta) < ANGLE_ERROR_THRESHOLD) eta = 0.f;

    float xvel = K1*rho;
    float zvel = K2*alpha + K3*eta;

    // ROS_INFO("(xvel, zvel) = %f %f", xvel, zvel);
    if (xvel > MAX_X_VEL) msg_out.x = MAX_X_VEL;
    else if (xvel < -MAX_X_VEL) msg_out.x = -MAX_X_VEL;
    else msg_out.x = xvel;
    if (zvel > MAX_TH_VEL) msg_out.z = MAX_TH_VEL;
    else if (zvel < -MAX_TH_VEL) msg_out.z = -MAX_TH_VEL;
    else msg_out.z = zvel;

    // msg_out.x = (xvel > 3.0) ? 3.0 : xvel;
    // msg_out.z = (zvel > 3.0) ? 3.0 : zvel;
    ROS_INFO("msg_out = %f %f %f", msg_out.x, msg_out.y, msg_out.z);

    local_vel_pub.publish(msg_out);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_controller");
    ros::NodeHandle nodeHandle;

    desired_pose_sub = nodeHandle.subscribe("pose_controller", 1, desired_pose_recv_callback);
    curr_pose_sub = nodeHandle.subscribe("pose_est", 1, curr_pose_recv_callback);
    local_vel_pub = nodeHandle.advertise<geometry_msgs::Vector3>("local_velocities", 1);

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}