#include <ros/ros.h>
#include <pose_est/pose_est_msg.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt8.h>
#include <keyboard/Key.h> // This is where the keyboard/Key message type is defined
#include <math.h>

#define K1                          (0.75)
#define K2                          (1)
#define K3                          (1)

#define MAX_X_VEL                   (75)
#define MAX_TH_VEL                  (0.4)
#define DISTANCE_ERROR_THRESHOLD    (5.0)
#define ANGLE_ERROR_THRESHOLD       (0.03)
#define OVERSHOOT_THRESHOLD         (0.5)

#define S_KEY		(115)
#define R_KEY		(114)
#define G_KEY		(103)

typedef enum MoveState
{
    STOP,
    CORRECT_DIRECTION_ERROR,
    CORRECT_DISTANCE_ERROR,
    CORRECT_ORIENTATION_ERROR,
} MoveState;

enum PoseRequest
{
    RELOAD = 0,
    NEXT_POSE = 1
};

ros::Subscriber desired_pose_sub;
geometry_msgs::Vector3 desired_pose{};

ros::Subscriber curr_pose_sub;
geometry_msgs::Point curr_pose{};

ros::Subscriber keydown_sub;
keyboard::Key keydown_in;

ros::Publisher local_vel_pub;
geometry_msgs::Vector3 vel_out{};

ros::Publisher pose_request_pub;
std_msgs::UInt8 pose_request{};

float desired_global_x = 0.f;
float desired_global_y = 0.f;
float desired_global_th = 0.f;
MoveState move_state;

void desired_pose_recv_callback(const geometry_msgs::Vector3& msg)
{
    ROS_INFO("============= Pose received! Attempting to reach goal... =============");
    desired_pose = msg;
    // desired_global_x = desired_pose.x * cos(curr_pose.z) - desired_pose.y * sin(curr_pose.z) + curr_pose.x;
    // desired_global_y = desired_pose.x * sin(curr_pose.z) + desired_pose.y * cos(curr_pose.z) + curr_pose.y;
    // desired_global_th = desired_pose.z + curr_pose.z;
    desired_global_x = desired_pose.x;
    desired_global_y = desired_pose.y;
    desired_global_th = desired_pose.z;
    ROS_INFO("Desired global pose: %f %f %f", desired_global_x, desired_global_y, desired_global_th);
    ROS_INFO("======================================================================");
    move_state = CORRECT_DIRECTION_ERROR;
    return;
}

inline float normalize_angle(float angle)
{
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return angle;
}

void curr_pose_recv_callback(const pose_est::pose_est_msg& msg)
{
    float dx = 0.f;
    float dy = 0;
    float rho = 0.f;
    float alpha = 0.f;
    float eta = 0.f;
    float zspeed = 0.f;
    float xspeed = 0.f;
    static float last_rho;
    curr_pose = msg.point;
    switch (move_state)
    {
    case CORRECT_DIRECTION_ERROR:
        dx = desired_global_x - curr_pose.x;
        dy = desired_global_y - curr_pose.y;
        alpha = normalize_angle(atan2(dy, dx) - curr_pose.z);        
        ROS_INFO("ALPHA = %f", alpha);

        if (abs(alpha) < ANGLE_ERROR_THRESHOLD) 
        {
            move_state = CORRECT_DISTANCE_ERROR;
            vel_out.z = 0.f;
            // last_rho = sqrt(pow(dx, 2) + pow(dy, 2)) + DISTANCE_ERROR_THRESHOLD;
            // if (rho < DISTANCE_ERROR_THRESHOLD)
            // {
            //     move_state = CORRECT_ORIENTATION_ERROR;
            //     msg_out.x = 0.f;
            // }
            // else
            // {
            //     speed = K1*rho;
            //     msg_out.x = (speed > MAX_X_VEL) ? MAX_X_VEL : speed;
            //     last_rho = rho;
            //     break;
            // }
        }
        else 
        {
            zspeed = K2*alpha;
            ROS_INFO("Speed = %f", zspeed);
            vel_out.z = (zspeed > MAX_TH_VEL) ? MAX_TH_VEL : (zspeed < -MAX_TH_VEL) ? -MAX_TH_VEL : zspeed;
        }
        ROS_INFO("msg_out = %f %f %f", vel_out.x, vel_out.y, vel_out.z);
        break;

    case CORRECT_DISTANCE_ERROR:
        dx = desired_global_x - curr_pose.x;
        dy = desired_global_y - curr_pose.y;
        rho = sqrt(pow(dx, 2) + pow(dy, 2));
        alpha = normalize_angle(atan2(dy, dx) - curr_pose.z);
        ROS_INFO("RHO = %f   Last RHO = %f", rho, last_rho);
        ROS_INFO("msg_out = %f %f %f", vel_out.x, vel_out.y, vel_out.z);

        if ((rho < DISTANCE_ERROR_THRESHOLD))
        {
            move_state = CORRECT_ORIENTATION_ERROR;
            vel_out.x = 0.f;
            vel_out.z = 0.f;
            // last_rho = 0.f;
        }
        else
        {
            xspeed = K1 * rho;
            zspeed = K2 * alpha;
            vel_out.x = (xspeed > MAX_X_VEL) ? MAX_X_VEL : xspeed;
            vel_out.z = (zspeed > MAX_TH_VEL) ? MAX_TH_VEL : (zspeed < -MAX_TH_VEL) ? -MAX_TH_VEL : zspeed;
            last_rho = rho;
        }
        break;

    case CORRECT_ORIENTATION_ERROR:
        eta = normalize_angle(desired_global_th - curr_pose.z);
        ROS_INFO("msg_out = %f %f %f", vel_out.x, vel_out.y, vel_out.z);
        if (abs(eta) < ANGLE_ERROR_THRESHOLD)
        {
            move_state = STOP;
            vel_out.z = 0.f;
            ROS_INFO("======================================================================");
            ROS_INFO("============================= Arrived!!! =============================");
            ROS_INFO("======================================================================");
            pose_request.data = NEXT_POSE;
            pose_request_pub.publish(pose_request);
        }
        else
        {
            zspeed = K3*eta;
            vel_out.z = (zspeed > MAX_TH_VEL) ? MAX_TH_VEL : (zspeed < -MAX_TH_VEL) ? -MAX_TH_VEL : zspeed;
        }
        break;

    case STOP:
    default:
        vel_out.x = 0.f;
        vel_out.y = 0.f;
        vel_out.z = 0.f;
        break;
    }

    local_vel_pub.publish(vel_out);

}

void keydown_recv_callback(const keyboard::Key& msg)
{
    keydown_in = msg;
    ROS_INFO("Key pressed has code: %d", keydown_in.code);
    switch(keydown_in.code)
    {
    case G_KEY:
        move_state = CORRECT_DIRECTION_ERROR;
        break;  
    case R_KEY:
        pose_request.data = RELOAD;
        pose_request_pub.publish(pose_request);
    case S_KEY:
    default:
        move_state = STOP;
        vel_out.x = 0.f;
        vel_out.z = 0.f;
        local_vel_pub.publish(vel_out);
        break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_controller");
    ros::NodeHandle nodeHandle;

    desired_pose_sub = nodeHandle.subscribe("pose_controller_global", 1, desired_pose_recv_callback);
    curr_pose_sub = nodeHandle.subscribe("pose_est", 1, curr_pose_recv_callback);
    keydown_sub = nodeHandle.subscribe("keyboard/keydown", 1, keydown_recv_callback);
    local_vel_pub = nodeHandle.advertise<geometry_msgs::Vector3>("local_velocities", 1);
    pose_request_pub = nodeHandle.advertise<std_msgs::UInt8>("pose_request", 1);
    pose_request.data = RELOAD;
    move_state = STOP;
    while (move_state == STOP)
    {
        ros::spinOnce();
        pose_request_pub.publish(pose_request);
    }

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}