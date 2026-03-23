#include <ros/ros.h>
#include <pose_est/pose_est_msg.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <math.h>

#define K1      (0.9)
#define K2      (1)
#define K3      (1)
#define MAX_X_VEL   (100)
#define MAX_TH_VEL  (0.75)
#define DISTANCE_ERROR_THRESHOLD    (9.0)
#define ANGLE_ERROR_THRESHOLD       (0.03)
#define OVERSHOOT_THRESHOLD         (0.5)

typedef enum MoveState
{
    INACTIVE,
    CORRECT_DIRECTION_ERROR,
    CORRECT_DISTANCE_ERROR,
    CORRECT_ORIENTATION_ERROR
} MoveState;

ros::Subscriber desired_pose_sub;
ros::Subscriber curr_pose_sub;
ros::Publisher local_vel_pub;

geometry_msgs::Vector3 desired_pose{};
geometry_msgs::Vector3 msg_out{};
geometry_msgs::Point curr_pose{};
float desired_global_x = 0.f;
float desired_global_y = 0.f;
float desired_global_th = 0.f;
MoveState move_state = INACTIVE;

void desired_pose_recv_callback(const geometry_msgs::Vector3& msg)
{
    ROS_INFO("============= Pose received! Attempting to reach goal... =============");
    desired_pose = msg;
    desired_global_x = desired_pose.x * cos(curr_pose.z) - desired_pose.y * sin(curr_pose.z) + curr_pose.x;
    desired_global_y = desired_pose.x * sin(curr_pose.z) + desired_pose.y * cos(curr_pose.z) + curr_pose.y;
    desired_global_th = desired_pose.z + curr_pose.z;
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
    float eta= 0.f;
    float speed = 0.f;
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
            msg_out.z = 0.f;
            rho = sqrt(pow(dx, 2) + pow(dy, 2));
            if (rho < DISTANCE_ERROR_THRESHOLD)
            {
                move_state = CORRECT_ORIENTATION_ERROR;
                msg_out.x = 0.f;
            }
            else
            {
                speed = K1*rho;
                msg_out.x = (speed > MAX_X_VEL) ? MAX_X_VEL : speed;
                last_rho = rho;
                break;
            }
            msg_out.z = 0.f;
            last_rho = 0.f;
        }
        else 
        {
            speed = K2*alpha;
            ROS_INFO("Speed = %f", speed);
            msg_out.z = (speed > MAX_TH_VEL) ? MAX_TH_VEL : speed;
        }
        ROS_INFO("msg_out = %f %f %f", msg_out.x, msg_out.y, msg_out.z);
        break;

    case CORRECT_DISTANCE_ERROR:
        dx = desired_global_x - curr_pose.x;
        dy = desired_global_y - curr_pose.y;
        rho = sqrt(pow(dx, 2) + pow(dy, 2));
        speed = K1*rho;
        ROS_INFO("RHO = %f   Last RHO = %f", rho, last_rho);
        msg_out.x = (speed > MAX_X_VEL) ? MAX_X_VEL : speed * (rho > DISTANCE_ERROR_THRESHOLD) * (last_rho > rho);
        ROS_INFO("msg_out = %f %f %f", msg_out.x, msg_out.y, msg_out.z);

        if (msg_out.x == 0.0)
        {
            move_state = CORRECT_ORIENTATION_ERROR;
            last_rho = 0.f;
        }
        else
        {
            last_rho = rho;
            break;
        }

    case CORRECT_ORIENTATION_ERROR:
        eta = desired_global_th - curr_pose.z;
        ROS_INFO("msg_out = %f %f %f", msg_out.x, msg_out.y, msg_out.z);
        if (abs(eta) < ANGLE_ERROR_THRESHOLD)
        {
            move_state = INACTIVE;
            msg_out.z = 0.f;
            ROS_INFO("======================================================================");
            ROS_INFO("============================= Arrived!!! =============================");
            ROS_INFO("======================================================================");
        }
        else
        {
            speed = K3*eta;
            msg_out.z = (speed > MAX_TH_VEL) ? MAX_TH_VEL : speed;
            break;
        }

    case INACTIVE:
    default:
        msg_out.x = 0.f;
        msg_out.y = 0.f;
        msg_out.z = 0.f;
        break;
    }

    // ROS_INFO("desired global x - curr pose x: %f - %f = %f", desired_global_x, curr_pose.x, dx);
    // ROS_INFO("rho = %f", rho);

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