#include <ros/ros.h>
#include <pose_est/pose_est_msg.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>

#define STATE_STRING            "ganymede"
#define WHEEL_SEPARATION        (245.f)   // mm
#define WHEEL_RADIUS            (33.1)  // mm

ros::Publisher my_publisher_object;
ros::Subscriber enc_left_sub;
ros::Subscriber enc_right_sub;
ros::Subscriber local_vel_sub;
geometry_msgs::Vector3 local_velocity;

float enc_left = 0;
float enc_right = 0;
float last_enc_left = 0.f; 
float last_enc_right = 0.f;
bool left_enc_flag = false;
bool right_enc_flag = false;

void enc_left_recv_callback(const std_msgs::Float64& msg)
{
    last_enc_left = (left_enc_flag) ? enc_left : msg.data;
    enc_left = msg.data;
    left_enc_flag = true;
}

void enc_right_recv_callback(const std_msgs::Float64& msg)
{
    last_enc_right = (right_enc_flag) ? enc_right : msg.data;
    enc_right = msg.data;
    right_enc_flag = true;
}

int main(int argc, char **argv)
{
    pose_est::pose_est_msg current_position;
    ros::Time time_obj;
    
    float delta_enc_left = 0.f; 
    float delta_enc_right = 0.f;
    float d_inv = 1 / WHEEL_SEPARATION;
    float left_ang_vel = 0.f, right_ang_vel = 0.f;
    float delta_th_left = 0.f, delta_th_right = 0.f;
    float left_vel = 0.f;
    float right_vel = 0.f;
    float x_vel = 0;
    float y_vel = 0;
    float th_vel = 0;
    float dt = 0.f;
    float x = 0.f;
    float y = 0.f;
    float th = 0.f;
    ros::init(argc, argv, "pose_est");
    ros::NodeHandle nodeHandle;

    enc_left_sub = nodeHandle.subscribe("left_wheel_enc", 1, enc_left_recv_callback);
    enc_right_sub = nodeHandle.subscribe("right_wheel_enc", 1, enc_right_recv_callback);

    my_publisher_object = nodeHandle.advertise<pose_est::pose_est_msg>("pose_est", 1);

    float rate = 20.f;
    ros::Rate r((int)rate);
    float Ts = 1/rate;
    while (ros::ok())
    {
        delta_enc_left = enc_left - last_enc_left;
        delta_enc_right = enc_right - last_enc_right;
        // ROS_INFO("Delta enc (L/R): %f %f", delta_enc_left, delta_enc_right);

        delta_th_left = delta_enc_left;
        delta_th_right = delta_enc_right;
        // ROS_INFO("Delta theta (L/R): %f %f", delta_th_left, delta_th_right);

        left_ang_vel = delta_th_left * rate;
        right_ang_vel = delta_th_right * rate;
        // ROS_INFO("left and right ang vel: %f %f", left_ang_vel, right_ang_vel);

        left_vel = WHEEL_RADIUS * left_ang_vel;
        right_vel = WHEEL_RADIUS * right_ang_vel;
        // ROS_INFO("left and right vel: %f %f", left_vel, right_vel);

        x_vel = 0.5 * cos(th) * (left_vel + right_vel);
        y_vel = 0.5 * sin(th) * (left_vel + right_vel);
        th_vel = d_inv * (right_vel - left_vel);
        // ROS_INFO("x_vel = %f", x_vel);
        // ROS_INFO("y_vel = %f", y_vel);
        // ROS_INFO("th_vel = %f", th_vel);

        x += x_vel * Ts;
        y += y_vel * Ts;
        th += th_vel * Ts;
        // ROS_INFO("Pose: %f %f %f", x, y, th);
        current_position.point.x = x;
        current_position.point.y = y;
        current_position.point.z = th;
        current_position.header.stamp = time_obj.now();
        current_position.name = STATE_STRING;
        current_position.id = 18273645;
        my_publisher_object.publish(current_position);
        last_enc_left = enc_left;
        last_enc_right = enc_right;
        r.sleep();

        ros::spinOnce();
    }


    return 0;
}