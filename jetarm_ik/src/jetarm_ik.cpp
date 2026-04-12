#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <hiwonder_interfaces/JointMove.h>
#include <sensor_msgs/JointState.h>
#include <ctime>
#include <cstdlib>

#define JOINT1          (0)
#define JOINT2          (1)
#define JOINT3          (2)
#define JOINT4          (3)
#define JOINT5          (4)
#define MIN             (0)
#define MAX             (1)

#define JOINT1_MIN      (-2.09)
#define JOINT1_MAX      (2.09)
#define JOINT2_MIN      (-1.67)
#define JOINT2_MAX      (1.65)
#define JOINT3_MIN      (-2.09)
#define JOINT3_MAX      (2.09)
#define JOINT4_MIN      (-1.85)
#define JOINT4_MAX      (2.00)
#define JOINT5_MIN      (-2.09)
#define JOINT5_MAX      (2.09)


using namespace Eigen;

typedef float q_t;
typedef Matrix<float, 5, 1> config_t;
typedef Matrix<float, 3, 5> jac_t;
typedef Matrix<float, 5, 3> invjac_t;   // not necessary?
typedef Vector3f taskspace_t;

ros::Subscriber desired_taskspace_sub;
ros::Subscriber curr_config_sub;
config_t curr_config{};
bool config_recv = false;

ros::Publisher set_joint_pub;
hiwonder_interfaces::JointMove joint_pos_out;

taskspace_t calc_endaffector_position(config_t q)
{
    taskspace_t lambda;
    q_t q1 = q(0);
    q_t q2 = q(1);
    q_t q3 = q(2);
    q_t q4 = q(3);
    q_t q5 = q(4);

    q_t xpos = 0.06471*sin(q1 - q2) - 0.05743*sin(q2 - q1 + q3 + q4) - 0.06471*sin(q1 + q2 + q3) - 0.05743*sin(q1 + q2 + q3 + q4) - 0.06471*sin(q1 + q2) - 0.06471*sin(q2 - q1 + q3);
    q_t ypos = 0.06471*cos(q1 + q2 + q3) - 0.05743*cos(q2 - q1 + q3 + q4) - 0.06471*cos(q1 - q2) + 0.05743*cos(q1 + q2 + q3 + q4) + 0.06471*cos(q1 + q2) - 0.06471*cos(q2 - q1 + q3);
    q_t zpos = 0.11486*cos(q2 + q3 + q4) + 0.12942*cos(q2 + q3) + 0.12942*cos(q2) + 0.10432;
    lambda << xpos, ypos, zpos;
    return lambda;
}

jac_t jacobian(config_t q)
{
    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];
    float q4 = q[3];
    float q5 = q[4];
    // ROS_INFO("[q] = [%f %f %f %f %f]", q1, q2, q3, q4, q5);
    jac_t J;
    J << 
        (6471*cos(q1 - q2))/100000 - (6471*cos(q1 + q2 + q3))/100000 + (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (6471*cos(q1 + q2))/100000 + (5743*cos(q2 - q1 + q3 + q4))/100000, - (6471*cos(q1 - q2))/100000 - (6471*cos(q1 + q2 + q3))/100000 - (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (6471*cos(q1 + q2))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, - (6471*cos(q1 + q2 + q3))/100000 - (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, - (5743*cos(q1 + q2 + q3 + q4))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, 0,
        (6471*sin(q1 - q2))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (6471*sin(q2 - q1 + q3))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 - (6471*sin(q1 + q2))/100000 - (5743*sin(q2 - q1 + q3 + q4))/100000,   (6471*sin(q2 - q1 + q3))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (6471*sin(q1 - q2))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 - (6471*sin(q1 + q2))/100000 + (5743*sin(q2 - q1 + q3 + q4))/100000,   (6471*sin(q2 - q1 + q3))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 + (5743*sin(q2 - q1 + q3 + q4))/100000,   (5743*sin(q2 - q1 + q3 + q4))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000, 0,
                                                                                                                                                                                                            0,                                                                                                                          -(5743*sin(q2 + q3 + q4))/50000 - (6471*sin(q2 + q3))/50000 - (6471*sin(q2))/50000,                                                                                       -(5743*sin(q2 + q3 + q4))/50000 - (6471*sin(q2 + q3))/50000,                                               -(5743*sin(q2 + q3 + q4))/50000, 0
    ;
    // ROS_INFO("[J] = [%f %f %f %f %f]", J(0, 0), J(0, 1), J(0, 2), J(0, 3), J(0, 4));
    // ROS_INFO("      [%f %f %f %f %f]", J(1, 0), J(1, 1), J(1, 2), J(1, 3), J(1, 4));
    // ROS_INFO("      [%f %f %f %f %f]", J(2, 0), J(2, 1), J(2, 2), J(2, 3), J(2, 4));

    return J;
}

config_t calc_jetarm_ik(taskspace_t lambda_d)
{
    float K = 0.1;
    int max_iter = 1000;
    float error_threshold = 0.001;
    q_t joint_limits[5][2] = {
        {JOINT1_MIN, JOINT1_MAX},
        {JOINT2_MIN, JOINT2_MAX},
        {JOINT3_MIN, JOINT3_MAX},
        {JOINT4_MIN, JOINT4_MAX},
        {JOINT5_MIN, JOINT5_MAX},
    };

    taskspace_t lambda_error, curr_lambda;
    config_t q;
    jac_t J;
    invjac_t Jinv;
    
    q << curr_config;
    
    for (int i = 0; i < max_iter; i++)
    {
        curr_lambda << calc_endaffector_position(q);
        lambda_error << lambda_d - curr_lambda;
        if (lambda_error.norm() <= error_threshold)
            {break;}

        J << jacobian(q);

        Jinv << J.transpose() * (J*J.transpose()).inverse();

        if (!Jinv.allFinite())
        {
            // ROS_INFO("Reached Singularity!");
            q << q + MatrixXf::Random(5, 1)*0.005;
            continue;
        }
        q << q + K*Jinv*lambda_error;
        
    }
    
    for (int i = 0; i < 4; i++)
    {
        // Wrap to pi
        while (q[i] > M_PI) q[i] -= 2*M_PI;
        while (q[i] < -M_PI) q[i] += 2*M_PI;

        // Make sure each joint is within joint limits
        q[i] = (q[i] > joint_limits[i][MAX]) ? 
                    joint_limits[i][MAX] : 
                    (q[i] < joint_limits[i][MIN]) ?
                        joint_limits[i][MIN] :
                        q[i];
    }
    q(4) = 0.0; // set joint 5 to 0.0 as it has no effect on xyz position
    return q;
}

void virtual_ik()
{
    config_t q;
    jac_t J;
    invjac_t Jinv2;
    taskspace_t lambda_d, lambda_n, lambda_err;

    q_t joint_limits[5][2] = {
        {JOINT1_MIN, JOINT1_MAX},
        {JOINT2_MIN, JOINT2_MAX},
        {JOINT3_MIN, JOINT3_MAX},
        {JOINT4_MIN, JOINT4_MAX},
        {JOINT5_MIN, JOINT5_MAX},
    };

    q << -2.00, -1.65, 2.30, -1.9, 0.0;
    ROS_INFO("initial [q0] = [%f %f %f %f %f]", q[0], q[1], q[2], q[3], q[4]);

    // Make sure each joint is within joint limits
    for (int i = 0; i < 4; i++)
    {
        // Wrap to pi
        while (q[i] > M_PI) q[i] -= 2*M_PI;
        while (q[i] < -M_PI) q[i] += 2*M_PI;

        // Make sure each joint is within joint limits
        q[i] = (q[i] > joint_limits[i][MAX]) ? 
                    joint_limits[i][MAX] : 
                    (q[i] < joint_limits[i][MIN]) ?
                        joint_limits[i][MIN] :
                        q[i];
    }

    ROS_INFO("adjusted [q0] = [%f %f %f %f %f]", q[0], q[1], q[2], q[3], q[4]);

    
    lambda_n << calc_endaffector_position(curr_config);
    ROS_INFO("initial endaffector position: [%f %f %f]", lambda_n(0), lambda_n(1), lambda_n(2));
    
    J << jacobian(q);    
    ROS_INFO("Jacobian complete!");
    
    Jinv2 << J.transpose() * (J*J.transpose()).inverse();
    ROS_INFO("[Jinv] = [%f %f %f]", Jinv2(0, 0), Jinv2(0, 1), Jinv2(0, 2));
    ROS_INFO("         [%f %f %f]", Jinv2(1, 0), Jinv2(1, 1), Jinv2(1, 2));
    ROS_INFO("         [%f %f %f]", Jinv2(2, 0), Jinv2(2, 1), Jinv2(2, 2));
    ROS_INFO("         [%f %f %f]", Jinv2(3, 0), Jinv2(3, 1), Jinv2(3, 2));
    ROS_INFO("         [%f %f %f]", Jinv2(4, 0), Jinv2(4, 1), Jinv2(4, 2));
    
    lambda_d << 0.3737, 0.0, 0.10432;
    q << calc_jetarm_ik(lambda_d);
    ROS_INFO("final [q] = [%f %f %f %f %f]", q[0], q[1], q[2], q[3], q[4]);
}

void set_joints(config_t q)
{
    ros::Rate r(0.25);
    joint_pos_out.name = "joint1";
    joint_pos_out.rad = q(0);
    joint_pos_out.duration = 2.0;
    set_joint_pub.publish(joint_pos_out);
    r.sleep();
    
    joint_pos_out.name = "joint2";
    joint_pos_out.rad = q(1);
    joint_pos_out.duration = 2.0;
    set_joint_pub.publish(joint_pos_out);
    r.sleep();
    
    joint_pos_out.name = "joint3";
    joint_pos_out.rad = q(2);
    joint_pos_out.duration = 2.0;
    set_joint_pub.publish(joint_pos_out);
    r.sleep();
    
    joint_pos_out.name = "joint4";
    joint_pos_out.rad = q(3);
    joint_pos_out.duration = 2.0;
    set_joint_pub.publish(joint_pos_out);
    r.sleep();
    
    joint_pos_out.name = "joint5";
    joint_pos_out.rad = q(4);
    joint_pos_out.duration = 2.0;
    set_joint_pub.publish(joint_pos_out);
}

void curr_config_recv_callback(const sensor_msgs::JointState& q_msg)
{
    curr_config <<  q_msg.position[JOINT1],
                    q_msg.position[JOINT2],
                    q_msg.position[JOINT3],
                    q_msg.position[JOINT4],
                    q_msg.position[JOINT5];

    // ROS_INFO("Current config space = [%f %f %f %f %f]", 
        curr_config(0), curr_config(1), curr_config(2), curr_config(3), curr_config(4));
    config_recv = true;
    curr_config_sub.shutdown();
}

void desired_taskspace_recv_callback(const geometry_msgs::Vector3& lambda_d_msg)
{
    taskspace_t lambda_d;
    // ROS_INFO("New desired taskspace received!");
    lambda_d << lambda_d_msg.x, lambda_d_msg.y, lambda_d_msg.z;
    curr_config << calc_jetarm_ik(lambda_d);
    set_joints(curr_config);
}

int main(int argc, char **argv)
{
    std::srand((unsigned int) std::time(nullptr));
    ros::init(argc, argv, "jetarm_ik");

    ros::NodeHandle nodeHandle;
    
    curr_config_sub = nodeHandle.subscribe("/joint_states", 1, curr_config_recv_callback);
    ROS_INFO("Waiting for config space reading...");
    while (!config_recv) ros::spinOnce();

    ROS_INFO("Ready!"); 
    set_joint_pub = nodeHandle.advertise<hiwonder_interfaces::JointMove>("/controllers/set_joint", 1);
    desired_taskspace_sub = nodeHandle.subscribe("arm_desired_task_space", 1, desired_taskspace_recv_callback);

    while (ros::ok())
    {
        ros::spinOnce();
    }

}