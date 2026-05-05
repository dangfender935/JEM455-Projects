#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <hiwonder_interfaces/JointMove.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <ctime>
#include <cstdlib>
#include <vector>

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
typedef Matrix<float, 5, 5> jac_t;
typedef Vector3f taskspace_t;
typedef Matrix<float, 5, 1> lambda_t;
typedef Matrix<float, 1, 5> jac_row_t;

ros::Subscriber desired_taskspace_sub;
ros::Subscriber curr_config_sub;
config_t curr_config{};
bool config_recv = false;

ros::Publisher set_joint_pub;
hiwonder_interfaces::JointMove joint_pos_out;

ros::Subscriber jac_sub;

lambda_t calc_endaffector_position(config_t q)
{
    lambda_t lambda;
    q_t q1 = q(0);
    q_t q2 = q(1);
    q_t q3 = q(2);
    q_t q4 = q(3);
    q_t q5 = q(4);

    q_t xpos = 0.06471*sin(q1 - q2) - 0.05743*sin(q2 - q1 + q3 + q4) - 0.06471*sin(q1 + q2 + q3) - 0.05743*sin(q1 + q2 + q3 + q4) - 0.06471*sin(q1 + q2) - 0.06471*sin(q2 - q1 + q3);
    q_t ypos = 0.06471*cos(q1 + q2 + q3) - 0.05743*cos(q2 - q1 + q3 + q4) - 0.06471*cos(q1 - q2) + 0.05743*cos(q1 + q2 + q3 + q4) + 0.06471*cos(q1 + q2) - 0.06471*cos(q2 - q1 + q3);
    q_t zpos = 0.11486*cos(q2 + q3 + q4) + 0.12942*cos(q2 + q3) + 0.12942*cos(q2) + 0.10432;
    q_t zzepos = -cos(q2 + q3 + q4);
    q_t yyepos = 0.5*sin(q2 - q1 + q3 + q4)*sin(q5) - cos(q5)*cos(q1) - 0.5*sin(q1 + q2 + q3 + q4)*sin(q5);
    lambda << xpos, ypos, zpos, zzepos, yyepos;
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
    jac_row_t Jx, Jy, Jz, Jzze, Jyye;
    jac_t J;
    Jx << 0.05743*cos(q2 - 1.0*q1 + q3 + q4) - 0.06471*cos(q1 + q2 + q3) + 0.06471*cos(q1 - 1.0*q2) - 0.05743*cos(q1 + q2 + q3 + q4) - 0.06471*cos(q1 + q2) + 0.06471*cos(q2 - 1.0*q1 + q3), - 0.06471*cos(q1 + q2 + q3) - 0.05743*cos(q2 - 1.0*q1 + q3 + q4) - 0.06471*cos(q1 - 1.0*q2) - 0.05743*cos(q1 + q2 + q3 + q4) - 0.06471*cos(q1 + q2) - 0.06471*cos(q2 - 1.0*q1 + q3), - 0.06471*cos(q1 + q2 + q3) - 0.05743*cos(q2 - 1.0*q1 + q3 + q4) - 0.05743*cos(q1 + q2 + q3 + q4) - 0.06471*cos(q2 - 1.0*q1 + q3), - 0.05743*cos(q2 - 1.0*q1 + q3 + q4) - 0.05743*cos(q1 + q2 + q3 + q4), 0;
    Jy << 0.06471*sin(q1 - 1.0*q2) - 0.05743*sin(q2 - 1.0*q1 + q3 + q4) - 0.06471*sin(q1 + q2 + q3) - 0.05743*sin(q1 + q2 + q3 + q4) - 0.06471*sin(q1 + q2) - 0.06471*sin(q2 - 1.0*q1 + q3),   0.05743*sin(q2 - 1.0*q1 + q3 + q4) - 0.06471*sin(q1 + q2 + q3) - 0.06471*sin(q1 - 1.0*q2) - 0.05743*sin(q1 + q2 + q3 + q4) - 0.06471*sin(q1 + q2) + 0.06471*sin(q2 - 1.0*q1 + q3), 0.05743*sin(q2 - 1.0*q1 + q3 + q4) - 0.06471*sin(q1 + q2 + q3) - 0.05743*sin(q1 + q2 + q3 + q4) + 0.06471*sin(q2 - 1.0*q1 + q3), 0.05743*sin(q2 - 1.0*q1 + q3 + q4) - 0.05743*sin(q1 + q2 + q3 + q4), 0;
    Jz << 0, - 0.11486*sin(q2 + q3 + q4) - 0.12942*sin(q2 + q3) - 0.12942*sin(q2), - 0.11486*sin(q2 + q3 + q4) - 0.12942*sin(q2 + q3), -0.11486*sin(q2 + q3 + q4), 0;
    Jzze << 0, sin(q2 + q3 + q4), sin(q2 + q3 + q4), sin(q2 + q3 + q4), 0;
    Jyye << - 1.0*cos(q5)*(-1.0*sin(q1)) - 0.5*cos(q2 - 1.0*q1 + q3 + q4)*sin(q5) - 0.5*cos(q1 + q2 + q3 + q4)*sin(q5), 0.5*cos(q2 - 1.0*q1 + q3 + q4)*sin(q5) - 0.5*cos(q1 + q2 + q3 + q4)*sin(q5),   0.5*cos(q2 - 1.0*q1 + q3 + q4)*sin(q5) - 0.5*cos(q1 + q2 + q3 + q4)*sin(q5),   0.5*cos(q2 - 1.0*q1 + q3 + q4)*sin(q5) - 0.5*cos(q1 + q2 + q3 + q4)*sin(q5), 0.5*sin(q2 - 1.0*q1 + q3 + q4)*cos(q5) + 1.0*sin(q5)*cos(q1) - 0.5*sin(q1 + q2 + q3 + q4)*cos(q5);
//     // J << 
//     //     (6471*cos(q1 - q2))/100000 - (6471*cos(q1 + q2 + q3))/100000 + (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (6471*cos(q1 + q2))/100000 + (5743*cos(q2 - q1 + q3 + q4))/100000, - (6471*cos(q1 - q2))/100000 - (6471*cos(q1 + q2 + q3))/100000 - (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (6471*cos(q1 + q2))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, - (6471*cos(q1 + q2 + q3))/100000 - (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, - (5743*cos(q1 + q2 + q3 + q4))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, 0,
//     //     (6471*sin(q1 - q2))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (6471*sin(q2 - q1 + q3))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 - (6471*sin(q1 + q2))/100000 - (5743*sin(q2 - q1 + q3 + q4))/100000,   (6471*sin(q2 - q1 + q3))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (6471*sin(q1 - q2))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 - (6471*sin(q1 + q2))/100000 + (5743*sin(q2 - q1 + q3 + q4))/100000,   (6471*sin(q2 - q1 + q3))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 + (5743*sin(q2 - q1 + q3 + q4))/100000,   (5743*sin(q2 - q1 + q3 + q4))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000, 0,
//     //                                                                                                                                                                                                         0,                                                                                                                          -(5743*sin(q2 + q3 + q4))/50000 - (6471*sin(q2 + q3))/50000 - (6471*sin(q2))/50000,                                                                                       -(5743*sin(q2 + q3 + q4))/50000 - (6471*sin(q2 + q3))/50000,                                               -(5743*sin(q2 + q3 + q4))/50000, 0
//     // ;
    J << Jx,
        Jy,
        Jz,
        Jzze,
        Jyye
    ;
    // ROS_INFO("[J] = [%f %f %f %f %f]", J(0, 0), J(0, 1), J(0, 2), J(0, 3), J(0, 4));
    // ROS_INFO("      [%f %f %f %f %f]", J(1, 0), J(1, 1), J(1, 2), J(1, 3), J(1, 4));
    // ROS_INFO("      [%f %f %f %f %f]", J(2, 0), J(2, 1), J(2, 2), J(2, 3), J(2, 4));
    // ROS_INFO("      [%f %f %f %f %f]", J(3, 0), J(3, 1), J(3, 2), J(3, 3), J(3, 4));
    // ROS_INFO("      [%f %f %f %f %f]", J(4, 0), J(4, 1), J(4, 2), J(4, 3), J(4, 4));

    return J;
}

config_t calc_jetarm_ik(lambda_t lambda_d)
{
    float K = 0.005;
    float K2 = 1.0;
    int max_iter = 10000;
    int max_iter_q5 = 500;
    float error_threshold = 0.001;
    float desired_yye = -1.0;
    float yye = 0;
    float yye_error = 0;
    q_t joint_limits[5][2] = {
        {JOINT1_MIN, JOINT1_MAX},
        {JOINT2_MIN, JOINT2_MAX},
        {JOINT3_MIN, JOINT3_MAX},
        {JOINT4_MIN, JOINT4_MAX},
        {JOINT5_MIN, JOINT5_MAX},
    };

    Matrix<float, 4, 1> constrained_q;
    
    lambda_t lambda_error, curr_lambda;
    Matrix<float, 4, 1> constrained_error;
    config_t q;
    jac_t J, Jinv;
    Matrix<float, 4, 4> constrained_J, constrained_Jinv;
    std::vector<int> fk_indx{1, 2, 3, 4};
    // ROS_INFO("Assigning constrained_q...");
    
    q << curr_config;
    constrained_q << q(0),
                     q(1),
                     q(2),
                     q(3);
    
    for (int i = 0; i < max_iter; i++)
    {
        curr_lambda << calc_endaffector_position(q);
        lambda_error << lambda_d - curr_lambda;
        constrained_error << lambda_error(1),
                            lambda_error(2),
                            lambda_error(3),
                            lambda_error(4);
        if (constrained_error.norm() <= error_threshold)
            {break;}
        J << jacobian(q);
        // ROS_INFO("assigning constrained_J...");
        constrained_J << J(0, 0), J(0, 1), J(0, 2), J(0, 3),
                         J(1, 0), J(1, 1), J(1, 2), J(1, 3),
                         J(2, 0), J(2, 1), J(2, 2), J(2, 3),
                         J(3, 0), J(3, 1), J(3, 2), J(3, 3);

        // Jinv << J.transpose() * (J*J.transpose()).inverse();
        constrained_Jinv << constrained_J.inverse();
        if (!constrained_Jinv.allFinite())
        {
            ROS_INFO("Reached Singularity!");
            constrained_q << constrained_q + MatrixXf::Random(4, 1)*0.005;
            continue;
        }
        // ROS_INFO("Assigning constrained_q in loop...");

        constrained_q << constrained_q + K*constrained_Jinv*constrained_error;
        // ROS_INFO("Adjusting constrained_q to be within joint limits...");

        for (int i = 0; i < 4; i++)
        {
            // Make sure each joint is within joint limits
            constrained_q[i] = (constrained_q[i] > joint_limits[i][MAX]) ? 
                        joint_limits[i][MAX] : 
                        (constrained_q[i] < joint_limits[i][MIN]) ?
                            joint_limits[i][MIN] :
                            constrained_q[i];
        } 
        ROS_INFO("Assigning back to q...");

        q << constrained_q(0),
             constrained_q(1),
             constrained_q(2),
             constrained_q(3),
             q(4);
    }

    for (int j = 0; j < max_iter_q5; j++)
    {
        curr_lambda << calc_endaffector_position(q);
        yye = curr_lambda(4);
        yye_error = desired_yye - yye;
        if (abs(yye_error) < error_threshold)
        {
            break;
        }
        q(4) = q(4) - K2*yye_error;
    }

    return q;
}

void set_joints(config_t q)
{
    ROS_INFO("final [q] = [%f %f %f %f %f]", q[0], q[1], q[2], q[3], q[4]);
    // ros::Rate r(0.5);
    // joint_pos_out.name = "joint1";
    // joint_pos_out.rad = q(0);
    // joint_pos_out.duration = 2.0;
    // set_joint_pub.publish(joint_pos_out);
    // // r.sleep();
    
    // joint_pos_out.name = "joint2";
    // joint_pos_out.rad = q(1);
    // joint_pos_out.duration = 2.0;
    // set_joint_pub.publish(joint_pos_out);
    // // r.sleep();
    
    // joint_pos_out.name = "joint3";
    // joint_pos_out.rad = q(2);
    // joint_pos_out.duration = 2.0;
    // set_joint_pub.publish(joint_pos_out);
    // r.sleep();
    
    // joint_pos_out.name = "joint4";
    // joint_pos_out.rad = q(3);
    // joint_pos_out.duration = 2.0;
    // set_joint_pub.publish(joint_pos_out);
    // // r.sleep();
    
    // joint_pos_out.name = "joint5";
    // joint_pos_out.rad = q(4);
    // joint_pos_out.duration = 2.0;
    // set_joint_pub.publish(joint_pos_out);
}

void curr_config_recv_callback(const sensor_msgs::JointState& q_msg)
{
    curr_config <<  q_msg.position[JOINT1],
                    q_msg.position[JOINT2],
                    q_msg.position[JOINT3],
                    q_msg.position[JOINT4],
                    q_msg.position[JOINT5];

    // ROS_INFO("Current config space = [%f %f %f %f %f]", 
        // curr_config(0), curr_config(1), curr_config(2), curr_config(3), curr_config(4));
    config_recv = true;
    curr_config_sub.shutdown();
}

void desired_taskspace_recv_callback(const geometry_msgs::Vector3& lambda_d_msg)
{
    lambda_t lambda_d;
    ROS_INFO("New desired taskspace received!");
    lambda_d << lambda_d_msg.x, lambda_d_msg.y, lambda_d_msg.z, 1.0, -1.0;
    curr_config << calc_jetarm_ik(lambda_d);
    ROS_INFO("final [q] = [%f %f %f %f %f]", curr_config[0], curr_config[1], curr_config[2], curr_config[3], curr_config[4]);
    set_joints(curr_config);
}

void jac_recv_callback(const std_msgs::Float32MultiArray& msg)
{
    config_t q;
    jac_t J;
    lambda_t lambda;
    for (const auto& value: msg.data)
    {
        ROS_INFO("Value = %f", value);
    }
    q << msg.data[0],
        msg.data[1],
        msg.data[2], 
        msg.data[3],
        msg.data[4];

    lambda << calc_endaffector_position(q);
    ROS_INFO("[lambda] = [%f %f %f %f %f]", lambda(0), lambda(1), lambda(2), lambda(3), lambda(4));
    ROS_INFO("");

    J << jacobian(q);
    ROS_INFO("[J] = [%f %f %f %f %f]", J(0, 0), J(0, 1), J(0, 2), J(0, 3), J(0, 4));
    ROS_INFO("      [%f %f %f %f %f]", J(1, 0), J(1, 1), J(1, 2), J(1, 3), J(1, 4));
    ROS_INFO("      [%f %f %f %f %f]", J(2, 0), J(2, 1), J(2, 2), J(2, 3), J(2, 4));
    ROS_INFO("      [%f %f %f %f %f]", J(3, 0), J(3, 1), J(3, 2), J(3, 3), J(3, 4));
    ROS_INFO("      [%f %f %f %f %f]", J(4, 0), J(4, 1), J(4, 2), J(4, 3), J(4, 4));
    ROS_INFO("");

    curr_config << q;

    Matrix<float, 4, 4> constrained_J;

    constrained_J << J(0, 0), J(0, 1), J(0, 2), J(0, 3),
                         J(1, 0), J(1, 1), J(1, 2), J(1, 3),
                         J(2, 0), J(2, 1), J(2, 2), J(2, 3),
                         J(3, 0), J(3, 1), J(3, 2), J(3, 3);

    ROS_INFO("[constrained_J] = [%f %f %f %f]", constrained_J(0, 0), constrained_J(0, 1), constrained_J(0, 2), constrained_J(0, 3));
    ROS_INFO("      [%f %f %f %f]", constrained_J(1, 0), constrained_J(1, 1), constrained_J(1, 2), constrained_J(1, 3));
    ROS_INFO("      [%f %f %f %f]", constrained_J(2, 0), constrained_J(2, 1), constrained_J(2, 2), constrained_J(2, 3));
    ROS_INFO("      [%f %f %f %f]", constrained_J(3, 0), constrained_J(3, 1), constrained_J(3, 2), constrained_J(3, 3));
    ROS_INFO("");

    Matrix<float, 4, 4> inv_J;
    inv_J << constrained_J.inverse();
    ROS_INFO("[inv_J] = [%f %f %f %f]", inv_J(0, 0), inv_J(0, 1), inv_J(0, 2), inv_J(0, 3));
    ROS_INFO("          [%f %f %f %f]", inv_J(1, 0), inv_J(1, 1), inv_J(1, 2), inv_J(1, 3));
    ROS_INFO("          [%f %f %f %f]", inv_J(2, 0), inv_J(2, 1), inv_J(2, 2), inv_J(2, 3));
    ROS_INFO("          [%f %f %f %f]", inv_J(3, 0), inv_J(3, 1), inv_J(3, 2), inv_J(3, 3));
    ROS_INFO("");
}

int main(int argc, char **argv)
{
    std::srand((unsigned int) std::time(nullptr));
    ros::init(argc, argv, "jetarm_ik");

    ros::NodeHandle nodeHandle;
    
    set_joint_pub = nodeHandle.advertise<hiwonder_interfaces::JointMove>("/controllers/set_joint", 1);
    // curr_config_sub = nodeHandle.subscribe("/joint_states", 1, curr_config_recv_callback);
    // ROS_INFO("Waiting for config space reading...");
    // while (!config_recv) ros::spinOnce();
    ROS_INFO("Ready!"); 
    desired_taskspace_sub = nodeHandle.subscribe("arm_desired_task_space", 1, desired_taskspace_recv_callback);

    jac_sub = nodeHandle.subscribe("jac_test", 1, jac_recv_callback);

    

    while (ros::ok())
    {
        ros::spinOnce();
    }

}