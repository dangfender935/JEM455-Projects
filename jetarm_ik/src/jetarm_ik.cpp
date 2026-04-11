#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <hiwonder_interfaces/JointMove.h>
#include <ctime>
#include <cstdlib>

using namespace Eigen;

typedef float q_t;
typedef Matrix<float, 5, 1> config_t;
typedef Matrix<float, 3, 5> jac_t;
typedef Matrix<float, 5, 3> invjac_t;   // not necessary?
typedef Vector3f xyz_vec;

ros::Subscriber desired_taskspace_sub;
ros::Subscriber curr_taskspace_sub;
xyz_vec curr_taskspace {0.f, 0.f, 0.f};

ros::Publisher set_joint_pub;
geometry_msgs::Vector3 joint_pos_out;

xyz_vec calc_endaffector_position(config_t q)
{
    xyz_vec lambda;
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
    ROS_INFO("[q] = [%f %f %f %f %f]", q1, q2, q3, q4, q5);
    jac_t J;
    J << 
        (6471*cos(q1 - q2))/100000 - (6471*cos(q1 + q2 + q3))/100000 + (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (6471*cos(q1 + q2))/100000 + (5743*cos(q2 - q1 + q3 + q4))/100000, - (6471*cos(q1 - q2))/100000 - (6471*cos(q1 + q2 + q3))/100000 - (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (6471*cos(q1 + q2))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, - (6471*cos(q1 + q2 + q3))/100000 - (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, - (5743*cos(q1 + q2 + q3 + q4))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, 0,
        (6471*sin(q1 - q2))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (6471*sin(q2 - q1 + q3))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 - (6471*sin(q1 + q2))/100000 - (5743*sin(q2 - q1 + q3 + q4))/100000,   (6471*sin(q2 - q1 + q3))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (6471*sin(q1 - q2))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 - (6471*sin(q1 + q2))/100000 + (5743*sin(q2 - q1 + q3 + q4))/100000,   (6471*sin(q2 - q1 + q3))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 + (5743*sin(q2 - q1 + q3 + q4))/100000,   (5743*sin(q2 - q1 + q3 + q4))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000, 0,
                                                                                                                                                                                                            0,                                                                                                                          -(5743*sin(q2 + q3 + q4))/50000 - (6471*sin(q2 + q3))/50000 - (6471*sin(q2))/50000,                                                                                       -(5743*sin(q2 + q3 + q4))/50000 - (6471*sin(q2 + q3))/50000,                                               -(5743*sin(q2 + q3 + q4))/50000, 0
    ;
    ROS_INFO("[J] = [%f %f %f %f %f]", J(0, 0), J(0, 1), J(0, 2), J(0, 3), J(0, 4));
    ROS_INFO("      [%f %f %f %f %f]", J(1, 0), J(1, 1), J(1, 2), J(1, 3), J(1, 4));
    ROS_INFO("      [%f %f %f %f %f]", J(2, 0), J(2, 1), J(2, 2), J(2, 3), J(2, 4));



    return J;
}

config_t calc_jetarm_ik(xyz_vec lambda_d)
{
    float K = 0.1;
    int max_iter = 1000;
    float error_threshold = 0.001;
    xyz_vec lambda_error, curr_lambda;
    config_t q;
    jac_t J;
    invjac_t Jinv;
    
    q << 0.0,
         0.0,
         0.0,
         0.0,
         0.0;
    
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
            ROS_INFO("Reached Singularity!");
            q << q + MatrixXf::Random(5, 1)*0.005;
            q(4) = 0.0; // set joint 5 to zero as it has no bearing on the xyz of end affector
            continue;
        }
        q << q + K*Jinv*lambda_error;
        
    }
    
    for (int i = 0; i < 5; i++)
    {
        while (q[i] > M_PI) q[i] -= 2*M_PI;
        while (q[i] < -M_PI) q[i] += 2*M_PI;
    }

    return q;
}

void desired_taskspace_recv_callback(const geometry_msgs::Vector3& lambda_d)
{
    Vector3f lambda_vec;
    config_t qd;
    lambda_vec << lambda_d.x, lambda_d.y, lambda_d.z;
    ROS_INFO("Taskspace received!");
    qd << calc_jetarm_ik(lambda_vec);
    ROS_INFO("qd = %f %f %f %f %f", qd[0], qd[1], qd[2], qd[3], qd[4]);
}

void curr_taskspace_recv_callback(const geometry_msgs::Vector3& lambda)
{
    curr_taskspace << lambda.x, lambda.y, lambda.z;
    ROS_INFO("Received current taskspace!");
    curr_taskspace_sub.shutdown();
}

int main(int argc, char **argv)
{
    std::srand((unsigned int) std::time(nullptr));
    ros::init(argc, argv, "jetarm_ik");

    ros::NodeHandle nodeHandle;

    // desired_taskspace_sub = nodeHandle.subscribe("arm_desired_task_space", 1, desired_taskspace_recv_callback);
    // curr_taskspace_sub = nodeHandle.subscribe("arm_task_space", 1, curr_taskspace_recv_callback);
    

    // set_joint_pub = nodeHandle.advertise<geometry_msgs::Vector3>("/controllers/set_joint", 1);

    config_t q;
    jac_t J;
    invjac_t Jinv2;
    xyz_vec lambda_d, lambda_n, lambda_err;
    q << 0.01,
         -0.023,
         0.02,
         -0.09,
         0.06;
    ROS_INFO("initial [q] = [%f %f %f %f %f]", q[0], q[1], q[2], q[3], q[4]);

    lambda_n << calc_endaffector_position(q);
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


    while (ros::ok())
    {
        ros::spinOnce();
    }

}