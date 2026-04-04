#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <hiwonder_interfaces/JointMove.h>

using namespace Eigen;
typedef Matrix<float, 5, 1> config_t;
ros::Subscriber desired_taskspace_sub;
ros::Subscriber curr_taskspace_sub;
Vector3f curr_taskspace {0.f, 0.f, 0.f};

ros::Publisher set_joint_pub;
geometry_msgs::Vector3 joint_pos_out;


Matrix<float, 3, 5> jacobian(config_t q)
{
    float q1 = q[0];
    float q2 = q[1];
    float q3 = q[2];
    float q4 = q[3];
    float q5 = q[4];
    Matrix<float, 3, 5> J;
    J << 
        (6471*cos(q1 - q2))/100000 - (6471*cos(q1 + q2 + q3))/100000 + (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (6471*cos(q1 + q2))/100000 + (5743*cos(q2 - q1 + q3 + q4))/100000, - (6471*cos(q1 - q2))/100000 - (6471*cos(q1 + q2 + q3))/100000 - (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (6471*cos(q1 + q2))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, - (6471*cos(q1 + q2 + q3))/100000 - (6471*cos(q2 - q1 + q3))/100000 - (5743*cos(q1 + q2 + q3 + q4))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, - (5743*cos(q1 + q2 + q3 + q4))/100000 - (5743*cos(q2 - q1 + q3 + q4))/100000, 0,
        (6471*sin(q1 - q2))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (6471*sin(q2 - q1 + q3))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 - (6471*sin(q1 + q2))/100000 - (5743*sin(q2 - q1 + q3 + q4))/100000,   (6471*sin(q2 - q1 + q3))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (6471*sin(q1 - q2))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 - (6471*sin(q1 + q2))/100000 + (5743*sin(q2 - q1 + q3 + q4))/100000,   (6471*sin(q2 - q1 + q3))/100000 - (6471*sin(q1 + q2 + q3))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000 + (5743*sin(q2 - q1 + q3 + q4))/100000,   (5743*sin(q2 - q1 + q3 + q4))/100000 - (5743*sin(q1 + q2 + q3 + q4))/100000, 0,
                                                                                                                                                                                                            0,                                                                                                                          -(5743*sin(q2 + q3 + q4))/50000 - (6471*sin(q2 + q3))/50000 - (6471*sin(q2))/50000,                                                                                       -(5743*sin(q2 + q3 + q4))/50000 - (6471*sin(q2 + q3))/50000,                                               -(5743*sin(q2 + q3 + q4))/50000, 0
    ;

    return J;
}

config_t calc_jetarm_ik(Vector3f lambda_d)
{
    float K = 0.1;
    int max_iter = 1000;
    float error_threshold = 0.001;
    Vector3f lambda_error;
    config_t q;
    q << 0.0, 0.0, 0.0, 0.0, 0.0;
    Matrix<float, 3, 5> J;
    Matrix<float, 5, 3> Jinv;

    
    for (int i = 0; i < max_iter; i++)
    {
        lambda_error << lambda_d - curr_taskspace;
        if (lambda_error.norm() <= error_threshold)
            {break;}

        J << jacobian(q);

        Jinv << J.transpose() * (J*J.transpose()).inverse();

        if (!Jinv.allFinite())
        {
            ROS_INFO("Reached Singularity!");
            q << q + MatrixXf::Random(5, 1)*0.01;
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
    ros::init(argc, argv, "jetarm_ik");

    ros::NodeHandle nodeHandle;

    desired_taskspace_sub = nodeHandle.subscribe("arm_desired_task_space", 1, desired_taskspace_recv_callback);
    curr_taskspace_sub = nodeHandle.subscribe("arm_task_space", 1, curr_taskspace_recv_callback);
    

    // set_joint_pub = nodeHandle.advertise<geometry_msgs::Vector3>("/controllers/set_joint", 1);

    Vector3f myvec;
    myvec << 0.f, 0.f, 0.f;
    ROS_INFO("My vec = [%f %f %f]", myvec[0], myvec[1], myvec[2]);

    while (ros::ok())
    {
        ros::spinOnce();
    }

}