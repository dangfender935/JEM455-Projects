#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt8.h>
#include "pose_checkpoint.h"
#include <math.h>
#include <vector>
#include <string>

#define BLOCK1                  (0)
#define BLOCK2                  (1)
#define BLOCK3                  (2)
#define BLOCK_DROP              (3)

#define ERROR_THRESHOLD         (0.005)

#define GRIPPER_OPEN_TIME       (1)
#define GRIPPER_CLOSE_TIME      (1)

enum GripperState : int
{
    OPEN = 0,
    CLOSE = 1
};

ros::Subscriber arm_state_sub;

ros::Publisher arm_state_pub;
std_msgs::UInt8 arm_state_out;

ros::Publisher gripper_state_pub;
std_msgs::UInt8 gripper_state_out;

ros::Subscriber task_space_sub;

ros::Publisher task_space_pub;
geometry_msgs::Vector3 task_space_out;

int arm_state = -1;

std::string json_path;
std::vector<Pose> taskspaces;
Pose desired_pose;

void arm_state_callback(const std_msgs::UInt8& state)
{
    arm_state = state.data;
    switch (arm_state)
    {
    case DROP_BLOCK:
        gripper_state_out.data = OPEN;
        gripper_state_pub.publish(gripper_state_out);
        ros::Duration(GRIPPER_OPEN_TIME).sleep();
        arm_state_out.data = HAS_DROPPED;
        arm_state_pub.publish(arm_state_out);
        break;

    case PICKUP_BLOCK1:
        desired_pose = taskspaces[BLOCK1];
        task_space_out.x = taskspaces[BLOCK1].x;
        task_space_out.y = taskspaces[BLOCK1].y;
        task_space_out.z = taskspaces[BLOCK1].theta;
        task_space_pub.publish(task_space_out);
        break;

    case PICKUP_BLOCK2:
        desired_pose = taskspaces[BLOCK2];
        task_space_out.x = taskspaces[BLOCK2].x;
        task_space_out.y = taskspaces[BLOCK2].y;
        task_space_out.z = taskspaces[BLOCK2].theta;
        task_space_pub.publish(task_space_out);
        break;

    case PICKUP_BLOCK3:
        desired_pose = taskspaces[BLOCK3];
        task_space_out.x = taskspaces[BLOCK3].x;
        task_space_out.y = taskspaces[BLOCK3].y;
        task_space_out.z = taskspaces[BLOCK3].theta;
        task_space_pub.publish(task_space_out);
        break;

    case MOVING_TO_DROP:
        desired_pose = taskspaces[BLOCK_DROP];
        task_space_out.x = desired_pose.x;
        task_space_out.y = desired_pose.y;
        task_space_out.z = desired_pose.theta;
        task_space_pub.publish(task_space_out);
        break;

    default:
        break;
    }
}

void task_space_callback(const geometry_msgs::Vector3& msg)
{
    switch (arm_state)
    {
    case PICKUP_BLOCK1:
    case PICKUP_BLOCK2:
    case PICKUP_BLOCK3:
        if ((abs(msg.x - desired_pose.x) < ERROR_THRESHOLD) &&
            (abs(msg.y - desired_pose.y) < ERROR_THRESHOLD) &&
            (abs(msg.z - desired_pose.theta) < ERROR_THRESHOLD))
        {
            gripper_state_out.data = CLOSE;
            gripper_state_pub.publish(gripper_state_out);
            ros::Duration(GRIPPER_CLOSE_TIME).sleep();
            arm_state_out.data = MOVING_TO_DROP;
            arm_state_pub.publish(arm_state_out);
        };
        break;

    case MOVING_TO_DROP:
        if ((abs(msg.x - desired_pose.x) < ERROR_THRESHOLD) &&
            (abs(msg.y - desired_pose.y) < ERROR_THRESHOLD) &&
            (abs(msg.z - desired_pose.theta) < ERROR_THRESHOLD))
        {
            arm_state_out.data = READY_TO_DROP;
            arm_state_pub.publish(arm_state_out);
        };
        break;

    default:
        break;
    }

}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "arm_manager"); /* This is how you create a node. The only thing that
    changes for this function is the string argument which indicates the name of the node.*/
    ros::NodeHandle nodeHandle; /* This creates something called an object that is used in object-
    oriented programming. This object is responsible for handling the publications and 
    subscriptions that this node talks to.*/

    arm_state_sub = nodeHandle.subscribe("arm_state", 1, arm_state_callback);
    arm_state_pub = nodeHandle.advertise<std_msgs::UInt8>("arm_state", 1);

    gripper_state_pub = nodeHandle.advertise<std_msgs::UInt8>("gripper_state", 1);

    task_space_sub = nodeHandle.subscribe("arm_task_space", 1, task_space_callback);
    task_space_pub = nodeHandle.advertise<geometry_msgs::Vector3>("arm_desired_task_space", 1);

    nodeHandle.getParam("/arm_manager/arm_taskspaces", json_path);
    
    loadPoses(json_path, taskspaces);

    ros::Duration(1).sleep();

    gripper_state_out.data = OPEN;
    gripper_state_pub.publish(gripper_state_out);

    while (ros::ok()) // The ros::ok() function returns true as long as ROS is running
    {
        ros::spinOnce(); // This function checks for anything published to a topic we subscribe to
    }

}

void loadPoses(const std::string& file_path, std::vector<Pose>& poses) {
	std::ifstream file(file_path); // Load file and check if successful
	if (!file.is_open()) {
		ROS_ERROR("Could not open JSON file %s", file_path.c_str());
		return;
	}

	json j; // Use variable j to reference in the file
	file >> j;

    poses.clear(); // Clear list if not empty

	for (const auto& item : j) {
		Pose p; // Load a new item from the JSON file
		p.x = item.value("x", 0.0);
		p.y = item.value("y", 0.0);
		p.theta = item.value("theta", 0.0);

		poses.push_back(p); // Push the pose on to the stack
	}
    ROS_INFO("JSON file loaded");
}