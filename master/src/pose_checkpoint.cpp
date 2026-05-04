#include "pose_checkpoint.h"

ros::Subscriber pose_request_sub;
std_msgs::UInt8 pose_request{};

ros::Publisher pose_controller_pub;
geometry_msgs::Vector3 desired_pose;

ros::Publisher bot_state_pub;
std_msgs::UInt8 bot_state_out;

std::string pickup_json_path; 
std::string dropoff_json_path; 

std::vector<Pose> pickup_pose_list; 
std::vector<Pose> dropoff_pose_list; 


void pose_request_recv_callback(const std_msgs::UInt8& pose_request)
{
    Pose curr_pose;
    static int vec_index = 0;

    switch (pose_request.data)
    {
    case RELOAD:
        loadPoses(pickup_json_path, pickup_pose_list);
        loadPoses(dropoff_json_path, dropoff_pose_list);
        vec_index = 0;
        // break;
        return;

    case NEXT_PICKUP_POSE:
        if (vec_index < pickup_pose_list.size())
        {
            curr_pose = pickup_pose_list.at(vec_index++);
            desired_pose.x = curr_pose.x;
            desired_pose.y = curr_pose.y;
            desired_pose.z = curr_pose.theta;
            pose_controller_pub.publish(desired_pose);
        }
        else
        {
            bot_state_out.data = READY_TO_RECEIVE_BLOCK;
            bot_state_pub.publish(bot_state_out);
            vec_index = 0;
        }
        break;

    case NEXT_DROPOFF_POSE:
        if (vec_index < dropoff_pose_list.size())
        {
            curr_pose = dropoff_pose_list.at(vec_index++);
            desired_pose.x = curr_pose.x;
            desired_pose.y = curr_pose.y;
            desired_pose.z = curr_pose.theta;
            pose_controller_pub.publish(desired_pose);
        }
        else
        {
            bot_state_out.data = UNLOADING;
            bot_state_pub.publish(bot_state_out);
            vec_index = 0;
        }
        break;

    default:
        break;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_checkpoint");
    ros::NodeHandle nodeHandle; // Create the node handle object

    pose_request_sub = nodeHandle.subscribe("pose_request", 1, pose_request_recv_callback);
    pose_controller_pub = nodeHandle.advertise<geometry_msgs::Vector3>("pose_controller_global", 1);

    bot_state_pub = nodeHandle.advertise<std_msgs::UInt8>("bot_state", 1);

    // Load the argument that gives the name of the json file
    nodeHandle.getParam("/pose_checkpoint/SHC2C", pickup_json_path);
    nodeHandle.getParam("/pose_checkpoint/C2SHC", dropoff_json_path);
    // ROS_INFO("Name of the json file is: %s", json_path.c_str());

    // Load the list of poses from the json file
    loadPoses(pickup_json_path, pickup_pose_list);
    loadPoses(dropoff_json_path, dropoff_pose_list);

    // Print out all poses loaded from the JSON file
    // for (int i = 0; i < pose_list.size(); i++) {
    // 	ROS_INFO("Pose %d: x=%.2f, y=%.2f, theta=%.2f", i, pose_list[i].x, pose_list[i].y, pose_list[i].theta); 
    // }

    while (ros::ok()) // The ros::ok() function returns true as long as ROS is running
    {
        ros::spinOnce();
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
