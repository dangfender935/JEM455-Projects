#include "pose_checkpoint.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_checkpoint"); // Create the "master" node
    ros::NodeHandle n; // Create the node handle object

    std::string json_path; // Load the argument that gives the name of the json file
    n.getParam("/pose_checkpoint/json_file", json_path);
    ROS_INFO("Name of the json file is: %s", json_path.c_str());

    std::vector<Pose> pose_list; // Load the list of poses from the json file
    loadPoses(json_path, pose_list);

    // Print out all poses loaded from the JSON file
    for (int i = 0; i < pose_list.size(); i++) {
    	ROS_INFO("Pose %d: x=%.2f, y=%.2f, theta=%.2f", i, pose_list[i].x, pose_list[i].y, pose_list[i].theta); 
    }
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
