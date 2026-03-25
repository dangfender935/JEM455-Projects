#include <ros/ros.h>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct Pose {
	double x;
	double y;
	double theta;
};

void loadPoses(const std::string& file_path, std::vector<Pose>& poses);