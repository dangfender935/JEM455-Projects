#include <ros/ros.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Vector3.h>
#include <vector>

using json = nlohmann::json;

struct Pose {
	double x;
	double y;
	double theta;
};

enum PoseRequest
{
    RELOAD = 0,
    NEXT_PICKUP_POSE = 1,
	NEXT_DROPOFF_POSE = 2
};

enum BotState : int
{
    UNLOADING = 0,
    MOVING_TO_PICKUP = 1,
    READY_TO_RECEIVE_BLOCK = 2,
    TRANSPORTING_BLOCK = 3
};

enum ArmState : int
{
    DROP_BLOCK = 0,
    PICKUP_BLOCK1 = 1,
    PICKUP_BLOCK2 = 2,
    PICKUP_BLOCK3 = 3,
    MOVING_TO_DROP = 4,
    READY_TO_DROP = 5,
    HAS_DROPPED = 6
};

void loadPoses(const std::string& file_path, std::vector<Pose>& poses);