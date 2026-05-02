#include <ros/ros.h>  // This .h file must always be included in all ROS code
// There must be a .h file for every message type used
#include <std_msgs/UInt16.h>
#include <string>
#include <iostream>

#define SECONDS_TO_WAIT         (3)

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
    READY_TO_DROP = 4,
    HAS_DROPPED = 5
};

ros::Subscriber bot_state_sub;
std_msgs::UInt16 bot_state_out;
int bot_state = UNLOADING;

ros::Publisher bot_state_pub;

ros::Subscriber arm_state_sub;
std_msgs::UInt16 arm_state_out;
int arm_state = DROP_BLOCK;

ros::Publisher arm_state_pub;

int sequence[3];
int curr_sequence_indx = 0;

void state_machine()
{
    static int blocks_delivered = 0;
    if (bot_state == READY_TO_RECEIVE_BLOCK)
    {
        switch (arm_state)
        {
            case READY_TO_DROP:
                arm_state_out.data = DROP_BLOCK;
                arm_state_pub.publish(arm_state_out);
                break;

            case HAS_DROPPED:
                bot_state_out.data = TRANSPORTING_BLOCK;
                bot_state_pub.publish(bot_state_out);
                if (blocks_delivered < 2)
                {
                    arm_state_out.data = sequence[blocks_delivered + 1];
                    arm_state_pub.publish(arm_state_out);
                }
                break;

            default:
                break;
        }
        return;
    }

    if (bot_state == UNLOADING)
    {
        ros::Duration(SECONDS_TO_WAIT).sleep();
        blocks_delivered++;
        if (blocks_delivered < 3)
        {
            bot_state_out.data = MOVING_TO_PICKUP;
            bot_state_pub.publish(bot_state_out);
        }
    }
}

void bot_state_callback(const std_msgs::UInt16& msg)
{
    bot_state = msg.data;
    state_machine();
}

void arm_state_callback(const std_msgs::UInt16& msg)
{
    arm_state = msg.data;
    state_machine();
}

void get_sequence()
{
    int sequence_indx;
    std::string sequence_string;
    while (true)
    {
        ROS_INFO("Enter your three number sequence: ");
        std::getline(std::cin, sequence_string);
        sequence_indx = 0;
        for (int i = 0; i < sequence_string.length(); i++)
        {
            switch (sequence_string.at(i))
            {
                case '1':
                    sequence[sequence_indx++] = 1;
                    break;

                case '2':
                    sequence[sequence_indx++] = 2;
                    break;

                case '3':
                    sequence[sequence_indx++] = 3;
                    break;
                
                default:
                    continue;
            }
            if (sequence_indx >= 3) break;
        }

        if ((sequence[0] == sequence[1]) || (sequence[0] == sequence[2]) || (sequence[1] == sequence[2]) 
            || (sequence_indx < 2))
        {
            ROS_INFO("Invalid Sequence!");
            continue;
        }
        break;
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "master"); /* This is how you create a node. The only thing that
    changes for this function is the string argument which indicates the name of the node.*/
    ros::NodeHandle nodeHandle; /* This creates something called an object that is used in object-
    oriented programming. This object is responsible for handling the publications and 
    subscriptions that this node talks to.*/

    bot_state_sub = nodeHandle.subscribe("bot_state", 1, bot_state_callback);
    bot_state_pub = nodeHandle.advertise<std_msgs::UInt16>("bot_state", 1);

    arm_state_sub = nodeHandle.subscribe("arm_state", 1, arm_state_callback);
    arm_state_pub = nodeHandle.advertise<std_msgs::UInt16>("arm_state", 1);
    
    get_sequence();
    ROS_INFO("Sequence: [%i %i %i]", sequence[0], sequence[1], sequence[2]);
    
    arm_state_out.data = sequence[curr_sequence_indx++];
    arm_state_pub.publish(arm_state_out);

    bot_state_out.data = MOVING_TO_PICKUP;
    bot_state_pub.publish(bot_state_out);

    while (ros::ok()) // The ros::ok() function returns true as long as ROS is running
    {
        ros::spinOnce(); // This function checks for anything published to a topic we subscribe to
    }

}