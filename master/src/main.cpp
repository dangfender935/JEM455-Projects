#include <ros/ros.h>  // This .h file must always be included in all ROS code
// There must be a .h file for every message type used
#include <std_msgs/Float64.h> // This is type std_msgs/Float64 which is a standard 64 bit floating point number
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3.h>
#include <keyboard/Key.h> // This is where the keyboard/Key message type is defined

#define Q_KEY           (113)
#define W_KEY           (119)
#define A_KEY           (97)
#define S_KEY           (115)
#define D_KEY           (100)
#define E_KEY           (101)
#define Z_KEY           (122)
#define X_KEY           (120)
#define C_KEY           (99)
#define LINEAR_SPEED    (1)
#define ANGULAR_SPEED   (1)

// Global variables
keyboard::Key message_in; // This will hold the message from the topic we subscribe to
// std_msgs::UInt16 message_out;
geometry_msgs::Vector3 message_out;
ros::Publisher my_publisher_object;
ros::Subscriber keydown_in;


/* This is our callback function that is called when something is published to the keyboard/keydown
topic. Callback functions take in 1 parameter that is an object of the message type that topic 
receives. Callback functions always return void, so we will store this data in our global variable
so we can use it in other functions.*/
void keydown_callback(const keyboard::Key& msg)
{
    message_in = msg; // Store the data from this topic into our global variable
    ROS_INFO("Key pressed has code: %d",message_in.code); /* This is how we do print statements in
    ROS. Here we are just printing the code for the key that was pressed.*/
    switch (message_in.code)
    {
    case Q_KEY:
        message_out.x = LINEAR_SPEED;
        message_out.z = ANGULAR_SPEED;
        break;

    case W_KEY:
        message_out.x = LINEAR_SPEED;
        message_out.z = 0;
        break;

    case E_KEY:
        message_out.x = LINEAR_SPEED;
        message_out.z = -ANGULAR_SPEED;
        break;

    case A_KEY:
        message_out.x = 0;
        message_out.z = ANGULAR_SPEED;
        break;

    case S_KEY:
        message_out.x = 0;
        message_out.z = 0;
        break;

    case D_KEY:
        message_out.x = 0;
        message_out.z = -ANGULAR_SPEED;
        break;

    case Z_KEY:
        message_out.x = -LINEAR_SPEED;
        message_out.z = ANGULAR_SPEED;
        break;

    case X_KEY:
        message_out.x = -LINEAR_SPEED;
        message_out.z = 0;
        break;

    case C_KEY:
        message_out.x = -LINEAR_SPEED;
        message_out.z = -ANGULAR_SPEED;
        break;

    default:
        // message_out.x = 0;
        // message_out.z = 0;
        break;
    
    }
    my_publisher_object.publish(message_out);


}

int main(int argc, char **argv) 
{
    message_out.y = 0;

    ros::init(argc, argv, "master"); /* This is how you create a node. The only thing that
    changes for this function is the string argument which indicates the name of the node.*/
    ros::NodeHandle n; /* This creates something called an object that is used in object-
    oriented programming. This object is responsible for handling the publications and 
    subscriptions that this node talks to.*/

    my_publisher_object = n.advertise<geometry_msgs::Vector3>("local_velocities", 1);
    /* This creates a publisher object. This is done by calling the function n.advertise which
    is a function tied to the object that we created earlier. The <> brackets indicate what
    message type is going to be used for this topic. The first argument is the name of the 
    topic. The "1" argument says to use a buffer size of 1; could make larger, if expect network 
    backups */

    keydown_in = n.subscribe("/keyboard/keydown", 1, keydown_callback);
    /* This creates a subscriber object. This is done by calling the function n.subscribe which
    is also a function tied to the object of class ros::NodeHandle. The first argument is the 
    name of the topic that you want to subscribe to. The second argument is the buffer size which
    we will just set to 1. The third argument is the callback function that gets called when data
    is published to this topic. This works like an interupt.*/

    while (ros::ok()) // The ros::ok() function returns true as long as ROS is running
    {
        // input_float.data = input_float.data + 0.001; //increment by 0.001 each iteration

        // my_publisher_object.publish(input_float); /* This is the function that we call to publish
        // the data in the input_float object. Note that we use the my_publisher_object to call the 
        // function.*/

        // if (message_in.code == Q_KEY)
        // {
        //     message_out.data += 1;
        //     my_publisher_object.publish(message_out);
        //     message_in.code = 114;
        // }

        
	    ros::spinOnce(); // This function checks for anything published to a topic we subscribe to
    }

}