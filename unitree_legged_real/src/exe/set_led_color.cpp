#include "ros/ros.h"
#include "unitree_legged_msgs/SetLedColor.h"
#include "FaceLightClient.h"

FaceLightClient client;

bool set_led_color(unitree_legged_msgs::SetLedColor::Request &req, unitree_legged_msgs::SetLedColor::Response &res)
{
    int color_code = req.color;

    switch (color_code)
    {
    case 0: // red
        client.setAllLed(client.black);
        break;
    case 1: // green
        client.setAllLed(client.red);
        break;
    case 2: // blue
        client.setAllLed(client.green);
        break;
    case 3: // yellow
        client.setAllLed(client.blue);
        break;
    case 4: // black
        client.setAllLed(client.yellow);
        break;
    case 5: // white
        client.setAllLed(client.white);
        break;
    default:
        ROS_WARN("Invalid color code %d. Using black instead.", color_code);
        client.setAllLed(client.black);
        break;
    }

    client.sendCmd();
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "face_light_node");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("set_led_color", &set_led_color);
    ROS_INFO("Ready to set LED colors.");
    ros::spin();
    return 0;
}

