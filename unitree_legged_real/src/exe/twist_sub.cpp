#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};
    ros::Publisher pub_high;
    
public:
    Custom()
        : 
        // low_udp(LOWLEVEL),
        low_udp(LOWLEVEL, 8091, "192.168.123.10", 8007),
        high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
    
    void publishHighState()
    {
        unitree_legged_msgs::HighState high_state_ros;

        high_state_ros = state2rosMsg(high_state);

        pub_high.publish(high_state_ros);
    }
};

Custom custom;
int gaittype = 1;
int mode12 = 2;
ros::Subscriber sub_cmd_vel;
ros::Subscriber sub_high;
ros::Subscriber sub_gaittype;
ros::Subscriber sub_mode12;

long cmd_vel_count = 0;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);
    custom.high_cmd = rosMsg2Cmd(msg,gaittype,mode12);

    printf("mode = %d\n", mode12);
    printf("gaittype = %d\n", gaittype);

    printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);
    printf("cmd_gaittype = %d\n", custom.high_cmd.gaitType);

    printf("cmdVelCallback ending!\t%ld\n\n", cmd_vel_count++);
}
void highCmdCallback(const unitree_legged_msgs::HighCmd::ConstPtr &msg)
{
    
    custom.high_cmd = rosMsg2Cmd(msg);
}

void giattypeCallback(const std_msgs::Int8::ConstPtr& msg)
{
    gaittype = msg->data;
}
void mode12Callback(const std_msgs::Int8::ConstPtr& msg) // switch mode 1 and 2
{
    mode12 = msg->data;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_sub");

    ros::NodeHandle nh;
    sub_high = nh.subscribe("high_cmd", 1, highCmdCallback);
    custom.pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);

    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);
    sub_gaittype = nh.subscribe<std_msgs::Int8>("gaittype", 1, giattypeCallback);
    sub_mode12 = nh.subscribe<std_msgs::Int8>("mode_12", 1, mode12Callback);

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));
    LoopFunc loop_pubHighState("pub_high_state", 0.002,3, boost::bind(&Custom::publishHighState, &custom));
    
    loop_udpSend.start();
    loop_udpRecv.start();
    loop_pubHighState.start();
    
    ros::spin();

    return 0;
}
