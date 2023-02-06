#include "ros/ros.h"

int main(int argc,char *argv[])
{
    //执行ROS节点初始化
    ros::init(argc,argv,"hello");
    //创建ros节点句柄
    ros::NodeHandle n;
    //控制台输出hello world
    ROS_INFO("hello auto steer");
    return 0;
}