#include "ros/ros.h"

int main(int argc,char *argv[])
{
    //ִ��ROS�ڵ��ʼ��
    ros::init(argc,argv,"hello");
    //����ros�ڵ���
    ros::NodeHandle n;
    //����̨���hello world
    ROS_INFO("hello auto steer");
    return 0;
}