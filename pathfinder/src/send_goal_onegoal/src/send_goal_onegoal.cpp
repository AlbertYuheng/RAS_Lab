#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Int16.h"
#include <iostream>
#include <visualization_msgs/Marker.h>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher marker_pub_;
visualization_msgs::Marker marker;
void plot_marker(move_base_msgs::MoveBaseGoal &goal1)
{
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    // 设置marker的类型
    marker.type = visualization_msgs::Marker::SPHERE;
    // 设置marker的操作
    marker.action = visualization_msgs::Marker::MODIFY;
    // 设置marker的大小
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    // 设置marker的颜色和透明度
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    // 设置marker的位姿
    marker.pose.position.x = goal1.target_pose.pose.position.x;
    marker.pose.position.y = goal1.target_pose.pose.position.y;
    marker.pose.position.z = goal1.target_pose.pose.position.z;
    marker.pose.orientation.x = goal1.target_pose.pose.orientation.x;
    marker.pose.orientation.y = goal1.target_pose.pose.orientation.y;
    marker.pose.orientation.z = goal1.target_pose.pose.orientation.z;
    marker.pose.orientation.w = goal1.target_pose.pose.orientation.w;
    // 发布marker
    marker_pub_.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goal_onegoal");
    ros::NodeHandle nh123;//创建一个句柄
    MoveBaseClient ac("move_base", true);//创建movebase的一个动作的订阅者
    ros::Publisher pub_ = nh123.advertise<std_msgs::Int16>("/goal_flag", 1);//创建一个发布者告诉导航已到达目标点
    marker_pub_ = nh123.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    std_msgs::Int16 Int16_flag;

    uint8_t goal_number = 4;

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal[4];

    // 待发送的 目标点 在 map 坐标系下的坐标位置
    // 左
    goal[0].target_pose.pose.position.x = 3.786541281847143;
    goal[0].target_pose.pose.position.y = 0.4139120219479551;
    goal[0].target_pose.pose.orientation.z = 0.7013865848187414;
    goal[0].target_pose.pose.orientation.w = 0.7127810734273903;

    //     中
    // goal[0].target_pose.pose.position.x = 9.542451390468182;
    // goal[0].target_pose.pose.position.y = 0.7223462791828574;
    // goal[0].target_pose.pose.orientation.z = 0.7913752221477446;
    // goal[0].target_pose.pose.orientation.w = 0.6113307269969407;

    //          右
    // goal[0].target_pose.pose.position.x = 9.215377807617188;
    // goal[0].target_pose.pose.position.y = -0.8106219172477722;
    // goal[0].target_pose.pose.orientation.z = -0.18787541637705749;
    // goal[0].target_pose.pose.orientation.w = 0.9821928669671488;

    ROS_INFO(" Init success!!! ");

    goal[0].target_pose.header.frame_id = "map";
    goal[0].target_pose.header.stamp = ros::Time::now();
    ac.sendGoal(goal[0]);
    plot_marker(goal[0]);
    ROS_INFO("move to one goal");

    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("success to final goal!");
        system("rosrun cvfinder f_2");
        // cout << "发送给红圈检测flag" << endl;
        // Int16_flag.data = 1;
        // goal_number -- ;
        // pub_.publish(Int16_flag);
        // ros::Duration(0.4).sleep();
        // pub_.publish(Int16_flag);
        // ros::Duration(0.4).sleep();
        // pub_.publish(Int16_flag);
        // ros::Duration(0.4).sleep();
        // pub_.publish(Int16_flag);
        // ros::Duration(0.4).sleep();
        // pub_.publish(Int16_flag);
    }
    else
    {
        ROS_WARN("The final Goal Planning Failed for some reason");//调试信息，未到达目标点
    }

    return 0;
}
