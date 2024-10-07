#include "robot_bringup/robot.h"
#include "ros/ros.h"
#include "mbot_linux_serial.h"

double RobotV_ = 0;
double RobotYawRate_ = 0;

//double testSend1=20.0;
//double testSend2=0.0;
//unsigned char testSend3=0x07;


//test receive value
double testRece1=0.0;
double testRece2=0.0;
double testRece3=0.0;
unsigned char testRece4=0x00;
 

// 速度控制消息的回调函数
void cmdCallback(const geometry_msgs::Twist& msg)
{
    RobotV_  = msg.linear.x * 1000;//mm/s//发布的速度是m/s为单位
    RobotYawRate_ = msg.angular.z;//rad/s
    //myrobot.deal(RobotV_, RobotYawRate_);
    //writeSpeed(RobotV_,RobotYawRate_,0);
    
    // ROS_INFO("go to callback");
    //myrobot.deal(RobotV_,RobotYawRate_);
}
    
int main(int argc, char** argv)
{
    //初始化ROS节点
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "mbot_bringup");
    ros::NodeHandle nh;
    robot::robot myrobot;

    //serialInit();

    //初始化robot
    
    if(!myrobot.init())
    ROS_ERROR("myrobot initialized failed.");
    
    ROS_INFO("myrobot initialized successful.");

    ros::Subscriber sub = nh.subscribe("cmd_vel", 50, cmdCallback);
    ROS_INFO("ready");
    
    
    ROS_INFO("complete");
    //循环运行
    ros::Rate loop_rate(50);//50hz
    while (ros::ok()) 
    {
        //ROS_INFO("hello");
        ros::spinOnce();
        // 机器人控制      
        
        myrobot.deal(RobotV_,RobotYawRate_);
        
	
        loop_rate.sleep();
    }

    return 0;
}

