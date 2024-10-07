#ifndef MBOT_LINUX_SERIAL_H
#define MBOT_LINUX_SERIAL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include <ctime> 

extern void serialInit();
extern void readSpeed1();
extern void readSpeed2();
extern void setSpeed(uint8_t ID,union sendData  wheelV);
extern void void servo_TransAngle(uint8_t servoId,float angle, uint16_t interval, uint16_t power);
extern void writeSpeed(double RobotV, double YawRate);
extern bool readSpeed(double &vx,double &vth,double &th,unsigned char &ctrlFlag);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

#endif