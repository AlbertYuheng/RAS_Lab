#include "robot_bringup/mbot_linux_serial.h"
#include "string.h"
#include <boost/bind.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include <ctime> 

using namespace std;
using namespace boost::asio;

serial::Serial ser;
serial::Serial ser2;
#define sBUFFER_SIZE 1024
#define rBUFFER_SIZE 9
unsigned char s_buffer[sBUFFER_SIZE];
unsigned char r_buffer[rBUFFER_SIZE];

//串口相关对象
//boost::asio::io_service iosev;
//boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");
//boost::system::error_code err;


//发送左右轮速控制速度共用体,传感器的X，Z，Angle
union sendData
{
    short d;
    uint8_t data[2];
} leftVelSet, rightVelSet,num,wheelV,angleSet;

//接收数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
union receiveData
{
    short d;
    uint8_t data[2];
} leftVelNow, rightVelNow, angleNow;

const double ROBOT_WIDE = 400.00; // 两轮之间距离  
const double ROBOT_RADIUS = 76.2;  //  轮子半径  mm
const double ROBOT_LENGTH = 456.0;  //前后轮距离  


//延时函数
void   Delay(int   time)//单位us 
{ 
clock_t   now   =   clock(); 

while(clock()-now < time); 
} 

/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void serialInit()
{
    
    //sp.set_option(serial_port::baud_rate(115200));
    //sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    //sp.set_option(serial_port::parity(serial_port::parity::none));
    //sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    //sp.set_option(serial_port::character_size(8));
    
    try
    {
        ser.setPort("/dev/ttyUSB1");
        // ser.setPort("/dev/ttyUSB0");　// 这个端口号就是之前用cutecom看到的端口名称
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_INFO_STREAM("Failed to open port ");
        // return -1;
    }
    ROS_INFO_STREAM("Succeed to open port");
    
    try
    {
        ser2.setPort("/dev/ttyUSB0");
        ser2.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser2.setTimeout(to);
        ser2.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open ttyUSB1 ");
    }

    if (ser2.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized.\n");
    }
    //初始化ttyUSB1
}

/********************************************************
函数功能：读取下位机返还的数据并打印
入口参数:
出口参数：
********************************************************/
void readSpeed1()
{
    if (ser.available())
		{
			// 读取串口数据ROBOT_LENGTH
			size_t bytes_read = ser.read(r_buffer, ser.available());
            ROS_INFO("%s",r_buffer);
		}
}

void readSpeed2()
{
    if (ser2.available())
		{
			// 读取串口数据ROBOT_LENGTH
			size_t bytes_read = ser2.read(r_buffer, ser2.available());
            ROS_INFO("%s",r_buffer);
		}
}

/********************************************************
函数功能：将机器人的轮子速度打包发送给下位机
入口参数：电机名称，线速度
出口参数：
********************************************************/
uint8_t buf[9] = {0};
void setSpeed(uint8_t ID,union sendData  wheelV)
{
    
    buf[0]=0x86;
    buf[1]=0xc1;  //帧头
    buf[2]=ID;    //电机编号
    buf[3]=0x01;  //几号模式
    buf[4]=sizeof(buf);  //数组长度
    if(wheelV.d>=0)
    {

        buf[5]=wheelV.data[1];   //电机速度，舵机时为角度,已将数值转化为0～32767的数字范围
        buf[6]=wheelV.data[0];   //电机速度，舵机时为角度
        buf[7]=0x00;     //正反转
    }
    else {
        wheelV.d=-wheelV.d;
        buf[5]=wheelV.data[1];
        buf[6]=wheelV.data[0];
        buf[7]=0x01;
    }

    uint8_t sum = 0;
    for(int i=0;i<8;i++)
    {
        sum += buf[i]; 
    }
    buf[8]=sum % 256;
    ser.write(buf,sizeof(buf));
    // ROS_INFO("串口1发送了%x %x %x %x %x %x %x %x %x",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8]);
    readSpeed1();
    Delay(40000);  //增加延时函数，保证下位机能处理完上一个指令，否则数据会堵塞
}

/********************************************************
函数功能：将机器人的舵机角度打包发送给舵机转接板
入口参数：舵机id，角度，转动时间，功率
出口参数：
********************************************************/

uint8_t CMD_TransAngle[12] = {0x12,0x4C,0x08,0x07,0,0,0,0,0,0,0,0};
    
// 【舵机】中断发送设定角度指令
void servo_TransAngle(uint8_t servoId,float angle, uint16_t interval, uint16_t power)
{
    angle = angle+20;//舵机初始位置不准，需要映射
	CMD_TransAngle[4] = servoId;
    
	int16_t cont[3] = {(int16_t)(angle*10), (int16_t)interval , (int16_t)power};
	for(uint8_t i=0;i<3;i++){
		CMD_TransAngle[5+i*2] = (uint8_t)(cont[i] & 0xFF);
		CMD_TransAngle[6+i*2] = (uint8_t)((cont[i] >> 8) &0xFF);
	}

	uint8_t sum=0;
	for(uint8_t i=0;i<11;i++){
		sum+=CMD_TransAngle[i];
	}
	CMD_TransAngle[11] = sum % 256;
    
	ser2.write(CMD_TransAngle,12);
    ROS_INFO(" 串口2发送了%x %x %x %x %x %x %x %x %x %x %x %x",CMD_TransAngle[0],CMD_TransAngle[1],CMD_TransAngle[2],CMD_TransAngle[3],CMD_TransAngle[4],
    CMD_TransAngle[5],CMD_TransAngle[6],CMD_TransAngle[7],CMD_TransAngle[8],CMD_TransAngle[9],CMD_TransAngle[10],CMD_TransAngle[11]);
    Delay(100000);
}


/********************************************************
函数功能：将机器人的线速度和角速度分解成左右轮子和舵机速度，交给setSpeed处理
入口参数：机器人线速度、角速度
出口参数：
********************************************************/
void writeSpeed(double RobotV, double YawRate)
{
    // YawRate = YawRate*3;
    
    double r = RobotV / YawRate; //转弯半径 mm
    float angle = 0;
    double leftSpeed = 0,rightSpeed = 0;
    int16_t leftwheelV,rightwheelV,anglewheelV,flag=0;
    // 计算左右轮期望速度
    if (RobotV == 0 && YawRate!=0) //原地掉头
    {
        // leftSpeed = -YawRate * ROBOT_WIDE / 2 / ROBOT_RADIUS*32747/30.0*3; // rad/s
        // rightSpeed = YawRate * ROBOT_WIDE / 2 / ROBOT_RADIUS*32747/30.0*3; 
        // double w = YawRate;
        // double v = (rightSpeed/32747.0*30.0/3.0*ROBOT_RADIUS-leftSpeed/32747.0*30.0/3.0*ROBOT_RADIUS)/2.0;
        // angle = atan(ROBOT_LENGTH*w/v);
        angle = YawRate*4;

        // if(YawRate>0)
        // {angle=30;}
        // else {angle=-30;}
    }
    else if (YawRate == 0 || RobotV != 0) //直线
    {
        leftSpeed = RobotV / ROBOT_RADIUS*32747/30.0*2; // rad/s
        rightSpeed = RobotV / ROBOT_RADIUS*32747/30.0*2;
    }
    if(RobotV!=0 && YawRate != 0) //速度不一致
    {
        if(RobotV > 0)
        angle = atan(YawRate * ROBOT_LENGTH / RobotV)*2;  //舵机转角
        else
        {angle = -atan(YawRate * ROBOT_LENGTH / RobotV)*2;}  //舵机转角
        // leftSpeed = RobotV / ROBOT_RADIUS*32747/30.0*2; // rad/s
        // rightSpeed = RobotV / ROBOT_RADIUS*32747/30.0*2;
        if(RobotV > 0)
        {
        leftSpeed = YawRate * (r - ROBOT_WIDE/2) / ROBOT_RADIUS*32747/30.0*2; // rad/s
        rightSpeed = YawRate * (r + ROBOT_WIDE/2) / ROBOT_RADIUS*32747/30.0*2;
        }
        else{
        rightSpeed = YawRate * (r - ROBOT_WIDE/2) / ROBOT_RADIUS*32747/30.0*2; // rad/s
        leftSpeed = YawRate * (r + ROBOT_WIDE/2) / ROBOT_RADIUS*32747/30.0*2;

        }
    }

    ROS_INFO("RobotV %f, YawRate %f",RobotV,YawRate);
    // ROS_INFO("left %f,right %f angle %f",leftSpeed/32747.0*30.0,rightSpeed/32747.0*30.0,angle );
    // if(leftSpeed<=30 || leftSpeed>=-30)
    // {
        leftVelSet.d=(short)leftSpeed;
        setSpeed(0x01,leftVelSet);
    // } //电机转速范围是-30～30rad
    // else 
    // {ROS_INFO("leftSpeed is too big");}  

    // if(rightSpeed<=30 || rightSpeed>=-30)
    // {
        
        rightVelSet.d=(short)rightSpeed;
        setSpeed(0x02,rightVelSet);
    // }
    // else 
    // {ROS_INFO("rightSpeed is too big");}
    angle=angle*20;
    if(angle<=-45)
    {
        angle=-30;
    }
    if(angle>=45)
    {
        angle=30;
    }
    // if(angle<=90 || angle>=-90)
    // {
        servo_TransAngle(0, -angle, 100, 0);
    // }  //舵机范围是-35～35

}


/********************************************************
函数功能：从下位机读取数据，解析出线速度、角速度、角度
入口参数：机器人线速度、角速度、角度，引用参数
出口参数：bool
********************************************************/
bool readSpeed(double &vx, double &vth, double &th, unsigned char &ctrlFlag)
{
    
		if (ser.available())
		{
			// 读取串口数据ROBOT_LENGTH
			size_t bytes_read = ser.read(r_buffer, ser.available());
            ROS_INFO_STREAM(r_buffer);
		}
		//imu_pub.publish(msg);
    return true;
}


/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/ 
// unsigned char getCrc8(unsigned char *ptr, unsigned short len)
// {
//     unsigned char crc;
//     unsigned char i;
//     crc = 0;
//     while (len--)
//     {
//         crc ^= *ptr++;
//         for (i = 0; i < 8; i++)
//         {
//             if (crc & 0x01)
//                 crc = (crc >> 1) ^ 0x8C;
//             else
//                 crc >>= 1;
//         }
//     }
//     return crc;
// }
