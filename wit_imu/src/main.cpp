
/**
 * boost read io data  push rosimu msgs
 */

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <stdio.h>
#include <math.h>

//ros include file
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace boost::asio;

//buffer angle
double a[3],w[3],Angle[3],T;
double g = 9.8;

/**
 * DecodeIMudata
 * @param chrTemp
 * @return
 */
void DecodeIMUData(unsigned char chrTemp[])
{
    switch(chrTemp[1])
    {
        case 0x51:
            a[0] = (short(chrTemp[3]<<8|chrTemp[2]))/32768.0*16 * g;
            a[1] = (short(chrTemp[5]<<8|chrTemp[4]))/32768.0*16 * g;
            a[2] = (short(chrTemp[7]<<8|chrTemp[6]))/32768.0*16 * g;
            T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
            printf("a = %4.3f\t%4.3f\t%4.3f\t\r\n",a[0],a[1],a[2]);
            break;
        case 0x52:
            w[0] = (short(chrTemp[3]<<8|chrTemp[2]))/32768.0*2000;
            w[1] = (short(chrTemp[5]<<8|chrTemp[4]))/32768.0*2000;
            w[2] = (short(chrTemp[7]<<8|chrTemp[6]))/32768.0*2000;
            T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
            printf("w = %4.3f\t%4.3f\t%4.3f\t\r\n",w[0],w[1],w[2]);
            break;
        // case 0x53:
        //     Angle[0] = (short(chrTemp[3]<<8|chrTemp[2]))/32768.0*180;
        //     Angle[1] = (short(chrTemp[5]<<8|chrTemp[4]))/32768.0*180;
        //     Angle[2] = (short(chrTemp[7]<<8|chrTemp[6]))/32768.0*180;
        //     T = (short(chrTemp[9]<<8|chrTemp[8]))/340.0+36.25;
        //     printf("Angle = %4.2f\t%4.2f\t%4.2f\tT=%4.2f\r\n",Angle[0],Angle[1],Angle[2],T);
        //     break;
    }
}

/**
 * 填存imu_msgs
 */
sensor_msgs::Imu push_info_to_imu_msgs(void)
{
    sensor_msgs::Imu imu_data;
    double  deg2rad =  M_PI / 180;
    imu_data.header.stamp = ros::Time::now();
    imu_data.header.frame_id = "imu_link";

    //angle_velocity rad
    imu_data.angular_velocity.x = w[0] * deg2rad ;
    imu_data.angular_velocity.y = w[1] * deg2rad ;
    imu_data.angular_velocity.z = w[2] * deg2rad ;
    
    //liear_acceleration
    imu_data.linear_acceleration.x = - a[0] ;
    imu_data.linear_acceleration.y = - a[1] ;
    imu_data.linear_acceleration.z = - a[2] ;

    return imu_data ;
}

int main(int argc , char ** argv) {
    
    /**
     * ros node and node name
     */
    ros::init(argc,argv,"Wit_imu"); //node name
    ros::NodeHandle n;  
    ros::Publisher wit_pub = n.advertise<sensor_msgs::Imu>("imu",1000);
    //ros::Rate loop_rate(100);;                  //while rate
    
    //create sensor_msgs/IMU_msgs
    sensor_msgs::Imu imu_data_msgs;
    
    /**
     * set boost io
     */
    boost::asio::io_service iosev_;             //io object

    boost::asio::serial_port serial_port_object(iosev_, "/dev/ttyUSB0");

    serial_port_object.set_option(boost::asio::serial_port::baud_rate(230400));
    serial_port_object.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial_port_object.set_option(serial_port::parity(serial_port::parity::none));
    serial_port_object.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    serial_port_object.set_option(serial_port::character_size(8));
   
    int flage_num_count = 0;
    
    while(1)
    {
        //--------------------------------------
        unsigned char chrBuffer[100] = {0};
        unsigned char chrTemp[100] = {0};
        unsigned short usRxLength = 100;

        /* every once read 100 Bytes*/
        read(serial_port_object, buffer(chrBuffer));

        /**
        * 解析UART data
        */
        while (usRxLength >= 11) {

            memcpy(chrTemp, chrBuffer, usRxLength); //copy usRxLength size come  char Buffer 到chrTemp

            if (!((chrTemp[0] == 0x55) & ((chrTemp[1] == 0x51) | (chrTemp[1] == 0x52)  /*| (chrTemp[1] == 0x53)*/ ))) {
                for (int i = 1; i < usRxLength; i++) chrBuffer[i - 1] = chrBuffer[i];  //整体前进左移，丢弃一个
                usRxLength--;
                continue;
            }

            DecodeIMUData(chrTemp);
            
            flage_num_count ++;
            if(flage_num_count = 2)
            {
                //pushlish_msgs flage 两次push_data once
                wit_pub.publish( push_info_to_imu_msgs() );
                flage_num_count = 0;
            }
            
            //减去前面11个
            for (int i = 11; i < usRxLength; i++) chrBuffer[i - 11] = chrBuffer[i];
            usRxLength -= 11;
            
            ros::spinOnce(); 
            //loop_rate.sleep(); 
        }
        //other thread operation
        iosev_.run();
    }

    return 0;
}
