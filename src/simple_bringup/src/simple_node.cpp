#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <signal.h>

#include "simple_bringup/com.h"

float wheelDiameter=150.0, wheelLength=450.0, gearRatio=1;
Com Com;


void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
void pauseCallback(const std_msgs::Int8::ConstPtr& msg);
bool isPauseSignReceived = false;


int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_bringup");
    ROS_INFO("simple bringup test");
    ros::NodeHandle nh;
    ros::Subscriber sub_cmd = nh.subscribe("cmd_vel", 10, cmdCallback);
    ros::Subscriber sub_pause = nh.subscribe("pause_order", 1, pauseCallback);
    ros::Publisher pub_pause = nh.advertise<std_msgs::Int8>("pause_order", 1); // for test

    // Com.initSerial("/dev/ttyACM0", 19200);
    
    std_msgs::Int8 msg;
    msg.data = 0;
    int count = 0;

    ros::Rate lr(1);
    
    while(ros::ok()) {
        
        if(count > 5) msg.data = 1;
        pub_pause.publish(msg);
        ros::spinOnce();
        
        count++;
        if(count > 10) {
            count = 0;
            msg.data = 0;
        }

        lr.sleep();
    }

    ros::shutdown();
    return 0;
}


void cmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
    double theta, vel;

    if(isPauseSignReceived == true) {
        theta = 0.0;
        vel = 0.0;
    } else {
        theta = cmd_msg->angular.z;  //rad
        vel = cmd_msg->linear.x * 1000.0;  // mm/s    
    }

    double rps = vel / (wheelDiameter * M_PI) * gearRatio; // wheelRev/s * gearRatio
    double rpm = rps * 60.0;

    double rpm_diff = (wheelLength / 2.0 * theta ) / (wheelDiameter * M_PI) 
                        * 60.0 * gearRatio;

    ROS_INFO("rpm-rpm_diff: %lf \t rpm+rpm_diff: %lf", rpm - rpm_diff, rpm + rpm_diff);
    Com.setTargetVelocity(rpm - rpm_diff, rpm + rpm_diff);
}

void pauseCallback(const std_msgs::Int8::ConstPtr& msg) {
    if(msg->data == 0) {
        isPauseSignReceived = false;
    } else if (msg->data == 1) {
        isPauseSignReceived = true;
    }

    // ROS_INFO("pause: %s", isPauseSignReceived ? "True" : "False");
}