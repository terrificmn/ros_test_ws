#include <ros/ros.h>
#include <obstacle_check/obstacle.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_check_node");

    ObstacleCheck ob_check;
    
    ros::Rate lr(1);
    while(ros::ok()) {

        ros::spinOnce();
        lr.sleep();
    }
}