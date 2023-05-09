#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <vector>

class ObstacleCheck {
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_grid_map;
    ros::Publisher pub_marker;

public:
    ObstacleCheck();
    ~ObstacleCheck();
    void obstacleCb(const nav_msgs::OccupancyGrid::ConstPtr & msg);
    void marker(double x, double y);

};

#endif