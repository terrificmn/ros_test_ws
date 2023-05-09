#include <obstacle_check/obstacle.h>


ObstacleCheck::ObstacleCheck() {
    ROS_INFO("obstacle check class init");
    sub_grid_map = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 10, &ObstacleCheck::obstacleCb, this);
    pub_marker = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
}

ObstacleCheck::~ObstacleCheck() {}

void ObstacleCheck::obstacleCb(const nav_msgs::OccupancyGrid::ConstPtr & msg) {
    ROS_INFO("width: %d", msg->info.width);
    ROS_INFO("height: %d", msg->info.height);
    ROS_INFO("size: %ld", msg->data.size());
    ROS_INFO("data: %d", msg->data[2000]);

    ROS_INFO("origin x: %lf", msg->info.origin.position.x);
    ROS_INFO("origin y: %lf", msg->info.origin.position.y);

    std::vector<std::vector<int>> grid_map {};
    grid_map.clear();
    // ROS_INFO("data: %d", msg->data[0 * 1 + 1]);
    // grid_map.push_back(std::vector<int>());
    // grid_map[0].push_back(msg->data[0 * 1 + 1]);


    for(long int i=0; i < msg->info.width; i++) {
        grid_map.push_back(std::vector<int>());

        for(long int j=0; j < msg->info.height; j++) {
            long int height_size = msg->info.height;
            grid_map[i].push_back(msg->data[height_size * i + j]);
        }
    }

    ROS_INFO("complete.. grid_map x size: %ld", grid_map.size());
    ROS_INFO("complete.. grid_map y size: %ld", grid_map[0].size());

    ROS_WARN("grid_map data30 50: %d", grid_map[30][50]);
    ROS_WARN("grid_map data40 50: %d", grid_map[40][50]);
    ROS_WARN("grid_map data50 60: %d", grid_map[50][60]);

    this->marker(4.51, -4.06);   
}


// marker는 잘 표시되지만, grid_map으로 만든 것과 좌표를 어떻게 공유할 수 있을지 생각해봐야함
void ObstacleCheck::marker(double x, double y) {
    visualization_msgs::Marker marker_msg;

    marker_msg.header.frame_id = "map";
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.ns = "robot";
    marker_msg.id = 1;
    marker_msg.type = visualization_msgs::Marker::ARROW;
    marker_msg.action = visualization_msgs::Marker::ADD;

    marker_msg.pose.position.x = x; 
    marker_msg.pose.position.y = y;
    marker_msg.pose.position.z = 0.0;

    marker_msg.pose.orientation.x =0.0;
    marker_msg.pose.orientation.y =0.0;
    marker_msg.pose.orientation.z =0.0;
    marker_msg.pose.orientation.w =0.0;
    marker_msg.scale.x = 0.5;  // display scale
    marker_msg.scale.y = 0.5;
    marker_msg.scale.z = 0.5;

    marker_msg.color.r = 0.0; 
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;

    pub_marker.publish(marker_msg);
}