#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using std::string;

#ifndef GLOBAL_PLANNER_PLUGIN_H
#define GLOBAL_PLANNER_PLUGIN_H

namespace global_planner {

    class GlobalPlannerPlugin : public nav_core::BaseGlobalPlanner {
        public:

        GlobalPlannerPlugin();
        GlobalPlannerPlugin(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan);
    };
};
#endif
