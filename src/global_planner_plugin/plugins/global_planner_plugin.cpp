#include <pluginlib/class_list_macros.h>
// #include "global_planner_plugin/global_planner_test.h"
#include "global_planner_plugin.h"

PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlannerPlugin, nav_core::BaseGlobalPlanner)

namespace global_planner {

    GlobalPlannerPlugin::GlobalPlannerPlugin() {}

    GlobalPlannerPlugin::GlobalPlannerPlugin(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        initialize(name, costmap_ros);
    }


    void GlobalPlannerPlugin::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {

    }

    bool GlobalPlannerPlugin::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        std::vector<geometry_msgs::PoseStamped>& plan)      {
        
        ROS_INFO("test");
        plan.push_back(start);
        // 시작과 골 사이에 차이르 ㄹ넣어줌
        double diff_x = goal.pose.position.x - start.pose.position.x;
        double diff_y = goal.pose.position.y - start.pose.position.y;

        // 두 포인트 사이에 point 추가
        for(int i=0; i < 2; i++) {
            // goal을 그대로 받지만, x, y만 다르게 설정해준다 
            geometry_msgs::PoseStamped mid_point = goal;

            // 골과 시작의 차이점과 곱한 값을 차이점의 중간 지점을 ...
            mid_point.pose.position.x = start.pose.position.x + (0.5 * i * diff_x);
            mid_point.pose.position.y = start.pose.position.y + (0.5 * i * diff_y);

            // 번갈아 가면서 0.5 값이 더 하거나 빼진다 (z)
            if(i % 2) {
                mid_point.pose.position.y += 0.5;
            } else {
                mid_point.pose.position.y -= 0.5;
            }
            // push_back 해서 publish 될 수 있게 한다 
            plan.push_back(mid_point);
            // 이제 zig zag로 움직이게 되는데 global path는 만들어 지지만 실제로는 그렇게 가지는 않음
            // 왜냐하면 local planner가 개입을 해서 compensates를 하기 때문에 ...
        }
        
        plan.push_back(goal);

        return true;
    }

    
};