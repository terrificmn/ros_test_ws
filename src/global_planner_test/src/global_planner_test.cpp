#include <pluginlib/class_list_macros.h>
#include "global_planner_test/global_planner_test.h"

// 이 cpp파일은 dummy static plan 을 만든다   

//register this planner as a BaseGlobalPlanner plugin
// BaseGlobalPlanner로 플러그인 등록, 위의 pluginlib/class_list_macros.h 파일이 필요하다
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlannerTest, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {

GlobalPlannerTest::GlobalPlannerTest () {
}

GlobalPlannerTest::GlobalPlannerTest(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros);
}


void GlobalPlannerTest::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {

}

bool GlobalPlannerTest::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ) {

    plan.push_back(start);  // initial location과 target location 을 위해 사용
    for (int i=0; i<20; i++){
        geometry_msgs::PoseStamped new_goal = goal;
        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

        new_goal.pose.position.x = -2.5+(0.05*i);
        new_goal.pose.position.y = -3.5+(0.05*i);

        new_goal.pose.orientation.x = goal_quat.x();
        new_goal.pose.orientation.y = goal_quat.y();
        new_goal.pose.orientation.z = goal_quat.z();
        new_goal.pose.orientation.w = goal_quat.w();

        plan.push_back(new_goal);
    }
    plan.push_back(goal);
    // 만들어진 vector plan은 move_base global planner 모듈로 전송되고 nav_msgs/Path 토픽으로 publish된다 
    return true;
}

}; //namespace