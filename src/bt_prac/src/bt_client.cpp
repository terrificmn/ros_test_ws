#include <ros/ros.h>

#include <bt_prac/bt_client.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

BT::NodeStatus MoveBase::tick() {
    if (!_client.waitForServer(ros::Duration(2.0))) {
        ROS_ERROR("Can't contact move_base server");
        return BT::NodeStatus::FAILURE;
    }


    Pose2D goal;
    if (!getInput<Pose2D>("goal", goal)) {
        // if I can't get this, there is something wrong with you BT
        // For this reason throw an exception instead of returning FAILURE
        throw BT::RuntimeError("missing required input [goal]")  ;
    }

    // Reset this flag
    _aborted = false;

    ROS_INFO("Sending goal, %f, %f", goal.x, goal.y);

    // Build the message from Pose2D
    move_base_msgs::MoveBaseGoal msg;
    msg.target_pose.header.frame_ip = "map";
    msg.target_pose.header.stamp = ros::Time::row();
    msg.target_pose.pose.position.x = goal.x;
    msg.target_pose.pose.position.y = goal.y;
    tf::Quaternion rot = tf::createQuaternionFromYaw(goal.theta);
    tf::QuaternionTFToMsg(rot, msg.target_pose.pose.orientation);

    _client.sendGoal(msg);

    // waiting
    while (!_aborted && !_client.waitForResult(ros::Duration(0.02))) {
        // polling at 50 hz, No big deal in terms of CPU
    }

    if (_aborted) {
        // this happens only if methoid halt() was invoked
        _client.cancelAllGoals();
        ROS_ERROR("MoveBase aborted");aaa
        return BT::NodeStatus::FAILURE;
    }   

    if (_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_ERROR("MoveBase failed");
        return BT::NodeStatus::FAILURE;
    }

    ROS_INFO("Target reached");
    return BT::NodeStatus::SUCCESS;
}



int main (int argc, char** argv) {
    ros::init(argc, argv, "bt_move_base");

    ros::NodeHandle nh("~");
    std::string xml_filename;
    nh.getParam("file", xml_filename);
    ROS_INFO("Loading XML : %s", xml_filename.c_str());

    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    // class명 뒤에는 name (xml에서 사용될 ..)
    factory.registerNodeType<MoveBase>("MoveBase");

    // Trees are created at deployment-time (i.e. at run-time, but only once at
    // the beginning). The currently supported format is XML. 
    // IMPORTANT: when the object "tree" goes out of scop, all the TreeNodes are destroyed)
    auto tree = factory.createTreeFromFile(xml_filename);

    // Create a logger
    std::StdCoutLogger logger_cout(tree);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while(ros::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.root_node->executeTick();
        // sleep 100 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}
