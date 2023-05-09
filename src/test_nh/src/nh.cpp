#include <ros/ros.h>
#include <std_msgs/Int8.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "mynode");
    ros::NodeHandle nh;

    // ros::NodeHandle nh(""), ros::NodeHandle nh("~"), ros::NodeHandle nh("my_special_node")
    // 노드 핸들에 이름을 부여하면 private NodeHandle이 되게 된다 
    // rostopic list로 확인해보기
    ROS_INFO("start");

    std::string path = std::getenv("PATH");
    ROS_INFO("env path: %s", path.c_str());
    
    /// param test
    std::string test_param1, test_param2;
    bool test_param3;

    nh.getParam("test_param1", test_param1);
    nh.getParam("test_param2", test_param2);
    nh.getParam("test_param3", test_param3);

    ROS_INFO("test param1: %s", test_param1.c_str());
    ROS_INFO("test param2: %s", test_param2.c_str());
    ROS_INFO("test param3: %s", test_param3 ? "true" : "false");

    

    ros::Publisher my_pub;
    my_pub = nh.advertise<std_msgs::Int8>("my_topic", 1);

    ros::Rate lr(1);
    while(ros::ok()) {
        std_msgs::Int8 msg;
        // ROS_INFO("loop?");
        msg.data = 1;
        my_pub.publish(msg);

        ros::spinOnce();
        lr.sleep();
    }

    ros::shutdown();
    return 0;
}