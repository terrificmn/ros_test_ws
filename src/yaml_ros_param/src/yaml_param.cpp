#include <ros/ros.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "ros_yaml_param_test");
	ros::NodeHandle nh;
	
	// 변수를 사용하기 위해 먼저 디폴트 값으로 선언
	int vertical_pwm = 300;
	int horizontal_pwm = 200;

	// 파라미터를 yaml파일로부터 받으려면 일단 런치파일로 실행을 해서 파라미터 서버에 등록
	// 런치파일로 rosparam 태그를 이용해서 읽음 - 이후 노드 핸들로 getParam()을 해온다
	nh.getParam("vertical_motor/pwm_value", vertical_pwm);
	nh.getParam("horizontal_motor/pwm_value", horizontal_pwm);

    ROS_INFO("yaml param1: %d", vertical_pwm);
    ROS_INFO("yaml param2: %d", horizontal_pwm);

    ros::shutdown();
}