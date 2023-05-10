# ROS_TEST_WS
ros / cpp 패키지 테스트용 workspace

일반적으로 패키지 하나만 있는 repo가 아닌, 워크 스페이스 통으로 사용   
그래서 ros패키지들이 서로 전혀 관련이 없을 수 있다.  

> 패키지는 src 디렉토리에서 확인. devel, build, 디렉토리 등은 ignore 되어 올리지 않음

- 주로 c++ 짜여진 ros package 테스트용으로 사용함
- reference로 사용


## 최초 클론 및 빌드
최초 깃 클론 후 cakin build 를 해준다 
```
git clone https://github.com/terrificmn/ros_test_ws.git
```

이후 
```
cd ros_test_ws
catkin build
```

아마도 다른 곳에 설치를 하면 bt (behavior tree 의존성에서 빌드 실패할 듯..) 등에서 빌드 실패할 수 있음  
관련 문서(md) 찾아보기 

> 추후 필요한 의존성 있다면, 업데이트 예정

source 해준 후 사용
```
source devel/setup.bash
```

