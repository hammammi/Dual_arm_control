# mservo_manipulator_control

## massage   
mservo_msg : mservo lab에서 사용하는 ros msgs   

## EtherCAT   


## MoveIt   

package :   
   
- dualarm_config : solidworks로 만든 urdf 파일과 mesh 파일 (STL)   

 * 아래 moveit 패키지 만들 시 urdf가 수정됨
 * urdf 생성 시 dualarm으로 바로 생성되지 않아 ver1/ver2 각각 urdf 생성 후 파일 수정하여 base link에 연결   
 * dummy link, joint를 생성하여야 moveit에서 vurtual joint 활용하여 world에 attatch 가능   
 * griper 끝을 eef position으로 planning 하기 위해서는 urdf 생성 시 eef joint coordinate를 원하는 위치에 생성해야 함   

     
ver2. limit   
joint 4 : +- 120   
joint 5 : +- 130   

joint 6만 CCW가 + 방향 이므로 작동 시 확인 바람   

- dualarm_moveit : dualarm_config로 moveit_setup_assistant 이용해서 생성된 패키지   

 * setup assistant에서 manipulator뿐 아니라 eef 그룹 또한 생성 필요 (현재는 manipulator 마지막 링크와 solid 그리퍼로만 생성)   
 * config 폴더 및 launch 폴더의 파일 수정 필요   
   
-> config 폴더 내 파일 수정내역   
dualarm.rviz : bringup_rviz launch 시 초기화면 설정을 위한 rviz config 파일, rviz에서 다른이름 저장으로 생성   
dualarm_controllers.yaml : MoveIt에서 불러오는 controller list를 제공하기 위해 생성   
gazebo_joint_states.yaml : joint_state를 publish하기 위해 생성   
ros_contollers.yaml : gazebo와 ros_control을 위해 MoveIt에서 생성된 파일 수정

-> launch 폴더 내 파일 수정내역

---------------------------------------------------------------------   

move_robot : c++/c/python 이용해서 매니퓰레이터 제어하는 코드 모음   

---------------------------------------------------------------------
## MoveIt 활용 시 execution order

terminal1   
$ roslaunch dualarm_moveit dualarm_bringup_gazebo.launch   
terminal2   
$ roslaunch dualarm_moveit dualarm_bringup_rviz.launch   

