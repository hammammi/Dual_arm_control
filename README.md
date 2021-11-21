# mservo_manipulator_control

## massage   
> **mservo_msg** : mservo lab에서 사용하는 ros msgs   

## EtherCAT   
> xenomai, soem 필요   
> 관리자 권한 터미널  (sudo -s) 에서 실행   
> 사용 시 ecat_ifname 항상 확인..   
> dualarm 사용 시 junction 사용, 배터리 항상 확인 필요
   
> **ethercat_test**
> - ecat_profile_pos_1.cpp,ecat_profile_pos_2.cpp,mani_ecat_homing.cpp : junction 없이 사용
> - dualarm_ecat_ctrl.cpp
     
> **참고**    
> * ethtool parameter persist 변경 방법       
> ```    
> $ cd /etc/NetworkManager/dispatcher.d    
> $ sudo nano 20-ethtool    
> ```    
> ```    
> #!/bin/bash    
> /sbin/ethtool -C eno1 rx-usecs 0    
> ```    
> ```      
> $ sudo chmod +x 20-ethtool    
> $ reboot    
> ```    
> * lan port를 찾지 못하는 경우
> ```
> $ sudo nano /etc/netplan/*.ymal    
> ```    
> 내에 네트워크를 추가한다    
> ```    
> network:    
>   ethernets:    
>      eno1:    
>         dhcp4:yes    
>         dhcp6:yes (안해도됨)    
> ```    
> ```    
> $ sudo netplan apply        
> ```    
> ifconfig 명령을 통해 적용된 내용확인   
   

## Gazebo & MoveIt   
   
> **dualarm_config** : solidworks로 만든 urdf 파일과 mesh 파일 (STL)   
>
> * 아래 moveit 패키지 만들 시 urdf가 수정됨
> * urdf 생성 시 dualarm으로 바로 생성되지 않아 ver1/ver2 각각 urdf 생성 후 파일 수정하여 base link에 연결   
> * dummy link, joint를 생성하여야 moveit에서 vurtual joint 활용하여 world에 attatch 가능   
> * griper 끝을 eef position으로 planning 하기 위해서는 urdf 생성 시 eef joint coordinate를 원하는 위치에 생성해야 함   
>
>     
> ver2. limit   
> joint 4 : +- 120   
> joint 5 : +- 130   
>
> joint 6만 CCW가 + 방향 이므로 작동 시 확인 바람   
   
   
> **dualarm_moveit** : dualarm_config로 moveit_setup_assistant 이용해서 생성된 패키지   
>   
> * setup assistant에서 manipulator뿐 아니라 eef 그룹 또한 생성 필요 (현재는 manipulator 마지막 링크와 solid 그리퍼로만 생성)   
> * config 폴더 및 launch 폴더의 파일 수정 필요   
>   
> -> config 폴더 내 파일 수정내역   
> dualarm.rviz : bringup_rviz launch 시 초기화면 설정을 위한 rviz config 파일, rviz에서 다른이름 저장으로 생성   
> dualarm_controllers.yaml : MoveIt에서 불러오는 controller list를 제공하기 위해 생성   
> (controller의 name과 action_ns가 gazebo가 받는 topic과 일치할 수 있도록 작성)
> gazebo_joint_states.yaml : joint_state를 publish하기 위해 생성   
> ros_contollers.yaml : gazebo와 ros_control을 위해 MoveIt에서 생성된 파일 수정   
> (MoveIt controller != ros_control controller 이므로 yaml 파일을 두개로 나누어 사용(dualarm/ros controllers.yaml))   
>   
> -> launch 폴더 내 파일 수정내역   
> dualarm_bringup_gazebo.launch : gazebo 활용 위해 생성   
> dualarm_bringup_rviz.launch : rviz 활용 위해 생성   
> dualarm_moveit_controller_manager.launch : dualarm_controllers.yaml 참조 위해 수정  
> (joint_controller_spawner의 ns가 urdf의 gazebo library 캡션 내의 ns와 같은지 확인 필요. Controller Spawner couldn't find the expected controller_manager ROS interface error 해결)   
> gazebo_states.launch : gazebo_joint_states.yaml 참조 및 robot state publisher 위해 생성   
> moveit_rviz.launch : defalt rviz config 파일 경로 수정   
> planning_context.launch : srdf파일 추가 (rviz planning scene error 해결)   
> ros_controllers.launch : 내용 잘 맞는지 확인
>   
> -> execution order
>   
> terminal1   
> ```
> $ roslaunch dualarm_moveit dualarm_bringup_gazebo.launch   
> ```
> terminal2   
> ```
> $ roslaunch dualarm_moveit dualarm_bringup_rviz.launch   
> ```
 
   
> **move_robot** : c++/c/python 이용해서 매니퓰레이터 제어하는 코드 모음   

> **참고**   
>   https://github.com/eYSIP-2017/eYSIP-2017_Robotic_Arm/wiki/Interfacing-MoveIt%21-with-Gazebo    
>   https://answers.ros.org/question/214712/gazebo-controller-spawner-warning/    
>   http://gazebosim.org/tutorials/?tut=ros_control

