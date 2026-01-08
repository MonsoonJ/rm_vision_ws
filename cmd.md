
source install/setup.bash
colcon build --packages-select rm_ros_interfaces
#分别启动控制
ros2 launch rm_driver rm_eco63_driver.launch.py
ros2 launch rm_description rm_eco63_display.launch.py
ros2 launch rm_control rm_eco63_control.launch.py
ros2 launch rm_eco63_config real_moveit_demo.launch.py
#夹爪控制
python3 gripper_arm_sleep.py  #取水固定动作演示
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 1, block: true, timeout: 3}"  #单独控制夹爪
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 999, block: true, timeout: 3}"

#弃用部分
python3 src/vi_grab/vi_grab/object_tf_node.py  
python3 src/vi_grab/vi_grab/grab_node.py      
ros2 run vi_grab yolo_rs_publisher            
ros2 topic echo /object_pose                   
cd src/vi_grab/vi_grab
    ros2 run vi_grab yolo_rs_publisher
    python3 object_tf_node.py
cd .. && cd .. && cd ..


#流程命令
conda activate recogn && source install/setup.bash	#环境
colcon build && source install/setup.bash	#编译

ros2 launch rm_bringup_lite.launch.py	#机械臂驱动

ros2 run vi_grab yolo_rs_publisher	#yolo发布

python3 src/vi_grab/vi_grab/camera_to_base.py	#坐标发布

python3 src/vi_grab/vi_grab/grasp_executor.py	#执行

python3 gripper_arm_sleep.py 

一键启动
python3 src/vi_grab/vi_grab/start.launch.py

