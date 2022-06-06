

#roslaunch mavros px4.launch & sleep 1;
sudo chmod 777 /dev/ttyUSB0 & sleep 1;

rosrun serial_node 6inc_ctrl_node & sleep 1; 

roslaunch realsense2_camera rs_camera.launch & sleep 10;
rosrun vins vins_node /home/hyy/Safety_planner/src/realflight_modules/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml & sleep 1;


#roslaunch plan_manage /home/hyy/Safety_planner/src/adaptive_planner/plan_manage/launch/experiment.launch & sleep 1;
roslaunch plan_manage experiment.launch & sleep 1;

wait;
------------
