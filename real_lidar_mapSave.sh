
# 1) 保存地图
gnome-terminal -t "carto gmapping save" -- bash -c "cd /home/cfl/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash;rosservice call /write_state "{filename: '/home/cfl/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mapf.pbstream'}"; exec bash"
sleep 5
# 2）转换 pbstream → 栅格地图 保存到 Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map 文件夹下
gnome-terminal -t "pbstream to pgm" -- bash -c "cd /home/cfl/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash;rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/cfl/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mapf.pbstream -map_filestem=/home/cfl/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mapf -resolution=0.1; exec bash"
sleep 5

# 3）转换 pbstream → 栅格地图 保存 RealSlamRos1/src/tri_steer_gazebo/map 文件夹下
gnome-terminal -t "pbstream to pgm2" -- bash -c "cd /home/cfl/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash;rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/cfl/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mapf.pbstream -map_filestem=/home/cfl/RealSlamRos1/src/tri_steer_gazebo/map/carto_mapf -resolution=0.1; exec bash"

# 4）转换 pbstream → 栅格地图 保存 RealSlamRos1/src/robot_costmap/maps 文件夹下
gnome-terminal -t "pbstream to pgm2" -- bash -c "cd /home/cfl/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash;rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/cfl/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mapf.pbstream -map_filestem=/home/cfl/RealSlamRos1/src/robot_costmap/maps/carto_mapf -resolution=0.1; exec bash"