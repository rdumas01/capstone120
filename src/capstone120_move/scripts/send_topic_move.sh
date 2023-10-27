#!/bin/bash


echo -e "${Info} press enter to start" 
echo && stty erase '^H' && read -p "click" 

rostopic pub /start_topic std_msgs/String "data: 'start'" -1


#rostopic pub /my_point_topic1 geometry_msgs/Point '{x: 0.035, y: -0.02, z: 0.0}' -1 &
#rostopic pub /my_point_topic2 geometry_msgs/Point '{x: 0.035, y: 0.02, z: 0.0}' -1 &

