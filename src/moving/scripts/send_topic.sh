#!/bin/bash

echo -e "===== CAPSTONE 120 ====="
echo -e "" 
echo -e "${Info}Wait until the robot arm beeps" 
echo -e "${Info}When ready, press Enter to start，otherwise press 'Ctrl + c' to stop" 

echo && stty erase '^H' && read -p "Press Enter:" 

rostopic pub /start_topic std_msgs/String "data: 'start'" -1
