#!/bin/bash

if [ $# -eq 0 ]; then
  echo "No rosbag name provided. Using default."
  rosbag_name="olympus_mpc_rosbag"
elif [ $# -eq 1 ]; then 
    rosbag_name=$1 
fi
mkdir $rosbag_name
cd $rosbag_name

rosbag record /leg2_node/motor_statuses /leg2_node/command_position /qualisys/olympus/odom /leg_mpc_status /controller_current_phase -O $rosbag_name

rostopic echo -b $rosbag_name.bag /leg2_node/motor_statuses   -p > $rosbag_name\_motor_statuses.txt
rostopic echo -b $rosbag_name.bag /qualisys/olympus/odom      -p > $rosbag_name\_odom.txt
rostopic echo -b $rosbag_name.bag /leg_mpc_status             -p > $rosbag_name\_mpc_status.txt
rostopic echo -b $rosbag_name.bag /controller_current_phase   -p > $rosbag_name\_controller_phase.txt
rostopic echo -b $rosbag_name.bag /leg2_node/command_position -p > $rosbag_name\_reference.txt

echo "Exported the topics in txts"
