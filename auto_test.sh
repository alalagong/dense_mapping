#!/bin/sh
#MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult 
#V1_01_easy V1_02_medium V1_03_difficult 
#V2_01_easy V2_02_medium V2_03_difficult

rosrun lt_slam lt_kitti 00 
sleep 10s
# rosnode kill /lt_slam

#rosrun lt_slam lt_kitti 01 &
#sleep 10s
#rosnode kill /lt_slam

#rosrun lt_slam lt_kitti 02 
#sleep 10s
#rosnode kill /lt_slam

rosrun lt_slam lt_kitti 03 
sleep 10s
#rosnode kill /lt_slam

rosrun lt_slam lt_kitti 04 
sleep 10s
#rosnode kill /lt_slam

rosrun lt_slam lt_kitti 05 
sleep 10s
#rosnode kill /lt_slam

rosrun lt_slam lt_kitti 06 
sleep 10s
#rosnode kill /lt_slam

rosrun lt_slam lt_kitti 07 
sleep 10s
#rosnode kill /lt_slam

rosrun lt_slam lt_kitti 08 
sleep 10s
#rosnode kill /lt_slam

#rosrun lt_slam lt_kitti 09 
#sleep 10s
#rosnode kill /lt_slam

rosrun lt_slam lt_kitti 10 
sleep 10s
#rosnode kill /lt_slam

