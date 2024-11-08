#!/bin/bash

ros2 param get /simple_pure_pursuit_node map_vel_gain

echo "map_vel_gain? :"
read input_value

ros2 param set /simple_pure_pursuit_node map_vel_gain $input_value
ros2 param get /simple_pure_pursuit_node map_vel_gain

