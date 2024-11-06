#!/bin/bash

ros2 param get /simple_pure_pursuit_node external_target_vel

echo "external_target_vel? :"
read input_value

ros2 param set /simple_pure_pursuit_node external_target_vel $input_value
ros2 param get /simple_pure_pursuit_node external_target_vel

