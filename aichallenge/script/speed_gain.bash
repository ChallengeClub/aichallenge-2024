#!/bin/bash

ros2 param get /simple_pure_pursuit_node speed_proportional_gain

echo "speed_proportional_gain? :"
read input_value

ros2 param set /simple_pure_pursuit_node speed_proportional_gain $input_value
ros2 param get /simple_pure_pursuit_node speed_proportional_gain

