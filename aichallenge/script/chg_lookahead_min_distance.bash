#!/bin/bash

ros2 param get /simple_pure_pursuit_node lookahead_min_distance

echo "lookahead_min_distance? :"
read input_value

ros2 param set /simple_pure_pursuit_node lookahead_min_distance $input_value
ros2 param get /simple_pure_pursuit_node lookahead_min_distance

