#!/bin/bash

ros2 param get /simple_pure_pursuit_node steering_tire_angle_gain

echo "steering_tire_angle_gain? :"
read input_value

ros2 param set /simple_pure_pursuit_node steering_tire_angle_gain $input_value
ros2 param get /simple_pure_pursuit_node steering_tire_angle_gain

