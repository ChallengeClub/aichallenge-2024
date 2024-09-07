#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand


class Command_setter(Node):
    def __init__(self):
        super().__init__('Command_setter')

        # simple_pure_pursuit からトピックを受信
        self.create_subscription(AckermannControlCommand, "/control/command/pre_control_cmd", self.onTrigger, 1)

        # raw_vehicle_cmd_converter にトピックを送信
        self.vehicle_inputs_pub_ = self.create_publisher(AckermannControlCommand, "/control/command/control_cmd", 1)

        # 時速 20km/h で固定
        target_vel = 20.0

        # km/h ->m/s
        self.target_vel =  target_vel / 3.6

        # 比例係数 (simple_pure_pursuitと同値である必要あり)
        self.speed_proportional_gain_ = 1.0

        # log を出力するかどうか
        self.log = False

    def onTrigger(self, msg):
        
        # 現在の速度を逆計算 
        current_longitudinal_vel = msg.longitudinal.speed - (msg.longitudinal.acceleration / self.speed_proportional_gain_) 
                                                             
        if self.log:
            self.get_logger().info("current vel {}".format(current_longitudinal_vel * 3.6))
        
        msg.longitudinal.speed = self.target_vel

        # accel を計算
        msg.longitudinal.acceleration = self.speed_proportional_gain_ * (self.target_vel - current_longitudinal_vel) 

        # トピックを送信
        self.vehicle_inputs_pub_.publish(msg)

def main(args=None):
    print('Hi from Command_setter.')
    rclpy.init(args=args)
    node = Command_setter()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()