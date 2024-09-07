#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rclpy
from ament_index_python.packages import get_package_share_directory
import os
from rclpy.node import Node
import numpy as np
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_control_msgs.msg import AckermannControlCommand
from onnxruntime import InferenceSession

class Control_cmd_predictor(Node):
    def __init__(self):
        super().__init__('Control_cmd_predictor')
        self.package_dir = get_package_share_directory("control_command_setter")

        # simple_pure_pursuit からトピックを受信
        self.create_subscription(AckermannControlCommand, 
                                 "/control/command/pre_control_cmd", 
                                 self.onTrigger, 
                                 1)
        # Trajectory のsubscriber
        self.create_subscription(Trajectory, 
                                 "/planning/scenario_planning/trajectory", 
                                 self.onTrajectory, 
                                 1)
        
        # raw_vehicle_cmd_converter にトピックを送信
        self.vehicle_inputs_pub_ = self.create_publisher(AckermannControlCommand, "/control/command/control_cmd", 1)

        # 比例係数 (simple_pure_pursuitと同値である必要あり)
        self.speed_proportional_gain_ = 1.0

        # Trajectory 格納
        self.trajectory = None

        # log を出力するかどうか
        self.log = False

        self.get_logger().info("package path {}".format(self.package_dir))

        # モデルファイルのロード
        model_file = self.package_dir + "/velocity_transformer_20230106.onnx"
        if not os.path.exists(model_file):
            self.get_logger().info("Error: File {} is not existed!!".format(model_file))
            exit()
        self.input_node = "input.1" # 入力レイヤ
        self.output_node = "255" # 出力レイヤ
        self.session = InferenceSession(model_file)

        self.target_vel = 50.0

        # 前処理パラメータ
        self.traj_token_mode = "degree"
        self.trajectory_points_interval = 10
        self.minimum_trajectory_points_num = 150
        self.traj_token_dim = 12

    # Trajectory トピックを受信
    def onTrajectory(self, msg: Trajectory):
        trajectory = self._Trajectory2Numpy(msg)
        self.trajectory = parse_traj(trajectory,
                                     mode=self.traj_token_mode,
                                     interval=self.trajectory_points_interval,
                                     minimum_num=self.minimum_trajectory_points_num,
                                     point_num=self.traj_token_dim,
                                     )

    # 制御コマンドのトピックを受信。 速度・アクセル値を変更し、トピックを再送信
    def onTrigger(self, msg: AckermannControlCommand):
        
        # 現在の速度を逆計算 
        current_longitudinal_vel = msg.longitudinal.speed - (msg.longitudinal.acceleration / self.speed_proportional_gain_) 
                                                             
        #if self.log:
       #
        
        if self.trajectory is not None:
            input_data = {self.input_node:[self.trajectory]}
            result = self.session.run(output_names=[self.output_node], input_feed=input_data)
            target_vel = float(result[0])
            self.target_vel = min(max(target_vel, 30.0), 200)
            self.get_logger().info("current vel {}".format(current_longitudinal_vel * 3.6))
            self.get_logger().info("predicted target vel {}".format(self.target_vel))
        else:
            self.target_vel = 50.0

        msg.longitudinal.speed = self.target_vel / 3.6

        # accel を計算
        msg.longitudinal.acceleration = float(self.speed_proportional_gain_ * (msg.longitudinal.speed - current_longitudinal_vel))

        # トピックを送信
        self.vehicle_inputs_pub_.publish(msg)
    
    # Trajectory データをnumpyに変換
    def _Trajectory2Numpy(self, trajectory):
        points_pose_list = []
        for p in trajectory.points:
            points_pose_list.append([p.pose.position.x, p.pose.position.y])
        return np.array(points_pose_list)

def parse_traj(traj: np.array,
            mode: str = "degree",
            interval: int = 10,
            minimum_num: int = 150,
            point_num: int= 12,
            dtype="int32"
            ):

    # 出力するフォーマットを指定
    if dtype =="int16":
        dtype=np.int16
    else:
        dtype=np.int32
    # trajectory points が不足している場合は None を返す
    if len(traj) < minimum_num:
        return None

    degrees = []
    for k in range(point_num):
        # 2つのtrajectory points から角度を出力する
        index = interval * k
        vec = traj[index][0:2] - traj[index - interval][0:2]
        base_vec = np.array([1, 0])
        degree = angle_between_vectors(vec, base_vec)
        degrees.append(degree)
        #points.append(traj[index - interval][0:2]) # For debug
    #print(degrees) # For debug
    #plot_point(points) # For debug
    return np.array(degrees, dtype=dtype)

def angle_between_vectors(v1, v2):
    # ベクトルの大きさを計算
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)

    # ベクトルがゼロベクトルであるかどうかを確認
    if magnitude_v1 == 0 or magnitude_v2 == 0:
        print("Warning: It's zero vector.")
        return 0

    # ドット積を計算
    dot_product = np.dot(v1, v2)

    # arccosを使用して角度を計算（ラジアンから度に変換）
    angle_rad = np.arccos(dot_product / (magnitude_v1 * magnitude_v2))
    angle_deg = np.degrees(angle_rad)
    
    if v1[1] < 0:
        angle_deg = 360 - angle_deg

    return angle_deg

def main(args=None):
    print('Hi from Control_cmd_predictor.')
    print(os.getcwd())
    rclpy.init(args=args)
    node = Control_cmd_predictor()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()