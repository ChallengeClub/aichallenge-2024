#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import math
import rclpy
import pygame
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand

"""
Logitech Driveing Force Proのアサイン
Axis 0: -32767 - 32767 Steering 左Maxが-32767
Axis 1: -32767 - 32767 accel 32767が開放（踏んでない位置）
Axis 2: -32767 - 32767 break 32767が開放（踏んでない位置）
Axis 3: +キーの横方向 -1, 0, 1 左が-1
Axis 4: +キーの縦方向 -1, 0, 1 上が-1
Buttons
0 ×
1 □
2 ○
3 △
4 R
5 L
6 R2
7 L2
8 SELECT
9 START
10 R3
11 L3
12 Shift +
13 Shift -

"""

class Controller_cmd_setter(Node):
    def __init__(self, joystick=None):
        super().__init__('Controller_cmd_setter')

        # simple_pure_pursuit からトピックを受信
        self.create_subscription(AckermannControlCommand, "/control/command/pre_control_cmd", self.onTrigger, 1)

        # raw_vehicle_cmd_converter にトピックを送信
        self.vehicle_inputs_pub_ = self.create_publisher(AckermannControlCommand, "/control/command/control_cmd", 1)
    
        # Gear用パブリッシャーの作成
        self.gear_pub = self.create_publisher(GearCommand, '/control/command/gear_cmd',1)
        
        self.gear_cmd = GearCommand()
        
        # 初期値を時速 20km/h とする
        self.target_vel = 20.0

        # 比例係数 (simple_pure_pursuitと同値である必要あり)
        self.speed_proportional_gain_ = 1.0

        # log を出力するかどうか
        self.log = False

        # ステアリングを手動にするか
        self.manual_steering = True # False

        # バックギア状態
        self.back_gear = False
        
        # 現在のステアリング値
        self.target_steering = 0.0
        self.target_angle = 0.0

        # 加速度
        self.accel = 0.0
        
        # 車両特性
        # https://automotiveaichallenge.github.io/aichallenge-documentation-2024/specifications/simulator.html
        self.max_vel      = 30    # kmph
        self.max_acc      = 3.2   # 駆動時最大加速度 3.2 m/s^2
        self.max_stearang = 80    # Max Steer Angle Input 80
        self.wheel_base = 1.087   # ホイールの長さ[m]
        
        # ジョイスティックの移動で何度動かすか
        self.steering_scale = self.max_stearang 
        self.button_cout = 0

        # 0.1 秒ごとにコントローラーの入力を更新　■これ、多分遅い。
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        if joystick is None:
            print("ジョイスティックが見つかりません。")
            sys.exit()
        self.joystick = joystick

    def cnv_handle(self, value):
        return - value * (math.pi / 180.0 * self.max_stearang) # プラマイ32767の範囲だった。操舵角の単位はrad。車の仕様で操舵角はプラマイ80度。

    def cnv_accel(self, value):
        return (1.0 - value) / 2.0 * self.max_acc # 車両のパラメータから、最大加速度は、3.2m/s^2

    # コントローラーからの入力を更新
    def timer_callback(self):
        self.button_cout += 1 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
	
	# ** 速度制御
	# Xボタン: ブレーキ
        if self.joystick.get_button(0): 
            self.target_vel = 0
	
	# Aボタン: 加速
        if self.joystick.get_button(2): 
            self.target_vel += min(self.max_acc  * self.timer_period * 3.6,self.max_vel)
        else:
            self.target_vel -= max(self.max_acc  * self.timer_period * 3.6,0)
        
        # Yボタン: 後退
        if self.joystick.get_button(1):
            self.target_vel = -1.0
            
        # ギア制御
        """
        gear = self.gear_cmd.command
        if self.joystick.get_button(12):#Shift +
            if self.button_cout > 2: #チャタリング防止
                self.button_cout = 0
                gear += 1
        elif self.joystick.get_button(13):#Shift -
            if self.button_cout > 2: #チャタリング防止
                self.button_cout = 0
                gear -= 1
        self.gear_cmd.command = min(255, max(gear, 0))
        """
        if self.joystick.get_button(12):#Shift +
            if self.button_cout > 2: #チャタリング防止
                self.button_cout = 0
                self.back_gear = False
        elif self.joystick.get_button(13):#Shift -
            if self.button_cout > 2: #チャタリング防止
                self.button_cout = 0
                self.back_gear = True

        """
        # self.gear_cmd.stamp = rospy.Time.now()
        self.gear_cmd.stamp =  self.get_clock().now().to_msg()

        if self.target_vel < 0:
            # ギア：Rにする
            self.gear_cmd.command = GearCommand.REVERSE  # バックギアに設定
        else:
     	    # ギア：D
            self.gear_cmd.command = GearCommand.DRIVE  # ドライブ
        """
       
	# Rボタンで速度を 110 % 
        # if self.joystick.get_button(5): 
        #     self.target_vel = self.target_vel * 1.10
        # Lボタンで速度を 90 % 
        # if self.joystick.get_button(7): 
        #     self.target_vel = self.target_vel * 0.90
	
	# ** ステアリング制御
        # B ボタンでステアリングモードを切り替え
        if self.joystick.get_button(3):
            if self.button_cout > 2: #チャタリング防止
                self.manual_steering = not self.manual_steering
                self.button_cout = 0
        
        # 十字キー操作
        """
        if self.manual_steering:
            # LRキー押されていたらステアリングを大きくする
            if self.joystick.get_button(4) or self.joystick.get_button(5): 
                self.steering_scale = self.max_stearang
            else:
                self.steering_scale = self.max_stearang * 0.3                
            
            hatLR = 0
            if self.joystick.get_hat(0) < 0:
                hatLR = 1 # Right
            elif self.joystick.get_hat(0) > 0:
                hatLR = -1 # Left
            	
            self.target_angle = hatLR * self.steering_scale
            self.target_angle = max(min(self.target_angle, self.max_stearang ), -self.max_stearang)
            self.target_steering = self.target_angle /(self.wheel_base * 360)
        """
         
        # Joystick の値を反映
        if self.joystick.get_hat(0):
            # joystic_x, joystic_y = self.joystick.get_hat(0)
#            joystic_x = -self.joystick.get_axis(2) # ELECOM JC-U4113S 
#            joystic_y = -self.joystick.get_axis(0) # ELECOM JC-U4113S
            gas_pedal = self.joystick.get_axis(1)
            break_pedal = self.joystick.get_axis(2)
            steering_wheel = self.joystick.get_axis(0)
#            self.target_vel  =  self.target_vel + joystic_x * 1.0

            # LRキー押されていたらステアリングを大きくする
            if self.joystick.get_button(4) or self.joystick.get_button(5): 
                self.steering_scale = self.max_stearang
            else:
                self.steering_scale = self.max_stearang * 0.3    
            
            if self.manual_steering:
#                self.target_angle = joystic_y * self.steering_scale
                self.target_angle = max(min(self.target_angle, self.max_stearang ), -self.max_stearang)
#                self.target_steering = self.target_angle /(self.wheel_base * 360)
                self.target_steering = self.cnv_handle(steering_wheel)
                #誤差のためか、ブレーキ、アクセルともしきい値以下は０に設定する。1.0のときに、変換後の値が０になる
                if gas_pedal > 0.999:
                    gas_pedal = 1.0
                if break_pedal > 0.999:
                    break_pedal = 1.0
                if break_pedal < 1.0:
                    self.accel = -self.cnv_accel(break_pedal) * 10
                    self.target_vel = 0
                else:
                    self.accel = self.cnv_accel(gas_pedal)
                    self.target_vel = self.max_vel / 3.6
                self.gear_cmd.command = 2 #DRIVE
                if self.back_gear:
                    self.gear_cmd.command = 20 #REVERSE
#                    self.accel *= -1
                    self.target_vel *= -1

#        if self.joystick.get_axis(3):
#            self.target_vel  =  self.target_vel + self.joystick.get_axis(3) * (-1.0)
    
        self.target_vel = min(max( self.target_vel, -10), 200)
#        self.get_logger().info("accel:{} target_vel:{} steering: {} steering_mode: {}".format(self.accel, self.target_vel,
#                                                                   self.target_angle,
#                                                                   self.manual_steering
#                                                                   ))
        self.get_logger().info("gas_pedal:{} accel:{} gear_cmd:{} steering: {}".format(gas_pedal, self.accel, self.gear_cmd.command, self.target_angle))

    # 制御コマンドを送受信
    def onTrigger(self, msg):
        # km/h ->m/s
#        target_vel = self.target_vel / 3.6
#        target_vel = self.max_vel / 3.6

        # 現在の速度を逆計算 
        current_longitudinal_vel = msg.longitudinal.speed - (msg.longitudinal.acceleration / self.speed_proportional_gain_) 

        if self.log:
            self.get_logger().info("current vel {}".format(current_longitudinal_vel * 3.6))
        
        msg.longitudinal.speed = float(self.target_vel)

        # accel を計算
#        msg.longitudinal.acceleration = float(self.speed_proportional_gain_ * (target_vel - current_longitudinal_vel))
        msg.longitudinal.acceleration = self.accel

        # 自動ステアリングモードではステアリング値を取得
        if not (self.manual_steering):
            self.target_steering = msg.lateral.steering_tire_angle
            self.target_angle = (self.target_steering * 360.0 * self.wheel_base)
        else:
            msg.lateral.steering_tire_angle = self.target_steering

        # トピックを送信
        self.vehicle_inputs_pub_.publish(msg)

        # メッセージをパブリッシュ
        self.gear_pub.publish(self.gear_cmd)
        

def main(args=None):
    print('Hi from Controller_cmd_setter.')

    # rosノードを初期化
    rclpy.init(args=args)

    # Pygameの初期化
    pygame.init()

    # pygame を初期化
    pygame.joystick.init()

    # 利用可能なジョイスティックの数を取得
    joystick_count = pygame.joystick.get_count()

    if joystick_count == 0:
        print("ジョイスティックが見つかりません。")
        sys.exit()

    # 0番のジョイスティックを取得
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print('ジョイスティックの名前:', joystick.get_name())
    print('ボタン数 :', joystick.get_numbuttons())

    node = Controller_cmd_setter(joystick=joystick)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
