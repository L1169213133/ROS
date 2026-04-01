#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class FormationController:
    def __init__(self):
        # 存储三只乌龟的位姿
        self.pose1 = None  # 主控
        self.pose2 = None  # follower 1
        self.pose3 = None  # follower 2

        # 订阅位姿话题
        rospy.Subscriber('/turtle1/pose', Pose, self.callback1)
        rospy.Subscriber('/turtle2/pose', Pose, self.callback2)
        rospy.Subscriber('/turtle3/pose', Pose, self.callback3)

        # 发布速度指令
        self.pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        self.pub3 = rospy.Publisher('/turtle3/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(20)  # 20 Hz，更平滑

    def callback1(self, msg):
        self.pose1 = msg

    def callback2(self, msg):
        self.pose2 = msg

    def callback3(self, msg):
        self.pose3 = msg

    def get_target_position(self, leader_pose, dx_local, dy_local):
        """
        根据主乌龟位姿和局部偏移，计算世界坐标系下的目标位置
        :param leader_pose: 主乌龟位姿 (Pose)
        :param dx_local: 局部x偏移（前进方向）
        :param dy_local: 局部y偏移（左为正）
        :return: (x_target, y_target)
        """
        x, y, theta = leader_pose.x, leader_pose.y, leader_pose.theta
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        # 旋转局部偏移到世界坐标
        x_world = dx_local * cos_t - dy_local * sin_t
        y_world = dx_local * sin_t + dy_local * cos_t
        return x + x_world, y + y_world

    def run(self):
        # 编队参数（单位：米）
        D = 1.5  # 编队边长
        # turtle2: 左前方 60°
        dx2, dy2 = D * math.cos(math.pi / 3), D * math.sin(math.pi / 3)   # (0.75, 1.3)
        # turtle3: 右前方 60°
        dx3, dy3 = D * math.cos(math.pi / 3), -D * math.sin(math.pi / 3)  # (0.75, -1.3)

        Kp = 2.0  # 比例增益

        while not rospy.is_shutdown():
            if self.pose1 is None or self.pose2 is None or self.pose3 is None:
                self.rate.sleep()
                continue

            # 计算目标位置
            x2_target, y2_target = self.get_target_position(self.pose1, dx2, dy2)
            x3_target, y3_target = self.get_target_position(self.pose1, dx3, dy3)

            # 当前位置
            x2_curr, y2_curr = self.pose2.x, self.pose2.y
            x3_curr, y3_curr = self.pose3.x, self.pose3.y

            # 速度指令（P控制）
            twist2 = Twist()
            twist2.linear.x = Kp * (x2_target - x2_curr)
            twist2.linear.y = Kp * (y2_target - y2_curr)

            twist3 = Twist()
            twist3.linear.x = Kp * (x3_target - x3_curr)
            twist3.linear.y = Kp * (y3_target - y3_curr)

            # 发布
            self.pub2.publish(twist2)
            self.pub3.publish(twist3)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('formation_controller', anonymous=True)
    controller = FormationController()
    rospy.loginfo("Formation Controller Started: Triangle Formation")
    controller.run()
