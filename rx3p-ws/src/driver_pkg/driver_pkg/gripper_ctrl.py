#!/usr/bin/env python3

import rclpy
import rclpy.qos
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from .rosboard_drv.rosboard_drv import RosBoardDrv
import time

class GripperCtrl(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self.qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history = QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.__grippercmd_sub = self.create_subscription(String, 'Gripper_cmd', self._on_gripper_cmd, 10)
        self._gripper_states =['open','close'] 
        self._gripper_state = ''
        self._gripper_pos = [50, 180]
        self._is_busy = False
        self._default_delay = 1000
        self._dofbot_arm = RosBoardDrv(RosBoardDrv.CARTYPE_X3_PLUS, com="/dev/ttyUSB1")

    def _ctrl_gripper(self, cmd, delay):
        self._is_busy = True
        indx = self._gripper_states.index(cmd)
        self._dofbot_arm.set_uart_servo_angle(6, self._gripper_pos[indx], self._default_delay)
        time.sleep(1.0)
        self._gripper_state = cmd
        self._is_busy = False

    def _on_gripper_cmd(self, msg:String):
        self.get_logger().info(f"message obtained: {msg}")
        state = msg.data.lower()
        if state in self._gripper_states:
            if (not state != self._gripper_state) and (not self._is_busy):
                self._ctrl_gripper(state, self._default_delay)


def main(args=None):
    rclpy.init(args=args)
    client = GripperCtrl("NOdoGRipper")
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
