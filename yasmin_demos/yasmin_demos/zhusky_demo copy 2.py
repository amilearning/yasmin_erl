#!/usr/bin/env python3

import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from ament_index_python.packages import get_package_share_directory
from yasmin_ros.basic_outcomes import TIMEOUT, CANCEL
import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_ros import MonitorState, set_ros_loggers
from yasmin_viewer import YasminViewerPub


# === MonitorState Subclasses ===

class NavToTreeState(MonitorState):
    def __init__(self, goal_x, goal_y) -> None:
        super().__init__(
            Odometry,
            "/odom",
            ["arrived", "moving"],
            self.check_goal_reached,
            msg_queue=10,
            timeout=15.0
        )
        self.goal = np.array([goal_x, goal_y])

    def check_goal_reached(self, blackboard, msg) -> str:
        yasmin.YASMIN_LOG_INFO(f"Odometry: {msg.pose.pose.position}")
        pos = msg.pose.pose.position
        current = np.array([pos.x, pos.y])
        dist = np.linalg.norm(current - self.goal)
        return "arrived" if dist < 0.3 else "moving"


class MoveArmState(MonitorState):
    def __init__(self, target_joints) -> None:
        super().__init__(
            JointState,
            "/joint_states",
            ["done", "moving"],
            self.check_arm_position,
            msg_queue=10,
            timeout=10.0
        )
        self.target = np.array(target_joints)

    def check_arm_position(self, blackboard, msg) -> str:
        yasmin.YASMIN_LOG_INFO(f"JointState: {msg.position}")
        current = np.array(msg.position[:len(self.target)])
        return "done" if np.allclose(current, self.target, atol=0.05) else "moving"


class CheckTempState(MonitorState):
    def __init__(self, threshold) -> None:
        super().__init__(
            Float32,
            "/soil_temp",
            ["done", "waiting"],
            self.check_temp,
            msg_queue=10,
            timeout=10.0
        )
        self.threshold = threshold

    def check_temp(self, blackboard, msg) -> str:
        yasmin.YASMIN_LOG_INFO(f"Temperature: {msg.data}")
        blackboard["last_temp"] = msg.data
        return "done" if msg.data >= self.threshold else "waiting"


class HomeArmState(MoveArmState):
    def check_arm_position(self, blackboard, msg) -> str:
        yasmin.YASMIN_LOG_INFO("⚠️ HOMING: returning to home position...")
        return super().check_arm_position(blackboard, msg)


# === Load Config ===

def load_config():
    pkg_share = get_package_share_directory("yasmin_demos")
    config_path = os.path.join(pkg_share, "config", "mission_config.yaml")
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


# === Main ===



class FSM(Node):

    def __init__(self):
        super().__init__('fsm')        
        yasmin.YASMIN_LOG_INFO("🌳 Starting Full Soil Temp FSM with HOMING fallback")            
        self.config = load_config()
        self.sm = StateMachine(outcomes=["done", "aborted"])
        self.blackboard = Blackboard()
        self.blackboard["config"] = self.config

        self.goals = self.config["tree_goals"]
        self.arm_positions = self.config["arm_positions"]
        self.threshold = self.config["temperature_threshold"]
        self.home_joints = self.config["home_joint_position"]
        self.init_fsm()
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def init_fsm(self):
        self.sm.add_state("NAV_TO_TREE_1", NavToTreeState(self.goals[0]["x"], self.goals[0]["y"]),
                 transitions={"arrived": "ARM_TO_TREE_1", "moving": "NAV_TO_TREE_1",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})

        self.sm.add_state("ARM_TO_TREE_1", MoveArmState(self.arm_positions[0]),
                 transitions={"done": "CHECK_TEMP_TREE_1", "moving": "ARM_TO_TREE_1",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})
        
        self.sm.add_state("CHECK_TEMP_TREE_1", CheckTempState(self.threshold),
                    transitions={"done": "NAV_TO_TREE_2", "waiting": "CHECK_TEMP_TREE_1",
                                TIMEOUT: "HOMING", CANCEL: "HOMING"})

        # === Tree 2 ===
        self.sm.add_state("NAV_TO_TREE_2", NavToTreeState(self.goals[1]["x"], self.goals[1]["y"]),
                    transitions={"arrived": "ARM_TO_TREE_2", "moving": "NAV_TO_TREE_2",
                                TIMEOUT: "HOMING", CANCEL: "HOMING"})

        self.sm.add_state("ARM_TO_TREE_2", MoveArmState(self.arm_positions[1]),
                    transitions={"done": "CHECK_TEMP_TREE_2", "moving": "ARM_TO_TREE_2",
                                TIMEOUT: "HOMING", CANCEL: "HOMING"})

        self.sm.add_state("CHECK_TEMP_TREE_2", CheckTempState(self.threshold),
                    transitions={"done": "NAV_TO_TREE_3", "waiting": "CHECK_TEMP_TREE_2",
                                TIMEOUT: "HOMING", CANCEL: "HOMING"})

        # === Tree 3 ===
        self.sm.add_state("NAV_TO_TREE_3", NavToTreeState(self.goals[2]["x"], self.goals[2]["y"]),
                    transitions={"arrived": "ARM_TO_TREE_3", "moving": "NAV_TO_TREE_3",
                                TIMEOUT: "HOMING", CANCEL: "HOMING"})

        self.sm.add_state("ARM_TO_TREE_3", MoveArmState(self.arm_positions[2]),
                    transitions={"done": "CHECK_TEMP_TREE_3", "moving": "ARM_TO_TREE_3",
                                TIMEOUT: "HOMING", CANCEL: "HOMING"})

        self.sm.add_state("CHECK_TEMP_TREE_3", CheckTempState(self.threshold),
                    transitions={"done": "done", "waiting": "CHECK_TEMP_TREE_3",
                                TIMEOUT: "HOMING", CANCEL: "HOMING"})

        # === HOMING fallback state ===
        self.sm.add_state("HOMING", HomeArmState(self.home_joints),
                    transitions={"done": "aborted", "moving": "HOMING",
                                TIMEOUT: "HOMING", CANCEL: "HOMING"})
        
        YasminViewerPub("soil_temp_fsm_full", self.sm)
        
    def timer_callback(self):        
        self.get_logger().info('Publishing:')
        self.i += 1
        if self.i > 10:
            self.get_logger().info('i > 10 cancel state')
            if self.sm.is_running():
                self.sm.cancel_state()



def main(args=None):
    rclpy.init(args=args)
    
    minimal_publisher = FSM()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
        
def main():
    
    rclpy.init()
    set_ros_loggers()

    config = load_config()
    sm = StateMachine(outcomes=["done", "aborted"])
    blackboard = Blackboard()
    blackboard["config"] = config
    goals = config["tree_goals"]
    arm_positions = config["arm_positions"]
    threshold = config["temperature_threshold"]
    home_joints = config["home_joint_position"]

    # === Tree 1 ===
    sm.add_state("NAV_TO_TREE_1", NavToTreeState(goals[0]["x"], goals[0]["y"]),
                 transitions={"arrived": "ARM_TO_TREE_1", "moving": "NAV_TO_TREE_1",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})

    sm.add_state("ARM_TO_TREE_1", MoveArmState(arm_positions[0]),
                 transitions={"done": "CHECK_TEMP_TREE_1", "moving": "ARM_TO_TREE_1",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})

    sm.add_state("CHECK_TEMP_TREE_1", CheckTempState(threshold),
                 transitions={"done": "NAV_TO_TREE_2", "waiting": "CHECK_TEMP_TREE_1",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})

    # === Tree 2 ===
    sm.add_state("NAV_TO_TREE_2", NavToTreeState(goals[1]["x"], goals[1]["y"]),
                 transitions={"arrived": "ARM_TO_TREE_2", "moving": "NAV_TO_TREE_2",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})

    sm.add_state("ARM_TO_TREE_2", MoveArmState(arm_positions[1]),
                 transitions={"done": "CHECK_TEMP_TREE_2", "moving": "ARM_TO_TREE_2",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})

    sm.add_state("CHECK_TEMP_TREE_2", CheckTempState(threshold),
                 transitions={"done": "NAV_TO_TREE_3", "waiting": "CHECK_TEMP_TREE_2",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})

    # === Tree 3 ===
    sm.add_state("NAV_TO_TREE_3", NavToTreeState(goals[2]["x"], goals[2]["y"]),
                 transitions={"arrived": "ARM_TO_TREE_3", "moving": "NAV_TO_TREE_3",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})

    sm.add_state("ARM_TO_TREE_3", MoveArmState(arm_positions[2]),
                 transitions={"done": "CHECK_TEMP_TREE_3", "moving": "ARM_TO_TREE_3",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})

    sm.add_state("CHECK_TEMP_TREE_3", CheckTempState(threshold),
                 transitions={"done": "done", "waiting": "CHECK_TEMP_TREE_3",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})

    # === HOMING fallback state ===
    sm.add_state("HOMING", HomeArmState(home_joints),
                 transitions={"done": "aborted", "moving": "HOMING",
                              TIMEOUT: "HOMING", CANCEL: "HOMING"})

    # Visualize in Foxglove/Yasmin viewer
    YasminViewerPub("soil_temp_fsm_full", sm)

    try:
        result = sm()
        yasmin.YASMIN_LOG_INFO(f"✅ FSM completed with result: {result}")
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
