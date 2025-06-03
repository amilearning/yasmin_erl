#!/usr/bin/env python3

import os
import yaml
import numpy as np
import rclpy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float32
import time 
from ament_index_python.packages import get_package_share_directory
from yasmin_ros.basic_outcomes import TIMEOUT, CANCEL, SUCCEED
from yasmin import CbState
import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_ros import MonitorState, set_ros_loggers
from yasmin_viewer import YasminViewerPub
from yasmin_ros.yasmin_node import YasminNode
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import FrankaState
# === MonitorState Subclasses ===

class NavToTreeState(MonitorState):
    def __init__(self, goal_x, goal_y) -> None:
        super().__init__(
            Odometry,
            "/dlio/odom_node/odom",
            ["arrived", "moving"],
            self.check_goal_reached,
            msg_queue=10,
            timeout=2.0
        )
        self.goal = np.array([goal_x, goal_y])

    def check_goal_reached(self, blackboard, msg) -> str:
        
        pos = msg.pose.pose.position
        current = np.array([pos.x, pos.y])
        dist = np.linalg.norm(current - self.goal)
        # yasmin.YASMIN_LOG_INFO(f"goal: {self.goal}")
        # yasmin.YASMIN_LOG_INFO(f"Odometry: {msg.pose.pose.position} , dist: {dist}")
        yasmin.YASMIN_LOG_INFO(f"No moving, so skipping")
        # return "arrived" if dist < 0.3 else "moving"
        ''' 
        no moving .. so skipping 
        '''
        return "arrived"


class MoveArmState(MonitorState):
    def __init__(self, target_ee, ee_pub) -> None:
        
        super().__init__(
            FrankaState,
            "/franka/franka_robot_state_broadcaster/robot_state",
            ["done", "moving"],
            self.check_arm_position,
            msg_queue=10,
            timeout=2.0
        )
        self.target = np.array(target_ee)
        self.ee_msg = Float64MultiArray()
        self.ee_msg.data = np.array(target_ee).tolist()
        self.ee_pub = ee_pub

    def check_arm_position(self, blackboard, msg) -> str:
        # yasmin.YASMIN_LOG_INFO(f"JointState: {msg.position}")
        # joint_name_pos = dict(zip(msg.name, msg.position))
        # ordered_positions = [joint_name_pos[name] for name in sorted(joint_name_pos.keys()) if name in joint_name_pos]        
        o_t_ee = np.array(msg.o_t_ee).reshape(4, 4).T
        yasmin.YASMIN_LOG_INFO(f"End-effector transform (o_t_ee):\n{o_t_ee}")
        current_position = o_t_ee[:3, 3]  # Extract translation part
        yasmin.YASMIN_LOG_INFO(f"EE position: {current_position}")
        # current = np.array(ordered_positions[:len(self.target)])
        # yasmin.YASMIN_LOG_INFO(f"JointState (sorted): {current}")
        
        max_step = 0.15
        target_position = self.target[:3]
        direction_vector = target_position - current_position
        distance = np.linalg.norm(direction_vector)
        goal_ee_msg = self.ee_msg
        if distance > max_step:
            corrected_position = current_position + (direction_vector / distance) * max_step
            # print(corrected_position)
            goal_ee_msg.data[0] = corrected_position[0]
            goal_ee_msg.data[1] = corrected_position[1]
            goal_ee_msg.data[2] = corrected_position[2]
            # goal_ee_msg.data[:3] =  np.array(corrected_position).tolist()
            yasmin.YASMIN_LOG_INFO(f"⚠️ Target too far. Corrected target: {corrected_position}")
        else:
            yasmin.YASMIN_LOG_INFO("✅ Target within acceptable range.")

        self.ee_pub.publish(goal_ee_msg)
        
        # return "done" if np.allclose(current, self.target, atol=0.2) else "moving"
        return "done" if np.allclose(current_position, self.target[:3], atol=0.1) else "moving"


class CheckTempState(MonitorState):
    def __init__(self, threshold) -> None:
        super().__init__(
            Float32,
            "/soil_temp",
            ["done", "waiting"],
            self.check_temp,
            msg_queue=10,
            timeout=2.0
        )
        self.threshold = threshold

    def check_temp(self, blackboard, msg) -> str:        
        blackboard["last_temp"] = msg.data
        # yasmin.YASMIN_LOG_INFO(f"Temperature: {msg.data}")
        # return "done" if msg.data % self.threshold == 0 else "waiting"
        yasmin.YASMIN_LOG_INFO(f"Temperature monitoring skipping")
        time.sleep(1.0)
        return "done"

        
class HomeArmState(MoveArmState):
    def check_arm_position(self, blackboard, msg) -> str:
        yasmin.YASMIN_LOG_INFO("⚠️ HOMING: returning to home position...")
        return super().check_arm_position(blackboard, msg)


# === Load Config ===

def load_config():
    pkg_share = get_package_share_directory("husky_state_machine")
    config_path = os.path.join(pkg_share, "config", "mission_config.yaml")
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


# === Main ===

def Homing_func(blackboard: Blackboard) -> str:        
    yasmin.YASMIN_LOG_INFO("🌳 HOMING ~~~~~~~~~~~~~~~~~~")
    return SUCCEED
        
def main():
    yasmin.YASMIN_LOG_INFO("🌳 Starting Full Soil Temp FSM with HOMING fallback")

    rclpy.init()
    set_ros_loggers()       
    # _node = rclpy.create_node("soil_temp_fsm_node")
    _node = YasminNode.get_instance()        
    def joy_callback(msg):
        if any(msg.buttons):
            yasmin.YASMIN_LOG_INFO("🛑 Joy button pressed. Cancelling FSM and restarting...")
            if not blackboard["restart_requested"]:
                blackboard["restart_requested"] = True
                if sm.is_running():
                    sm.cancel_state()
                    
    _node.create_subscription(Joy, "/joy", joy_callback, 10)
    ee_pub = _node.create_publisher(Float64MultiArray, "/cartesian_impedance/pose_desired", 10)    

    
    config = load_config()
    sm = StateMachine(outcomes=["done", CANCEL])
    blackboard = Blackboard()
    blackboard["config"] = config
    blackboard["restart_requested"] = False

    goals = config["tree_goals"]
    # arm_positions = config["arm_positions"]
    # joint_positions = config["joint_positions"]
    ee_goals = config["arm_ee_goal"]
    threshold = config["temperature_threshold"]
    home_joints = config["home_joint_position"]


    sm.add_state("NAV_TO_TREE_1", NavToTreeState(goals[0]["x"], goals[0]["y"]),
                 transitions={"arrived": "ARM_TO_TREE_1", "moving": "NAV_TO_TREE_1",
                              TIMEOUT: "HOMING", CANCEL: CANCEL})

    sm.add_state("ARM_TO_TREE_1", MoveArmState(ee_goals[0],ee_pub),
                 transitions={"done": "CHECK_TEMP_TREE_1", "moving": "ARM_TO_TREE_1",
                              TIMEOUT: "HOMING", CANCEL: CANCEL})

    sm.add_state("CHECK_TEMP_TREE_1", CheckTempState(threshold),
                 transitions={"done": "NAV_TO_TREE_2", "waiting": "CHECK_TEMP_TREE_1",
                              TIMEOUT: "HOMING", CANCEL: CANCEL})

    # === Tree 2 ===
    sm.add_state("NAV_TO_TREE_2", NavToTreeState(goals[1]["x"], goals[1]["y"]),
                 transitions={"arrived": "ARM_TO_TREE_2", "moving": "NAV_TO_TREE_2",
                              TIMEOUT: "HOMING", CANCEL: CANCEL})

    sm.add_state("ARM_TO_TREE_2", MoveArmState(ee_goals[1],ee_pub),
                 transitions={"done": "CHECK_TEMP_TREE_2", "moving": "ARM_TO_TREE_2",
                              TIMEOUT: "HOMING", CANCEL: CANCEL})

    sm.add_state("CHECK_TEMP_TREE_2", CheckTempState(threshold),
                 transitions={"done": "NAV_TO_TREE_3", "waiting": "CHECK_TEMP_TREE_2",
                            #   TIMEOUT: "NAV_TO_TREE_1", CANCEL: CANCEL})
                              TIMEOUT: "HOMING", CANCEL: CANCEL})

    # === Tree 3 ===
    sm.add_state("NAV_TO_TREE_3", NavToTreeState(goals[2]["x"], goals[2]["y"]),
                 transitions={"arrived": "ARM_TO_TREE_3", "moving": "NAV_TO_TREE_3",
                              TIMEOUT: "HOMING", CANCEL: CANCEL})

    sm.add_state("ARM_TO_TREE_3", MoveArmState(ee_goals[2],ee_pub),
                 transitions={"done": "CHECK_TEMP_TREE_3", "moving": "ARM_TO_TREE_3",
                              TIMEOUT: "HOMING", CANCEL: CANCEL})

    sm.add_state("CHECK_TEMP_TREE_3", CheckTempState(threshold),
                 transitions={"done": "NAV_TO_TREE_1", "waiting": "CHECK_TEMP_TREE_3",
                              TIMEOUT: "NAV_TO_TREE_1", CANCEL: CANCEL})

    # === HOMING fallback state ===
    # sm.add_state("HOMING", HomeArmState(home_joints),
    #              transitions={"done": "aborted", "moving": "aborted"})

        
    sm.add_state(
        "HOMING",
        CbState([SUCCEED], Homing_func),
        transitions={
            SUCCEED: "NAV_TO_TREE_1",
        },
    )


    YasminViewerPub("soil_temp_fsm_full", sm)

    # try:
    result = sm()
    yasmin.YASMIN_LOG_INFO(f"✅ FSM completed with result: {result}")
    # except KeyboardInterrupt:
    #     if sm.is_running():
    #         sm.cancel_state()
    # finally:
    _node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
