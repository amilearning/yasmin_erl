#!/usr/bin/env python3

import os
import yaml
import numpy as np
import rclpy
from std_msgs.msg import Float32, Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float32
from std_srvs.srv import SetBool

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
from franka_msgs.srv import SetFullCollisionBehavior
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
        # yasmin.YASMIN_LOG_INFO(f"No moving, so skipping")
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
        self.arm_cmd_timeout = 5.0

    def check_arm_position(self, blackboard, msg) -> str:
        # yasmin.YASMIN_LOG_INFO(f"JointState: {msg.position}")
        # joint_name_pos = dict(zip(msg.name, msg.position))
        # ordered_positions = [joint_name_pos[name] for name in sorted(joint_name_pos.keys()) if name in joint_name_pos]        
        o_t_ee = np.array(msg.o_t_ee).reshape(4, 4).T
        # yasmin.YASMIN_LOG_INFO(f"End-effector transform (o_t_ee):\n{o_t_ee}")
        current_position = o_t_ee[:3, 3]  # Extract translation part
        # yasmin.YASMIN_LOG_INFO(f"EE position: {current_position}")
        # current = np.array(ordered_positions[:len(self.target)])
        # yasmin.YASMIN_LOG_INFO(f"JointState (sorted): {current}")
        
        
        max_step = 0.2
        
        target_position = self.target[:3]
        direction_vector = target_position - current_position
        distance = np.linalg.norm(direction_vector)
        ee_msg = Float64MultiArray()
        ee_msg.data = self.ee_msg.data
        goal_ee_msg = ee_msg
        if distance > 0.2:            
            corrected_position = current_position + (direction_vector / distance) * max_step            
            goal_ee_msg.data[0] = corrected_position[0]
            goal_ee_msg.data[1] = corrected_position[1]
            goal_ee_msg.data[2] = corrected_position[2]     
            # yasmin.YASMIN_LOG_INFO(f"⚠️ Target too far. max_step: {max_step}")
            # yasmin.YASMIN_LOG_INFO(f"⚠️ Target too far. Corrected target: {corrected_position}")
            # yasmin.YASMIN_LOG_INFO(f"⚠️ Target too far. distance: {distance}")            
        # else:        
        #      yasmin.YASMIN_LOG_INFO("✅ Target within acceptable range.")
        # elif distance > 0.2 and distance <= 0.5:
        #     max_step = 0.10
        
        #     corrected_position = current_position + (direction_vector / distance) * max_step            
        #     goal_ee_msg.data[0] = corrected_position[0]
        #     goal_ee_msg.data[1] = corrected_position[1]
        #     goal_ee_msg.data[2] = corrected_position[2]     

        else:
            goal_ee_msg.data[0] = self.target[0]
            goal_ee_msg.data[1] = self.target[1]
            goal_ee_msg.data[2] = self.target[2]
        
        self.ee_pub.publish(goal_ee_msg)
        
        distance_vector = self.target[:3] - current_position
        distance_eval = np.linalg.norm(distance_vector)
        # yasmin.YASMIN_LOG_INFO(f"⚠️ Target too far. max_step: {max_step}")                
        # yasmin.YASMIN_LOG_INFO(f"⚠️ Target too far. distance: {distance}")
        # yasmin.YASMIN_LOG_INFO(f"⚠️ Target too far. distance_eval: {distance_eval}")
        # yasmin.YASMIN_LOG_INFO(f"⚠️ Target 0: {goal_ee_msg.data[:3]}")
        
        if distance_eval < 0.15:
            return "done"
        else:
            return "moving"
            
        
        # return "done" if np.allclose(current, self.target, atol=0.2) else "moving"
        # return "done" if np.allclose(current_position, self.target[:3], atol=0.1) else "moving"


class CheckTempState(MonitorState):
    def __init__(self, threshold) -> None:
        super().__init__(
            Odometry,
            "/dlio/odom_node/odom",
            ["done", "waiting"],
            self.check_temp,
            msg_queue=10,
            timeout=2.0
        )
        self.threshold = threshold
        self.add_service_timeout = 8.0
        self.client = YasminNode.get_instance().create_client(SetBool, '/read_soil_sensors')

    def check_temp(self, blackboard, msg) -> str:
        if not self.client.wait_for_service(timeout_sec=2.0):
            yasmin.YASMIN_LOG_WARN("Soil sensor service not available. Waiting...")
            return "waiting"

        request = SetBool.Request()
        request.data = True  # trigger reading

        # try:
        #     future = self.client.call(request)  # ✅ synchronous call
        # except Exception as e:
        #     yasmin.YASMIN_LOG_WARN(f"⚠️ Service call failed: {str(e)}")
        #     return "waiting"

        future = self.client.call_async(request)

        # Wait for result (you could also set a timer and break early)
        start_time = time.time()
        while rclpy.ok() and not future.done():
            time_diff = time.time() - start_time
            if time_diff > self.add_service_timeout:
                yasmin.YASMIN_LOG_WARN("⏱️ Timeout waiting for soil sensor response. Go back to home.")
                return "done"
            else:
                yasmin.YASMIN_LOG_WARN(f"⏱️ Waiting... elapsed time: {time_diff:.2f} seconds")
            time.sleep(0.5)

        if future.result() is not None and future.result().success:
            yasmin.YASMIN_LOG_INFO(f"✅ Soil sensor success: {future.result().message}")
            return "done"
        else:
            yasmin.YASMIN_LOG_WARN(f"⚠️ Soil sensor failed: {future.result().message if future.result() else 'no response'}")
            return "done"


# class CheckTempState(MonitorState):
#     def __init__(self, threshold) -> None:
#         super().__init__(
#             Float32,
#             "/soil_temp",
#             ["done", "waiting"],
#             self.check_temp,
#             msg_queue=10,
#             timeout=2.0
#         )
#         self.threshold = threshold

#     def check_temp(self, blackboard, msg) -> str:        
#         blackboard["last_temp"] = msg.data
#         # yasmin.YASMIN_LOG_INFO(f"Temperature: {msg.data}")
#         # return "done" if msg.data % self.threshold == 0 else "waiting"
#         yasmin.YASMIN_LOG_INFO(f"Temperature monitoring skipping")
#         time.sleep(2.0)
#         return "done"
class IdleState(MonitorState):
    def __init__(self):
        super().__init__(
            Int32,
            "/tree_trigger",
            ["to_tree_1", "to_tree_2", "to_tree_3", "waiting", "timeout"],
            self.check_trigger,
            msg_queue=10,
            timeout=None  # Wait indefinitely
        )

    def check_trigger(self, blackboard, msg) -> str:
        if 'max_step' not in blackboard:        
            blackboard['max_step'] = 0.05
        if msg.data == 1:
            return "to_tree_1"
        elif msg.data == 2:
            return "to_tree_2"
        elif msg.data == 3:
            return "to_tree_3"
        return "waiting"
        
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
                # if sm.is_running():
                #     sm.cancel_state()
                    
    _node.create_subscription(Joy, "/joy", joy_callback, 10)
    ee_pub = _node.create_publisher(Float64MultiArray, "/cartesian_impedance/pose_desired", 10)    


    arm_config_cli = _node.create_client(
            SetFullCollisionBehavior,
            '/franka/panda_param_service_server/set_full_collision_behavior'
        )
  
    # Wait for the service to be available
    while not arm_config_cli.wait_for_service(timeout_sec=1.0):        
        yasmin.YASMIN_LOG_INFO("Waiting for set_full_collision_behavior service...")
    
    def response_callback(future):        
        yasmin.YASMIN_LOG_INFO("Service callback started...")
        try:
            response = future.result()
            if response.success:                
                yasmin.YASMIN_LOG_INFO("✅ Successfully set collision behavior: Robot is free!")
            else:
                yasmin.YASMIN_LOG_INFO(f"❌ Failed to set collision behavior: {response.error}")                
        except Exception as e:            
            yasmin.YASMIN_LOG_INFO("f'Service call failed: {str(e)}'")
        finally:
            req_done = True  

    
    def send_request():
        request = SetFullCollisionBehavior.Request()

        # Set very large thresholds to "free" the robot
        request.lower_torque_thresholds_acceleration = [-10000.0] * 7
        request.upper_torque_thresholds_acceleration = [10000.0] * 7
        request.lower_torque_thresholds_nominal = [-10000.0] * 7
        request.upper_torque_thresholds_nominal = [10000.0] * 7
        request.lower_force_thresholds_acceleration = [-5000.0] * 6
        request.upper_force_thresholds_acceleration = [5000.0] * 6
        request.lower_force_thresholds_nominal = [-5000.0] * 6
        request.upper_force_thresholds_nominal = [5000.0] * 6

        yasmin.YASMIN_LOG_INFO("Sending collision behavior request...")
        future = arm_config_cli.call_async(request)
        future.add_done_callback(response_callback)

    send_request()

    
    config = load_config()
    sm = StateMachine(outcomes=["done", CANCEL])
    blackboard = Blackboard()
    blackboard['max_step'] = 0.1
    blackboard["config"] = config
    blackboard["restart_requested"] = False

    goals = config["tree_goals"]
    # arm_positions = config["arm_positions"]
    # joint_positions = config["joint_positions"]
    ee_goals = config["arm_ee_goal"]
    arm_home_ee = config["arm_home_ee"][0]
    threshold = config["temperature_threshold"]
    home_joints = config["home_joint_position"]
    sm.add_state(
        "IDLE",
        IdleState(),
        transitions={
            "to_tree_1": "NAV_TO_TREE_1",
            "to_tree_2": "NAV_TO_TREE_2",
            "to_tree_3": "NAV_TO_TREE_3",
            "waiting": "IDLE",
            TIMEOUT: "IDLE",
            CANCEL: CANCEL
        }
    )
    for i in range(len(ee_goals)):
        tree_num = i + 1
        next_tree = (i + 1) % len(ee_goals) + 1  # for cycling back to tree 1 after the last

        # Add NAV_TO_TREE_i
        sm.add_state(
            f"NAV_TO_TREE_{tree_num}",
            NavToTreeState(goals[0]["x"], goals[0]["y"]),
            transitions={
                "arrived": f"ARM_TO_TREE_{tree_num}",
                "moving": f"NAV_TO_TREE_{tree_num}",
                TIMEOUT: "HOMING",
                CANCEL: CANCEL
            }
        )

        # Add ARM_TO_TREE_i
        sm.add_state(
            f"ARM_TO_TREE_{tree_num}",
            MoveArmState(ee_goals[i], ee_pub),
            transitions={
                "done": f"CHECK_TEMP_TREE_{tree_num}",
                "moving": f"ARM_TO_TREE_{tree_num}",
                TIMEOUT: "HOMING",
                CANCEL: CANCEL
            }
        )

        # Add CHECK_TEMP_TREE_i
        # For the last tree, go back to NAV_TO_TREE_1 or customize as needed
        # next_state = f"NAV_TO_TREE_{next_tree}" if i < len(ee_goals) - 1 else "NAV_TO_TREE_1"
       
        sm.add_state(
            f"CHECK_TEMP_TREE_{tree_num}",
            CheckTempState(threshold),
            transitions={
                "done": f"HOMING",
                # "waiting": f"CHECK_TEMP_TREE_{tree_num}",
                "waiting": f"HOMING",
                TIMEOUT: "HOMING" if i == len(ee_goals) - 1 else "HOMING",
                CANCEL: CANCEL
            }
        )


    

    sm.add_state(
        f"HOMING",
        MoveArmState(arm_home_ee, ee_pub),
        transitions={
            "done": f"IDLE",
            "moving": f"HOMING",
            TIMEOUT: "HOMING",
            CANCEL: CANCEL
        }
    )
    # === HOMING fallback state ===
    # sm.add_state("HOMING", HomeArmState(home_joints),
    #              transitions={"done": "aborted", "moving": "aborted"})

        
    sm.set_start_state("HOMING")

    YasminViewerPub(" ", sm)

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
