#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32
from std_srvs.srv import SetBool
import py_trees
import py_trees.visitors

# -------------------------
# GoToTree Node
# -------------------------
class GoToTree(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node, tree_pos):
        super().__init__(name=name)
        self.node = node
        self.tree_pos = tree_pos
        self.publisher = self.node.create_publisher(Float64MultiArray, "/arm/goal_position", 10)
        self.start_time = None

    def initialise(self):
        msg = Float64MultiArray()
        msg.data = self.tree_pos
        self.publisher.publish(msg)
        self.start_time = self.node.get_clock().now()
        self.node.get_logger().info(f"Moving to position: {self.tree_pos}")

    def update(self):
        elapsed = self.node.get_clock().now() - self.start_time
        if elapsed.nanoseconds / 1e9 > 3.0:
            self.node.get_logger().warn("Timeout while moving arm")
            return py_trees.common.Status.FAILURE
        if elapsed.nanoseconds / 1e9 > 2.0:  # Simulate success
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


# -------------------------
# TriggerSensorRead Node (Fixed for Service)
# -------------------------
class TriggerSensorRead(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node):
        super().__init__(name=name)
        self.node = node
        self.client = self.node.create_client(SetBool, "/read_soil_sensor")
        self.future = None

    def initialise(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("Sensor service not available")
            return
        request = SetBool.Request()
        request.data = True
        self.future = self.client.call_async(request)

    def update(self):
        if self.future is None:
            return py_trees.common.Status.FAILURE
        if self.future.done():
            if self.future.result() and self.future.result().success:
                self.node.get_logger().info("Soil sensor reading success")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().warn("Sensor reading failed or returned false")
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING


# -------------------------
# InterruptHandler (decorator-like)
# -------------------------
class InterruptMonitor(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="InterruptMonitor")
        self.node = node
        self.override_index = None
        self.sub = self.node.create_subscription(Int32, "/user/override_tree", self.override_cb, 10)

    def override_cb(self, msg):
        self.override_index = msg.data
        self.node.get_logger().info(f"Received override to Tree {self.override_index}")

    def update(self):
        if self.override_index is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


# -------------------------
# BT Tree Using plain py_trees
# -------------------------
class SoilMonitorTree(Node):
    def __init__(self):
        super().__init__("soil_monitor_bt")

        self.tree_positions = [
            [0.1, 0.2, 0.3],
            [0.4, 0.2, 0.3],
            [0.6, 0.2, 0.3],
        ]

        self.root = self.create_tree()
        self.bt = py_trees.trees.BehaviourTree(self.root)
        self.bt.setup(timeout=15)

        # Add snapshot visitor for visualization
        self.visitor = py_trees.visitors.SnapshotVisitor()
        self.bt.visitors.append(self.visitor)

    def create_tree(self):
        root = py_trees.composites.Selector(name="Root", memory=False)

        interrupt_seq = py_trees.composites.Sequence(name="InterruptSequence", memory=False)
        interrupt_monitor = InterruptMonitor(self)
        interrupt_seq.add_children([
            interrupt_monitor,
            GoToTree("GoToInterrupt", self, [0.0, 0.0, 0.0]),
            TriggerSensorRead("TriggerSensorRead_Interrupt", self)
        ])

        routine_seq = py_trees.composites.Sequence(name="RoutineSequence", memory=False)
        for i, pos in enumerate(self.tree_positions):
            routine_seq.add_children([
                GoToTree(f"GoToTree_{i}", self, pos),
                TriggerSensorRead(f"TriggerSensorRead_{i}", self)
            ])

        root.add_children([interrupt_seq, routine_seq])
        return root

    def tick(self):
        self.bt.tick()
        # tree_status_str = py_trees.display.unicode_tree(self.root, visited=self.visitor)
        tree_status_str = py_trees.display.unicode_tree(self.root, visited=self.visitor.visited)

        self.get_logger().info("\n" + tree_status_str)


# -------------------------
# Main Entry Point
# -------------------------
def main():
    rclpy.init()
    node = SoilMonitorTree()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
