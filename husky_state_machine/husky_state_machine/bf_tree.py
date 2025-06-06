#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, String
from std_srvs.srv import SetBool
import py_trees

class GoToTree(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node, tree_index: int, tree_pos):
        super().__init__(name=f"GoToTree{tree_index}")
        self.node = node
        self.tree_index = tree_index
        self.tree_pos = tree_pos
        self.publisher = self.node.create_publisher(Float64MultiArray, "/arm/goal_position", 10)
        self.reached = False
        self.start_time = None

    def initialise(self):
        msg = Float64MultiArray()
        msg.data = self.tree_pos
        self.publisher.publish(msg)
        self.start_time = self.node.get_clock().now()
        self.reached = False
        self.node.get_logger().info(f"Moving to Tree {self.tree_index}")

    def update(self):
        if self.reached:
            return py_trees.common.Status.SUCCESS
        elapsed = self.node.get_clock().now() - self.start_time
        if elapsed.nanoseconds / 1e9 > 3.0:
            self.node.get_logger().warn("Timeout while moving arm")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def set_reached(self):
        self.reached = True

class TriggerSensorRead(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="TriggerSensorRead")
        self.node = node
        self.client = self.node.create_client(SetBool, '/read_soil_sensor')
        self.response = None

    def update(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("Service not available")
            return py_trees.common.Status.FAILURE

        req = SetBool.Request()
        req.data = True
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() and future.result().success:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class InterruptHandler(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node):
        super().__init__(name="InterruptHandler")
        self.node = node
        self.override_index = None
        self.node.create_subscription(Int32, "/user/override_tree", self.override_cb, 10)

    def override_cb(self, msg):
        self.override_index = msg.data
        self.node.get_logger().info(f"Interrupt: Override to Tree {self.override_index}")

    def update(self):
        if self.override_index is not None:
            self.node.get_logger().info(f"Executing manual override to tree {self.override_index}")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__("behavior_tree_node")
        self.tree_positions = [
            [0.1, 0.2, 0.3],
            [0.4, 0.2, 0.3],
            [0.6, 0.2, 0.3]
        ]
        self.root = self.create_tree()
        self.bt = py_trees.trees.BehaviourTree(self.root)
        self.override_index = None

    def create_tree(self):
        root = py_trees.composites.Selector("Root",memory=False)

        interrupt = InterruptHandler(self)
        interrupt_seq = py_trees.composites.Sequence("InterruptSequence",memory=False)
        interrupt_seq.add_children([
            interrupt,
            GoToTree(self, 99, [0.0, 0.0, 0.0]),  # dummy fallback or override tree
            TriggerSensorRead(self)
        ])

        routine_seq = py_trees.composites.Sequence("RoutineSequence",memory=False)
        for i, pos in enumerate(self.tree_positions):
            routine_seq.add_children([
                GoToTree(self, i, pos),
                TriggerSensorRead(self)
            ])

        root.add_children([interrupt_seq, routine_seq])
        return root

    def tick(self):
        self.bt.tick()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()