import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from message.action import CommandRobotiqGripper

class MyNode(Node):

    def __init__(self):
        super().__init__('my_node')

        self.gripper_control = ActionClient(self, CommandRobotiqGripper, 'ur_right/robotiq_hand_e')
        self.gripper_control.wait_for_server()

        self.hand_ur_right = ['ur_right_bl_to_leftFinger', 'ur_right_leftFinger_to_rightFinger']


    def timer_callback(self):

        gripper_command          = CommandRobotiqGripper.Goal()
        gripper_command.position = 0.0
        gripper_command.speed    = 10.0
        gripper_command.force    = 0.0
        print(gripper_command)

        
        self._send_goal_future = self.gripper_control.send_goal_async(gripper_command, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        print("RIENTRO")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    def goal_response_callback(self, future):
        print("aaaaaaaa")
        # Get handle for the goal we just sent
        goal_handle = future.result()

        # Return early if goal is rejected
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Use goal handle to request the result
        self._get_result_future = goal_handle.get_result()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        # Log result and shut down ROS 2 cleanly
        self.get_logger().info('Result: {}'.format(result))
        rclpy.shutdown()




def main(args=None):
    rclpy.init(args=args)

    node = MyNode()
    future = node.timer_callback()

    rclpy.spin(node, future)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()