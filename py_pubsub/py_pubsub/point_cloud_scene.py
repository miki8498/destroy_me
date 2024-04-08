import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class PointCloud(Node):

    def __init__(self):
        super().__init__('point_cloud_scene')

        self.scene_service = self.create_client(Trigger, 'points_remove')
        self.scene_service.wait_for_service()

    def scene_req(self):

        scene_command = Trigger.Request()

        res = self.scene_service.call_async(scene_command)
        rclpy.spin_until_future_complete(self, res)

def main(args=None):
    rclpy.init(args=args)

    node = PointCloud()
    future = node.scene_req()

    rclpy.spin(node, future)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()