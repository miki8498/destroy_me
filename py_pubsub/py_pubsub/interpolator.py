import rclpy
from message.srv import Interpolate
from geometry_msgs.msg import Pose
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node

class Interpolator(Node):

    def __init__(self):
        
        super().__init__('interpolator')
            
        self.interpolate_client = self.create_client(Interpolate, 'interpolate')
        self.interpolate_client.wait_for_service()
        self.logger = RcutilsLogger()
  
    
    def interpolate(self, RATE, startpose, goalpose, period):
        
        if(startpose == goalpose):
            print("Start and goal pose are the same")
            pose = Pose()
            pose.position.x = startpose.position.x
            pose.position.y = startpose.position.y
            pose.position.z = startpose.position.z
            pose.orientation.x = startpose.orientation.x
            pose.orientation.y = startpose.orientation.y
            pose.orientation.z = startpose.orientation.z
            pose.orientation.w = startpose.orientation.w
            
            return [pose]
        
        goal = Interpolate.Request()
        
        goal.start_pose.position.x = startpose.position.x
        goal.start_pose.position.y = startpose.position.y
        goal.start_pose.position.z = startpose.position.z

        goal.start_pose.orientation.x = startpose.orientation.x
        goal.start_pose.orientation.y = startpose.orientation.y
        goal.start_pose.orientation.z = startpose.orientation.z
        goal.start_pose.orientation.w = startpose.orientation.w

        goal.goal_pose.position.x = goalpose.position.x
        goal.goal_pose.position.y = goalpose.position.y
        goal.goal_pose.position.z = goalpose.position.z

        goal.goal_pose.orientation.x = goalpose.orientation.x
        goal.goal_pose.orientation.y = goalpose.orientation.y
        goal.goal_pose.orientation.z = goalpose.orientation.z
        goal.goal_pose.orientation.w = goalpose.orientation.w
        
        goal.rate     = RATE       #rate cambia in base al robot (UR5 = 125, UR5e = 500 )
        goal.vel_cm_s = period
       
        res = self.interpolate_client.call_async(goal)
        rclpy.spin_until_future_complete(self, res)


        if(res.result().success == True):
            trajectory = res.result().trajectory
            return trajectory
        else:   
            self.logger.error("Error in interpolation")