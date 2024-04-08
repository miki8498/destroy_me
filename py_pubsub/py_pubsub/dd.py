import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import copy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
class MyNode(Node):

    def __init__(self):
        super().__init__('my_node')
        self.joint_pub  = self.create_publisher(Float64MultiArray, "ur_left_joint_group_pos_controller/commands",1)
        #moveit non può usare velocity controllers. solo con moveit servo
        self.velocity_pub  = self.create_publisher(Float64MultiArray, "ur_left_joint_group_vel_controller/commands",1)
        #create a subscriber to the joint state
        self.ur_type = "ur_left"
        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.joint_state_sub
        time.sleep(0.1)
        self.kp = 1
        self.ki = 0.5
        self.kd = 0.01
        self.integral_error = np.zeros(6)
        self.previous_error = np.zeros(6)

        timer_period = 1.0  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer = self.create_timer(1, self.vel_pub)
        self.i = 0
    
    def joint_state_callback(self, msg):
        indice = np.zeros(6)
        indice[0] = msg.name.index('{}_shoulder_pan_joint'.format(self.ur_type))
        indice[1] = msg.name.index('{}_shoulder_lift_joint'.format(self.ur_type))
        indice[2] = msg.name.index('{}_elbow_joint'.format(self.ur_type))
        indice[3] = msg.name.index('{}_wrist_1_joint'.format(self.ur_type))
        indice[4] = msg.name.index('{}_wrist_2_joint'.format(self.ur_type))
        indice[5] = msg.name.index('{}_wrist_3_joint'.format(self.ur_type))

        self.joint_position = [copy.deepcopy(msg.velocity[int(indice[0])]), copy.deepcopy(msg.velocity[int(indice[1])]), copy.deepcopy(msg.velocity[int(indice[2])]), 
                copy.deepcopy(msg.velocity[int(indice[3])]), copy.deepcopy(msg.velocity[int(indice[4])]), copy.deepcopy(msg.velocity[int(indice[5])])]
      

    def timer_callback(self):
        joint_trajectory = [[-3.14,-0.767,-1.676,-1.57,1.56,0.0]]
        # for i in range(1000): 
        #     new_pose = [joint_trajectory[-1][0]+z, joint_trajectory[-1][1]+z, joint_trajectory[-1][2]+z, joint_trajectory[-1][3]+z, joint_trajectory[-1][4]+z, joint_trajectory[-1][5]+z]
        #     joint_trajectory.append(new_pose)

        for i in joint_trajectory:
            joint_pose      = Float64MultiArray()
            joint_pose.data = copy.deepcopy(i)
            print(joint_pose.data)
            self.joint_pub.publish(joint_pose)
            time.sleep(0.002)
    
    def vel_pub(self):
        np_array = np.array(self.joint_position)
        print(np_array)
        joint_trajectory_list = [0.0,0.0,0.0,0.0,0.0,0.0]
        new_joint = np.array(joint_trajectory_list) 
        error = new_joint-np_array

        self.integral_error += error
        derivative_error = error - self.previous_error
        self.previous_error = error
        new_joint = new_joint + self.kp*error + self.ki*self.integral_error + self.kd*derivative_error
        joint_trajectory_list = [new_joint.tolist()]
        while True:
            for i in joint_trajectory_list:
                joint_pose      = Float64MultiArray()
                joint_pose.data = copy.deepcopy(i)
                self.velocity_pub.publish(joint_pose)
                time.sleep(0.002)
        print("done")

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()