import rclpy
from rclpy.node import Node
from message.srv import UrInverseKinematics
from geometry_msgs.msg import Pose
import numpy as np
import copy
from tqdm import tqdm
from std_msgs.msg import Float64MultiArray
import time
import PyKDL as kdl
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
from tf2_ros import TransformException
from tf2_ros import LookupException
from py_pubsub.interpolator import Interpolator
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor

prior_config = 0


class MyNode(Node):

    def __init__(self):
        super().__init__('my_node')
       
        self.ur_type = "ur_right"
        self.joint_sub  = self.create_subscription(JointState, "joint_states", self.joint_callback, 1)
        self.joint_sub
        time.sleep(0.1)
        self.start_servo = self.create_client(UrInverseKinematics, "ur_inverse_k")
        self.start_servo.wait_for_service()

        self.joint_pub  = self.create_publisher(Float64MultiArray, "ur_right_joint_group_pos_controller/commands",1)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)        

        self.tbase_world = self.tf_to_list(origin_frame = 'world', dest_frame = 'ur_right_base')
        self.tcp_end = self.tf_to_list(origin_frame = 'ur_right_flange', dest_frame = 'ur_right_finger_tip')
        
        self.desired_config = [0,2,5,7] 
        

    def joint_callback(self, msg):
        indice = np.zeros(6)
        indice[0] = msg.name.index('{}_shoulder_pan_joint'.format(self.ur_type))
        indice[1] = msg.name.index('{}_shoulder_lift_joint'.format(self.ur_type))
        indice[2] = msg.name.index('{}_elbow_joint'.format(self.ur_type))
        indice[3] = msg.name.index('{}_wrist_1_joint'.format(self.ur_type))
        indice[4] = msg.name.index('{}_wrist_2_joint'.format(self.ur_type))
        indice[5] = msg.name.index('{}_wrist_3_joint'.format(self.ur_type))


        self.joint_position = [copy.deepcopy(msg.position[int(indice[0])]), copy.deepcopy(msg.position[int(indice[1])]), copy.deepcopy(msg.position[int(indice[2])]), 
                copy.deepcopy(msg.position[int(indice[3])]), copy.deepcopy(msg.position[int(indice[4])]), copy.deepcopy(msg.position[int(indice[5])])]
      
        
    def tcp_wrt_world(self, position):
        #Homogeneous matrix of the TCP wrt the world
        T_tcworld   = kdl.Frame()
        T_tcworld.p = kdl.Vector(position[0],position[1],position[2])
        T_tcworld.M = kdl.Rotation.RPY(position[3],position[4],position[5])
        print("TCP WORLD ", T_tcworld.M.GetQuaternion())
        return T_tcworld
    
    def tf_to_list(self, origin_frame, dest_frame):

        tf_future = self._tf_buffer.wait_for_transform_async(origin_frame, dest_frame, time=rclpy.time.Time())
        
        rclpy.spin_until_future_complete(self, tf_future)
        print("Got it!")
        t = self._tf_buffer.lookup_transform(origin_frame, dest_frame, rclpy.time.Time())
        print(t)
        tf   = kdl.Frame()
        tf.p = kdl.Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
        tf.M = kdl.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
        print(tf.p)
        print(tf.M.GetQuaternion())
        print(tf.M.GetRPY())
        return tf
    
    def whole_kinematics(self,tcp_world):

        T = self.tbase_world.Inverse()*tcp_world*self.tcp_end.Inverse()

        ee_base = Pose()
        ee_base.position.x = T.p.x()
        ee_base.position.y = T.p.y()
        ee_base.position.z = T.p.z()
        ee_base.orientation.x = T.M.GetQuaternion()[0]
        ee_base.orientation.y = T.M.GetQuaternion()[1]
        ee_base.orientation.z = T.M.GetQuaternion()[2]
        ee_base.orientation.w = T.M.GetQuaternion()[3]

        return ee_base

    def kin_call(self, traj):

        poses = [Pose() for i in range(len(traj))]
        ik_solution  = np.zeros((len(poses),4,6))

        req = UrInverseKinematics.Request()

        req.reference_pose = copy.deepcopy(traj)
        req.desired_config = [0,2,5,7]
        req.ur_type = "UR5e"
        req.check_q6 = True
        req.verbose  = False
        req.last_joints = self.joint_position

        res = self.start_servo.call_async(req)
        rclpy.spin_until_future_complete(self, res)
    
        success   =  [res.result().success[self.desired_config[0]], res.result().success[self.desired_config[1]], res.result().success[self.desired_config[2]], res.result().success[self.desired_config[3]]]
        max_error =  [res.result().max_error[self.desired_config[0]], res.result().max_error[self.desired_config[1]],res.result().max_error[self.desired_config[2]], res.result().max_error[self.desired_config[3]]]
        
        for i in range(len(poses)):
            for j in range(6):
                for z in range(4):
                    ik_solution[i,z,j] = res.result().solution[i].joint_matrix[z].data[j]
        
        return success, max_error, ik_solution

    def check_config(self, solution, max_error, success):
       
        boolean_list = list(np.array(max_error) <= 0.002)
        print(boolean_list)
        if (not True in boolean_list):    
            ik_choice = 10
            return ik_choice
        print(boolean_list)
        matrix = copy.deepcopy(solution)
        
        for coloumn in tqdm(range(4)):
            if(success[coloumn] == False or boolean_list[coloumn] == False):
                boolean_list[coloumn] = False
            else:
                r = [True,True,True,True,True,True]
                for i,j,d in zip(matrix,matrix[1:],matrix[2:]):
                    sol1, sol2, sol3 = i[coloumn], j[coloumn], d[coloumn]
                    for z in range(6):
                        if (abs(sol1[z] - sol2[z]) > 0.30):
                            if(abs(sol1[z] - sol3[z]) > 0.30):
                                r[z] = False
                    if (False in r):
                        boolean_list[coloumn] = False
        print(boolean_list)

        ik_choice = self.priority(boolean_list)
        print(ik_choice)

        if (not True in boolean_list) or (ik_choice != prior_config):    
            ik_choice = 10
            return ik_choice
                                         
        return ik_choice  
    
    
    def priority(self, bool_list:list):

        config = []
        for sub,name in enumerate(bool_list):
            if (name == True):
                config.append(sub)         #salvo gli index della lista che tornano true = la configurazione che funziona 
        for val in config:
            if val == prior_config:
                res = val           #ritorna la configurazione a true con la più alta priorità
                break
            else:
                res = 10
        return res   
    
    def final_traj(self, solution, ik_choice):
        Final_traj = []
        for i in solution:
            column =  i[ik_choice ,:]
            Final_traj.append(column)

        return Final_traj  
    
    def move(self, traj):
         
        for i in traj:
            joint_pose      = Float64MultiArray()
            joint_pose.data = copy.deepcopy(list(i))
            self.joint_pub.publish(joint_pose)
            time.sleep(0.002)
        
def main(args=None):
    rclpy.init(args=args)

    node = MyNode()
    interpolator = Interpolator()
    start_rot =  kdl.Rotation.Quaternion(0.613, 0.789, 0.024, -0.022)
    print("start ", start_rot.GetRPY())
    start_pose = node.tcp_wrt_world([0.515, 0.370, 0.477,start_rot.GetRPY()[0], start_rot.GetRPY()[1], start_rot.GetRPY()[2]])
    ee_start_base = node.whole_kinematics(start_pose)
    print(ee_start_base)
    goal_pose = node.tcp_wrt_world([0.615, 0.370, 0.477,start_rot.GetRPY()[0], start_rot.GetRPY()[1], start_rot.GetRPY()[2]])
    ee_goal_base = node.whole_kinematics(goal_pose)
    cartesian_traj = interpolator.interpolate(500, ee_start_base, ee_goal_base, 5.0)
    # for i in range(len(cartesian_traj)):
    #     print("Trajectory ", i, " ", cartesian_traj[i].orientation)

    success, max_error, ik_solution = node.kin_call(cartesian_traj)
    ik_choice = node.check_config(ik_solution, max_error, success)
    final_traj = node.final_traj(ik_solution, ik_choice)

    # for i in range(len(final_traj)):
    #     print("Trajectory ", i, " ", final_traj[i])
    node.move(final_traj)

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()