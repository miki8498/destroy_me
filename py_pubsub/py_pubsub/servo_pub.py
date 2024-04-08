import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from std_srvs.srv import Trigger
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import PyKDL as kdl
import tf2_ros
import time
from message.srv import CablePos
from message.msg import ListMsg
import queue
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R

class ServoNode(Node):

    def __init__(self, ur_type):
        super().__init__('servo_node_{}'.format(ur_type))

        #Transform listener
        self.ur_type = ur_type
        self.tf_pub  = self.create_publisher(Float64MultiArray, "tf_transf",1)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self.timer = self.create_timer(0.2, self.current_pose)  

        #Servo topics 
        req = Trigger.Request() 
        self.twist_cmd_pub  = self.create_publisher(TwistStamped, "{}/servo_node/delta_twist_cmds".format(ur_type),1)
        self.start_servo = self.create_client(Trigger, "/{}/servo_node/start_servo".format(ur_type))
        self.start_servo.wait_for_service()
        self.start_servo.call_async(req)
        
        #Cable pos service
        self.cable_pos_client = self.create_client(CablePos, 'cable_pos')
        self.cable_pos_client.wait_for_service()

        #PID
        self.Kp_dist = 10.0
        self.Ki_dist = 0.01
        self.Kd_dist = 0.0052
        self.prev_error = [0, 0, 0]#, 0, 0, 0]

    def pub_cable_pos(self, actions):
        goal = CablePos.Request()

        for action in actions:
            listmsg = ListMsg()
            listmsg.action = action
            goal.cable_pos.append(listmsg)

        res = self.cable_pos_client.call_async(goal)
        rclpy.spin_until_future_complete(self, res)
        return res.result()

    
    def twist_cmd(self, desired_pose, msg, q: queue.Queue):
        #lui si sposta nelle direzioni indicate nel cartesiano, di quanto gli dai 
        
        twist_stamped = TwistStamped()
        twist = Twist()

        twist.linear.x = msg[0]
        twist.linear.y = msg[1]
        twist.linear.z = msg[2]
        # twist.angular.x = msg[3]
        # twist.angular.y = msg[4]
        # twist.angular.z = msg[5]

        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "world"
        twist_stamped.twist = twist
        
        self.twist_cmd_pub.publish(twist_stamped)
        q_new = queue.Queue

        while not (abs(self.robot_position.p.x()-desired_pose.p.x())< 0.001  and abs(self.robot_position.p.y()-desired_pose.p.y())< 0.001 and abs(self.robot_position.p.z()-desired_pose.p.z())< 0.001):
                #and abs(msg[3]) < 0.001 and abs(msg[4]) < 0.001 and abs(msg[5]) < 0.001):
            msg = self.proportional_control(desired_pose, False, q_new)
            twist.linear.x = msg[0]
            twist.linear.y = msg[1]
            twist.linear.z = msg[2]
            # twist.angular.x = msg[3]
            # twist.angular.y = msg[4]
            # twist.angular.z = msg[5]
            # print("twist ", twist)
            if(abs(self.robot_position.p.x()-desired_pose.p.x())< 0.001):
                twist.linear.x = 0.0
                print("xxxxxxxxxxxxxxxxxxx")
            if(abs(self.robot_position.p.y()-desired_pose.p.y())< 0.001):
                twist.linear.y = 0.0
                print("yyyyyyyyyyyyyyyyyyyyyyy")
            if(abs(self.robot_position.p.z()-desired_pose.p.z())< 0.001):
                twist.linear.z = 0.0
                print("zzzzzzzzzzzzzzzzzzz")
            # if(abs(msg[3]) < 0.001):
            #     twist.angular.x = 0.0
            #     self.position_reached[3] = True
            #     print("qqqqqqqqxxxxxxxxxxxxxxxxxxxx")
            # if(abs(msg[4]) < 0.001):
            #     twist.angular.y = 0.0
            #     self.position_reached[4] = True
            #     print("qqqqqqqqqyyyyyyyyyyyyyyyyyyyyyyyyyyyy")
            # if(abs(msg[5]) < 0.001):
            #     twist.angular.z = 0.0
            #     self.position_reached[5] = True
            #     print("qqqqqqqqzzzzzzzzzzzzz")
            

            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = "world"
            twist_stamped.twist = twist
            self.twist_cmd_pub.publish(twist_stamped)
                
        
        print("ECCOMI!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11")
        success = True
        q.put_nowait(success)

    def current_pose(self):

        origin_frame = "world"
        dest_frame = "{}_finger_tip".format(self.ur_type)

        try:
            # Get the transform between /map and /base_footprint
            t = self._tf_buffer.lookup_transform(origin_frame, dest_frame,rclpy.time.Time())

            self.robot_position   = kdl.Frame()
            self.robot_position.p = kdl.Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
            self.robot_position.M = kdl.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
            self.tf_pub.publish(Float64MultiArray(data=[t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]))
            print(self.robot_position.M.GetQuaternion())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')

    
    def proportional_control(self, desired_pose, thread, q: queue.Queue):
        self.robot_position = None

        while (self.robot_position is None):
            time.sleep(0.1)
    
        
        current_pose = self.robot_position
        print("Current pose ", current_pose.M.GetRPY())
        print("Desired pose ", desired_pose.M.GetRPY())
        
        err_x = desired_pose.p.x() - current_pose.p.x()
        err_y = desired_pose.p.y() - current_pose.p.y()
        err_z = desired_pose.p.z() - current_pose.p.z()

        #w_err = self.quaternion_error(current_pose.M, desired_pose.M)

        # print(err_x, err_y, err_z)
        l_vx = self.Kp_dist * err_x + self.Ki_dist * (err_x + self.prev_error[0]) + self.Kd_dist * (err_x - self.prev_error[0])
        l_vy = self.Kp_dist * err_y + self.Ki_dist * (err_y + self.prev_error[1]) + self.Kd_dist * (err_y - self.prev_error[1])
        l_vz = self.Kp_dist * err_z + self.Ki_dist * (err_z + self.prev_error[2]) + self.Kd_dist * (err_z - self.prev_error[2])
    
        self.prev_error = [err_x, err_y, err_z]
        #l_v = Kp_dist * abs(err_x) # + Ki_dist * integral_dist + Kd_dist * derivative_dist

        lv = [l_vx, l_vy, l_vz]# w_err[0], w_err[1], w_err[2]]
        print("lv ", lv)
        if thread:
            q.put_nowait(lv)
        else:
            return lv
    def quaternion_multiply(self,q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return np.array([x, y, z, w])
    
    def quaternion_error(self, q1, q2):
        #complex conjugate of q1 
        q1_new = kdl.Frame()
        q1_new.M = kdl.Rotation.Quaternion(-q1.GetQuaternion()[0], -q1.GetQuaternion()[1], -q1.GetQuaternion()[2], q1.GetQuaternion()[3])
        print("q1_new ", q1_new.M.GetQuaternion())
        #err_rot = self.quaternion_multiply(q1_new.M.GetQuaternion(), q2.GetQuaternion())
        err_rot = q1_new.M*q2
        #print("s ", s.GetQuaternion())

        #err_rot = kdl.Rotation.Inverse(q1) * q2
        q_w_err = err_rot.GetQuaternion()[3]
        print("q_w_err ", q_w_err)
        if(q_w_err < 0):
            q_w_err = -q_w_err
            q_x_err = -err_rot.GetQuaternion()[0]
            q_y_err = -err_rot.GetQuaternion()[1]
            q_z_err = -err_rot.GetQuaternion()[2]
        else:
            q_x_err = err_rot.GetQuaternion()[0]
            q_y_err = err_rot.GetQuaternion()[1]
            q_z_err = err_rot.GetQuaternion()[2]

        
        w_x = 2*(q_w_err*q_x_err + q_y_err*q_z_err)

        w_err = [w_x, w_y, w_z]
        print("w_err ", w_err)
        return w_err


