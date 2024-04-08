import rclpy
from rclpy.node import Node
from py_pubsub.servo_pub import ServoNode
import PyKDL as kdl
import threading
import queue
import numpy as np

class Deployment(Node):

    def __init__(self):
        super().__init__('deployment_node')

    def release_cable(self, x_centro,z_centro_iniziale, raggio_iniziale, raggio_finale, theta_iniziale, passo_raggio, passo_theta, passo_zeta):

        # Calcolo delle coordinate x e z per ogni circonferenza
        raggio_corrente = raggio_iniziale
        z_centro_corrente = z_centro_iniziale
        theta_corrente = theta_iniziale
        poses = []
        while raggio_corrente <= raggio_finale:
            # Calcolo del centro della circonferenza corrente
            z_centro     = z_centro_corrente
            theta_centro =  theta_corrente  
    
            # Calcolo delle coordinate x e z per ciascun angolo      
            x = x_centro + raggio_corrente * np.cos(theta_centro)
            z = z_centro + raggio_corrente * np.sin(theta_centro)
            
            poses.append(np.array([x, -0.117, z]))
            # Incremento del raggio
            raggio_corrente   += passo_raggio
            theta_corrente    += passo_theta
            z_centro_corrente += passo_zeta
        return poses

def main(args=None):
    rclpy.init(args=args)


    node = Deployment()

    # Coordinate del centro fisso della circonferenza
    x_centro = -0.05 #+ per il destro

    # Raggio iniziale e finale
    raggio_iniziale = 0.15
    raggio_finale   = 0.21

    # Passo di decremento del raggio
    passo_raggio = 0.01
    passo_theta  = 0.314
    passo_zeta   = -0.01
    theta_iniziale    = 1.57
    z_centro_iniziale = 0.15

    ur_type_left  = "ur_left"
    ur_type_right = "ur_right"   

    servo_left  = ServoNode(ur_type_left)

    action1 = [0.0, 0.1, 0.0, 0.4, 0.0, 0.0, 0.0]
    action2 = [24.0, 0.0, 0.0, 0.4, 0.0, 0.0, 0.0]

    actions = [action1, action2]
    result_cable_spawning = servo_left.pub_cable_pos(actions)

    # servo_right = ServoNode(ur_type_right)

    # q_l = queue.Queue()
    # q_r = queue.Queue()

    # poses_to_reach = node.release_cable(x_centro, z_centro_iniziale, raggio_iniziale, raggio_finale, theta_iniziale, passo_raggio, passo_theta, passo_zeta)
    
    # desired_pose_left   = kdl.Frame()
    # #for i in range(len(poses_to_reach)):
    # desired_pose_left.p = kdl.Vector( -0.157, -0.117, 0.422) #poses_to_reach[0][0],   poses_to_reach[0][1],    poses_to_reach[0][2])

    # desired_pose_left.M = kdl.Rotation.Quaternion(0.979, 0.008, 0.200, -0.033)
    # # print(desired_pose_left.M.GetRPY())

    # t_left  = threading.Thread(target = servo_left.proportional_control, args = (desired_pose_left,True, q_l,))
    # t_left.start()
    # t_left.join()

    # lv_left  = q_l.get_nowait()
    # t_left  = threading.Thread(target = servo_left.twist_cmd, args = (desired_pose_left,lv_left,q_l))
    # t_left.start()
    # t_left.join()

    # succcess_left = q_l.get_nowait()
    # print(succcess_left)

    # 0.999, 0.025, 0.008, -0.024

    # # for i in range(len(poses_to_reach)):
    # desired_pose_left   = kdl.Frame()
    # desired_pose_left.p = kdl.Vector( poses_to_reach[0][0], poses_to_reach[0][1], poses_to_reach[0][2])
    # desired_pose_left.M = kdl.Rotation.Quaternion(0.012, 0.371, 0.034, 0.928)

    # desired_pose_right   = kdl.Frame()
    # desired_pose_right.p = kdl.Vector(0.162, 0.118, 0.568)
    # desired_pose_right.M = kdl.Rotation.Quaternion(-0.012, 0.931, -0.032, -0.363)

    # t_left  = threading.Thread(target = servo_left.proportional_control, args = (desired_pose_left,True, q_l,))
    # # t_right = threading.Thread(target = servo_right.proportional_control, args = (desired_pose_right,True, q_r,))
    # t_left.start()
    # # t_right.start() 
    # t_left.join()
    # # t_right.join() 

    # lv_left  = q_l.get_nowait()
    # # lv_right = q_r.get_nowait()

    # t_left  = threading.Thread(target = servo_left.twist_cmd, args = (desired_pose_left,lv_left,q_l))
    # # t_right = threading.Thread(target = servo_right.twist_cmd, args = (desired_pose_right,lv_right,q_r))
    # t_left.start()
    # # t_right.start() 
    # t_left.join()
    # # t_right.join() 

    # succcess_left = q_l.get_nowait()
    # print(succcess_left)
    # success_right = q_r.get_nowait()
 
    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

     
    # action1 = [0.0, 0.1, 0.0, 0.2, 0.0, 0.0, 0.0]
    # action2 = [24.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0]

    # actions = [action1, action2]
    #result_cable_spawning = node.pub_cable_pos(actions)