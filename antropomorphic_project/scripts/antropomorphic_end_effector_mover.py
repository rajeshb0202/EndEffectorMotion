#!/usr/bin/env python3
import rospy
from planar_3dof_control.msg import EndEffector
from geometry_msgs.msg import Vector3
from ik_antropomorphic_arm import ComputeInverseKinematics
from move_joints import JointMover
from rviz_marker import MarkerBasics

class EndEffectorMover(object):

    def __init__(self, wait_reach_goal=True):

        # Start the RVIZ marker pblisher
        self.markerbasics_object = MarkerBasics()
        self.unique_marker_index = 0
        
        # We start the mover
        self.robot_mover = JointMover()

        # object to calculate the inverse kinematics
        self.IK_compute = ComputeInverseKinematics()
        
        # We subscribe to a topic for EE Pose commands

        ee_pose_commands_topic = "/ee_pose_commands"
        rospy.Subscriber(ee_pose_commands_topic, EndEffector, self.ee_pose_commands_clb)
        ee_pose_commands_data = None
        while ee_pose_commands_data is None and not rospy.is_shutdown():
            try:
                ee_pose_commands_data = rospy.wait_for_message(ee_pose_commands_topic, EndEffector, timeout=0.5)
            except:
                rospy.logwarn("Waiting for first EE command Pose in topic =" + str(ee_pose_commands_topic))
                pass
        
        self.Pee_x = ee_pose_commands_data.ee_xy_theta.x
        self.Pee_y = ee_pose_commands_data.ee_xy_theta.y
        self.Pee_z = ee_pose_commands_data.ee_xy_theta.z
        self.elbow_pol = ee_pose_commands_data.elbow_policy.data        


        # We susbcribe to the Real P_3 pose
        end_effector_real_pose_topic = "/end_effector_real_pose"
        rospy.Subscriber(end_effector_real_pose_topic, Vector3, self.end_effector_real_pose_clb)
        end_effector_real_pose_data = None
        while end_effector_real_pose_data is None and not rospy.is_shutdown():
            try:
                end_effector_real_pose_data = rospy.wait_for_message(end_effector_real_pose_topic, Vector3, timeout=0.5)
            except:
                rospy.logwarn("Waiting for first EE command Pose in topic =" + str(end_effector_real_pose_topic))
                pass
        
        self.Pee_x_real = end_effector_real_pose_data.x
        self.Pee_y_real = end_effector_real_pose_data.y
        self.Pee_z_real = end_effector_real_pose_data.z



    def end_effector_real_pose_clb(self,msg):

        self.Pee_x_real = msg.x
        self.Pee_y_real = msg.y
        self.Pee_z_real = msg.z

        rospy.loginfo("Pxx_REAL=["+str(self.Pee_x_real)+","+str(self.Pee_y_real)+"] --- Pee_z="+str(self.Pee_z_real))
        rospy.loginfo("Pxx_OBJE=["+str(self.Pee_x)+","+str(self.Pee_y)+"] --- Pee_z_real="+str(self.Pee_z))

        

    def ee_pose_commands_clb(self, msg):

        self.Pee_x = msg.ee_xy_theta.x
        self.Pee_y = msg.ee_xy_theta.y
        self.Pee_z = msg.ee_xy_theta.z

        self.elbow_pol = msg.elbow_policy.data
        possible_solution = False

        joint_angles_solution = self.IK_compute.calculate_ik(self.Pee_x, self.Pee_y, self.Pee_z, self.elbow_pol)
        # print(joint_angles_solution)
        if joint_angles_solution:
            possible_solution = joint_angles_solution[0][3]

        if possible_solution:
            theta_1 = joint_angles_solution[0][0]
            theta_2 = joint_angles_solution[0][1]
            theta_3 = joint_angles_solution[0][2]

            self.robot_mover.move_all_joints(theta_1, theta_2, theta_3)
            self.markerbasics_object.publish_point(self.Pee_x, self.Pee_y, self.Pee_z, index=self.unique_marker_index)
            self.unique_marker_index += 1 
        else:
            rospy.logerr("NO POSSIBLE SOLUTION FOUND, Robot Cant reach that pose")

    

def main():
    rospy.init_node('planar_end_effector_mover')

    planar_object = EndEffectorMover()
    rospy.spin()



if __name__ == '__main__':
    main()