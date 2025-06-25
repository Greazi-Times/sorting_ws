#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import tf2_ros
from geometry_msgs.msg import Pose, TransformStamped
from tf.transformations import quaternion_from_euler
from my_demo.msg import MoveToPoseAction, MoveToPoseGoal
from xarm_msgs.srv import SetInt16, Call

class RobotHandler:
    def __init__(self):
        # Maak verbinding met de action server voor het sturen van poses
        self.client = actionlib.SimpleActionClient('move_to_pose', MoveToPoseAction)
        rospy.loginfo("Wachten op action server...")
        self.client.wait_for_server()
        rospy.loginfo("Verbonden met action server.")

        # Initialiseer de gripper service
        rospy.loginfo("Wachten op gripper service...")
        rospy.wait_for_service('/ufactory/vacuum_gripper_set')
        self.gripper_service = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', SetInt16)
        rospy.loginfo("Gripper service beschikbaar.")

        # Initialiseer de TF broadcaster om frames te publiceren in RViz
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.latest_pose = None
        self.tf_timer = rospy.Timer(rospy.Duration(0.1), self.publish_tf_timer)

        # Definieer de standby positie voor de robotarm
        self.standby_pose = Pose()
        self.standby_pose.position.x = 0.0
        self.standby_pose.position.y = 0.075
        self.standby_pose.position.z = 0.22
        qx, qy, qz, qw = quaternion_from_euler(3.14, 0, -1.57)
        self.standby_pose.orientation.x = qx
        self.standby_pose.orientation.y = qy
        self.standby_pose.orientation.z = qz
        self.standby_pose.orientation.w = qw

    def publish_tf(self, pose, frame_id="world", child_frame_id="product_locatie"):
        # Publiceer een TF transform om het product visueel te maken in RViz
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z - 0.082
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_tf_timer(self, event):
        # Wordt periodiek aangeroepen door rospy.Timer om TF te blijven publiceren
        if self.latest_pose:
            self.publish_tf(self.latest_pose)

    def set_gripper(self, on, wait_time=2):
        # Activeer of deactiveer de gripper (vacuum aan of uit)
        try:
            state = 0 if on else 1
            rospy.loginfo("Gripper %s", "aan (vacuum)" if on else "uit (loslaten)")
            resp = self.gripper_service(state)
            rospy.sleep(wait_time)
            rospy.loginfo("Gripper service-oproep verstuurd, response: ret=%s", resp.ret)
            return resp.ret == 0
        except rospy.ServiceException as e:
            rospy.logerr("Fout bij aanroepen gripper service: %s", str(e))
            return False

    def move_to(self, pose, max_retries=10):
        # Stuur een pose naar de robot en probeer deze maximaal max_retries keer
        goal = MoveToPoseGoal()
        goal.target_pose = pose

        def feedback_cb(feedback):
            rospy.loginfo("Voortgang: %.1f%%", feedback.progress)

        attempt = 0
        while attempt < max_retries:
            rospy.loginfo("Beweging poging %d naar: x=%.3f, y=%.3f, z=%.3f",
                          attempt + 1, pose.position.x, pose.position.y, pose.position.z)

            self.client.send_goal(goal, feedback_cb=feedback_cb)
            self.client.wait_for_result()
            result = self.client.get_result()

            if result and result.success:
                rospy.loginfo("Resultaat succesvol op poging %d", attempt + 1)
                return True
            else:
                rospy.logwarn("Beweging mislukt op poging %d", attempt + 1)
                attempt += 1
                rospy.sleep(1.0)

        rospy.logerr("Beweging naar pose mislukt na %d pogingen", max_retries)
        return False

    def sort(self, x, y, z , orientation, object_type):
        # Volledige pick-and-place sequentie uitvoeren op basis van object type
        rospy.loginfo("Pick-and-place gestart voor '%s' op (%.3f, %.3f, %.3f)",
                      object_type, x, y, z)

        # Definieer de pickup poses
        pickup_pose = Pose()
        pickup_pose.position.x = x
        pickup_pose.position.y = y
        pickup_pose.position.z = z + 0.03 + 0.082
        pickup_pose.orientation = orientation

        pickup_down = Pose()
        pickup_down.position.x = x
        pickup_down.position.y = y
        pickup_down.position.z = z + 0.082
        pickup_down.orientation = orientation

        # Publiceer TF-frame tijdens benadering
        rospy.loginfo("TF Frame 'product_locatie' publiceren")
        self.latest_pose = pickup_down
        if not self.move_to(self.standby_pose): return False

        # Uitvoeren van pick and place stappen
        if not self.move_to(pickup_pose): return False
        if not self.set_gripper(False): return False
        if not self.move_to(pickup_down): return False

        # Stop TF-publicatie zodra we op pickup_down zijn
        self.latest_pose = None

        if not self.set_gripper(True): return False
        if not self.move_to(pickup_pose): return False
        if not self.move_to(self.standby_pose): return False

        # Doelpositie bepalen
        target_pose = Pose()
        if object_type == "hout":
            target_pose.position.x = 0.076
            target_pose.position.y = 0.175
            target_pose.position.z = 0.150
        elif object_type == "normaal":
            target_pose.position.x = -0.049
            target_pose.position.y = 0.175
            target_pose.position.z = 0.150
        elif object_type == "elektrisch":
            target_pose.position.x = 0.076
            target_pose.position.y = 0.305
            target_pose.position.z = 0.265
        elif object_type == "rager":
            target_pose.position.x = -0.049
            target_pose.position.y = 0.305
            target_pose.position.z = 0.265
        else:
            rospy.logwarn("Onbekend object_type: %s", object_type)
            return False

        # Zet oriëntatie voor plaatsing
        qx, qy, qz, qw = quaternion_from_euler(3.14, 0, -1.57)
        target_pose.orientation.x = qx
        target_pose.orientation.y = qy
        target_pose.orientation.z = qz
        target_pose.orientation.w = qw

        if not self.move_to(target_pose): return False
        if not self.set_gripper(False): return False
        if not self.move_to(self.standby_pose): return False
        if not self.set_gripper(True): return False

        # Stopservice voor de gripper aanroepen
        try:
            rospy.wait_for_service('/ufactory/stop_lite6_gripper', timeout=5.0)
            stop_service = rospy.ServiceProxy('/ufactory/stop_lite6_gripper', Call)
            resp = stop_service()
            rospy.loginfo("Stopservice gripper aangeroepen, response: ret=%s", resp.ret)
        except rospy.ServiceException as e:
            rospy.logerr("Fout bij stopservice gripper: %s", str(e))
        except rospy.ROSException:
            rospy.logerr("Timeout bij wachten op stopservice gripper")

        rospy.loginfo("Object succesvol verwerkt.")
        return True

    def reset_robot(self):
        # Reset de robot na een noodstop en keer terug naar standby
        try:
            rospy.loginfo("Noodstop resetten...")
            reset_resp = self.set_state_service(4)
            rospy.sleep(0.5)

            rospy.loginfo("Servo's weer inschakelen...")
            servo_resp = self.set_state_service(1)

            if reset_resp.ret == 0 and servo_resp.ret == 0:
                rospy.loginfo("Robot succesvol gereset en actief.")
                rospy.loginfo("Robot keert terug naar standby positie...")
                if self.move_to(self.standby_pose):
                    rospy.loginfo("Robot staat klaar in standby.")
                    return True
                else:
                    rospy.logwarn("Beweging naar standby faalde.")
                    return False
            else:
                rospy.logwarn("Robot reset faalde: reset=%s, servo=%s", reset_resp.ret, servo_resp.ret)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Fout bij resetten van de robot: %s", str(e))
            return False

