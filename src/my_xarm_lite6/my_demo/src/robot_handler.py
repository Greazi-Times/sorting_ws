#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import tf2_ros
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, TransformStamped
from tf.transformations import quaternion_from_euler
from my_demo.msg import MoveToPoseAction, MoveToPoseGoal
from xarm_msgs.srv import SetInt16, Call, SetInt16Request
from xarm_msgs.msg import RobotMsg

class RobotHandler:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_to_pose', MoveToPoseAction)
        rospy.loginfo("Wachten op action server...")
        self.client.wait_for_server()
        rospy.loginfo("Verbonden met action server.")

        rospy.loginfo("Wachten op gripper service...")
        rospy.wait_for_service('/ufactory/vacuum_gripper_set')
        self.gripper_service = rospy.ServiceProxy('/ufactory/vacuum_gripper_set', SetInt16)
        rospy.loginfo("Gripper service beschikbaar.")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.latest_pose = None
        self.tf_timer = rospy.Timer(rospy.Duration(0.1), self.publish_tf_timer)

        self.emergency_stop_active = False
        self.external_emergency_stop = False
        self.has_error = False
        self.has_warning = False
        self._estop_logged = False

        rospy.Subscriber("/ufactory/robot_states", RobotMsg, self._robot_state_callback)
        rospy.Subscriber("/custom/emergency_stop", Bool, self.external_estop_callback)

        self.standby_pose = Pose()
        self.standby_pose.position.x = 0.0
        self.standby_pose.position.y = 0.075
        self.standby_pose.position.z = 0.22
        qx, qy, qz, qw = quaternion_from_euler(3.14, 0, -1.57)
        self.standby_pose.orientation.x = qx
        self.standby_pose.orientation.y = qy
        self.standby_pose.orientation.z = qz
        self.standby_pose.orientation.w = qw

    def _robot_state_callback(self, msg):
        e_stop_now = (msg.err == 2)
        if e_stop_now and not self.emergency_stop_active:
            self.cancel_current_motion("Hardwarematige E-stop via robot_states")

        self.emergency_stop_active = e_stop_now
        self.has_error = (msg.err != 0)
        self.has_warning = (msg.warn != 0)

        if self.emergency_stop_active and not self._estop_logged:
            rospy.logwarn("Noodstop geactiveerd (err=2)")
            self._estop_logged = True
        elif not self.emergency_stop_active:
            self._estop_logged = False

        if not self.emergency_stop_active:
            if self.has_error:
                rospy.logerr("Robotfout actief: err={}".format(msg.err))
            elif self.has_warning:
                rospy.logwarn("Waarschuwing actief: warn={}".format(msg.warn))

    def external_estop_callback(self, msg):
        self.external_emergency_stop = msg.data
        if msg.data:
            self.cancel_current_motion("Externe E-stop via HMI (geen logging)")

    def cancel_current_motion(self, reason=""):
        if self.client.gh and self.client.gh.comm_state != actionlib.CommState.DONE:
            rospy.logwarn("Beweging afgebroken: {}".format(reason))
            self.client.cancel_goal()

    def is_emergency_stop_active(self):
        return self.emergency_stop_active or self.external_emergency_stop

    def publish_tf(self, pose, frame_id="world", child_frame_id="product_locatie"):
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
        if self.latest_pose:
            self.publish_tf(self.latest_pose)

    def set_gripper(self, on, wait_time=2):
        try:
            state = 0 if on else 1
            rospy.loginfo("Gripper {}".format("aan (vacuum)" if on else "uit (loslaten)"))
            resp = self.gripper_service(state)
            rospy.sleep(wait_time)
            rospy.loginfo("Gripper service-oproep verstuurd, response: ret={}".format(resp.ret))
            return resp.ret == 0
        except rospy.ServiceException as e:
            rospy.logerr("Fout bij aanroepen gripper service: {}".format(str(e)))
            return False

    def move_to(self, pose, max_retries=10):
        if self.is_emergency_stop_active():
            rospy.logerr("Beweging geblokkeerd: noodstop is actief!")
            return False

        goal = MoveToPoseGoal()
        goal.target_pose = pose

        def feedback_callback(feedback):
            rospy.loginfo("Status vanuit controller: {}".format(feedback.status))

        attempt = 0
        while attempt < max_retries:
            rospy.loginfo("Beweging poging {} naar: x={:.3f}, y={:.3f}, z={:.3f}".format(
                attempt + 1, pose.position.x, pose.position.y, pose.position.z))

            self.client.send_goal(goal, feedback_cb=feedback_callback)
            self.client.wait_for_result()
            result = self.client.get_result()

            if result and result.success:
                rospy.loginfo("Resultaat succesvol op poging {}".format(attempt + 1))
                return True
            else:
                rospy.logwarn("Beweging mislukt op poging {}".format(attempt + 1))
                attempt += 1
                rospy.sleep(1.0)

        rospy.logerr("Beweging naar pose mislukt na {} pogingen".format(max_retries))
        return False

    def sort(self, x, y, z , orientation, object_type):
        rospy.loginfo("Pick-and-place gestart voor '{}' op ({:.3f}, {:.3f}, {:.3f})".format(
            object_type, x, y, z))

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

        rospy.loginfo("TF Frame 'product_locatie' publiceren")
        self.latest_pose = pickup_down
        if not self.move_to(self.standby_pose): return False
        if not self.move_to(pickup_pose): return False
        if not self.set_gripper(False): return False
        if not self.move_to(pickup_down): return False

        self.latest_pose = None
        if not self.set_gripper(True): return False
        if not self.move_to(pickup_pose): return False
        if not self.move_to(self.standby_pose): return False

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
            rospy.logwarn("Onbekend object_type: {}".format(object_type))
            return False

        qx, qy, qz, qw = quaternion_from_euler(3.14, 0, -1.57)
        target_pose.orientation.x = qx
        target_pose.orientation.y = qy
        target_pose.orientation.z = qz
        target_pose.orientation.w = qw

        if not self.move_to(target_pose): return False
        if not self.set_gripper(False): return False
        if not self.move_to(self.standby_pose): return False
        if not self.set_gripper(True): return False

        try:
            rospy.wait_for_service('/ufactory/stop_lite6_gripper', timeout=5.0)
            stop_service = rospy.ServiceProxy('/ufactory/stop_lite6_gripper', Call)
            resp = stop_service()
            rospy.loginfo("Stopservice gripper aangeroepen, response: ret={}".format(resp.ret))
        except rospy.ServiceException as e:
            rospy.logerr("Fout bij stopservice gripper: {}".format(str(e)))
        except rospy.ROSException:
            rospy.logerr("Timeout bij wachten op stopservice gripper")

        rospy.loginfo("Object succesvol verwerkt.")
        return True

    def reset_robot(self):
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
                rospy.logwarn("Robot reset faalde: reset={}, servo={}".format(reset_resp.ret, servo_resp.ret))
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Fout bij resetten van de robot: {}".format(str(e)))
            return False

