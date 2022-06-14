import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
import tf_conversions
import tf
import geometry_msgs.msg
from tf.transformations import *
import PyKDL
import tf2_ros
import tf2_geometry_msgs #import the packages first

# OFFSET = 1243316.568249702
def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))

class Pub:
    def __init__(self):
        rospy.init_node('odom_to_pose')
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.odom_pub = rospy.Publisher("spot_odom", Odometry, queue_size=50) # republish for ros time
        self.br = tf2_ros.TransformBroadcaster()
        self.transform = None
        self.timer = rospy.Timer(rospy.Duration(0.05), self.callback)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(2400.0)) # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # self.tf_listener = tf.TransformListener()
        self.publisher = rospy.Publisher("/pose_estimation", geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
        rospy.spin()
    
    def odom_callback(self, msg):
        current_time = rospy.Time.now()
        # first, we'll publish the transform over tf
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        # t.child_frame_id = "unomove_odom"
        t.child_frame_id = "base_footprint"
        pose = msg.pose.pose
        t.transform.translation = pose.position
        t.transform.rotation = pose.orientation
        self.br.sendTransform(t)
        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = msg.pose

        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = msg.twist

        # publish the message
        self.odom_pub.publish(odom)

    def callback(self, event):
        stamp = rospy.Time.now()
        if self.transform:
            self.transform.header.stamp = stamp
            self.br.sendTransform(self.transform)

            # print ("hahahaha!!!")
            # print (self.tf_buffer)
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_footprint", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                return

            # print ("coming here!!!!")
            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Create and fill pose message for publishing
            pose = geometry_msgs.msg.PoseWithCovarianceStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "map"
            pose.pose.pose.position.x = trans.x
            pose.pose.pose.position.y = trans.y
            pose.pose.pose.position.z = trans.z
            pose.pose.pose.orientation.x = rot.x
            pose.pose.pose.orientation.y = rot.y
            pose.pose.pose.orientation.z = rot.z
            pose.pose.pose.orientation.w = rot.w

            pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0]
            # print ("base in map", pose)
            self.publisher.publish(pose)

    def pose_callback(self, msg):
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        # t.child_frame_id = "unomove_odom"
        t.child_frame_id = "odom"

        print ("initial pose in base_print", msg.pose)

        # map->odom transform = initialpose * odom^-1
        # odom_transform = self.tf_buffer.lookup_transform("base_link",
        #                     "unomove_odom", 
        #                     rospy.Time(0),
        #                     rospy.Duration(1.0))
        odom_transform = self.tf_buffer.lookup_transform("base_footprint",
                            "odom", 
                            rospy.Time(0),
                            rospy.Duration(1.0))
        
        print ("odom_transform", odom_transform)
        # tf2_pose = tf2_ros.convert(msg.pose, odom_transform.transform)
        pose = msg.pose
        f = PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                                                          pose.pose.orientation.z, pose.pose.orientation.w),
                                                PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)) * transform_to_kdl(odom_transform)

        t.transform.translation.x = f[(0, 3)]
        t.transform.translation.y = f[(1, 3)]
        t.transform.translation.z = f[(2, 3)]
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = f.M.GetQuaternion()

        print ("initial in odom coordinate", pose)

        print ("transform", t)
        self.br.sendTransform(t)
        self.transform = t 
        print ("br", self.br)

t = Pub()