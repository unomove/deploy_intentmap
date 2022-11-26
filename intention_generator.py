#! /usr/bin/env python3
import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from numpy import array
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
#import tf2_ros
#import tf2_geometry_msgs #import the packages first
import rdp
import tf_conversions
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
import fire
import time

# define functions 
def pose(m):
    if hasattr(m, 'position') and hasattr(m, 'orientation'):
        return m
    elif hasattr(m, 'pose'):
        return pose(m.pose)
    else:
        raise ValueError(m)

def angle(dir):
    """
    Returns the angles between vectors.

    Parameters:
       dir is a 2 D - array of shape(N, M) representing N vectors in M - dimensional space.

    The
    return value is a 1 D - array of values of shape(N - 1, ), with each value
    between 0 and pi.

    0 implies the vectors point in the same direction
    pi / 2 implies the vectors are orthogonal
    pi implies the vectors point in opposite directions 
    +  means left
    -  means right
    """
    dir2 = dir[1: ]
    dir1 = dir[: -1]
    sign = np.sign(np.cross(dir1, dir2))
    return sign * np.arccos((dir1 * dir2).sum(axis = 1) / (
       np.sqrt((dir1 ** 2).sum(axis = 1) * (dir2 ** 2).sum(axis = 1))))

class Robot(object):
    """
    Get the status of robot
    """
    goal = MoveBaseGoal()
    start = PoseStamped()
    end = PoseStamped()
    position = None

    def __init__(self, name="gazebo", with_move_base=False):
        self.name=name
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_footprint')
        if with_move_base:
            self.plan_service = rospy.get_param(
                '~plan_service', '/move_base/NavfnROS/make_plan')
        else:
            # self.plan_service = rospy.get_param(
            #     '~plan_service', '/global_planner/navfn_planner/make_plan')
            self.plan_service = rospy.get_param(
                '~plan_service', '/move_base/NavfnROS/make_plan')

        self.with_move_base = with_move_base
        if self.with_move_base:
            self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            self.client.wait_for_server()
        
        Robot.goal.target_pose.header.frame_id = "map"
        Robot.goal.target_pose.header.stamp = rospy.Time.now()

        rospy.wait_for_service(self.plan_service)

        self.make_plan = rospy.ServiceProxy(
            self.plan_service, GetPlan)
        Robot.start.header.frame_id = self.global_frame
        Robot.end.header.frame_id = self.global_frame

    def updatePosition(self, pose):
        Robot.start.pose = pose
        self.position = pose

    def updateAssignedPosition(self, pose):
        self.assigned_position = pose

    def updateAssignedGoal(self, pose):
        Robot.end.pose = pose
        self.assigned_goal = pose

    def sendGoal(self, pose):
        Robot.goal.target_pose.pose = pose
        self.client.send_goal(Robot.goal)
        self.updateAssignedGoal(pose)

    def cancelGoal(self):
        self.client.cancel_goal()
        self.updateAssignedGoal(self.position)

    def getState(self):
        return self.client.get_state()

    def makePlan(self):
        plan =  self.make_plan(start=Robot.start, goal=Robot.end, tolerance=0.0)
        return plan

    def Print(self):
        print ("position: ", self.position)

class Planner(object):
    min_angle = np.pi * 0.18
    # rdp tolerance
    tolerance = 0.8
    INPLACE_LEFT='inplace_left'
    INPLACE_RIGHT='inplace_right'
    FORWARD='forward'
    BACKWARD='backward'
    STOP='stop'
    LEFT='left'
    RIGHT='right'
    INTENTIONS=[
        FORWARD,
        LEFT,
        RIGHT,
        STOP,
        INPLACE_LEFT,
        INPLACE_RIGHT
    ]
    INTENTIONS_IDX={
        FORWARD:0,
        LEFT:1,
        RIGHT:2,
        STOP:3,
        INPLACE_LEFT:4,
        INPLACE_RIGHT:5
    }
    STATUS={
        "start":0,
        "progress":1,
        "end":2,
        "invalid":3
    }

    def __init__(self, with_move_base):
        print ("initialize planner")
        self.robot = Robot(with_move_base=with_move_base)
        rospy.Subscriber('/pose_estimation', PoseWithCovarianceStamped, self.cb_pose)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.cb_change_goal)
        self.vis = rospy.Publisher('/vis', Marker, queue_size=1)
        self.vis_tracker = rospy.Publisher('/vis_tracker', Marker, queue_size=1)
        self.vis_current = rospy.Publisher('/vis_current', Marker, queue_size=1)
        self.vis_intention = rospy.Publisher('/vis_intention', MarkerArray, queue_size=1)
        self.pub_intention = rospy.Publisher('/dlm_intention', String, queue_size=1)
        self.status = Planner.STATUS['invalid']
        self.simplified = None
        self.pts = None
        self.intentions = None
        print ("init done!")

    def path_to_rdp(self, path):
        # print ('path', path)
        points = []
        for p in path.plan.poses:
            pts = pose(p)
            points.append([pts.position.x, pts.position.y])
        simplified = np.array(rdp.rdp(points, Planner.tolerance))
        intention = self.simplified_to_intentions(simplified)
        self.marker_text(simplified, intention)
        return simplified

    def simplified_to_intentions(self, simplified):
        # print (simplified)
        directions = np.diff(simplified, axis = 0)
        # print (directions)
        theta = angle(directions)
        intentions = np.zeros(theta.shape).astype(np.int)
        idx = np.where(theta > Planner.min_angle)[0] 
        intentions[idx] = Planner.INTENTIONS_IDX[Planner.LEFT]
        idx = np.where(theta < -Planner.min_angle)[0] 
        intentions[idx] = Planner.INTENTIONS_IDX[Planner.RIGHT]

        return intentions
    
    def marker_text(self, simplified, intentions):
        marker_array = MarkerArray()
        pts = []
        idx = 0
        for idx, intent in enumerate(intentions):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.ns = "text"+str(idx)
            marker.scale.z = 1
            marker.scale.x = 0.7
            marker.scale.y = 1
            marker.color.a = 1
            marker.color.r = 0.1
            marker.color.g = 0.1
            marker.color.b = 1
            marker.pose.position.x  = simplified[idx+1][0]
            marker.pose.position.y  = simplified[idx+1][1]
            marker.pose.position.z = 0.5
            marker.text = Planner.INTENTIONS[intent]
            marker_array.markers.append(marker)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.SPHERE
            marker.ns = "idx"+str(idx)
            marker.scale.z = 0.5
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.color.a = 1
            marker.color.r = 0.1
            marker.color.g = 0.1
            marker.color.b = 1
            p = Point()
            p.x = (simplified[idx][0] + simplified[idx+1][0])/2
            p.y = (simplified[idx][1] + simplified[idx+1][1])/2
            pts.append([p.x, p.y])
            marker.pose.position  = p
            marker_array.markers.append(marker)

        # last marker
        idx += 1
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.ns = "idx"+str(idx)
        marker.scale.z = 0.5
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.color.a = 1
        marker.color.r = 0.1
        marker.color.g = 0.1
        marker.color.b = 1
        p = Point()
        p.x = (simplified[idx][0] + simplified[idx+1][0])/2
        p.y = (simplified[idx][1] + simplified[idx+1][1])/2
        pts.append([p.x, p.y])
        marker.pose.position  = p
        marker_array.markers.append(marker)
        self.vis_intention.publish(marker_array)

        self.simplified = simplified
        self.intentions = intentions
        self.pts = array(pts)

    def cb_pose(self, msg):
        # print ("cb_pose")
        self.robot.updatePosition(pose(msg.pose))

        if self.status != Planner.STATUS["invalid"]:
            self.track_rdp()
        else:
            # replan
            self.replan()

    def handle_orientation(self, dir2):
        k = tf_conversions.fromMsg(self.robot.position)
        r, p, y = k.M.GetRPY()
        #quat = self.robot.position.orientation
        #r, p, y = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w], axes='sxyz')

        dir1 = array([np.cos(y), np.sin(y)])
        # print ('dir1', dir1)
        # print ('dir2', dir2)
        sign = np.sign(np.cross(dir1, dir2))
        angle_diff = sign * np.arccos((dir1 * dir2).sum() / (
            np.sqrt((dir1 ** 2).sum() * (dir2 ** 2).sum())))
        # print ("angle_diff", angle_diff)
        intention = Planner.INTENTIONS_IDX[Planner.FORWARD]
        dist = np.linalg.norm(self.simplified[-1]-array([self.robot.position.position.x, self.robot.position.position.y]))
        if dist < 0.3:
            intention = Planner.INTENTIONS_IDX[Planner.STOP]
        if angle_diff > Planner.min_angle:
            intention = Planner.INTENTIONS_IDX[Planner.LEFT]
        elif angle_diff < -Planner.min_angle:
            intention = Planner.INTENTIONS_IDX[Planner.RIGHT]
        return intention

    def track_rdp(self):
        cur_pose = array([self.robot.position.position.x, self.robot.position.position.y])
        if self.simplified.shape[0] <= 2:
            self.pub_intention.publish(Planner.INTENTIONS[Planner.INTENTIONS_IDX[Planner.FORWARD]])
            return
        print ('simplifid', self.simplified, self.simplified.shape)
        print ('curpose', cur_pose)
        diff = self.simplified-cur_pose
        dist = np.linalg.norm(diff, axis=1)
        idx = np.argmin(dist)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.8
        marker.scale.y = 1
        marker.color.a = 1
        marker.color.r = 1
        marker.color.g = 0.1
        marker.color.b = 0.1

        intention = Planner.INTENTIONS_IDX[Planner.FORWARD]
        if idx == 0:
            self.status = Planner.STATUS["start"]
            p = Point()
            p.x = self.simplified[0][0]
            p.y = self.simplified[0][1]
            marker.points.append(p)

            p = Point()
            p.x = self.pts[0][0]
            p.y = self.pts[0][1]
            marker.points.append(p)
            intention = self.handle_orientation(self.pts[0]-self.simplified[0])
        elif idx == len(self.simplified)-1:
            self.status = Planner.STATUS["end"]
            p = Point()
            p.x = self.pts[idx-1][0]
            p.y = self.pts[idx-1][1]
            marker.points.append(p)

            p = Point()
            p.x = self.simplified[idx][0]
            p.y = self.simplified[idx][1]
            marker.points.append(p)
            intention = self.handle_orientation(self.simplified[idx]-self.pts[idx-1])
        else:
            self.status = Planner.STATUS["progress"]


            p = Point()
            p.x = self.pts[idx-1][0]
            p.y = self.pts[idx-1][1]
            marker.points.append(p)

            p = Point()
            p.x = self.simplified[idx][0]
            p.y = self.simplified[idx][1]
            marker.points.append(p)

            p = Point()
            p.x = self.pts[idx][0]
            p.y = self.pts[idx][1]
            marker.points.append(p)
            intention = self.intentions[idx-1]
        
        self.vis_tracker.publish(marker)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale.z = 2
        marker.scale.x = 0.7
        marker.scale.y = 1
        marker.color.a = 1
        marker.color.r = 0.1
        marker.color.g = 1
        marker.color.b = 0.1
        marker.pose = self.robot.position
        marker.text = Planner.INTENTIONS[intention]
        self.vis_current.publish(marker)
        print ('intention', Planner.INTENTIONS[intention])
        self.pub_intention.publish(Planner.INTENTIONS[intention])

    def cb_change_goal(self, msg):
        print ("call back goal")
        # delay 2 seconds to wait for synchronization
        time.sleep(2)

        self.robot.updateAssignedGoal(pose(msg.pose))
        self.status = Planner.STATUS['invalid']

    def replan(self):
        path = self.robot.makePlan()
        self.status=Planner.STATUS["invalid"]
        if path is not None:
            if len(path.plan.poses) > 4:
                simplified = self.path_to_rdp(path)
                self.vis.publish(self.marker_strip(simplified))
                self.robot.updateAssignedPosition(self.robot.position)
                self.status=Planner.STATUS["start"]

    def marker_strip(self, pts):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.POINTS
        marker.scale.x = 0.4
        marker.scale.y = 1
        marker.color.a = 1
        marker.color.r = 1
        marker.color.g = 0.1
        marker.color.b = 0.1

        for pos in pts:
            p = Point()
            p.x = pos[0]
            p.y = pos[1]
            marker.points.append(p)
        return marker

def main(with_move_base=False):
    rospy.init_node("intention_generator")
    planner = Planner(with_move_base)
    rospy.spin()

def debug():
    rospy.init_node("intention_generator")
    planner = Planner(False)
    import matplotlib.pyplot as plt
    a = np.array(
        [[  1.00000046 ,  1.80000047],
        [  5.01990103 ,  4.18867849],
        [  7.03415888 ,  9.44518186],
        [ 15.65028095,  13.93876266]]
    )
    x = a[:, 0]
    y = a[:, 1]
    intention = planner.simplified_to_intentions(a)
    plt.plot(x, y)
    plt.show()
    rospy.spin()

if __name__ == "__main__":
    fire.Fire(main)
    # debug()
