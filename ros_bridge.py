from math import nan, pi
import math
import cereal.messaging as messaging
import threading
import os
import sys
# # ros related package
import rospy
from sensor_msgs.msg import NavSatFix, Image
from nav_msgs.srv import GetMap, GetMapResponse
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion, PoseStamped, Pose2D
from pyquaternion import Quaternion
from std_msgs.msg import String
import tf
import cv2
import time
import json
import numpy as np

# global text
INPLACE_LEFT='inplace_left'
INPLACE_RIGHT='inplace_right'
FORWARD='forward'
BACKWARD='backward'
STOP='stop'
LEFT='left'
RIGHT='right'

# using ZMQ, since we will have remote subscriber
os.environ["ZMQ"] = "1"
messaging.context = messaging.Context()
config = json.load(open("config.json"))
ADDR=config['client']
checkpoints=config['checkpoints']
print (f"Client ADDR {ADDR}")

# # ros topics
FLOORPLAN_TOPIC="/floorplan/image"
ASSEST="assets/floorplans"

# deprecated, exploration test only
# import matplotlib.pyplot as plt
# from astar.gridmap import OccupancyGridMap
# from astar.a_star import a_star
# from astar.utils import plot_path

# import map graph
from lib.graph import load_graph, add_source_target, draw, dist, uv_to_position, smexit_to_dict, plan_with_checkpoints
import networkx as nx
import json

# ******************** GLOBAL VARIBLES *************** #
global cnt
global headingDeg
cnt = 0
headingDeg = 0
global pub_initial
global pub_goal
global pub_intention
global pub_gps_path
global pose # robot pose
global G # map graph
global milestone# global milestone in map graph
global path #global path
milestone='source'

# pose_dat = messaging.new_message("pose")
###************************************####
###This is the wrong use of global message with clear_write_flag() ####
###It will continuously increases the bandwidth but I don't know why###
# gps_dat = messaging.new_message('liveLocationKalman')

# load floorplans
# some JSON:
x =  open('map.json')
# parse x:
json_message = json.load(x)
floorplans = {}
for floorplan in json_message:
  floorplans[floorplan['id']] = floorplan

pm = messaging.PubMaster(['pose', 'gpsLocationExternal', 'state', "info"])

def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)

  # return roll_x, pitch_y, yaw_z # in radians
  return yaw_z # in radians

def __yaw_to_quat(yaw):
  """
  Computing corresponding quaternion q to angle yaw [rad]
  :param yaw
  :return: q
  """
  q = Quaternion(axis=[0, 0, 1], angle=yaw)
  return q.elements

def publish_goal(x, y, theta):
  global pub_goal
  goal = PoseStamped()

  goal.header.stamp = rospy.Time.now()
  goal.header.frame_id = "map"

  goal.pose.position.x = x
  goal.pose.position.y = y

  quaternion = __yaw_to_quat(theta)
  goal.pose.orientation.x = quaternion[1]
  goal.pose.orientation.y = quaternion[2]
  goal.pose.orientation.z = quaternion[3]
  goal.pose.orientation.w = quaternion[0]

  # print ("goal", goal)
  pub_goal.publish(goal)

def publish_initial_position(x, y, theta):
  global pub_initial
  """
  Publishing new initial position (x, y, theta) --> for localization
  :param x x-position of the robot
  :param y y-position of the robot
  :param theta theta-position of the robot
  """
  initpose = PoseWithCovarianceStamped()
  initpose.header.stamp = rospy.get_rostime()
  initpose.header.frame_id = "map"
  initpose.pose.pose.position.x = x
  initpose.pose.pose.position.y = y
  quaternion = __yaw_to_quat(theta)

  initpose.pose.pose.orientation.w = quaternion[0]
  initpose.pose.pose.orientation.x = quaternion[1]
  initpose.pose.pose.orientation.y = quaternion[2]
  initpose.pose.pose.orientation.z = quaternion[3]
  print ("initial", initpose)
  pub_initial.publish(initpose)
  return

# Deprecated
# def python_astar():
#   sm = messaging.SubMaster(['floorplan'])
#   while True:
#     sm.update()
#     if sm.updated['floorplan']:
#       # load the map
#       gmap = OccupancyGridMap.from_png(os.path.join(ASSEST, sm['floorplan'].name), sm['floorplan'].resolution)

#       print ("resolution", sm['floorplan'].resolution)
#       # # set a start and an end node (in meters)
#       start_node = (525.0*sm['floorplan'].resolution, 300.0*sm['floorplan'].resolution)
#       goal_node = (152.0*sm['floorplan'].resolution, 490.0*sm['floorplan'].resolution)

#       # # run A*
#       path, path_px = a_star(start_node, goal_node, gmap, movement='8N')

#       gmap.plot()

#       if path:
#         # plot resulting path in pixels over the map
#         plot_path(path_px)
#       else:
#         print('Goal is not reachable')

#       # plot start and goal points over the map (in pixels)
#       start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
#       goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

#       plt.plot(start_node_px[0], start_node_px[1], 'ro')
#       plt.plot(goal_node_px[0], goal_node_px[1], 'go')

#       plt.show()


def handle_milestone(G, milestone):
  node = G.nodes[milestone]
  time.sleep(0.3)
  if node['floorplanId'] != "outdoor":
    pos = uv_to_position(floorplans, node['floorplanId'], node['position']['x'], node['position']['y'])
    publish_goal(pos[0], pos[1], 0)
  else:
    pass
    # set gps goal
    # handled by client part
    # coords = (node['gps']['latitude'], node['gps']['longitude'])
    # print (f"destinations coords: lat {coords[0]} lon {coords[1]}")
    # dest = {"latitude": float(coords[0]), "longitude": float(coords[1])}
    # Params().put("NavDestination", json.dumps(dest))

def callback(msg):
    global pose, pm, milestone, G, path, checkpoints, cnt
    cnt += 1
    pose = msg.pose.pose.position
    quat = msg.pose.pose.orientation
    yaw = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
    # print (pose, math.degrees(yaw))
    pose_dat = messaging.new_message("pose")
    pose_dat.pose = {
      'x': pose.x,
      'y': pose.y,
      'theta': yaw
    }
    pm.send('pose', pose_dat)

    # handle map graph state change
    print ("current heading milestone", milestone)
    node = G.nodes[milestone]

    print ('milestone', milestone, 'node', node)
    if node['floorplanId'] != "outdoor":
      last_floorplan = node['floorplanId']
      pos = uv_to_position(floorplans, node['floorplanId'], node['position']['x'], node['position']['y'])
      # pos is already in real world, no need to multiply resolution. so pass 1.0 here.
      radius = dist(pos, (pose.x, pose.y), 1.0)
      print (f"floorplan {node['floorplanId']} distance to {milestone} {radius} m")

      # info_dat.info = {
      #   "msg" : f'current heading milestone {milestone}. distance to milestone {radius} m',
      #   "path" : f"Remaining milestones to go: {'-->'.join(path[1:])}"
      # }
      # info_dat.info.msg = f'current heading milestone {milestone}. distance to milestone {radius} m'
      print (f"resolution: {floorplan['resolution']}")
      print(f"Remaining milestones to go: {'-->'.join(path[1:])}")

      info_dat = messaging.new_message('info')
      info_dat.info = {
        "msg" : f"current heading {milestone}. floorplan {node['floorplanId']}, distance to milestone {radius} m",
        "path" : f"Remaining milestones to go: {'-->'.join(path[1:])}"
      }
      

      if len(checkpoints) > 0 and milestone == checkpoints[0]:
        checkpoints=checkpoints[1:]
      if radius < node['margin']*floorplan['resolution']:
        # change state
        # path = nx.shortest_path(G, source=milestone, target='target', weight='weight')
        # update checkpoints
        path = plan_with_checkpoints(G, milestone, 'target', checkpoints)
        info_dat.info.path = f"Remaining milestones to go: {'-->'.join(path[1:])}"
        print(f"Remaining milestones to go: {'-->'.join(path[1:])}")
        # # new path generated
        if node['floorplanId'] != "outdoor":
          publish_initial_position(pose.x, pose.y, yaw)
        milestone = path[1]
        node = G.nodes[milestone]
        # update transit if change map
        if last_floorplan != node["floorplanId"]:
          if node['floorplanId'] != "outdoor":
            pos = uv_to_position(floorplans, node['floorplanId'], node['position']['x'], node['position']['y'])
            if node['floorplanId'] == "as6_l1":
              # skew of stairs
              publish_initial_position(pos[0], pos[1], yaw+0.261799)
              # publish_initial_position(pos[0], pos[1], np.pi/2+0.261799)
            elif node['floorplanId'] == "com3":
              publish_initial_position(pos[0], pos[1], -np.pi/2-1.29161)
            elif last_floorplan == "as6_l1" and node['floorplanId'] == "com1_l1":
              # publish_initial_position(pos[0], pos[1], yaw-0.24299)
              # publish_initial_position(pos[0], pos[1], yaw-0.271799)
              publish_initial_position(pos[0], pos[1], -np.pi/2-0.03)
            else:
              publish_initial_position(pos[0], pos[1], yaw)
          milestone = path[1]
          handle_milestone(G, milestone)
          node = G.nodes[milestone]
          path = path[1:]
        else:
          handle_milestone(G, milestone)
        # update reset exit
        state_msg = messaging.new_message("state")
        state_msg.state=node
        pm.send("state", state_msg)
        # draw(G, path)
      if cnt % 30 == 0:
        pm.send('info', info_dat)
      # pm.send("info", info_dat)
    else:
      # delegate to GPS module
      pass

def pose_to_StampedPose(pose, time, frame=None):
    ps = PoseStamped()
    ps.header.stamp = time
    ps.header.frame_id = frame
    ps.pose.position = Point(pose[0], pose[1], 0)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, pose[2])
    ps.pose.orientation = Quaternion(*quaternion)
    return ps

# deprecated, used for gridmap
def floorplan_thread():
  global G, milestone, current_floorplan_id, path, pub_gps_path, pub_intention
  # load graph, graph logic here.
  G = load_graph()

  publisher = rospy.Publisher(FLOORPLAN_TOPIC, Image, queue_size=10)
    
  ## Publish raw GPS path
  pub_gps_path = rospy.Publisher("raw_gps_path", Path, queue_size=1)

  sm = messaging.SubMaster(['floorplan', 'source', 'target', 'navRoute', "navInstruction"], addr=ADDR)
  while True:
    # print ("here")
    sm.update()
    if sm.updated['floorplan']:
      current_floorplan_id = sm['floorplan'].id
      # publish image
      img = cv2.imread(os.path.join(ASSEST, sm['floorplan'].name), cv2.IMREAD_UNCHANGED)
      rosimage = Image()
      if img.dtype.itemsize == 2:
        if len(img.shape) == 3:
            if img.shape[2] == 3:
                rosimage.encoding = 'bgr16'
            if img.shape[2] == 4:
                rosimage.encoding = 'bgra16'
        else:
            rosimage.encoding = 'mono16'
      if img.dtype.itemsize == 1:
        if len(img.shape) == 3:
            if img.shape[2] == 3:
                rosimage.encoding = 'bgr8'
            if img.shape[2] == 4:
                rosimage.encoding = 'bgra8'
        else:
            rosimage.encoding = 'mono8'
      rosimage.width = img.shape[1]
      rosimage.height = img.shape[0]
      rosimage.step = img.strides[0]
      rosimage.data = img.tobytes()
      rosimage.header.stamp = rospy.Time.now()
      rosimage.header.frame_id = 'map'
      print (f"publish floorplan image from {os.path.join(ASSEST, sm['floorplan'].name)}")
      publisher.publish(rosimage)
    if sm.updated['source'] and sm.updated['target']:
      # check nodes
      if "source" in G:
        G.remove_node("source")
      if "target" in G:
        G.remove_node("target")

      print ("before source", sm['source'])
      source = smexit_to_dict(sm['source'])
      target = smexit_to_dict(sm['target'])
      # print ("source", source)
      # print ("target", target)
      # # add new source and target
      add_source_target(G, source, target)
      # path = nx.shortest_path(G, source=source['id'], target=target['id'], weight='weight') # this path is the shortest transit in map graph
      path = plan_with_checkpoints(G, source['id'], target['id'], checkpoints)
      print ("Subgoal milestones to go: ", path[1:])
      # info_dat.info.path = f"Remaining milestones to go: {'-->'.join(path[1:])}"

      # publish initial pose from source
      node = G.nodes["source"]
      if node['floorplanId'] != "outdoor":
        pos = uv_to_position(floorplans, node['floorplanId'], node['position']['x'], node['position']['y'])
        # initial calibration offset.
        publish_initial_position(pos[0], pos[1], pi/2+0.04)

      milestone = path[1]
      handle_milestone(G, milestone)
    if sm.updated["navRoute"]:
      # publish gps path
      gps_path = Path()
      gps_path.header.frame_id = 'map'
      gps_path.header.stamp = rospy.get_rostime()
      print ("navRoute", len(sm['navRoute'].coordinates))
      for x in list(sm['navRoute'].coordinates):
        map_pos = [x.longitude, x.latitude]
        gps_path.poses.append(pose_to_StampedPose([map_pos[0], map_pos[1], 0], gps_path.header.stamp, 'map'))
      pub_gps_path.publish(gps_path)
    if sm.updated["navInstruction"]:
      if "left" in sm['navInstruction'].maneuverModifier:
        pub_intention.publish(LEFT)
      elif "right" in sm['navInstruction'].maneuverModifier:
        pub_intention.publish(RIGHT)
      else:
        pub_intention.publish(FORWARD)

def sim_gps():
  route = [[103.772798,1.296927],[103.772737,1.296821],[103.772672,1.296731],[103.772573,1.296623],[103.77242,1.296484],[103.772377,1.296445],[103.772367,1.296438],[103.772229,1.296334],[103.772094,1.296285],[103.772082,1.296283],[103.771993,1.296262],[103.771844,1.296221],[103.771712,1.2962],[103.771004,1.2962],[103.770905,1.296202],[103.7709,1.296154],[103.770889,1.296054],[103.770864,1.295876],[103.770801,1.295687],[103.770734,1.29551],[103.770651,1.295347],[103.770603,1.295183],[103.770574,1.295056],[103.770558,1.294957],[103.770548,1.294791],[103.770571,1.294593],[103.770612,1.294453],[103.770646,1.294339],[103.770712,1.294174],[103.770752,1.294105],[103.770907,1.293902],[103.771023,1.29379],[103.771147,1.293698],[103.771275,1.293622],[103.771431,1.29355],[103.771597,1.293505],[103.771758,1.293476],[103.771965,1.293453],[103.772113,1.293427],[103.772125,1.293481],[103.772167,1.293467],[103.772202,1.293611],[103.772296,1.293801],[103.772475,1.294024],[103.772644,1.294166],[103.772762,1.294286],[103.772977,1.294585],[103.772956,1.294589],[103.772892,1.294579],[103.772847,1.294561],[103.773049,1.294927],[103.773079,1.294943],[103.773117,1.294959],[103.773142,1.294964],[103.773252,1.295083]]
  for i in range(20):
    for pos in route:
      gps = messaging.new_message("gpsLocationExternal")
      gps.gpsLocationExternal.latitude = pos[1]
      gps.gpsLocationExternal.longitude= pos[0]
      gps.gpsLocationExternal.altitude = 40
      gps.gpsLocationExternal.timestamp = 000
      print ("gps sent")
      pm.send("gpsLocationExternal", gps)
      time.sleep(1)

def bridge_gps(msg):
  global headingDeg
  # global gps_dat
  gps = messaging.new_message("gpsLocationExternal")
  gps.gpsLocationExternal.latitude = msg.latitude
  gps.gpsLocationExternal.longitude= msg.longitude
  gps.gpsLocationExternal.altitude = msg.altitude
  gps.gpsLocationExternal.bearingDeg = headingDeg
  gps.gpsLocationExternal.timestamp = int(msg.header.stamp.to_sec() * 10e3)
  print ("gps sent, current heading:", headingDeg)
  pm.send("gpsLocationExternal", gps)

def cb_orientation(msg):
  global headingDeg
  (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
  # print ("headingDeg:", roll, pitch, yaw)
  headingDeg = math.degrees(yaw)

def main():
  global pub_initial, pub_goal, pub_gps_path, pub_intention
  rospy.init_node("ros_bridge")
  rospy.Rate(30)
  # sub = rospy.Subscriber('/pose_estimation', PoseWithCovarianceStamped, callback)
  pub_initial = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
  pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

  #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$# for outdoor gps module
  rospy.Subscriber("/mavros/global_position/global", NavSatFix, bridge_gps)
  rospy.Subscriber("/map_odometry", Odometry, cb_orientation)
  pub_intention = rospy.Publisher('/dlm_intention', String, queue_size=1)

  # floorplan thread
  t1 = threading.Thread(target=floorplan_thread)
  t1.start()

  # sim_gps()

if __name__ == '__main__':
  main()
