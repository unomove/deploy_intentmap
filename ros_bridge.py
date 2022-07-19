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
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from pyquaternion import Quaternion
import cv2
import time
import json

# using ZMQ, since we will have remote subscriber
os.environ["ZMQ"] = "1"
messaging.context = messaging.Context()
config = json.load(open("config.json"))
ADDR=config['client']
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
from lib.graph import load_graph, add_source_target, draw, dist, uv_to_position, smexit_to_dict
import networkx as nx
import json

# ******************** GLOBAL VARIBLES *************** #
global pub_initial
global pub_goal
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

pm = messaging.PubMaster(['pose', 'liveLocationKalman', 'state'])

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
    global pose, pm, milestone, G, path
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

      if radius < node['margin']*floorplan['resolution']:
        # change state
        path = nx.shortest_path(G, source=milestone, target='target', weight='weight')
        # info_dat.info.path = f"Remaining milestones to go: {'-->'.join(path[1:])}"
        print(f"Remaining milestones to go: {'-->'.join(path[1:])}")
        # new path generated
        if node['floorplanId'] != "outdoor":
          pos = uv_to_position(floorplans, node['floorplanId'], node['position']['x'], node['position']['y'])
          publish_initial_position(pos[0], pos[1], yaw)

        milestone = path[1]
        handle_milestone(G, milestone)
        node = G.nodes[milestone]

        # update transit if change map
        if last_floorplan != node["floorplanId"]:
          if node['floorplanId'] != "outdoor":
            pos = uv_to_position(floorplans, node['floorplanId'], node['position']['x'], node['position']['y'])
            if node['floorplanId'] == "as6_l1":
              # skew of stairs
              publish_initial_position(pos[0], pos[1], yaw+0.261799)
            else:
              publish_initial_position(pos[0], pos[1], yaw)
          milestone = path[2]
          handle_milestone(G, milestone)
          node = G.nodes[milestone]
        # update reset exit
        state_msg = messaging.new_message("state")
        state_msg.state=node
        pm.send("state", state_msg)
        # draw(G, path)
      # pm.send("info", info_dat)
    else:
      # delegate to GPS module
      pass

# deprecated, used for gridmap
def floorplan_thread():
  global G, milestone, current_floorplan_id, path
  # load graph, graph logic here.
  G = load_graph()

  publisher = rospy.Publisher(FLOORPLAN_TOPIC, Image, queue_size=10)

  sm = messaging.SubMaster(['floorplan', 'source', 'target'], addr=ADDR)
  while True:
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
      print ("publish floorplan image")
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
      path = nx.shortest_path(G, source=source['id'], target=target['id'], weight='weight') # this path is the shortest transit in map graph
      print ("Subgoal milestones to go: ", path[1:])
      # info_dat.info.path = f"Remaining milestones to go: {'-->'.join(path[1:])}"
      milestone = path[1]
      handle_milestone(G, milestone)
      # publish initial pose from source
      node = G.nodes["source"]
      if node['floorplanId'] != "outdoor":
        pos = uv_to_position(floorplans, node['floorplanId'], node['position']['x'], node['position']['y'])
        # initial calibration offset.
        publish_initial_position(pos[0], pos[1], pi/2+0.04)
      # draw(G, path)
    # if sm.updated['state']:
    #   draw(G, path)

# ****************** Test Use ***************** #
# # in publisher
# pm = messaging.PubMaster(['liveLocationKalman'])
# dat = messaging.new_message('liveLocationKalman')
# gps_dat.liveLocationKalman = {
#   "gpsWeek":0,
#   "gpsTimeOfWeek":0,
#   "status":"uninitialized",
#   "unixTimestampMillis":1632948964999,
#   "inputsOK":True,
#   "gpsOK":False,
#   "velocityCalibrated":{
#     "value" : [29.00776253842281, -0.10930662366976479, 0.28999936306154739],
#     "std" : [0.22529312857341197, 0.34050586122310345, 0.19542205107799951],
#     "valid" : True },
#   "calibratedOrientationNED" : {
#     "value" : [0.00011318853898166816, -0.083885195422878811, -1.1017716045869208],
#     "std" : [nan, nan, nan],
#     "valid" : True }
# }

def update_gps_dat(lat, lon, alt, ts):
  global gps_dat
  gps_dat.liveLocationKalman.unixTimestampMillis = ts
  gps_dat.liveLocationKalman.positionGeodetic = {
    "value" : [lat, lon, alt],
    "std" : [nan, nan, nan],
    "valid" : True
  }
  gps_dat.liveLocationKalman.gpsOK = True
  gps_dat.liveLocationKalman.status = "valid"

# def send_once(pm):
#   global dat
#   pm.send('liveLocationKalman', dat)
#   dat.clear_write_flag()

# class RepeatTimer(threading.Timer):
#   def run(self):
#     while not self.finished.wait(self.interval):
#       self.function(*self.args,**self.kwargs)
#       print(' ')

def bridge_gps(msg):
  # global gps_dat
  gps_dat = messaging.new_message('liveLocationKalman')
  stamp = int(msg.header.stamp.to_sec() * 10e3)
  lat = msg.latitude
  lon = msg.longitude
  alt = msg.altitude
  # update_gps_dat(lat, lon, alt, stamp)
  gps_dat.liveLocationKalman.unixTimestampMillis = stamp
  gps_dat.liveLocationKalman.positionGeodetic = {
    "value" : [lat, lon, alt],
    "std" : [nan, nan, nan],
    "valid" : True
  }
  gps_dat.liveLocationKalman.gpsOK = True
  gps_dat.liveLocationKalman.status = "valid"
  print ("here")
  pm.send('liveLocationKalman', gps_dat)

def main():
  global pub_initial, pub_goal
  rospy.init_node("ros_bridge")
  rospy.Rate(30)
  sub = rospy.Subscriber('/pose_estimation', PoseWithCovarianceStamped, callback)
  pub_initial = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
  pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
  #rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, bridge_gps)

  # timer = RepeatTimer(0.1,send_once,[pm])
  # timer.start() #recalling run
  # print('Threading started')

  # floorplan thread
  t1 = threading.Thread(target=floorplan_thread)
  t1.start()
  # python_astar()

if __name__ == '__main__':
  main()
