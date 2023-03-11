import cereal.messaging as messaging
import math
from math import nan, pi
import threading
import time
import json
import os
import networkx as nx
from lib.graph import load_graph, add_source_target, dist, uv_to_position, smexit_to_dict, draw, plan_with_checkpoints
from lib.params import Params # used for Params write for mapbox GUI
from easydict import EasyDict as edict
os.environ["ZMQ"] = "1"
messaging.context = messaging.Context()

config = json.load(open("config.json"))
# print (f"client config {config} \n")
# # ***************Global Variables(Will be updated with GUI in later version)******************#
HOST=config['server']
print (f"Server ADDR {HOST}")
# # source, target are exits
# # alpha version, will input the source and target in GUI later.
source=config['source']
target=config['target']
checkpoints=config["checkpoints"]
# source = {
#   "id": "source",
#   "floorplanId": "com1_l1",
#   "type": "indoor",
#   "position": {
#     "x": 764,
#     "y": 72
#   }
# }

# target = {
#   "id": "target",
#   "type": "outdoor",
#   "floorplanId": "outdoor",
#   "gps": {
#     "latitude" : 1.2953665728338317,
#     "longitude": 103.77351743959628
#   }
# }

global current_floorplan_id

# load floorplans
# some JSON:
x =  open('map.json')
# parse x:
json_message = json.load(x)
floorplans = {}
for floorplan in json_message:
  floorplans[floorplan['id']] = floorplan
# ********************** End Global Variables **************** #

def send_exit(pm, data):
  global current_floorplan_id
  global floorplans
  assert ("source" in data.keys())
  assert ("target" in data.keys())
  # print (data['source'])
  source_msg = messaging.new_message("source")
  source_msg.source = data["source"]

  target_msg = messaging.new_message("target")
  target_msg.target = data["target"]
 
  # send
  pm.send("source", source_msg)
  pm.send("target", target_msg)
  id = data["source"]['floorplanId']
  if id != "outdoor":
    floorplan_msg = messaging.new_message('floorplan')
    current_floorplan_id = id
    floorplan_msg.floorplan = floorplans[id]
    pm.send("floorplan", floorplan_msg)

class RepeatTimer(threading.Timer):  
  def run(self):  
    while not self.finished.wait(self.interval):  
      self.function(*self.args,**self.kwargs)  
      print(' \n')  

# in subscriber
sm_remote = messaging.SubMaster({"gpsLocationExternal", "pose", "state", "info"}, addr=HOST)
sm = messaging.SubMaster({"navInstruction", "navRoute"}) # local message
pm = messaging.PubMaster({"source", "target", "floorplan", "liveLocationKalman"})

data = {
  "source": source,
  "target": target
}
# timer = RepeatTimer(4,send_exit,[pm, data]) 
# timer.start()
# print ("send source and target frequently.")
for i in range(5):
  send_exit(pm, data) #send once only with one source and target
  time.sleep(1)

################################ Deprecated, due to the latency, it may cause difference of backend and client ############################
# maintain the graph in client as well to save bandwidth
# load graph, graph logic here.
# G = load_graph()
# print (source)
# source = smexit_to_dict(edict(source))
# target = smexit_to_dict(edict(target))
# add_source_target(G, source, target)

# def handle_milestone(G, milestone):
#   node = G.nodes[milestone]
#   if node['floorplanId'] == "outdoor":
#     # set gps goal
#     coords = (node['gps']['latitude'], node['gps']['longitude'])
#     print (f"destinations coords: lat {coords[0]} lon {coords[1]}")
#     dest = {"latitude": float(coords[0]), "longitude": float(coords[1])}
#     Params().put("NavDestination", json.dumps(dest))

# milestone='source' 
# # path = nx.shortest_path(G, source=source['id'], target=target['id'], weight='weight') 
# path = plan_with_checkpoints(G, source['id'], target['id'], checkpoints)
# # draw(G, path)
# milestone = path[1]
# print ("milestone", milestone)
# handle_milestone(G, milestone)

def main():
  while 1:
    global path, milestone, current_floorplan_id, checkpoints
    sm.update()
    sm_remote.update()

    if sm_remote.updated['info']:
      print (sm_remote['info'].msg)
      print (sm_remote['info'].path)
    
    if sm_remote.updated["state"]:
      print (sm_remote['state'])
      if sm_remote['state'].floorplanId != current_floorplan_id and sm_remote['state'].floorplanId != "outdoor":
        floorplan_msg = messaging.new_message('floorplan')
        current_floorplan_id = sm_remote['state'].floorplanId
        floorplan_msg.floorplan = floorplans[current_floorplan_id]
        # for i in range(4): # need to publish multiple times to make sure ros update the floorplan.
        print ("new floorplan", floorplan_msg)
        print (f"resolution: {floorplans[current_floorplan_id]['resolution']}")
        pm.send("floorplan", floorplan_msg)
    if sm_remote.updated['gpsLocationExternal']:
        gps_dat = messaging.new_message('liveLocationKalman')
        #TODO add Kalman filter here.
        stamp = sm_remote['gpsLocationExternal'].timestamp
        lat = sm_remote['gpsLocationExternal'].latitude
        lon = sm_remote['gpsLocationExternal'].longitude
        alt = sm_remote['gpsLocationExternal'].altitude
        gps_dat.liveLocationKalman = {
          "gpsWeek":0,
          "gpsTimeOfWeek":0,
          "status":"uninitialized",
          "unixTimestampMillis":1632948964999,
          "inputsOK":True,
          "gpsOK":False,
          "velocityCalibrated":{
            "value" : [1.00776253842281, -0.10930662366976479, 0.28999936306154739],
            "std" : [0.22529312857341197, 0.34050586122310345, 0.19542205107799951],
            "valid" : True },
          "calibratedOrientationNED" : {
            "value" : [0.00011318853898166816, -0.083885195422878811, math.radians(sm_remote['gpsLocationExternal'].bearingDeg)],
            "std" : [nan, nan, nan],
            "valid" : True }
        }
        # update_gps_dat(lat, lon, alt, stamp)
        gps_dat.liveLocationKalman.unixTimestampMillis = stamp
        gps_dat.liveLocationKalman.positionGeodetic = {
          "value" : [lat, lon, alt],
          "std" : [nan, nan, nan],
          "valid" : True
        }
        gps_dat.liveLocationKalman.gpsOK = True
        gps_dat.liveLocationKalman.status = "valid"
        print ("receive GPS", gps_dat)
        pm.send('liveLocationKalman', gps_dat)
    if sm.updated["navRoute"]:
      print (sm["navRoute"])
      print (sm["navInstruction"])
    # print(sm['liveLocationKalman'])
    print ("#####################################")

main()