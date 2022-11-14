# This is the deployment package of intent_map
To run it properly, please config it properly.

In config.json, we define the server ip address and client ip address. Server and Client are connencted via P2P direct connection.

source and target are the starting point and target point of the navigation. Either source or target is an Exit type. "id", "floorplanId" and "type" fields must be given.
```
struct Exit{ # One specific Exit
    id @5 :Text; #global exclusive id 
    floorplanId @6 :Text; #floorplan id
    type @0 :ExitType; 
    margin @4 :UInt32; #margin pixels of one exit
    position @1 :Point2D; #position on floorplan
    gps @2 :Coordinate; #global position, if it connects outdoor
    connection @3 :Text; #direct connected exit id
    resolution @7 :Float32;
}
```
```
{
  "server":"192.168.1.249",
  "client":"192.168.1.136",
  "source":{
    "id": "source",
    "floorplanId": "com1_l1",
    "type": "indoor",
    "position": {
      "x": 764,
      "y": 72
    }
  },
  "target":{
    "id": "target",
    "type": "outdoor",
    "floorplanId": "outdoor",
    "gps": {
      "latitude" : 1.2953665728338317,
      "longitude": 103.77351743959628
    }
  }
}
```
map.json defines the floorplan connections. More details, see map.json.
### Running Steps
* In server side, run ros. 
```
roslaunch roslaunch/floorplan_to_gridmap.launch
# new terminal, run ros bridge
./dist_arm64/ros_bridge/ros_bridge
```

* In client side, run client and gui.
```
./run_gui.sh
# new terminal, run client
./dist_x86_64/client/client
```
## FAQ
### opencv issue on jetson
https://github.com/ros-perception/vision_opencv/issues/345

### Jetson AutoFanControl
https://github.com/Pyrestone/jetson-fan-ctl

### Ros Sync Time
https://www.layerstack.com/resources/tutorials/How-to-install-Network-Time-Sync-NTP-with-Chrony-on-Ubuntu20

### pipenv version (pipenv==2021.11.23)
