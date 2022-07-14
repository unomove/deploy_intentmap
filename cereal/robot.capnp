using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xd334fa223e0d3fd1;

struct Instruction {
    caption @0 :Text; # Human readable text descrption of current intention.

    enum Intention {
        none @0;
        left @1;
        right @2;
        forward @3;
        elevator @4;
    }
}

struct Point2D{
    x @0 :Float64;
    y @1 :Float64;
}

struct Pose{ # for initial pose 
    x @0 :Float64;
    y @1 :Float64;
    theta @2 :Float64;
}

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

struct Info{
    msg @0  :Text; #message information
    path @1 :Text; #path information
}

enum ExitType{
    stairs @0;
    door @1;
    outdoor @2;
    elevator @3;
    linkway @4;
    indoor @5; #for robot current position
}

struct Coordinate { # position on GPS
    latitude @0 :Float32;
    longitude @1 :Float32;
}   

struct Floorplan {
    name @0 :Text; # filename
    id @7: Text; # identity name
    width @1 :UInt32;
    height @2 :UInt32;
    resolution @3 :Float32; #map pixel resolution
    floor @4 :Int32 = 1; #floor level
    active @5 :Bool = true;
    exits @6 :List(Exit); #All the exits of the floorplan
}

