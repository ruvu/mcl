<!--
Copyright 2021 RUVU Robotics B.V.
-->

# MCL - Monte Carlo Localization
MCL is a probabilistic localization system for a robot moving in 2D.
It implements the Monte Carlo localization approach (as described by Dieter Fox), which uses a particle filter to track the pose of a robot against a known map.
See also http://wiki.ros.org/amcl

## Benchmarking

See also [BENCHMARK.md](./BENCHMARK.md) for a guide on how to compare different settings or localization algorithms.

## Design

The design of the MCL is split into a core library (mcl) and a ROS wrapper. This makes it easy in the future to migrate to ROS2 or other platforms.

```plantuml
class Node {
    tf2_ros::MessageFilter
    dynamic_reconfigure::Server
}
class Offline {
    rosbag::BagPlayer
}

class MclRos {
    tf lookup laser in base_link
    tf lookup base_link in odom
    translate between ROS and MCL datatypes
    publish pose & tf
}

class Mcl {
    ROS independent
}

Node --o MclRos
Offline --o MclRos
MclRos --o Mcl
```
