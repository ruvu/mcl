<!--
Copyright 2021 RUVU Robotics B.V.
-->

# MCL - Monte Carlo Localization
MCL is a probabilistic localization system for a robot moving in 2D.
It implements the Monte Carlo localization approach (as described by Dieter Fox), which uses a particle filter to track the pose of a robot against a known map.
Its design is based on [AMCL](http://wiki.ros.org/amcl), but is heaviliy improved due to modern C++ features.
Most of the AMCL parameters are still compatible with MCL.

The main addition that MCL brings is the addition of various landmark sensor models that can be used to localize on landmark maps.
Landmarks can be various things, for example lidar reflectors, QR markers or ground patterns.
Landmarks can be used alone, or in combination with lidar.
The codebase of MCL is also much easier to extend for resampling algorithms, adaptive particle algorithms or other sensor models.

## Algorithms
Many of the algorithms and their parameters are well-described in the book Probabilistic Robotics, by Thrun, Burgard, and Fox.
The user is advised to check there for more detail.
In particular, we use the following algorithms from that book: sample_motion_model_odometry, beam_range_finder_model, likelihood_field_range_finder_model, Augmented_MCL, KLD_Sampling_MCL and landmark_model_known_correspondence.

As currently implemented, this node works with laser scans and landmark measurments.
It is designed to be extended with other sensors.

## Nodes

### node

`node` is the main executable for running MCL. Takes in various measurments and outputs a position estimate.

#### Subscribed Topics
- `scan` Laser scan measurements
- `landmarks` Landmark measurements
- `map` Used to retrieve the map used for laser-based localization.
- `landmark_list` Used to retrieve the landmark map used for landmark-based localization.
- `tf` Used to calculate odometry movements and the distance of each sensor with respect to the center of the robot
- `initialpose` Mean and covariance with which to (re-)initialize the particle filter.

#### Published Topics
- `tf` Publishes the transform from odom (which can be remapped via the ~odom_frame_id parameter) to map.
- `~pose` Robot's estimated pose in the map, with covariance
- `~cloud` The set of pose estimates being maintained by the filter
- `~count` The number of pose estimates being maintained by the filter

#### Parameters
Most of the parameters are the same as AMCL for backwards compatibility.
For new parameters (for for example the gaussian landmark model) they are described with dynamic reconfigure.

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
