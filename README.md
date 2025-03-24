### SLAMVDB - the EDI SLAM Vector Data Base (pre-release version)

A collection of ROS2 software packages for constructing large-scale open-set semantic maps using LiDAR and RGB data. Not yet feature complete! This is the WIP conversion of the outdoor mapping technology demonstrator described in [the RoLISe T4.1 project report](http://edi.lv/RoLISe_T4_1) into a reusable open-source software package.

The reason it's called "mvdb" and not "slamvdb" in the code is that we are exploring uses beyond robotics - at which point it will become just the "map vector data base". Since the repository is not yet feature complete, major breaking internal and external API changes are to be expected.

Currently only tested on Ubuntu 22.04LTS running ROS2 humble.

TODO:
* GNSS constraints (present in prototype, not ported yet);
* IMU-based gravity alignment
* Publication, quantiative assessment, utilities related to validation and testing

Included packages:
* *mvdb* - the core map integration software, written in C++. Handle projection, integration, retrieval of semantic data in large-scale map where retroactive drift correction due to loop closures is possible. We ship our own tracker and mapper but in principle this system is SLAM provider agnostic, so long as the relevant interfaces are provided
* *mvdb_interface* - the ROS2 messages and services required for integration between the packages
* *mvdb_py* - shared Python dependencies for Python packages
* *mvdb_tracker* - simple LiDAR ICP odometry node written in Python using Open3D. TODO: gravity alignment
* *mvdb_mapper* - simple loop closing node written in Python using Open3D, with odometry and *ScanContext* based correspondence detection. **Note:** We provide our own implementation of the rotational version of *ScanContext* here, as a general policy of avoiding external dependencies wherever possible. TODO: GNSS constraints
* *langseg* - a fork of [LangSeg](https://github.com/isl-org/lang-seg/tree/65c0a0978fa2e562cda695afed1554c63ec35cb) with only the code relevant in image segmentation inference retained. TODO: refactor into proper ROS2 package, clean up the implementation.


## Build

Clone the repository, and recurse submodules to pull *langseg*:

```{bash}
git clone --recurse-submodules https://github.com/edi-administrator/SLAMVDB.git
```

After that, install the required dependencies [Section: TODO]

All other packages depend on *mvdb_interface*, so on a fresh build it needs to be built first and sourced

```{bash}
colcon build --packages-select mvdb_interface
source install/setup.bash
```

Once this is done, the workspace can be built by simply running

```{bash}
colcon build
```

For verbose builds and debugging, run

```{bash}
colcon build --event-handlers console_cohesion+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## License
The code directly contained in this repository is distributed under an Apache v2.0 license. On a per-package basis:
* *mvdb*, *mvdb_interface*, *mvdb_py*, *mvdb_tracker*, *mvdb_mapper* - Apache v2.0
* *langseg* - MIT