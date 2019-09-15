# Panda Controller

This is a set of nodes for simulate and control the Panda arm (Franka Emika)


## Table of Contents

* [Getting started](#getting-started)
  * [Dependencies](#dependencies)
  * [Building from source](#building-from-source)
  * [Execute](#execute)
* [Nodes](#nodes)
  * [load_scene](#load_scene)
* [VSCode](#vscode)
* [FAQ](#FAQ)
* [Author](#author)
* [License](#license)

---
## Getting Started

This package was written for ROS melodic running under Ubuntu 18.04
There is a script to install dependencies and build
~~~
git clone https://github.com/Envq/panda_controller.git

./build.sh
~~~


### Dependencies
- Install ROS [melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Build from source [libfranka](https://frankaemika.github.io/docs/installation_linux.html):
~~~
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

mkdir -p github

cd ~/github

git clone --recursive https://github.com/frankaemika/libfranka

mkdir libfranka/build

cd libfranka/build

cmake -DCMAKE_BUILD_TYPE=Release ..

cmake --build .
~~~

- Prepare workspace
~~~
mkdir -p ~/panda_ws/src

cd ~/panda_ws/src

git clone https://github.com/erdalpekel/panda_moveit_config.git

git clone --branch simulation https://github.com/erdalpekel/franka_ros.git

git clone https://github.com/Envq/panda_controller.git

cd ~/panda_ws

sudo apt-get install libboost-filesystem-dev clang-format

rosdep install --from-paths src --ignore-src -y --skip-keys libfranka

catkin config -j$(nproc) --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$HOME/github/libfranka/build

catkin init
~~~ 

---
### Building from source

For build:
~~~
catkin build
~~~

For setup the environment
~~~
cd

echo 'export LC_NUMERIC="en_US.UTF-8"' >> ~/.bashrc

echo 'source ~/panda_ws/devel/setup.bash' >> ~/.bashrc
~~~

---
### Execute
For lanch simulation:
~~~
roslaunch panda_controller simulation.launch
~~~

---
## Nodes
A brief description of what the nodes do

### **load_scene:**
![load_scene](screenshot/load_scene1.png?raw=true "load_scene")

This node loads in RViz a scene defined in a json file in scenes folder.

Json file must be in this form:
- "name": String
- "objects": List
    - "id": String
    - "type": ["box", "sphere", "cylinder", "cone"]
    - "color": List
        - r: float
        - g: float
        - b: float
        - a: float
    - "dimensions": List
        - x: double
        - y: double
        - z: double
        - r: double
        - h: double
    - "position": List
        - x: double
        - y: double
        - z: double
    - "orientation": List
        - w: double
        - x: double
        - z: double
        - z: double

Note:
- **objects**: it's the list of collision object
- **id**: it's the unique string that identify object
- **type**: it's the type of shape_msgs::SolidPrimitive
- **color**: it's defined in rgba format with double
- **dimensions**: 
    - for Box the values are the carthesian coordinates x, y, z
    - for Sphere the only value is radius
    - for cylinder the  values are radius and height
    - for cone the values are radius and height
- **position**: it's defined in carthesian coordinata and it represents the center of the solid
- **orientation**: it's a quaternion

---
## VSCode
The following commands are available:
- **build** : build panda_control package
- **simulation**: launch simulation
- **build&run**: build panda_control and run the actual developing node
- **load_scene**: run the node that load the scene
- **format**: format sources with clang-format
- **build_all**: build all packages
- **clean_all**: clean all packages
- **config_all**: config catkin

---
## FAQ:
#### Gazebo Rest error: 
You need to change in ~/.ignition/fuel/config.yaml the url: https://api.ignitionfuel.org to: https://api.ignitionrobotics.org

---
## Author

**Enrico Sgarbanti** [@**Envq**](https://github.com/Envq)

---
## License

This project is licensed under the GPL v3 License - see the [LICENSE.md](LICENSE.md) file for details