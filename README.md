## Step-by-Step Setup

### Clone the Warehouse Simulation (Docker Environment)
Clone the pre-built warehouse Docker setup:

```bash
git clone https://github.com/CollaborativeRoboticsLab/industrial-robots-and-systems-world.git
cd industrial-robots-and-systems-world

```
### Run the Simulation Container

Before starting the containers, allow Docker to access your display for GUI apps (so RViz and the warehouse simulator can open):

```bash
xhost +local:root
```
Then start the environment:
```bash
docker compose up
```
Two interfaces will open automatically:

-  Warehouse Simulation (Gazebo): the robot’s world view
-  RViz (MoveIt Arm Interface): arm planning and visualization
Keep this terminal running.

### Build Your ROS 2 Workspace Locally
Open a new terminal on your host and build your workspace:
```bash
cd ~/irslab_ws
colcon build
source install_local/setup.bash   # or source install/setup.bash
```
### Set Cyclone DDS (Middleware)
Run this once in every terminal that launches ROS 2 nodes:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
### Why?
ROS 2 uses DDS for node communication.
The default (Fast-DDS) can cause lag or timeouts with Nav2 + MoveIt.
CycloneDDS ensures smoother message exchange between:
-  Nav2 (navigation)
-  MoveIt action servers (arm)
-  Nodes outside Docker
Without this, the arm can become laggy or even crash after a while.

### Launch Navigation (Nav2 Stack)
In the same terminal (after exporting the variable):
```bash
ros2 launch hand_solo_virtual_nav nav_launch.py
```
This starts:
-  AMCL (localization)
-  Map Server + Planner + Controller
-  Behavior Tree Navigator

### Open the OpenPLC Dashboard
Open in your browser:
```bash
http://localhost:8080/dashboard
```
Then: 
1. Upload your PLC conveyor program .st file(from Conveyor_Data/).
2. Click Start PLC Runtime.
Inside the warehouse window:
-  Press P → open HMI → click Start PLC
-  Press R → select Autonomous Mode
The PLC logic will now control the conveyor and sensors.

### Run the Pick-and-Place Node
Open another new terminal and run:
```bash
cd ~/irslab_ws
colcon build
source install_local/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run hand_solo_arm hs_pick_place
```
The robot will:
-  Navigate between pick & place waypoints,
-  Use its arm to pick, carry, and drop objects.















