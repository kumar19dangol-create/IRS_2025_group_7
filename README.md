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
