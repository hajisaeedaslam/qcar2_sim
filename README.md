# QCar2 Simulation Project

This repository contains the ROS 2 Jazzy and Gazebo Harmonic simulation environment for the QCar2. 

## Prerequisites
* **OS:** Ubuntu 24.04 (via WSL2)
* **ROS 2 Distribution:** Jazzy Jalisco
* **Simulator:** Gazebo Harmonic
* **Python:** 3.12+

---

## Setup and Installation

Clone the Repository
```bash
git clone -b main [https://github.com/hajisaeedaslam/qcar2_sim.git](https://github.com/hajisaeedaslam/qcar2_sim.git)
cd ~/qcar2_sim
```
Fix Hardcoded Paths
Currently, URDF/Xacro meshes use absolute paths. Ensure you update these to match your local user directory:

Search for old paths
```bash
grep -r "home/OLD_USERNAME" src/urdf_representations
```
Update paths in the relevant URDF/Xacro files to:
/home/[YOUR_USERNAME]/qcar2_sim/...

Install Dependencies
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
Building the Project
```bash
cd ~/qcar2_sim
colcon build --symlink-install
```
## Running the Simulation
Always source the workspace before launching:
```bash
source install/setup.bash
ros2 launch qcar2 simulation.launch.py
```

## Github Flows
Creating a New Feature Branch
```bash
git checkout -b branchName
```
Pushing changes safely
```bash
git add .
git commit -m "Description of changes"
git push origin branchName
```

Reverting if things break
```bash
git checkout main
```
