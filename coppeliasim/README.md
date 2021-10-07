# CoppeliaSimSLAM
Read the wiki at [kanishkaganguly/GettingStarted](https://github.com/kanishkaganguly/GettingStarted/wiki) for more information on the Docker setup.

### Setup
1. Make the following directory structure: `/<dir>/CoppeliaSimProject/src/`
2. Clone the repo using `git clone git@github.com:kanishkaganguly/CoppeliaSimSLAM.git` into the `src` folder created previously.
3. Download and extract the [CoppeliaSim application](https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04.tar.xz) into the repository directory. Rename it to `coppeliasim`.
4. Merge the `models/components/sensors` directory into the `coppeliasim/models/components/sensors` directory. This will add two new LIDAR sensors into the simulator, with ROS support.
5. Copy the `programming/ros_packages/sim_ros_interface` directory from the `coppeliasim` directory into the `CoppeliaSimProject/src` directory. You should now have two packages in the `CoppeliaSimProject/src` directory.
6.
```bash
cd <repo>/docker
chmod +x docker_setup.sh
./docker_setup.sh build
./docker_setup.sh run
./docker_setup.sh exec
```

You should now have a bash instance from your container in your terminal.
All following commands are to be run inside the container's bash instance.

### Getting Started
1. Build and source your workspace
```bash
cd workspace
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```
2. Next, open up 6 instances of bash inside the container, using `./docker_setup.sh exec`. It's best to use something like `tmux` or `Terminator` for this.
3. In any one instance, run `roscore`. In another instance, run `rosparam set /use_sim_time true`, this allows the simulator to control the ROS clock.
4. Run the simulator:
```bash
cd workspace/src/CoppeliaSimSLAM/coppeliasim
./coppeliaSim.sh $(rospack find coppelia_sim_slam)/vrep_save/nav.ttt
```
5. Mapping Mode
```bash
roslaunch coppelia_sim_slam robot_mapping.launch mapping:=true navigation:=false
```
This launches the SLAM mapping node. You can now use the `rqt_robot_steering` menu to move the robot around, and create the required map. It will gradually populate in RViz as the map is built.
Once the mapping is complete, save the map using
`rosrun map_server map_saver -f $(rospack find coppelia_sim_slam)/vrep_save/maze_map`
6. Navigation Mode
```bash
roslaunch coppelia_sim_slam robot_mapping.launch mapping:=false navigation:=true
```
This launches the `map_server`, `amcl` and `move_base` nodes with their respective parameters. The robot should now be able to plan a path and navigate to it, given a `Pose` goal through RViz.