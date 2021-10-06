# CoppeliaSimSLAM
Read the wiki at [kanishkaganguly/GettingStarted](https://github.com/kanishkaganguly/GettingStarted/wiki) for more information on the Docker setup.

### Setup
1. Make the following directory structure: `/<dir>/src/`
2. Clone the repo using `git clone git@github.com:kanishkaganguly/CoppeliaSimSLAM.git` into the `src` folder created previously.
3. Download and extract the [CoppeliaSim application](https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04.tar.xz) into the repository directory. Rename it to `coppeliasim`.
4. Merge the `models/components/sensors` directory into the `coppeliasim/models/components/sensors` directory. This will add two new LIDAR sensors into the simulator, with ROS support.
5.
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
./coppeliaSim.sh
```
5. Load the `nav.ttt` scene from the `vrep_save` directory in the repository.
6. Each of the following commands should be run in a separate instance:
```bash
rosrun rqt_robot_steering rqt_robot_steering
rosrun gmapping slam_gmapping scan:=laser
rosrun rviz rviz
```
6. Load the `mapping_rviz.rviz` configuration file into rviz, once opened.

You can now use the `rqt_robot_steering` menu to move the robot around, and create the required map. It will gradually populate in RViz as the map is built.
