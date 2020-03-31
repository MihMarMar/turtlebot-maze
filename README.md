A simulation of a Turtlebot3 Waffle in a maze. The robot navigates the maze and goes to waypoints to scan qr codes. 

In order to run it 
0. Install dependencies. This project depends on libzbar0 and pyzbar. In order to install them

`sudo apt install -y libzbar0` 

`pip install pyzbar`


1. In a new terminal from the repository's directory run

`source devel/setup.sh` 

`roslaunch tb_navigation maze_nav.launch`


2. In a new terminal run

`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/turtlekin_ws/bettermap.yaml`

Note: you may need to change the path to the map file depending on your repository location. 

3. In `src/tb_navigation/src/send_waypoint.py` change the path so that it corresponds with your system

4. In rViz put the estimated robot position so that it is in sync with Gazeebo

5. In a new terminal

`source devel/setup.sh` 

`roslaunch tb_navigation maze_nav.launch` 

The robot will now start to navigate around the maze and scan QR codes. 

Demo video is available in [YouTube](https://www.youtube.com/watch?v=nKiNXyjyfzs)
