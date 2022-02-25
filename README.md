# biomed_drone
Simulation currently allows user to input an address to set a waypoint for a drone to fly to.
Clone the PX4 repo and make sure mavros is installed
1. Start the PX4 sitl: $ make px4_sitl gazebo
*Can create a custom world file in the px4 sitl worlds directory for drone to spawn at a certain location. If this is the case, run $ make px4_sitl gazebo___world
2. Start the mavros node: $ roslaunch mavros px4.launch fcu_url:="udp://:14540@YOUR_IP_ADDRESS:14557"
3. Start the drone_commander.py program. This will clear currently uploaded waypoints upon entering a new address then upload the new waypoint entered. Manual takeoff in QGroundControl is still necessary. $ python3 drone_commander.py
4. In QGroundControl, you may need to set some parameters to get the simulation working
 - COM_RC_IN_MODE 1
