# drone_navigation
A navigation package for drone.
* This repo required two launch files, <navigation_mananger.launch> and <PATH_PLANNERS>
* Launch <navigation_mananger.launch>, which manage the waypoints that drone should follows:
```
roslaunch drone_navigation navigation_mananger.launch
```
* And the <PATH_PLANNERS> are responsible for generate waypoints while they were activiated bt behaivor tree as action nodes, we take <waypoint_search.launch> as an example:
```
roslaunch drone_navigation waypoint_search.launch
```
## Params
* There are many params in <PATH_PLANNERS>, below are the explanations:
  * <node_name>: It will equal to name in behavior tree.
  * <mode_name>: There are four modes: waypoint_shifting (move without rotation) / waypoint_rotate (move only with rotation) / waypoint_heading (Rotate toward to the goal first, than move toward it) / waypoint_pose (totally follow the pose of goal) .
  * <origin>: The are two modes: drone (position relative to current drone state) / map (position relative to origin tf) .
  * <script_enable>: If script_enable is true, follow waypoints in script file, if script_enable is false, follow /move_base_simple/goal in rviz.
  * <cruise_height>: Cruise height will affect drone's height when script_enable is false (because /move_base_simple/goal does not have height information) .
  * <distance_margin>: This param effect how precise the drone's position is.
  * <heading_margin>: This param effect how precise the drone's position is.
  * <script_file>: If the param <script_enable> is true, drone will follow the waypoints inside this config files. (Notice: The format inside the script follows this rule: x, y, z, rotation)
