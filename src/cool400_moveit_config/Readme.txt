This was the package created to work with CHOMP planner. Unfortunately errors could not be resolved. 

The progress made with this package so far are :

1) added the chomp_planning_pipeline.launch.xml into launch directory

2)Made of copy of the file demo.launch (present inside launch directoy) and renamed it to file to demo_chomp.launch.

3)Open demo_chomp.launch and find the lines :

<include file="$(find jaco_moveit_config)/launch/move_group.launch">
  <arg name="allow_trajectory_execution" value="true"/>
  <arg name="fake_execution" value="true"/>
  <arg name="info" value="true"/>
  <arg name="debug" value="$(arg debug)"/>
</include>

and add <arg name="planner" value="chomp" /> before </include> so that it now looks like :

<include file="$(find jaco_moveit_config)/launch/move_group.launch">
  <arg name="allow_trajectory_execution" value="true"/>
  <arg name="fake_execution" value="true"/>
  <arg name="info" value="true"/>
  <arg name="debug" value="$(arg debug)"/>
  <arg name="planner" value="chomp" />
</include>

4) If you try to launch demo_chomp.launch at this stage it will throw an <arg> error. To solve it you have to add the following line
   to the move_group.launch file:

	<arg name="planner" default="ompl" />

  then change the line under <!-- Planning Functionality --> as follows:
	<arg name="pipeline" value="OMPL" /> to <arg name="pipeline" value="$(arg planner)" />

NOTE : There will be an unresolved error after this too. A fix couldnot be found yet.
