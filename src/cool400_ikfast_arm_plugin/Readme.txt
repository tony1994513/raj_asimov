This package contains the IKfast plugin created for cool400 robot.

The steps involved in this process are :

1) converting URDF into collada file.(No rounding was required since the plugin was generated within minutes using this collada file itself)

2)The following parameters were used to generate the IK solver:

	iktype=transform6d (since the robot has 6 DOF)
	baselink=0
	eelink=6
	savefile=ik.cpp

3)The next process was to create the plugin, the following were the parameters used here:

	myrobot_name - cool400_description (This was in the tag <robot name> in the URDF)
	planning_group_name - arm (as referenced in your SRDF and kinematics.yaml)
	moveit_ik_plugin_pkg - cool400_ikfast_arm_plugin
	ikfast_output_path - ~/catkin_ws/src/cool_ikfast_arm_plugin/src

4) The changes to the kinematics.yaml file in cool400_description_moveit_config were done automatically.
