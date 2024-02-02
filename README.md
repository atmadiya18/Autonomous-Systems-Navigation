 Mobile robots are used in different environments to perform a wide range of tasks such as cleaning,
 patrolling, material handling, rescue operations, etc. These tasks, require the robots to move freely and
 autonomously through either static or dynamic environments. Therefore, Navigation is a crucial aspect
 for robots.
 Navigation allows robots to move through a predefined path, to adjust its trajectory due to changes in
 the environment, or to correct motion online due to measurement errors. As part of navigation, it is
 also important to consider extreme cases in which the robot gets lost or crashes into something, by
 implementing recovery strategies that bring the robot back to a safe known state.
 This lab will show you some of the available tools in ROS to perform the Navigation task such as
 1) Mapping, 2) Localization, 3) Planning and 4) Execution of a trajectory using the ROS Navigation
 Stack Fig.1, which implements the nodes and algorithms to solve the planning, mapping, localization, and
 recovery.
 The ROS Navigation stack outputs Twist data through the \cmd_vel topic based on odometry, sensor
 data, and a goal pose. Although the planning and the localization algorithms are already part of the stack,
 it is possible to tune/modify the behavior by means of the ROS parameter server via *.yaml files.
 Finally you will implement your own path planning and path follower to better understand the afore
mentioned navigation concepts.
