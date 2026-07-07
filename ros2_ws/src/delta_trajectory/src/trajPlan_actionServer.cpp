// trajPlan_actionServer.cpp
// Exercise 6: a dynamically-linked component node (no main() - instantiated
// via RCLCPP_COMPONENTS_REGISTER_NODE(TrajectoryPlanServer)). Subscribes to
// /joint_states, sends each waypoint to ikin_server, publishes feedback at
// 1Hz, aborts on invalid response, succeeds after the last waypoint.
//
// TODO: paste in your actual action server implementation here.
