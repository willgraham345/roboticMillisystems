# Actions
/backup
/compute_path_through_poses
/compute_path_to_pose
/follow_path
/follow_waypoints
/navigate_through_poses
/navigate_to_pose
/spin
/wait

# bt_navigator stuff
/bt_navigator
  Subscribers:
    /bond: bond/msg/Status
    /goal_pose: geometry_msgs/msg/PoseStamped #received from...? 
      <!-- Pose stamped is a pose with reference coordinate frame and timestamp -->
    /odom: nav_msgs/msg/Odometry
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /tf: tf2_msgs/msg/TFMessage
    /tf_static: tf2_msgs/msg/TFMessage
  Publishers:
    /bond: bond/msg/Status
    /bt_navigator/transition_event: lifecycle_msgs/msg/TransitionEvent
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /bt_navigator/change_state: lifecycle_msgs/srv/ChangeState
    /bt_navigator/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /bt_navigator/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /bt_navigator/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /bt_navigator/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /bt_navigator/get_parameters: rcl_interfaces/srv/GetParameters
    /bt_navigator/get_state: lifecycle_msgs/srv/GetState
    /bt_navigator/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /bt_navigator/list_parameters: rcl_interfaces/srv/ListParameters
    /bt_navigator/set_parameters: rcl_interfaces/srv/SetParameters
    /bt_navigator/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /navigate_through_poses: nav2_msgs/action/NavigateThroughPoses
    /navigate_to_pose: nav2_msgs/action/NavigateToPose
  Action Clients:
    /navigate_to_pose: nav2_msgs/action/NavigateToPose

# Params
/Crazyflie:
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  robot_description (type: string)
  set_robot_state_publisher (type: boolean)
  use_sim_time (type: boolean)
/Ros2Supervisor:
  use_sim_time (type: boolean)
/amcl:
  /bond_disable_heartbeat_timeout (type: boolean)
  alpha1 (type: double)
  alpha2 (type: double)
  alpha3 (type: double)
  alpha4 (type: double)
  alpha5 (type: double)
  always_reset_initial_pose (type: boolean)
  base_frame_id (type: string)
  beam_skip_distance (type: double)
  beam_skip_error_threshold (type: double)
  beam_skip_threshold (type: double)
  do_beamskip (type: boolean)
  global_frame_id (type: string)
  initial_pose.x (type: double)
  initial_pose.y (type: double)
  initial_pose.yaw (type: double)
  initial_pose.z (type: double)
  lambda_short (type: double)
  laser_likelihood_max_dist (type: double)
  laser_max_range (type: double)
  laser_min_range (type: double)
  laser_model_type (type: string)
  map_topic (type: string)
  max_beams (type: integer)
  max_particles (type: integer)
  min_particles (type: integer)
  odom_frame_id (type: string)
  pf_err (type: double)
  pf_z (type: double)
  recovery_alpha_fast (type: double)
  recovery_alpha_slow (type: double)
  resample_interval (type: integer)
  robot_model_type (type: string)
  save_pose_rate (type: double)
  scan_topic (type: string)
  set_initial_pose (type: boolean)
  sigma_hit (type: double)
  tf_broadcast (type: boolean)
  transform_tolerance (type: double)
  update_min_a (type: double)
  update_min_d (type: double)
  use_sim_time (type: boolean)
  z_hit (type: double)
  z_max (type: double)
  z_rand (type: double)
  z_short (type: double)
/amcl_rclcpp_node:
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  use_sim_time (type: boolean)
/bt_navigator:
  /bond_disable_heartbeat_timeout (type: boolean)
  bt_loop_duration (type: integer)
  default_nav_through_poses_bt_xml (type: string)
  default_nav_to_pose_bt_xml (type: string)
  default_server_timeout (type: integer)
  enable_groot_monitoring (type: boolean)
  global_frame (type: string)
  goal_blackboard_id (type: string)
  goals_blackboard_id (type: string)
  groot_zmq_publisher_port (type: integer)
  groot_zmq_server_port (type: integer)
  odom_topic (type: string)
  path_blackboard_id (type: string)
  plugin_lib_names (type: string array)
  qos_overrides./tf.subscription.depth (type: integer)
  qos_overrides./tf.subscription.durability (type: string)
  qos_overrides./tf.subscription.history (type: string)
  qos_overrides./tf.subscription.reliability (type: string)
  qos_overrides./tf_static.subscription.depth (type: integer)
  qos_overrides./tf_static.subscription.history (type: string)
  qos_overrides./tf_static.subscription.reliability (type: string)
  robot_base_frame (type: string)
  transform_tolerance (type: double)
  use_sim_time (type: boolean)
/controller_server:
  /bond_disable_heartbeat_timeout (type: boolean)
  FollowPath.BaseObstacle.class (type: string)
  FollowPath.BaseObstacle.scale (type: double)
  FollowPath.BaseObstacle.sum_scores (type: boolean)
  FollowPath.GoalAlign.aggregation_type (type: string)
  FollowPath.GoalAlign.class (type: string)
  FollowPath.GoalAlign.forward_point_distance (type: double)
  FollowPath.GoalAlign.scale (type: double)
  FollowPath.GoalDist.aggregation_type (type: string)
  FollowPath.GoalDist.class (type: string)
  FollowPath.GoalDist.scale (type: double)
  FollowPath.Oscillation.class (type: string)
  FollowPath.Oscillation.oscillation_reset_angle (type: double)
  FollowPath.Oscillation.oscillation_reset_dist (type: double)
  FollowPath.Oscillation.oscillation_reset_time (type: double)
  FollowPath.Oscillation.scale (type: double)
  FollowPath.Oscillation.x_only_threshold (type: double)
  FollowPath.PathAlign.aggregation_type (type: string)
  FollowPath.PathAlign.class (type: string)
  FollowPath.PathAlign.forward_point_distance (type: double)
  FollowPath.PathAlign.scale (type: double)
  FollowPath.PathDist.aggregation_type (type: string)
  FollowPath.PathDist.class (type: string)
  FollowPath.PathDist.scale (type: double)
  FollowPath.RotateToGoal.class (type: string)
  FollowPath.RotateToGoal.lookahead_time (type: double)
  FollowPath.RotateToGoal.scale (type: double)
  FollowPath.RotateToGoal.slowing_factor (type: double)
  FollowPath.acc_lim_theta (type: double)
  FollowPath.acc_lim_x (type: double)
  FollowPath.acc_lim_y (type: double)
  FollowPath.angular_granularity (type: double)
  FollowPath.critics (type: string array)
  FollowPath.debug_trajectory_details (type: boolean)
  FollowPath.decel_lim_theta (type: double)
  FollowPath.decel_lim_x (type: double)
  FollowPath.decel_lim_y (type: double)
  FollowPath.default_critic_namespaces (type: string array)
  FollowPath.discretize_by_time (type: boolean)
  FollowPath.include_last_point (type: boolean)
  FollowPath.linear_granularity (type: double)
  FollowPath.marker_lifetime (type: double)
  FollowPath.max_speed_xy (type: double)
  FollowPath.max_vel_theta (type: double)
  FollowPath.max_vel_x (type: double)
  FollowPath.max_vel_y (type: double)
  FollowPath.min_speed_theta (type: double)
  FollowPath.min_speed_xy (type: double)
  FollowPath.min_vel_x (type: double)
  FollowPath.min_vel_y (type: double)
  FollowPath.plugin (type: string)
  FollowPath.prune_distance (type: double)
  FollowPath.prune_plan (type: boolean)
  FollowPath.publish_cost_grid_pc (type: boolean)
  FollowPath.publish_evaluation (type: boolean)
  FollowPath.publish_global_plan (type: boolean)
  FollowPath.publish_local_plan (type: boolean)
  FollowPath.publish_trajectories (type: boolean)
  FollowPath.publish_transformed_plan (type: boolean)
  FollowPath.short_circuit_trajectory_evaluation (type: boolean)
  FollowPath.shorten_transformed_plan (type: boolean)
  FollowPath.sim_time (type: double)
  FollowPath.time_granularity (type: double)
  FollowPath.trajectory_generator_name (type: string)
  FollowPath.trans_stopped_velocity (type: double)
  FollowPath.transform_tolerance (type: double)
  FollowPath.vtheta_samples (type: integer)
  FollowPath.vx_samples (type: integer)
  FollowPath.vy_samples (type: integer)
  FollowPath.xy_goal_tolerance (type: double)
  controller_frequency (type: double)
  controller_plugins (type: string array)
  failure_tolerance (type: double)
  general_goal_checker.plugin (type: string)
  general_goal_checker.stateful (type: boolean)
  general_goal_checker.xy_goal_tolerance (type: double)
  general_goal_checker.yaw_goal_tolerance (type: double)
  goal_checker_plugins (type: string array)
  min_theta_velocity_threshold (type: double)
  min_x_velocity_threshold (type: double)
  min_y_velocity_threshold (type: double)
  odom_topic (type: string)
  progress_checker.movement_time_allowance (type: double)
  progress_checker.plugin (type: string)
  progress_checker.required_movement_radius (type: double)
  progress_checker_plugin (type: string)
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  speed_limit_topic (type: string)
  use_sim_time (type: boolean)
/controller_server_rclcpp_node:
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  use_sim_time (type: boolean)
/crazyflie_webots_driver:
  use_sim_time (type: boolean)
/global_costmap/global_costmap:
  /bond_disable_heartbeat_timeout (type: boolean)
  always_send_full_costmap (type: boolean)
  clearable_layers (type: string array)
  filters (type: string array)
  footprint (type: string)
  footprint_padding (type: double)
  global_frame (type: string)
  height (type: integer)
  inflation_layer.cost_scaling_factor (type: double)
  inflation_layer.enabled (type: boolean)
  inflation_layer.inflate_around_unknown (type: boolean)
  inflation_layer.inflate_unknown (type: boolean)
  inflation_layer.inflation_radius (type: double)
  inflation_layer.plugin (type: string)
  lethal_cost_threshold (type: integer)
  map_topic (type: string)
  observation_sources (type: string)
  obstacle_layer.combination_method (type: integer)
  obstacle_layer.enabled (type: boolean)
  obstacle_layer.footprint_clearing_enabled (type: boolean)
  obstacle_layer.max_obstacle_height (type: double)
  obstacle_layer.observation_sources (type: string)
  obstacle_layer.plugin (type: string)
  obstacle_layer.scan.clearing (type: boolean)
  obstacle_layer.scan.data_type (type: string)
  obstacle_layer.scan.expected_update_rate (type: double)
  obstacle_layer.scan.inf_is_valid (type: boolean)
  obstacle_layer.scan.marking (type: boolean)
  obstacle_layer.scan.max_obstacle_height (type: double)
  obstacle_layer.scan.min_obstacle_height (type: double)
  obstacle_layer.scan.observation_persistence (type: double)
  obstacle_layer.scan.obstacle_max_range (type: double)
  obstacle_layer.scan.obstacle_min_range (type: double)
  obstacle_layer.scan.raytrace_max_range (type: double)
  obstacle_layer.scan.raytrace_min_range (type: double)
  obstacle_layer.scan.sensor_frame (type: string)
  obstacle_layer.scan.topic (type: string)
  origin_x (type: double)
  origin_y (type: double)
  plugins (type: string array)
  publish_frequency (type: double)
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  resolution (type: double)
  robot_base_frame (type: string)
  robot_radius (type: double)
  rolling_window (type: boolean)
  static_layer.enabled (type: boolean)
  static_layer.map_subscribe_transient_local (type: boolean)
  static_layer.map_topic (type: string)
  static_layer.plugin (type: string)
  static_layer.subscribe_to_updates (type: boolean)
  static_layer.transform_tolerance (type: double)
  track_unknown_space (type: boolean)
  transform_tolerance (type: double)
  trinary_costmap (type: boolean)
  unknown_cost_value (type: integer)
  update_frequency (type: double)
  use_maximum (type: boolean)
  use_sim_time (type: boolean)
  width (type: integer)
/global_costmap/global_costmap_rclcpp_node:
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  use_sim_time (type: boolean)
/lifecycle_manager_localization:
  autostart (type: boolean)
  bond_timeout (type: double)
  node_names (type: string array)
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  use_sim_time (type: boolean)
/lifecycle_manager_navigation:
  /bond_disable_heartbeat_timeout (type: boolean)
  autostart (type: boolean)
  bond_timeout (type: double)
  node_names (type: string array)
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  use_sim_time (type: boolean)
/local_costmap/local_costmap:
  /bond_disable_heartbeat_timeout (type: boolean)
  always_send_full_costmap (type: boolean)
  clearable_layers (type: string array)
  filters (type: string array)
  footprint (type: string)
  footprint_padding (type: double)
  global_frame (type: string)
  height (type: integer)
  inflation_layer.cost_scaling_factor (type: double)
  inflation_layer.enabled (type: boolean)
  inflation_layer.inflate_around_unknown (type: boolean)
  inflation_layer.inflate_unknown (type: boolean)
  inflation_layer.inflation_radius (type: double)
  inflation_layer.plugin (type: string)
  lethal_cost_threshold (type: integer)
  map_topic (type: string)
  observation_sources (type: string)
  origin_x (type: double)
  origin_y (type: double)
  plugins (type: string array)
  publish_frequency (type: double)
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  resolution (type: double)
  robot_base_frame (type: string)
  robot_radius (type: double)
  rolling_window (type: boolean)
  track_unknown_space (type: boolean)
  transform_tolerance (type: double)
  trinary_costmap (type: boolean)
  unknown_cost_value (type: integer)
  update_frequency (type: double)
  use_maximum (type: boolean)
  use_sim_time (type: boolean)
  voxel_layer.combination_method (type: integer)
  voxel_layer.enabled (type: boolean)
  voxel_layer.footprint_clearing_enabled (type: boolean)
  voxel_layer.mark_threshold (type: integer)
  voxel_layer.max_obstacle_height (type: double)
  voxel_layer.observation_sources (type: string)
  voxel_layer.origin_z (type: double)
  voxel_layer.plugin (type: string)
  voxel_layer.publish_voxel_map (type: boolean)
  voxel_layer.scan.clearing (type: boolean)
  voxel_layer.scan.data_type (type: string)
  voxel_layer.scan.expected_update_rate (type: double)
  voxel_layer.scan.inf_is_valid (type: boolean)
  voxel_layer.scan.marking (type: boolean)
  voxel_layer.scan.max_obstacle_height (type: double)
  voxel_layer.scan.min_obstacle_height (type: double)
  voxel_layer.scan.observation_persistence (type: double)
  voxel_layer.scan.obstacle_max_range (type: double)
  voxel_layer.scan.obstacle_min_range (type: double)
  voxel_layer.scan.raytrace_max_range (type: double)
  voxel_layer.scan.raytrace_min_range (type: double)
  voxel_layer.scan.sensor_frame (type: string)
  voxel_layer.scan.topic (type: string)
  voxel_layer.unknown_threshold (type: integer)
  voxel_layer.z_resolution (type: double)
  voxel_layer.z_voxels (type: integer)
  width (type: integer)
/local_costmap/local_costmap_rclcpp_node:
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  use_sim_time (type: boolean)
/map_server:
  /bond_disable_heartbeat_timeout (type: boolean)
  frame_id (type: string)
  topic_name (type: string)
  use_sim_time (type: boolean)
  yaml_filename (type: string)
/planner_server:
  /bond_disable_heartbeat_timeout (type: boolean)
  GridBased.allow_unknown (type: boolean)
  GridBased.plugin (type: string)
  GridBased.tolerance (type: double)
  GridBased.use_astar (type: boolean)
  GridBased.use_final_approach_orientation (type: boolean)
  expected_planner_frequency (type: double)
  planner_plugins (type: string array)
  use_sim_time (type: boolean)
/planner_server_rclcpp_node:
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  use_sim_time (type: boolean)
/recoveries_server:
  /bond_disable_heartbeat_timeout (type: boolean)
  backup.plugin (type: string)
  costmap_topic (type: string)
  cycle_frequency (type: double)
  footprint_topic (type: string)
  global_frame (type: string)
  max_rotational_vel (type: double)
  min_rotational_vel (type: double)
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  recovery_plugins (type: string array)
  robot_base_frame (type: string)
  rotational_acc_lim (type: double)
  simulate_ahead_time (type: double)
  spin.plugin (type: string)
  transform_tolerance (type: double)
  use_sim_time (type: boolean)
  wait.plugin (type: string)
/recoveries_server_rclcpp_node:
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  use_sim_time (type: boolean)
/robot_state_publisher:
  frame_prefix (type: string)
  ignore_timestamp (type: boolean)
  publish_frequency (type: double)
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  qos_overrides./tf.publisher.depth (type: integer)
  qos_overrides./tf.publisher.durability (type: string)
  qos_overrides./tf.publisher.history (type: string)
  qos_overrides./tf.publisher.reliability (type: string)
  qos_overrides./tf_static.publisher.depth (type: integer)
  qos_overrides./tf_static.publisher.history (type: string)
  qos_overrides./tf_static.publisher.reliability (type: string)
  robot_description (type: string)
  use_sim_time (type: boolean)
  use_tf_static (type: boolean)
/rviz:
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  use_sim_time (type: boolean)
/slam_toolbox:
  angle_variance_penalty (type: double)
  base_frame (type: string)
  ceres_dogleg_type (type: string)
  ceres_linear_solver (type: string)
  ceres_loss_function (type: string)
  ceres_preconditioner (type: string)
  ceres_trust_strategy (type: string)
  coarse_angle_resolution (type: double)
  coarse_search_angle_offset (type: double)
  correlation_search_space_dimension (type: double)
  correlation_search_space_resolution (type: double)
  correlation_search_space_smear_deviation (type: double)
  debug_logging (type: boolean)
  distance_variance_penalty (type: double)
  do_loop_closing (type: boolean)
  enable_interactive_mode (type: boolean)
  fine_search_angle_offset (type: double)
  interactive_mode (type: boolean)
  link_match_minimum_response_fine (type: double)
  link_scan_maximum_distance (type: double)
  loop_match_maximum_variance_coarse (type: double)
  loop_match_minimum_chain_size (type: integer)
  loop_match_minimum_response_coarse (type: double)
  loop_match_minimum_response_fine (type: double)
  loop_search_maximum_distance (type: double)
  loop_search_space_dimension (type: double)
  loop_search_space_resolution (type: double)
  loop_search_space_smear_deviation (type: double)
  map_file_name (type: string)
  map_frame (type: string)
  map_name (type: string)
  map_start_at_dock (type: not set)
  map_start_pose (type: not set)
  map_update_interval (type: double)
  max_laser_range (type: double)
  minimum_angle_penalty (type: double)
  minimum_distance_penalty (type: double)
  minimum_time_interval (type: double)
  minimum_travel_distance (type: double)
  minimum_travel_heading (type: double)
  mode (type: string)
  odom_frame (type: string)
  paused_new_measurements (type: boolean)
  paused_processing (type: boolean)
  qos_overrides./clock.subscription.depth (type: integer)
  qos_overrides./clock.subscription.durability (type: string)
  qos_overrides./clock.subscription.history (type: string)
  qos_overrides./clock.subscription.reliability (type: string)
  qos_overrides./parameter_events.publisher.depth (type: integer)
  qos_overrides./parameter_events.publisher.durability (type: string)
  qos_overrides./parameter_events.publisher.history (type: string)
  qos_overrides./parameter_events.publisher.reliability (type: string)
  qos_overrides./tf.publisher.depth (type: integer)
  qos_overrides./tf.publisher.durability (type: string)
  qos_overrides./tf.publisher.history (type: string)
  qos_overrides./tf.publisher.reliability (type: string)
  resolution (type: double)
  scan_buffer_maximum_scan_distance (type: double)
  scan_buffer_size (type: integer)
  scan_topic (type: string)
  solver_plugin (type: string)
  tf_buffer_duration (type: double)
  throttle_scans (type: integer)
  transform_publish_period (type: double)
  transform_timeout (type: double)
  use_response_expansion (type: boolean)
  use_scan_barycenter (type: boolean)
  use_scan_matching (type: boolean)
  use_sim_time (type: boolean)
/waypoint_follower:
  /bond_disable_heartbeat_timeout (type: boolean)
  loop_rate (type: integer)
  stop_on_failure (type: boolean)
  use_sim_time (type: boolean)
  wait_at_waypoint.enabled (type: boolean)
  wait_at_waypoint.plugin (type: string)
  wait_at_waypoint.waypoint_pause_duration (type: integer)
  waypoint_task_executor_plugin (type: string)
  waypoint_task_executor_plugin.plugin (type: string)

# services within repo
/Crazyflie/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/Crazyflie/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/Crazyflie/get_parameters [rcl_interfaces/srv/GetParameters]
/Crazyflie/list_parameters [rcl_interfaces/srv/ListParameters]
/Crazyflie/set_parameters [rcl_interfaces/srv/SetParameters]
/Crazyflie/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/Ros2Supervisor/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/Ros2Supervisor/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/Ros2Supervisor/get_parameters [rcl_interfaces/srv/GetParameters]
/Ros2Supervisor/list_parameters [rcl_interfaces/srv/ListParameters]
/Ros2Supervisor/set_parameters [rcl_interfaces/srv/SetParameters]
/Ros2Supervisor/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/amcl/change_state [lifecycle_msgs/srv/ChangeState]
/amcl/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/amcl/get_available_states [lifecycle_msgs/srv/GetAvailableStates]
/amcl/get_available_transitions [lifecycle_msgs/srv/GetAvailableTransitions]
/amcl/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/amcl/get_parameters [rcl_interfaces/srv/GetParameters]
/amcl/get_state [lifecycle_msgs/srv/GetState]
/amcl/get_transition_graph [lifecycle_msgs/srv/GetAvailableTransitions]
/amcl/list_parameters [rcl_interfaces/srv/ListParameters]
/amcl/set_parameters [rcl_interfaces/srv/SetParameters]
/amcl/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/amcl_rclcpp_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/amcl_rclcpp_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/amcl_rclcpp_node/get_parameters [rcl_interfaces/srv/GetParameters]
/amcl_rclcpp_node/list_parameters [rcl_interfaces/srv/ListParameters]
/amcl_rclcpp_node/set_parameters [rcl_interfaces/srv/SetParameters]
/amcl_rclcpp_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/bt_navigator/change_state [lifecycle_msgs/srv/ChangeState]
/bt_navigator/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/bt_navigator/get_available_states [lifecycle_msgs/srv/GetAvailableStates]
/bt_navigator/get_available_transitions [lifecycle_msgs/srv/GetAvailableTransitions]
/bt_navigator/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/bt_navigator/get_parameters [rcl_interfaces/srv/GetParameters]
/bt_navigator/get_state [lifecycle_msgs/srv/GetState]
/bt_navigator/get_transition_graph [lifecycle_msgs/srv/GetAvailableTransitions]
/bt_navigator/list_parameters [rcl_interfaces/srv/ListParameters]
/bt_navigator/set_parameters [rcl_interfaces/srv/SetParameters]
/bt_navigator/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/bt_navigatornavigate_through_poses_rclcpp_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/bt_navigatornavigate_through_poses_rclcpp_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/bt_navigatornavigate_through_poses_rclcpp_node/get_parameters [rcl_interfaces/srv/GetParameters]
/bt_navigatornavigate_through_poses_rclcpp_node/list_parameters [rcl_interfaces/srv/ListParameters]
/bt_navigatornavigate_through_poses_rclcpp_node/set_parameters [rcl_interfaces/srv/SetParameters]
/bt_navigatornavigate_through_poses_rclcpp_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/bt_navigatornavigate_to_pose_rclcpp_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/bt_navigatornavigate_to_pose_rclcpp_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/bt_navigatornavigate_to_pose_rclcpp_node/get_parameters [rcl_interfaces/srv/GetParameters]
/bt_navigatornavigate_to_pose_rclcpp_node/list_parameters [rcl_interfaces/srv/ListParameters]
/bt_navigatornavigate_to_pose_rclcpp_node/set_parameters [rcl_interfaces/srv/SetParameters]
/bt_navigatornavigate_to_pose_rclcpp_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/controller_server/change_state [lifecycle_msgs/srv/ChangeState]
/controller_server/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/controller_server/get_available_states [lifecycle_msgs/srv/GetAvailableStates]
/controller_server/get_available_transitions [lifecycle_msgs/srv/GetAvailableTransitions]
/controller_server/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/controller_server/get_parameters [rcl_interfaces/srv/GetParameters]
/controller_server/get_state [lifecycle_msgs/srv/GetState]
/controller_server/get_transition_graph [lifecycle_msgs/srv/GetAvailableTransitions]
/controller_server/list_parameters [rcl_interfaces/srv/ListParameters]
/controller_server/set_parameters [rcl_interfaces/srv/SetParameters]
/controller_server/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/controller_server_rclcpp_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/controller_server_rclcpp_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/controller_server_rclcpp_node/get_parameters [rcl_interfaces/srv/GetParameters]
/controller_server_rclcpp_node/list_parameters [rcl_interfaces/srv/ListParameters]
/controller_server_rclcpp_node/set_parameters [rcl_interfaces/srv/SetParameters]
/controller_server_rclcpp_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/crazyflie_webots_driver/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/crazyflie_webots_driver/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/crazyflie_webots_driver/get_parameters [rcl_interfaces/srv/GetParameters]
/crazyflie_webots_driver/list_parameters [rcl_interfaces/srv/ListParameters]
/crazyflie_webots_driver/set_parameters [rcl_interfaces/srv/SetParameters]
/crazyflie_webots_driver/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/global_costmap/clear_around_global_costmap [nav2_msgs/srv/ClearCostmapAroundRobot]
/global_costmap/clear_entirely_global_costmap [nav2_msgs/srv/ClearEntireCostmap]
/global_costmap/clear_except_global_costmap [nav2_msgs/srv/ClearCostmapExceptRegion]
/global_costmap/get_costmap [nav2_msgs/srv/GetCostmap]
/global_costmap/global_costmap/change_state [lifecycle_msgs/srv/ChangeState]
/global_costmap/global_costmap/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/global_costmap/global_costmap/get_available_states [lifecycle_msgs/srv/GetAvailableStates]
/global_costmap/global_costmap/get_available_transitions [lifecycle_msgs/srv/GetAvailableTransitions]
/global_costmap/global_costmap/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/global_costmap/global_costmap/get_parameters [rcl_interfaces/srv/GetParameters]
/global_costmap/global_costmap/get_state [lifecycle_msgs/srv/GetState]
/global_costmap/global_costmap/get_transition_graph [lifecycle_msgs/srv/GetAvailableTransitions]
/global_costmap/global_costmap/list_parameters [rcl_interfaces/srv/ListParameters]
/global_costmap/global_costmap/set_parameters [rcl_interfaces/srv/SetParameters]
/global_costmap/global_costmap/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/global_costmap/global_costmap_rclcpp_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/global_costmap/global_costmap_rclcpp_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/global_costmap/global_costmap_rclcpp_node/get_parameters [rcl_interfaces/srv/GetParameters]
/global_costmap/global_costmap_rclcpp_node/list_parameters [rcl_interfaces/srv/ListParameters]
/global_costmap/global_costmap_rclcpp_node/set_parameters [rcl_interfaces/srv/SetParameters]
/global_costmap/global_costmap_rclcpp_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/global_costmap_client/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/global_costmap_client/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/global_costmap_client/get_parameters [rcl_interfaces/srv/GetParameters]
/global_costmap_client/list_parameters [rcl_interfaces/srv/ListParameters]
/global_costmap_client/set_parameters [rcl_interfaces/srv/SetParameters]
/global_costmap_client/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/lifecycle_manager_localization/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/lifecycle_manager_localization/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/lifecycle_manager_localization/get_parameters [rcl_interfaces/srv/GetParameters]
/lifecycle_manager_localization/is_active [std_srvs/srv/Trigger]
/lifecycle_manager_localization/list_parameters [rcl_interfaces/srv/ListParameters]
/lifecycle_manager_localization/manage_nodes [nav2_msgs/srv/ManageLifecycleNodes]
/lifecycle_manager_localization/set_parameters [rcl_interfaces/srv/SetParameters]
/lifecycle_manager_localization/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/lifecycle_manager_localization_service_client/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/lifecycle_manager_localization_service_client/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/lifecycle_manager_localization_service_client/get_parameters [rcl_interfaces/srv/GetParameters]
/lifecycle_manager_localization_service_client/list_parameters [rcl_interfaces/srv/ListParameters]
/lifecycle_manager_localization_service_client/set_parameters [rcl_interfaces/srv/SetParameters]
/lifecycle_manager_localization_service_client/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/lifecycle_manager_navigation/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/lifecycle_manager_navigation/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/lifecycle_manager_navigation/get_parameters [rcl_interfaces/srv/GetParameters]
/lifecycle_manager_navigation/is_active [std_srvs/srv/Trigger]
/lifecycle_manager_navigation/list_parameters [rcl_interfaces/srv/ListParameters]
/lifecycle_manager_navigation/manage_nodes [nav2_msgs/srv/ManageLifecycleNodes]
/lifecycle_manager_navigation/set_parameters [rcl_interfaces/srv/SetParameters]
/lifecycle_manager_navigation/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/lifecycle_manager_navigation_service_client/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/lifecycle_manager_navigation_service_client/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/lifecycle_manager_navigation_service_client/get_parameters [rcl_interfaces/srv/GetParameters]
/lifecycle_manager_navigation_service_client/list_parameters [rcl_interfaces/srv/ListParameters]
/lifecycle_manager_navigation_service_client/set_parameters [rcl_interfaces/srv/SetParameters]
/lifecycle_manager_navigation_service_client/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/local_costmap/clear_around_local_costmap [nav2_msgs/srv/ClearCostmapAroundRobot]
/local_costmap/clear_entirely_local_costmap [nav2_msgs/srv/ClearEntireCostmap]
/local_costmap/clear_except_local_costmap [nav2_msgs/srv/ClearCostmapExceptRegion]
/local_costmap/get_costmap [nav2_msgs/srv/GetCostmap]
/local_costmap/local_costmap/change_state [lifecycle_msgs/srv/ChangeState]
/local_costmap/local_costmap/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/local_costmap/local_costmap/get_available_states [lifecycle_msgs/srv/GetAvailableStates]
/local_costmap/local_costmap/get_available_transitions [lifecycle_msgs/srv/GetAvailableTransitions]
/local_costmap/local_costmap/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/local_costmap/local_costmap/get_parameters [rcl_interfaces/srv/GetParameters]
/local_costmap/local_costmap/get_state [lifecycle_msgs/srv/GetState]
/local_costmap/local_costmap/get_transition_graph [lifecycle_msgs/srv/GetAvailableTransitions]
/local_costmap/local_costmap/list_parameters [rcl_interfaces/srv/ListParameters]
/local_costmap/local_costmap/set_parameters [rcl_interfaces/srv/SetParameters]
/local_costmap/local_costmap/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/local_costmap/local_costmap_rclcpp_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/local_costmap/local_costmap_rclcpp_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/local_costmap/local_costmap_rclcpp_node/get_parameters [rcl_interfaces/srv/GetParameters]
/local_costmap/local_costmap_rclcpp_node/list_parameters [rcl_interfaces/srv/ListParameters]
/local_costmap/local_costmap_rclcpp_node/set_parameters [rcl_interfaces/srv/SetParameters]
/local_costmap/local_costmap_rclcpp_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/local_costmap_client/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/local_costmap_client/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/local_costmap_client/get_parameters [rcl_interfaces/srv/GetParameters]
/local_costmap_client/list_parameters [rcl_interfaces/srv/ListParameters]
/local_costmap_client/set_parameters [rcl_interfaces/srv/SetParameters]
/local_costmap_client/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/map_server/change_state [lifecycle_msgs/srv/ChangeState]
/map_server/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/map_server/get_available_states [lifecycle_msgs/srv/GetAvailableStates]
/map_server/get_available_transitions [lifecycle_msgs/srv/GetAvailableTransitions]
/map_server/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/map_server/get_parameters [rcl_interfaces/srv/GetParameters]
/map_server/get_state [lifecycle_msgs/srv/GetState]
/map_server/get_transition_graph [lifecycle_msgs/srv/GetAvailableTransitions]
/map_server/list_parameters [rcl_interfaces/srv/ListParameters]
/map_server/set_parameters [rcl_interfaces/srv/SetParameters]
/map_server/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/planner_server/change_state [lifecycle_msgs/srv/ChangeState]
/planner_server/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/planner_server/get_available_states [lifecycle_msgs/srv/GetAvailableStates]
/planner_server/get_available_transitions [lifecycle_msgs/srv/GetAvailableTransitions]
/planner_server/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/planner_server/get_parameters [rcl_interfaces/srv/GetParameters]
/planner_server/get_state [lifecycle_msgs/srv/GetState]
/planner_server/get_transition_graph [lifecycle_msgs/srv/GetAvailableTransitions]
/planner_server/list_parameters [rcl_interfaces/srv/ListParameters]
/planner_server/set_parameters [rcl_interfaces/srv/SetParameters]
/planner_server/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/planner_server_rclcpp_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/planner_server_rclcpp_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/planner_server_rclcpp_node/get_parameters [rcl_interfaces/srv/GetParameters]
/planner_server_rclcpp_node/list_parameters [rcl_interfaces/srv/ListParameters]
/planner_server_rclcpp_node/set_parameters [rcl_interfaces/srv/SetParameters]
/planner_server_rclcpp_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/recoveries_server/change_state [lifecycle_msgs/srv/ChangeState]
/recoveries_server/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/recoveries_server/get_available_states [lifecycle_msgs/srv/GetAvailableStates]
/recoveries_server/get_available_transitions [lifecycle_msgs/srv/GetAvailableTransitions]
/recoveries_server/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/recoveries_server/get_parameters [rcl_interfaces/srv/GetParameters]
/recoveries_server/get_state [lifecycle_msgs/srv/GetState]
/recoveries_server/get_transition_graph [lifecycle_msgs/srv/GetAvailableTransitions]
/recoveries_server/list_parameters [rcl_interfaces/srv/ListParameters]
/recoveries_server/set_parameters [rcl_interfaces/srv/SetParameters]
/recoveries_server/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/recoveries_server_rclcpp_node/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/recoveries_server_rclcpp_node/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/recoveries_server_rclcpp_node/get_parameters [rcl_interfaces/srv/GetParameters]
/recoveries_server_rclcpp_node/list_parameters [rcl_interfaces/srv/ListParameters]
/recoveries_server_rclcpp_node/set_parameters [rcl_interfaces/srv/SetParameters]
/recoveries_server_rclcpp_node/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/robot_state_publisher/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/robot_state_publisher/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/robot_state_publisher/get_parameters [rcl_interfaces/srv/GetParameters]
/robot_state_publisher/list_parameters [rcl_interfaces/srv/ListParameters]
/robot_state_publisher/set_parameters [rcl_interfaces/srv/SetParameters]
/robot_state_publisher/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/rviz/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/rviz/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/rviz/get_parameters [rcl_interfaces/srv/GetParameters]
/rviz/list_parameters [rcl_interfaces/srv/ListParameters]
/rviz/set_parameters [rcl_interfaces/srv/SetParameters]
/rviz/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/slam_toolbox/clear_changes [slam_toolbox/srv/Clear]
/slam_toolbox/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/slam_toolbox/deserialize_map [slam_toolbox/srv/DeserializePoseGraph]
/slam_toolbox/dynamic_map [nav_msgs/srv/GetMap]
/slam_toolbox/get_interactive_markers [visualization_msgs/srv/GetInteractiveMarkers]
/slam_toolbox/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/slam_toolbox/get_parameters [rcl_interfaces/srv/GetParameters]
/slam_toolbox/list_parameters [rcl_interfaces/srv/ListParameters]
/slam_toolbox/manual_loop_closure [slam_toolbox/srv/LoopClosure]
/slam_toolbox/pause_new_measurements [slam_toolbox/srv/Pause]
/slam_toolbox/save_map [slam_toolbox/srv/SaveMap]
/slam_toolbox/serialize_map [slam_toolbox/srv/SerializePoseGraph]
/slam_toolbox/set_parameters [rcl_interfaces/srv/SetParameters]
/slam_toolbox/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/slam_toolbox/toggle_interactive_mode [slam_toolbox/srv/ToggleInteractive]
/spawn_urdf_robot [webots_ros2_msgs/srv/SpawnUrdfRobot]
/waypoint_follower/change_state [lifecycle_msgs/srv/ChangeState]
/waypoint_follower/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/waypoint_follower/get_available_states [lifecycle_msgs/srv/GetAvailableStates]
/waypoint_follower/get_available_transitions [lifecycle_msgs/srv/GetAvailableTransitions]
/waypoint_follower/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/waypoint_follower/get_parameters [rcl_interfaces/srv/GetParameters]
/waypoint_follower/get_state [lifecycle_msgs/srv/GetState]
/waypoint_follower/get_transition_graph [lifecycle_msgs/srv/GetAvailableTransitions]
/waypoint_follower/list_parameters [rcl_interfaces/srv/ListParameters]
/waypoint_follower/set_parameters [rcl_interfaces/srv/SetParameters]
/waypoint_follower/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
