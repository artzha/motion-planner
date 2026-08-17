function deg2rad(deg)
  return deg * math.pi / 180
end

NavigationParameters = {
    dt = 0.05;
    -- 0.05 is the default value for system latency on real robot
    system_latency = 1.4;

    max_linear_accel = 0.3;
    max_linear_deccel = 1.0;
    max_linear_speed = 0.5;

    max_angular_accel = 0.5;
    max_angular_deccel = 0.5;
    max_angular_speed = 1.4;

    max_curvature = 2.0;
    max_path_length = 5.0;
    max_clearance = 1.0;

    clearance_weight = 10.0;
    velocity_weight = 3.0;
    distance_weight = 5.0;

    goal_tolerance = 0.1;

    robot_length = 1.100;
    robot_width = 0.550;
    robot_wheelbase = 0.55;
    obstacle_margin = 0.05;
    lidar_offset = 0.0;
}

GridParameters = {
    resolution = 0.1;
    half_extent = 9.6;
}

AStarParameters = {
    -- Safety cap on hybrid A* expansions. 7000 was sized for the 12.8 m demo
    -- window (~0.2 s worst case). 14000 is 2x that, matching the 19.2 m window
    -- (2x linear size) so 9 m diverse detours still finish inside the cap.
    max_edge_expansions = 14000;
}

DifferentialSampler = {
    max_curvature = 2.0;
    clearance_path_clip_fraction = 0.05;
    max_fov = deg2rad(180);
}