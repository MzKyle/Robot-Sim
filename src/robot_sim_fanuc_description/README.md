# robot_sim_fanuc_description

ROS 2 simulation wrapper for a FANUC M-20iD/12L industrial robot.

The DAE visual meshes and STL collision meshes under `models/m20id12l/meshes`
were downloaded from FANUC Corporation's official `fanuc_description` repository:

```text
fanuc_m20_description/meshes/m20_12-23d/
```

The upstream Apache-2.0 license text is preserved in
`upstream/FANUC_DESCRIPTION_APACHE_2_0_LICENSE`. The xacro in this package is
ROS 2/Gazebo specific and preserves the official M-20iD/12L mesh geometry,
inertials, and joint limits while adding ros2_control, Gazebo sensors, and
profile-driven arguments.
