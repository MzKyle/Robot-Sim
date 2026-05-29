# robot_sim_fanuc_description

ROS 2 simulation wrapper for a FANUC M-20iA/10L industrial robot.

The STL visual and collision meshes under `models/m20ia10l/meshes` were downloaded
from the ROS-Industrial `ros-industrial/fanuc` repository, `noetic-devel` branch:

```text
fanuc_m20ia_support/meshes/m20ia10l/
```

The upstream BSD license text is preserved in `upstream/ROS_INDUSTRIAL_FANUC_LICENSE`.
The xacro in this package is ROS 2/Gazebo specific and preserves the joint geometry
and limits from the upstream support package while adding inertials, ros2_control,
Gazebo sensors, and profile-driven arguments.
