"""Compatibility wrapper for robot_sim_bringup.robot_domain.sim_launch_builder."""

from robot_sim_bringup.common.compat import reexport_module

_impl = reexport_module("robot_sim_bringup.robot_domain.sim_launch_builder", globals())

