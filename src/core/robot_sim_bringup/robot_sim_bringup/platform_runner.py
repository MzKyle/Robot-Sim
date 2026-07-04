"""Compatibility wrapper for robot_sim_bringup.platform.runner."""

from robot_sim_bringup.common.compat import reexport_module

_impl = reexport_module("robot_sim_bringup.platform.runner", globals())

