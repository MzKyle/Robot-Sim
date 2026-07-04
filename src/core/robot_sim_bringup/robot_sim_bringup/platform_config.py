"""Compatibility wrapper for robot_sim_bringup.platform.config."""

from robot_sim_bringup.common.compat import reexport_module

_impl = reexport_module("robot_sim_bringup.platform.config", globals())

