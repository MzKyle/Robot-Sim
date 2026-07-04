"""Compatibility wrapper for robot_sim_bringup.platform.run_suite."""

from robot_sim_bringup.common.compat import reexport_module

_impl = reexport_module("robot_sim_bringup.platform.run_suite", globals())

if __name__ == "__main__":
    raise SystemExit(_impl.main())

