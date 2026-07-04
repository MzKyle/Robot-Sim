"""Compatibility wrapper for robot_sim_bringup.legacy_integrations.module_runner."""

from robot_sim_bringup.common.compat import reexport_module

_impl = reexport_module("robot_sim_bringup.legacy_integrations.module_runner", globals())

if __name__ == "__main__":
    raise SystemExit(_impl.main())

