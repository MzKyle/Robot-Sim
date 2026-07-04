"""Compatibility wrapper for robot_sim_bringup.common.migrate_config."""

from robot_sim_bringup.common.compat import reexport_module

_impl = reexport_module("robot_sim_bringup.common.migrate_config", globals())

if __name__ == "__main__":
    raise SystemExit(_impl.main())

