# External Project Assets

External ROS packages can provide robot_sim assets without changing core code.
Install assets under:

```text
share/<pkg>/robot_sim/
  profiles/
  validation_cases/
  suites/
  data_sources/
  adapters/
```

`profiles/` may contain either schema v3 `sim_profile` files or schema v4
`system_profile` files. `suites/` is the preferred suite path; the legacy
`validation_suites/` path is still accepted for compatibility.

Generate starter assets with:

```bash
robot-sim scaffold-system --package my_robot_sim --name minimal_system --output /tmp
robot-sim scaffold-case --package my_robot_sim --name smoke_case --system minimal_system --output /tmp
robot-sim scaffold-suite --package my_robot_sim --name smoke_suite --case smoke_case --output /tmp
robot-sim scaffold-adapter --package my_robot_sim --name smoke_adapter --output /tmp
```

Use `--case-package`, `--suite-package`, or a direct YAML path when running
assets from an external package.
