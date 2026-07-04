"""Schema v4 generic ROS pipeline validation runtime.

If this package is accidentally imported as top-level ``platform`` because an
inner source directory is on PYTHONPATH, expose the standard-library platform
module attributes so third-party imports such as numpy still work.
"""

if __name__ == "platform":
    import importlib.util
    from pathlib import Path
    import sysconfig

    _stdlib_platform = Path(sysconfig.get_path("stdlib")) / "platform.py"
    _spec = importlib.util.spec_from_file_location("_stdlib_platform", _stdlib_platform)
    if _spec is not None and _spec.loader is not None:
        _module = importlib.util.module_from_spec(_spec)
        _spec.loader.exec_module(_module)
        for _name in dir(_module):
            if not _name.startswith("__"):
                globals()[_name] = getattr(_module, _name)
