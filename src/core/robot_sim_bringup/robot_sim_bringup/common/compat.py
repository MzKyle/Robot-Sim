from __future__ import annotations

import importlib
from types import ModuleType
from typing import Any


def reexport_module(module_name: str, namespace: dict[str, Any]) -> ModuleType:
    """Populate a compatibility module with every public and private symbol."""
    module = importlib.import_module(module_name)
    exported = []
    for name in dir(module):
        if name.startswith("__"):
            continue
        namespace[name] = getattr(module, name)
        exported.append(name)
    namespace["__all__"] = exported
    return module

