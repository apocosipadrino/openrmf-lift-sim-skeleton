"""Run every test_*.py in this directory without requiring pytest.

Usage: python3 tests/run_tests.py

The test modules are also pytest-compatible: `pytest tests/` works too.
"""

import importlib
import os
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)


def main() -> int:
    failures = 0
    ran = 0
    for filename in sorted(os.listdir(HERE)):
        if not (filename.startswith("test_") and filename.endswith(".py")):
            continue
        module_name = filename[:-3]
        module = importlib.import_module(module_name)
        for name in sorted(dir(module)):
            if not name.startswith("test_"):
                continue
            fn = getattr(module, name)
            if not callable(fn):
                continue
            ran += 1
            try:
                fn()
            except Exception:
                failures += 1
                print(f"FAIL {module_name}.{name}")
                traceback.print_exc()
            else:
                print(f"PASS {module_name}.{name}")
    print(f"\n{ran - failures}/{ran} tests passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
