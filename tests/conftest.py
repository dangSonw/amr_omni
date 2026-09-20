import sys
from pathlib import Path

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
for p in [str(WORKSPACE_ROOT), str(WORKSPACE_ROOT / "src" / "omni_control"), str(WORKSPACE_ROOT / "web" / "backend")]:
    if p not in sys.path:
        sys.path.insert(0, p)
