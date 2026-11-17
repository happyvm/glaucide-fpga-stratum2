import os
import sys

# `python3 -m unittest discover tools/golden` makes tools/golden the top-level directory;
# make the package importable as `golden` from its parent.
_parent = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _parent not in sys.path:
    sys.path.insert(0, _parent)
