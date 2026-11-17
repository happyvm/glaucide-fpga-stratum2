import os
import sys

# `python3 -m unittest discover tools/golden/ntpclient`: make `ntpclient` importable
# (a namespace package under tools/golden).
_parent = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _parent not in sys.path:
    sys.path.insert(0, _parent)
