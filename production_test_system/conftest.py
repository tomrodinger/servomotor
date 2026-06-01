"""Ensure the project root is importable so ``backend`` and ``tests`` resolve
when pytest is run from anywhere."""
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
