import sys

try:
    from .pinocchio_solver import PinocchioSolver
    # Eagerly check if pinocchio is valid to trigger failure here if broken
    import pinocchio as pin
    _ = pin.SE3
except (ImportError, AttributeError, Exception):
    PinocchioSolver = None
    # print("Warning: PinocchioSolver unavailable due to import/attribute error.", file=sys.stderr)

try:
    from .pyroki_solver import PyrokiSolver
except (ImportError, AttributeError, Exception):
    PyrokiSolver = None
