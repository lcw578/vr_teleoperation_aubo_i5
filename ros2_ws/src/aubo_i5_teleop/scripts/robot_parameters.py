"""Read the sourced i5/CB4 model specification, shared with generated artifacts."""
from pathlib import Path
import yaml
try:
    from ament_index_python.packages import get_package_share_directory
    SPEC_PATH = Path(get_package_share_directory("aubo_i5_teleop")) / "config/robot_spec.yaml"
except (ImportError, LookupError):
    SPEC_PATH = Path(__file__).resolve().parents[1] / "config/robot_spec.yaml"
if not SPEC_PATH.exists():
    SPEC_PATH = Path(__file__).resolve().parents[1] / "config/robot_spec.yaml"
SPEC = yaml.safe_load(SPEC_PATH.read_text())
POSITION_LIMIT = SPEC["position_limit_rad"]
VELOCITY_LIMITS = tuple(SPEC["velocity_limits_rad_s"])
WORKING_LIMIT = SPEC["working_limit_rad"]
GROUP = ("shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint")
