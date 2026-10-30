"""Python follower for queries only. This does not run IK or dispatch motion."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading

from .feedback import FeedbackCache

QUERY_NAMES = (
    "server_version", "device_info", "work_mode", "robot_state", "is_real_robot",
    "online_mode", "online_master_mode", "mac_communication_ok", "joint_angles",
    "current_waypoint", "joint_status", "joint_ranges", "joint_position_limits",
    "joint6_rotate360_enabled", "tool_kinematics", "tool_dynamics", "safety_config", "diagnosis",
)

# Keep the native allowlist intact. This exclusion is a probe default, not a
# claim that the getter is unsupported by every AUBO controller.
DEFAULT_QUERY_EXCLUSIONS = {
    "joint_position_limits": (
        "Excluded from the default probe after GetJointPositionLimit returned "
        "10007 in 5.000143 s on server V4.5.111.71f2ee1-Alpha, 2026-10-19. "
        "Support is unconfirmed; use explicit --query only for diagnosis."
    ),
}
DEFAULT_QUERY_NAMES = tuple(name for name in QUERY_NAMES if name not in DEFAULT_QUERY_EXCLUSIONS)
_NATIVE_LOAD_LOCK = threading.RLock()


def _mapped_sdk_libraries():
    mapped = set()
    for line in Path("/proc/self/maps").read_text().splitlines():
        fields = line.split(None, 5)
        if len(fields) == 6 and Path(fields[5]).name.startswith("libaubo_sdk"):
            mapped.add(Path(fields[5]).resolve())
    return mapped


class MotionUnavailable(RuntimeError):
    pass


def load_native(runtime=None):
    # Serialize our own imports so another caller cannot observe a half-loaded
    # SDK. No Python-wide dlopen flags or shared environment are changed.
    with _NATIVE_LOAD_LOCK:
        return _load_native(runtime)


def _load_native(runtime):
    directory = Path(runtime or os.environ.get("AUBO_RUNTIME_DIR", "/data/VR_teleoperation_runtime"))
    manifest = json.loads((directory / "readonly-build.json").read_text())
    library = Path(manifest["release_library"])
    if hashlib.sha256(library.read_bytes()).hexdigest() != manifest["release_sha256"]:
        raise RuntimeError("SDK changed since build; rebuild and review before connecting")
    modules = list(Path(manifest["module_directory"]).glob("_aubo_readonly*.so"))
    if len(modules) != 1:
        raise RuntimeError("Build the read-only extension for this Python interpreter first")
    module = sys.modules.get("_aubo_readonly")
    if module is not None:
        if Path(module.__file__).resolve() != modules[0].resolve():
            raise RuntimeError("A different read-only extension is already loaded")
        if not hasattr(module, "_isolated_sdk_handle"):
            raise RuntimeError("SDK was loaded without symbol isolation; restart this Python process")
    else:
        if _mapped_sdk_libraries():
            raise RuntimeError("AUBO SDK is already mapped outside this loader; restart this Python process")
        if not hasattr(os, "RTLD_DEEPBIND"):
            raise RuntimeError("This SDK requires Linux RTLD_DEEPBIND symbol isolation")
        import ctypes
        # The vendor SDK exports tiny-regex re_match(pattern, text, int*).
        # libc exports an incompatible function with the same name. Eagerly
        # bind SDK references to its own scope before importing the extension.
        # Loading the library does not construct ServiceInterface or log in.
        sdk_handle = ctypes.CDLL(str(library.resolve()),
                                 mode=os.RTLD_NOW | os.RTLD_LOCAL | os.RTLD_DEEPBIND)
        spec = importlib.util.spec_from_file_location("_aubo_readonly", modules[0])
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module._isolated_sdk_handle = sdk_handle
        sys.modules["_aubo_readonly"] = module
    # Import can load SDK static initializers, but never constructs/logs in a client.
    # Verify the actual mapped SDK as LD_LIBRARY_PATH can override ELF RUNPATH.
    if _mapped_sdk_libraries() != {library.resolve()}:
        raise RuntimeError("Loaded SDK path differs from the recorded release library")
    if tuple(module.query_names) != QUERY_NAMES:
        raise RuntimeError("Native query allowlist differs from Python definition")
    return module


class ReadOnlyFollower:
    motion_supported = False

    def __init__(self, client, cache=None):
        self.client = client
        self.cache = cache if cache is not None else FeedbackCache()
        self.connected = False

    def connect(self, *, host, port, user, password):
        if self.connected:
            raise RuntimeError("Already connected")
        self.cache.invalidate("new_connection_requires_feedback")
        result = self.client.login(host, port, user, password)
        self.connected = result["return_code"] == 0
        return result

    def query(self, name):
        if name not in QUERY_NAMES:
            raise ValueError("Query is not in the read-only allowlist")
        if not self.connected:
            raise RuntimeError("Not connected")
        try:
            result = self.client.query(name)
        except Exception:
            self.cache.invalidate("query_exception")
            raise
        try:
            result = dict(result)
            if type(result.get("return_code")) is not int:
                raise ValueError("SDK result lacks an integer return code")
        except (TypeError, ValueError):
            self.cache.invalidate("invalid_sdk_result", source=name)
            raise
        result["feedback_validated"] = None
        if name == "tool_kinematics":
            result["data_source"] = "sdk_client_move_profile"
            result["notes"] = (
                "GetToolKinematicsParam reads the SDK client's move profile; "
                "this is not verification of the pendant's configured TCP."
            )
        if result["return_code"] != 0:
            # Conservative: a failed query cannot leave feedback marked valid.
            self.cache.invalidate("sdk_query_failed", source=name)
        elif name in ("joint_angles", "current_waypoint", "joint_status"):
            try:
                self.cache.update(result["data"]["joint_positions_rad"], source=name,
                                  received_monotonic_s=result["host_received_monotonic_s"])
                if not self.cache.latest().valid:
                    raise ValueError(self.cache.latest().reason)
                result["feedback_validated"] = True
            except Exception as exc:
                self.cache.invalidate("invalid_query_result", source=name)
                result["feedback_validated"] = False
                result["validation_error"] = {"type": type(exc).__name__, "message": str(exc)}
        return result

    def get_observation(self, *, max_age_s):
        if not self.connected:
            raise RuntimeError("Not connected")
        sample = self.cache.require_recent(max_age_s)
        return {"joint_positions_rad": sample.q_rad, "source": sample.source,
                "host_received_monotonic_s": sample.received_monotonic_s,
                "controller_sample_time": None, "feedback_valid": sample.valid}

    def send_action(self, action):
        raise MotionUnavailable("Read-only build exports no real motion or startup API")

    def stop(self):
        raise MotionUnavailable("AUBO stop path is unverified and is not implemented here")

    def disconnect(self):
        self.cache.invalidate("disconnected")
        if not self.connected:
            return None
        self.connected = False
        return self.client.logout()
