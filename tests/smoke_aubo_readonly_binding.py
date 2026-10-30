"""Load and inspect the compiled module without constructing the SDK or logging in."""
from pathlib import Path
import ctypes
import os
import subprocess
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.aubo.readonly import QUERY_NAMES, load_native

dlopen_flags = sys.getdlopenflags()
module = load_native()
assert sys.getdlopenflags() == dlopen_flags
assert load_native() is module

# Regress the actual SDK relocation, without SDK construction or a login. Merely
# calling dlsym(handle, "re_match") would not detect the original libc collision.
sdk = module._isolated_sdk_handle
class DlInfo(ctypes.Structure):
    _fields_ = [("fname", ctypes.c_char_p), ("fbase", ctypes.c_void_p),
                ("sname", ctypes.c_char_p), ("saddr", ctypes.c_void_p)]
process = ctypes.CDLL(None)
process.dladdr.argtypes = [ctypes.c_void_p, ctypes.POINTER(DlInfo)]
info = DlInfo()
sdk_re_match = ctypes.cast(sdk.re_match, ctypes.c_void_p).value
assert process.dladdr(sdk_re_match, ctypes.byref(info)) != 0
relocations = subprocess.check_output(["readelf", "-rW", sdk._name], text=True)
offsets = [int(fields[0], 16) for line in relocations.splitlines()
           if len(fields := line.split()) >= 5 and fields[2] == "R_X86_64_JUMP_SLOT"
           and fields[4] == "re_match"]
assert len(offsets) == 1, "Expected one x86-64 SDK re_match relocation"
assert ctypes.c_void_p.from_address(info.fbase + offsets[0]).value == sdk_re_match

# Loading without isolation first cannot be repaired by dlopen later. Refuse it
# before any SDK client is constructed, and require a clean process instead.
code = '''
import ctypes, os, sys
from teleop.aubo.readonly import load_native
preloaded = ctypes.CDLL(sys.argv[1], mode=os.RTLD_NOW | os.RTLD_LOCAL)
try:
    load_native()
except RuntimeError as exc:
    assert "already mapped" in str(exc), str(exc)
else:
    raise AssertionError("Unsafe preloaded SDK was accepted")
'''
child_env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1]))
subprocess.run([sys.executable, "-B", "-c", code, sdk._name], env=child_env, check=True)
client = module.ReadOnlyClient()  # Lazy wrapper, ServiceInterface is not constructed.
assert {x for x in dir(client) if not x.startswith("_")} == {"login", "logout", "query", "fk"}
for name in QUERY_NAMES:
    try:
        client.query(name)
    except RuntimeError as exc:
        assert "Not connected" in str(exc)
    else:
        raise AssertionError(f"Unconnected query was accepted: {name}")
for name in ("startup", "follow", "stop", "set_tool_kinematics"):
    try:
        client.query(name)
    except ValueError:
        pass
    else:
        raise AssertionError(f"Operation outside allowlist accepted: {name}")
for q in ([0.] * 5, [float("nan")] * 6):
    try:
        client.fk(q)
    except (ValueError, TypeError):
        pass
    else:
        raise AssertionError("Invalid FK input accepted")
try:
    client.fk([0.] * 6)
except RuntimeError as exc:
    assert "Not connected" in str(exc)
else:
    raise AssertionError("Unconnected FK was accepted")
print("Binding import/allowlist and SDK re_match isolation checks passed; "
      "unsafe preload refused; no SDK construction, login or hardware call.")
