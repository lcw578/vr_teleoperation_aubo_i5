"""Inspect the real ARCS binding without connecting to any controller."""
import os
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.aubo.servoj_probe_cli import load_probe, PROBE_METHODS

runtime = os.environ.get("AUBO_RUNTIME_DIR", "/data/VR_teleoperation_runtime")
flags = sys.getdlopenflags()
native, manifest = load_probe(runtime)
assert sys.getdlopenflags() == flags
assert {x for x in dir(native.ServoJReadOnlyProbe) if not x.startswith("_")} == PROBE_METHODS
for timeout in (0, 99, 5001):
    try:
        native.ServoJReadOnlyProbe(timeout)
    except ValueError:
        pass
    else:
        raise AssertionError("Unbounded RPC timeout accepted")
client = native.ServoJReadOnlyProbe(1000)
for method in (client.inspect, client.sample_joint_positions):
    try:
        method()
    except RuntimeError as exc:
        assert "login required" in str(exc), str(exc)
    else:
        raise AssertionError("Unconnected query accepted")
assert client.logout()["disconnect_return_code"] == 0

# An already-loaded old SDK must be rejected before constructing an ARCS client.
code = '''
import sys
from teleop.aubo.readonly import load_native
from teleop.aubo.servoj_probe_cli import load_probe, ProbeError
load_native()
try:
    load_probe(sys.argv[1])
except ProbeError as exc:
    assert 'already loaded' in str(exc), str(exc)
else:
    raise AssertionError('Old and new SDKs loaded in the same process')
'''
subprocess.run([sys.executable, "-c", code, runtime], cwd=Path(__file__).resolve().parents[1], check=True)
print("ARCS build/import, read-only API, unconnected lifecycle and SDK isolation passed; no controller connected.")
