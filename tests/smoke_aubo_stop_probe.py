"""Optional built stop diagnostic boundary check; no SDK construction or login."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.aubo.stationary_stop_probe import load_stop_native

module = load_stop_native()
client = module.StationaryStopClient()
assert {x for x in dir(client) if not x.startswith("_")} == {
    "login", "logout", "state", "joints", "stop_at_rest"}
for call in (client.state, client.joints, client.logout,
             lambda: client.stop_at_rest("public_stop"),
             lambda: client.stop_at_rest("slow_control")):
    try:
        call()
    except RuntimeError as exc:
        assert "Not connected" in str(exc)
    else:
        raise AssertionError("Unconnected diagnostic call accepted")
try:
    client.stop_at_rest("continue")
except ValueError:
    pass
else:
    raise AssertionError("Non-stop method accepted")
print("Stop-only native boundary checks passed; no SDK construction, connection or stop.")
