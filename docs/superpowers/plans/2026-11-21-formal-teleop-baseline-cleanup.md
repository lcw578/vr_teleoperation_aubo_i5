# Formal Teleop Baseline Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with review checkpoints.

**Goal:** Reduce the repository to the verified real AUBO six-axis teleoperation baseline and the verified MuJoCo teleoperation baseline, while removing unrelated historical code and generated artifacts.

**Architecture:** Keep two executable roots, `scripts/aubo_quest_follow.sh` for real hardware and `scripts/run_mujoco_teleop.sh` for MuJoCo. Move the two MuJoCo parameters currently read from `ros2_ws` into `teleop/`, then delete the old ROS workspace and non-baseline entries after dependency scans confirm no kept runtime imports remain.

**Tech Stack:** Bash entry points, Python 3.10, MuJoCo, AUBO Native C++ extension, pytest/unittest, Git.

**Spec:** `docs/superpowers/specs/2026-11-21-formal-teleop-baseline-cleanup-design.md`

## Global Constraints

- Preserve the formal real baseline `aubo-c2-refined90-2026-11-12`.
- Preserve the formal MuJoCo baseline `sim-world-rg-2026-10-12`.
- Do not delete `/data/robot_SDK` or `/data/VR_teleoperation_runtime`.
- Do not move the existing `robot_SDK` symlink target.
- Remove `ros2_ws` only after `teleop/mujoco_session.py` no longer imports from it.
- Do not commit runtime logs, cache directories, generated build outputs, SDK binaries, recordings, or local reference PDFs.

---

### Task 1: Create a recoverable cleanup checkpoint and inventory

**Files:**
- Create: local Git branch `codex/formal-teleop-baseline-cleanup-20261121`
- Inspect: `docs/baselines/aubo-c2-refined90-2026-11-12/source.sha256`
- Inspect: `docs/baselines/sim-world-rg-2026-10-12.sha256`

**Interfaces:**
- Consumes: current working tree and tag `aubo-c2-refined90-2026-11-12`.
- Produces: a local recovery branch and a text inventory of current tracked and untracked paths.

- [ ] **Step 1: Record the current state before cleanup**

```bash
git status --short --branch
git rev-parse HEAD
git tag --points-at HEAD
git ls-files > /tmp/formal-baseline-cleanup-tracked.txt
find . -path './.git' -prune -o -type f -print | sort > /tmp/formal-baseline-cleanup-files.txt
```

- [ ] **Step 2: Create the local recovery branch**

```bash
git switch -c codex/formal-teleop-baseline-cleanup-20261121
```

If the branch already exists, keep the existing branch and record its current commit before continuing.

- [ ] **Step 3: Confirm the formal tags are present**

```bash
git show-ref --tags aubo-c2-refined90-2026-11-12
rg -n 'sim-world-rg-2026-10-12' docs README.md
```

Expected result: the real baseline tag resolves, and the MuJoCo baseline documents remain present.

---

### Task 2: Decouple MuJoCo from `ros2_ws`

**Files:**
- Create: `teleop/gripper_profile.py`
- Create: `teleop/config/robot_spec.yaml`
- Modify: `teleop/mujoco_session.py`
- Modify: `docs/simulation_baseline.md`
- Modify: `docs/mujoco_teleop.md`
- Test: `tests/test_mujoco_teleop.py`, `tests/smoke_mujoco_transport.py`

**Interfaces:**
- Consumes: existing `ros2_ws/src/aubo_i5_teleop/scripts/gripper_profile.py` and `ros2_ws/src/aubo_i5_teleop/config/robot_spec.yaml`.
- Produces: `teleop.gripper_profile.resolve(name: str | None) -> dict` and project-local `teleop/config/robot_spec.yaml` consumed by `mujoco_session.py`.

- [ ] **Step 1: Copy the two source files without changing their values**

```bash
mkdir -p teleop/config
cp ros2_ws/src/aubo_i5_teleop/scripts/gripper_profile.py teleop/gripper_profile.py
cp ros2_ws/src/aubo_i5_teleop/config/robot_spec.yaml teleop/config/robot_spec.yaml
```

- [ ] **Step 2: Update imports and parameter paths**

In `teleop/mujoco_session.py`, remove the `sys.path` insertion for `ros2_ws/src/aubo_i5_teleop/scripts`, replace `from gripper_profile import resolve` with `from teleop.gripper_profile import resolve`, and load `teleop/config/robot_spec.yaml` from `ROOT`.

- [ ] **Step 3: Verify values and references**

```bash
cmp -s teleop/gripper_profile.py ros2_ws/src/aubo_i5_teleop/scripts/gripper_profile.py
cmp -s teleop/config/robot_spec.yaml ros2_ws/src/aubo_i5_teleop/config/robot_spec.yaml
rg -n -F 'ros2_ws' teleop/mujoco_session.py teleop/gripper_profile.py teleop/config/robot_spec.yaml
```

Expected result: the copies match byte-for-byte before the old workspace is removed, and the runtime files contain no `ros2_ws` reference.

- [ ] **Step 4: Run the MuJoCo baseline tests before deleting the source workspace**

```bash
python3 -m unittest tests.test_mujoco_teleop
python3 tests/smoke_mujoco_transport.py --backend kinematic
```

Expected result: both commands exit successfully without connecting to hardware.

- [ ] **Step 5: Update the formal MuJoCo documentation**

Replace references to `ros2_ws/src/aubo_i5_teleop/scripts/gripper_profile.py` and `ros2_ws/src/aubo_i5_teleop/config/robot_spec.yaml` with `teleop/gripper_profile.py` and `teleop/config/robot_spec.yaml`. Keep historical ROS documents unchanged until the legacy deletion task.

---

### Task 3: Define the retained baseline source and tests

**Files:**
- Modify: `README.md`
- Modify: `docs/README.md`
- Modify: `docs/aubo_teleop_baseline_2026-11-12.md`
- Modify: `docs/simulation_baseline.md`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: the two verified entry points and their test manifests.
- Produces: a repository index that points only at retained entry points and excludes generated artifacts and local-only references.

- [ ] **Step 1: Enumerate direct runtime imports for the two entry points**

```bash
python3 - <<'PY'
import ast
from pathlib import Path
roots = [Path('teleop/aubo/quest_follow.py'), Path('teleop/mujoco_session.py')]
seen = set()
queue = list(roots)
while queue:
    path = queue.pop()
    if path in seen or not path.exists():
        continue
    seen.add(path)
    tree = ast.parse(path.read_text(), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.level == 0 and node.module and node.module.startswith('teleop'):
            candidate = Path(node.module.replace('.', '/'))
            if candidate.with_suffix('.py').exists(): queue.append(candidate.with_suffix('.py'))
        if isinstance(node, ast.Import) and any(n.name.startswith('teleop') for n in node.names):
            for n in node.names:
                candidate = Path(n.name.replace('.', '/'))
                if candidate.with_suffix('.py').exists(): queue.append(candidate.with_suffix('.py'))
print('\n'.join(sorted(map(str, seen))))
PY
```

- [ ] **Step 2: Mark retained tests**

Retain tests whose names cover the real six-axis follow, passthrough, trajectory, motion profile, priming, hot clutch, process state, feedback, stop order, TCP acknowledgement, runtime startup, and MuJoCo transport. Retain their native C++ fixtures under `tests/native/`. Exclude tests dedicated only to deleted J6, placement, recording, dataset export, old probes, or legacy ROS routes.

- [ ] **Step 3: Update the repository indexes**

Keep the real and MuJoCo baseline links in `README.md`, add the two entry points and their verification commands to `docs/README.md`, and remove links that point to deleted historical launchers. Do not rewrite historical evidence documents beyond links required by retained files.

- [ ] **Step 4: Extend ignore rules for generated local state**

Ensure `.gitignore` covers `__pycache__/`, `.pytest_cache/`, `ros2_ws/`, `build/`, `install/`, `log/`, local runtime data, and local-only PDF references under `docs/reference/` while keeping the tracked AUBO manual intact.

---

### Task 4: Remove legacy ROS, exploratory, and generated content

**Files:**
- Delete: `ros2_ws/`
- Delete: legacy ROS entry points and build scripts that only target Route B
- Delete: exploratory AUBO probe scripts and tests excluded by Task 3
- Delete: `docs/archive_route_a/`
- Delete: stale caches and generated files such as `.pytest_cache/`, `__pycache__/`, and `scripts/__pycache__/`
- Delete: local-only untracked PDFs from the repository workspace or keep them ignored under `docs/reference/`

**Interfaces:**
- Consumes: the dependency closure and retained-test list from Tasks 2 and 3.
- Produces: a working tree containing only formal baseline content.

- [ ] **Step 1: Check for remaining runtime references before deletion**

```bash
rg -n -F 'ros2_ws' teleop scripts environment README.md docs --glob '!docs/archive_route_a/**'
rg -n -E 'aubo_(j6|servoj|stationary_stop|ack_stationary|batch_probe|phase1|readonly_probe|program_sine|staged_acceptance)' scripts teleop tests
```

Classify every match as retained documentation evidence, a historical document, or a live runtime reference. Resolve live runtime references before deletion.

- [ ] **Step 2: Remove only paths classified as legacy or generated**

Use `git rm -r` for tracked legacy directories and `rm -rf` for ignored caches. Do not remove `assets/aubo_i5/` files referenced by either baseline scene, `third_party/vr-teleop-kit/`, or the retained Native sources.

- [ ] **Step 3: Remove obsolete links and comments**

Update retained documents and scene comments so they point to `teleop/gripper_profile.py` and `teleop/config/robot_spec.yaml`, then verify that retained runtime files contain no deleted path.

---

### Task 5: Verify the reduced repository

**Files:**
- Test: retained Python and Native tests
- Inspect: all retained entry points and Git status

**Interfaces:**
- Consumes: cleaned repository.
- Produces: evidence that both formal baselines remain runnable and no deleted runtime dependency remains.

- [ ] **Step 1: Run static path and syntax checks**

```bash
git diff --check
python3 -m compileall -q teleop scripts tests
rg -n -F 'ros2_ws' teleop scripts environment README.md docs/project docs/simulation_baseline.md docs/mujoco_teleop.md
```

Expected result: no runtime references to `ros2_ws`; historical documents outside the retained baseline set may be absent.

- [ ] **Step 2: Run MuJoCo verification**

```bash
python3 -m unittest tests.test_mujoco_teleop
python3 tests/smoke_mujoco_transport.py --backend kinematic
```

Expected result: all selected tests pass and the synthetic transport smoke test exits with code 0.

- [ ] **Step 3: Run the formal AUBO offline regression subset**

```bash
python3 -m unittest \
  tests.test_aubo_six_axis \
  tests.test_aubo_follow \
  tests.test_aubo_fixed_tick_batch \
  tests.test_aubo_motion_profiles \
  tests.test_aubo_priming \
  tests.test_aubo_hot_clutch \
  tests.test_aubo_process_state \
  tests.test_aubo_process_feedback \
  tests.test_aubo_stop_order_sdk \
  tests.test_aubo_tcp_ack
```

Expected result: every selected test passes without a robot connection.

- [ ] **Step 4: Verify the final tree and size**

```bash
git status --short
git diff --stat
du -sh . --exclude=.git
find . -path './.git' -prune -o -type d \( -name build -o -name install -o -name log -o -name __pycache__ -o -name .pytest_cache \) -print
```

Expected result: no generated build/cache directories remain in the project tree, and only the two formal baseline families appear in the final diff.

- [ ] **Step 5: Commit the cleanup as one reviewable change**

```bash
git add -A
git commit -m "refactor: retain formal teleop baselines only"
```

Do not push to GitHub until the user explicitly requests the cleaned repository to be published.
