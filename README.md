# frc-2026-robot

## Run simulator (Choreo)

From `robot/`:

```bash
./.venv/bin/robotpy sim
```

Then in the simulator dashboard:

- Select auto: `choreo/soufleuse_simple` (or `choreo/soufleuse_plus_shoot`)
- Enable **Autonomous**
- Watch `Pose_Estimator/field`:
  - robot pose (estimated)
  - `choreo_ref` object (reference pose)

Useful NetworkTables/SmartDashboard keys:

- `Choreo/Active`
- `Choreo/Trajectory`
- `Choreo/t`
- `Choreo/error_x`
- `Choreo/error_y`
- `Choreo/error_theta`
- `Choreo/LastEvent`

## Choreo events

Event markers in `.traj` are supported.

The recommended pattern is subclass + decorator:

- Base class: `robot/robot2026/autonomous/choreo.py`
- Decorator: `@choreo_event("EventName")`
- Current implementation example: `Team3117ChoreoAuto` in `robot/robot.py`

Currently wired event names:

- `StartFeeding` / `StopFeeding`
- `StartIndexing` / `StopIndexing`
- `StartShooting` / `StopShooting`
- `StartAll` / `StopAll`
