from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Any

from frctools import RobotBase, Timer, Alliance
from frctools.autonomous import AutonomousSequence
from frctools.drivetrain import SwerveDrive

from wpimath.geometry import Pose2d, Rotation2d
from wpilib import SmartDashboard


@dataclass
class _ChoreoSample:
    t: float
    vx: float
    vy: float
    omega: float
    x: float
    y: float
    heading: float


@dataclass
class _ChoreoEvent:
    name: str
    t: float
    triggered: bool = False


def choreo_event(name: str):
    """Decorator used by ChoreoSwerveSequence subclasses to bind event callbacks."""
    def _decorate(fn):
        setattr(fn, '__choreo_event_name__', name)
        return fn
    return _decorate


class ChoreoSwerveSequence(AutonomousSequence):
    """
    Minimal Choreo trajectory follower for the current frctools architecture.

    It reads a `.traj` file exported by Choreo and feeds normalized vx/vy/omega
    into the existing field-centric SwerveDrive override axes.
    """

    def __init__(self,
                 traj_name: str,
                 reset_pose: bool = True,
                 kP_xy: float = 1.4,
                 kP_theta: float = 2.0,
                 event_handlers: dict[str, Callable[[], None]] | None = None,
                 mirror_for_red: bool = True,
                 mirror_x: bool = True,
                 mirror_y: bool = False,
                 mirror_omega: bool = True,
                 vmax: float | None = None,
                 omega_max: float | None = None,
                 min_omega_norm: float = 1.5,
                 max_omega_correction: float = 1.0):
        super().__init__()

        self._traj_name = traj_name if traj_name.endswith('.traj') else f'{traj_name}.traj'
        self._reset_pose = reset_pose

        self._kP_xy = kP_xy
        self._kP_theta = kP_theta
        self._mirror_for_red = mirror_for_red
        self._mirror_x = mirror_x
        self._mirror_y = mirror_y
        self._mirror_omega = mirror_omega

        self._user_vmax = None if vmax is None else float(vmax)
        self._user_omega_max = None if omega_max is None else float(omega_max)

        self._min_omega_norm = float(min_omega_norm)
        self._max_omega_correction = float(max_omega_correction)

        self._event_handlers: dict[str, Callable[[], None]] = {}

        self._external_event_handlers = event_handlers or {}

        self._samples: list[_ChoreoSample] = []
        self._events: list[_ChoreoEvent] = []
        self._duration = 0.0
        self._is_mirrored = False

        self._max_vx = 1.0
        self._max_vy = 1.0
        self._max_omega = 1.0

        self._swerve: SwerveDrive | None = None
        self._pose_estimator = None
        self._ref_object = None
        self._robot: RobotBase | None = None

    def on_start(self):
        super().on_start()

        self._load_traj()

        robot = RobotBase.instance()
        self._robot = robot
        self._swerve = robot.get_component('Swerve')

        self._event_handlers = self._collect_decorated_event_handlers()
        if event_handlers := getattr(self, '_external_event_handlers', None):
            self._event_handlers.update(event_handlers)

        try:
            self._pose_estimator = robot.get_component('Pose_Estimator')
            self._ref_object = self._pose_estimator.get_object('choreo_ref')
        except Exception:
            self._pose_estimator = None
            self._ref_object = None

        SmartDashboard.putString('Choreo/Trajectory', self._traj_name)
        SmartDashboard.putBoolean('Choreo/Active', True)
        SmartDashboard.putNumber('Choreo/Duration', self._duration)
        SmartDashboard.putBoolean('Choreo/Mirrored', self._is_mirrored)
        SmartDashboard.putBoolean('Choreo/MirrorX', self._mirror_x)
        SmartDashboard.putBoolean('Choreo/MirrorY', self._mirror_y)
        SmartDashboard.putBoolean('Choreo/MirrorOmega', self._mirror_omega)
        SmartDashboard.putNumber('Choreo/MaxVNorm', self._max_vx)
        SmartDashboard.putNumber('Choreo/MaxOmegaNorm', self._max_omega)
        SmartDashboard.putNumber('Choreo/MaxOmegaCorrection', self._max_omega_correction)
        SmartDashboard.putString('Choreo/LastEvent', '')

        if self._reset_pose and len(self._samples) > 0 and self._pose_estimator is not None:
            first = self._samples[0]
            self._pose_estimator.set_current_pose(Pose2d(first.x, first.y, Rotation2d(first.heading)))
            # Keeps frctools heading convention aligned with the pose estimator usage.
            self._swerve.set_current_heading(-first.heading)

    def on_end(self):
        if self._swerve is not None:
            self._swerve.override_axes(0.0, 0.0, 0.0)

        SmartDashboard.putBoolean('Choreo/Active', False)

        super().on_end()

    def loop(self):
        yield from ()

        if self._swerve is None or len(self._samples) == 0:
            return

        start = Timer.get_current_time()
        while True:
            self.before_choreo_loop()

            t = Timer.get_elapsed(start)
            if t >= self._duration:
                break

            self._process_events(t)
            s = self._sample_at(t)

            # Feedforward from trajectory
            vx_cmd = s.vx
            vy_cmd = s.vy
            omega_cmd = s.omega

            ex = 0.0
            ey = 0.0
            etheta = 0.0

            # Pose feedback correction (closed-loop path tracking)
            if self._pose_estimator is not None:
                cur = self._pose_estimator.get_current_pose()
                ex = s.x - cur.x
                ey = s.y - cur.y
                etheta = self._angle_diff(s.heading, cur.rotation().radians())

                vx_cmd += self._kP_xy * ex
                vy_cmd += self._kP_xy * ey
                omega_cmd += self._clamp(self._kP_theta * etheta,
                                         -self._max_omega_correction,
                                         self._max_omega_correction)

            if self._ref_object is not None:
                self._ref_object.setPose(Pose2d(s.x, s.y, Rotation2d(s.heading)))

            SmartDashboard.putNumber('Choreo/t', t)
            SmartDashboard.putNumber('Choreo/ref_x', s.x)
            SmartDashboard.putNumber('Choreo/ref_y', s.y)
            SmartDashboard.putNumber('Choreo/ref_heading', s.heading)
            SmartDashboard.putNumber('Choreo/error_x', ex)
            SmartDashboard.putNumber('Choreo/error_y', ey)
            SmartDashboard.putNumber('Choreo/error_theta', etheta)

            # Normalize to the existing axis-based swerve interface.
            x_cmd = self._clamp(vx_cmd / self._max_vx)
            y_cmd = self._clamp(vy_cmd / self._max_vy)
            r_cmd = self._clamp(omega_cmd / self._max_omega)

            if Alliance.get_alliance() == Alliance.RED:
                x_cmd *= -1
                y_cmd *= -1

            self._swerve.override_axes(horizontal=x_cmd, vertical=y_cmd, rotation=r_cmd)

            self.after_choreo_loop()
            yield None

        self._swerve.override_axes(0.0, 0.0, 0.0)

    def before_choreo_loop(self):
        pass

    def after_choreo_loop(self):
        pass

    def _process_events(self, t: float):
        for event in self._events:
            if event.triggered:
                continue
            if t >= event.t:
                event.triggered = True
                self._invoke_event(event.name)

    def _invoke_event(self, name: str):
        SmartDashboard.putString('Choreo/LastEvent', name)

        handler = self._event_handlers.get(name)
        if handler is not None:
            try:
                handler()
            except Exception as e:
                print(f'Choreo event handler failed for {name}: {e}')

    def get_component(self, name: str) -> Any:
        if self._robot is None:
            raise RuntimeError('Robot is not initialized yet. Components are available after on_start().')
        return self._robot.get_component(name)

    def _collect_decorated_event_handlers(self) -> dict[str, Callable[[], None]]:
        handlers: dict[str, Callable[[], None]] = {}

        for attr_name in dir(self):
            attr = getattr(self, attr_name)
            event_name = getattr(attr, '__choreo_event_name__', None)
            if event_name is None:
                continue
            if callable(attr):
                handlers[str(event_name)] = attr

        return handlers

    def _load_traj(self):
        path = self._resolve_traj_path()
        with path.open('r', encoding='utf-8') as f:
            data = json.load(f)

        raw_samples = data['trajectory']['samples']
        self._samples = [
            _ChoreoSample(
                t=float(s['t']),
                vx=float(s['vx']),
                vy=float(s['vy']),
                omega=float(s['omega']),
                x=float(s['x']),
                y=float(s['y']),
                heading=float(s['heading']),
            )
            for s in raw_samples
        ]

        field_length, field_width = self._extract_field_size(data)
        self._is_mirrored = self._mirror_for_red and Alliance.get_alliance() == Alliance.RED and (self._mirror_x or self._mirror_y)
        if self._is_mirrored:
            self._samples = [self._mirror_sample_for_red(s, field_length, field_width) for s in self._samples]

        self._duration = self._samples[-1].t if len(self._samples) > 0 else 0.0
        auto_max_vx = max(1e-6, max(abs(s.vx) for s in self._samples))
        auto_max_vy = max(1e-6, max(abs(s.vy) for s in self._samples))
        auto_max_omega = max(self._min_omega_norm, max(abs(s.omega) for s in self._samples))

        if self._user_vmax is not None:
            user_v = max(1e-6, abs(self._user_vmax))
            self._max_vx = user_v
            self._max_vy = user_v
        else:
            self._max_vx = auto_max_vx
            self._max_vy = auto_max_vy

        if self._user_omega_max is not None:
            self._max_omega = max(1e-6, abs(self._user_omega_max))
        else:
            self._max_omega = auto_max_omega

        self._events = []
        for e in data.get('events', []):
            name = str(e.get('name', '')).strip()
            if name == '':
                continue

            from_data = e.get('from', {}) or {}
            t_event = float(from_data.get('targetTimestamp', 0.0))

            offset_data = from_data.get('offset', {}) or {}
            t_event += float(offset_data.get('val', 0.0))
            t_event = max(0.0, min(t_event, self._duration))

            self._events.append(_ChoreoEvent(name=name, t=t_event))

        self._events.sort(key=lambda ev: ev.t)

    def _resolve_traj_path(self) -> Path:
        here = Path(__file__).resolve()
        robot_dir = here.parents[2]  # .../robot
        repo_dir = here.parents[3]   # .../frc-2026-robot

        candidates = [
            robot_dir / 'auto' / self._traj_name,
            repo_dir / 'auto' / self._traj_name,
        ]

        for p in candidates:
            if p.exists():
                return p

        raise FileNotFoundError(
            f'Could not find Choreo trajectory "{self._traj_name}" in: '
            + ', '.join(str(p) for p in candidates)
        )

    def _extract_field_size(self, traj_data: dict) -> tuple[float, float]:
        # Defaults for FRC field units used by Choreo trajectories (meters)
        field_length = 16.541
        field_width = 8.0692

        constraints = traj_data.get('snapshot', {}).get('constraints', [])
        for c in constraints:
            c_data = c.get('data', {}) or {}
            if c_data.get('type') != 'KeepInRectangle':
                continue

            props = c_data.get('props', {}) or {}
            try:
                field_length = float(props.get('w', field_length))
                field_width = float(props.get('h', field_width))
                break
            except Exception:
                pass

        return field_length, field_width

    def _mirror_sample_for_red(self, s: _ChoreoSample, field_length: float, field_width: float) -> _ChoreoSample:
        x = (field_length - s.x) if self._mirror_x else s.x
        y = (field_width - s.y) if self._mirror_y else s.y

        vx = -s.vx if self._mirror_x else s.vx
        vy = -s.vy if self._mirror_y else s.vy

        heading = s.heading
        if self._mirror_x and self._mirror_y:
            heading = self._wrap_angle(s.heading + math.pi)
        elif self._mirror_x:
            heading = self._wrap_angle(math.pi - s.heading)
        elif self._mirror_y:
            heading = self._wrap_angle(-s.heading)

        omega = -s.omega if self._mirror_omega else s.omega

        return _ChoreoSample(
            t=s.t,
            vx=vx,
            vy=vy,
            omega=omega,
            x=x,
            y=y,
            heading=heading,
        )

    def _sample_at(self, t: float) -> _ChoreoSample:
        if t <= self._samples[0].t:
            return self._samples[0]
        if t >= self._samples[-1].t:
            return self._samples[-1]

        lo = 0
        hi = len(self._samples) - 1

        while lo <= hi:
            mid = (lo + hi) // 2
            mt = self._samples[mid].t
            if mt < t:
                lo = mid + 1
            elif mt > t:
                hi = mid - 1
            else:
                return self._samples[mid]

        a = self._samples[max(0, hi)]
        b = self._samples[min(len(self._samples) - 1, lo)]

        dt = b.t - a.t
        if dt <= 1e-9:
            return a

        k = (t - a.t) / dt
        return _ChoreoSample(
            t=t,
            vx=self._lerp(a.vx, b.vx, k),
            vy=self._lerp(a.vy, b.vy, k),
            omega=self._lerp(a.omega, b.omega, k),
            x=self._lerp(a.x, b.x, k),
            y=self._lerp(a.y, b.y, k),
            heading=self._lerp(a.heading, b.heading, k),
        )

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        return a + (b - a) * t

    @staticmethod
    def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
        return max(lo, min(hi, v))

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        return math.atan2(math.sin(target - current), math.cos(target - current))

    @staticmethod
    def _wrap_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))
