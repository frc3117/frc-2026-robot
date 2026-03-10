from frctools import Component, WPI_CANSparkMax
from frctools.sensor import Encoder
from frctools.frcmath import Range, lerp, between, repeat, SlewRateLimiter
from frctools.input import Input


from frc_ballistic_solver import CRT, CRTEncoder


from wpiutil import SendableBuilder
from wpimath.geometry import Translation3d


import math


class BallisticSolution:
    valid: bool = False
    time: float = 0.
    speed: float = 0.
    aim_dir: Translation3d = Translation3d(0., 0., 0.)
    impact_velocity: Translation3d = Translation3d(0., 0., 0.)
    impact_dir: Translation3d = Translation3d(0., 0., 0.)
    heading: float = 0.
    elevation: float = 0.
    score: float = math.inf

    def __str__(self):
        return f'BallisticSolution(valid={self.valid}, time={self.time:.3f}, speed={self.speed:.3f}, heading={self.heading:.3f}, elevation={self.elevation:.3f})'


class BallisticSolver:
    def __init__(self,
                 target_pos: Translation3d,
                 speed_range: Range,
                 airtime_range: Range,
                 impact_cone_axis: Translation3d = Translation3d(0., 0., -1),
                 impact_cone_tolerance: float = 0.785,
                 gravity: Translation3d = Translation3d(0., 0., -9.81),
                 sample_count: int=15,
                 prefer_high_arc: bool=True):
        self.__target_pos = target_pos
        self.__speed_range = speed_range
        self.__airtime_range = airtime_range
        self.__impact_cone_axis = impact_cone_axis / impact_cone_axis.norm()
        self.__impact_cone_tolerance = impact_cone_tolerance
        self.__impact_cone_cos_max = math.cos(impact_cone_tolerance * 0.5)
        self.__gravity = gravity
        self.__half_gravity = gravity * 0.5
        self.__sample_count = sample_count
        self.__prefer_high_arc = prefer_high_arc

    def solve(self, shooter_pos: Translation3d, shooter_world_vel: Translation3d) -> BallisticSolution:
        best = BallisticSolution()

        relative_target_pos = self.__target_pos - shooter_pos
        relative_target_vel = shooter_world_vel * -1

        for i in range(self.__sample_count):
            # Normalized centered sampling between [0, 1]
            u = (i + 0.5) / self.__sample_count

            # Apply arc bias
            if self.__prefer_high_arc:
                u = 1. - ((1. - u)**2)
            else:
                u = u**2

            # Convert to time
            t = lerp(self.__airtime_range.min, self.__airtime_range.max, u)
            if t < 1e-6:
                continue

            # Estimate total displacement including shoot world velocity and gravity
            total_displacement = relative_target_pos + (relative_target_vel * t) - (self.__half_gravity * (t*t))
            displacement_distance = total_displacement.norm()
            if displacement_distance < 1e-6:
                continue

            # Estimate speed required to make the shot
            required_speed = displacement_distance / t
            if not between(self.__speed_range.min, self.__speed_range.max, required_speed):
                continue

            aim_dir = total_displacement / displacement_distance

            # Estimate the impact velocity and direction
            v0 = shooter_world_vel + aim_dir * required_speed
            v_impact = v0 + self.__gravity * t

            if v_impact.squaredNorm() < 1e-8:
                continue

            impact_dir = v_impact /  v_impact.norm()
            if not self.__in_cone__(impact_dir):
                continue

            # Estimate the score of this solution and keep it if it is currently the best
            score = required_speed - 0.05 * t

            if not best.valid or score < best.score:
                best.valid = True
                best.time = t
                best.speed = required_speed
                best.aim_dir = aim_dir
                best.impact_velocity = v_impact
                best.impact_dir = impact_dir
                best.heading, best.elevation = BallisticSolver.__aim_dir_to_heading_elevation(aim_dir)
                best.score = score

        return best

    def __in_cone__(self, impact_dir: Translation3d) -> bool:
        return impact_dir.dot(self.__impact_cone_axis) >= self.__impact_cone_cos_max

    @staticmethod
    def __aim_dir_to_heading_elevation(aim_dir: Translation3d) -> (float, float):
        horizontal_mag = math.sqrt(
            aim_dir.x * aim_dir.x +
            aim_dir.y * aim_dir.y
        )

        heading = math.atan2(aim_dir.x, -aim_dir.y)
        elevation = math.atan2(aim_dir.z, horizontal_mag)

        return heading, elevation


class Shooter(Component):
    __encoder_a: Encoder
    __encoder_b: Encoder
    __heading_motor: WPI_CANSparkMax

    __crt_encoder_a: CRTEncoder
    __crt_encoder_b: CRTEncoder
    __crt: CRT
    __crt_inverted: CRT

    __current_angle: float = 0.

    __speed_input: Input
    __speed_ratio: float = 0.1

    __limiter: SlewRateLimiter

    def __init__(self, encoder_a: Encoder, encoder_b: Encoder, heading_motor: WPI_CANSparkMax):
        super().__init__()

        self.__encoder_a = encoder_a
        self.__encoder_b = encoder_b
        self.__heading_motor = heading_motor

        self.__crt_encoder_a = CRTEncoder(18, 207, 1.)
        self.__crt_encoder_b = CRTEncoder(23, 207, 1.)
        self.__crt = CRT(self.__crt_encoder_a, self.__crt_encoder_b, 207)

        self.__speed_input = Input.get_input('rotation')

        self.__limiter = SlewRateLimiter(4)

    def update_disabled(self):
        self.__crt_encoder_a.set_angle01(self.__encoder_a.get())
        self.__crt_encoder_b.set_angle01(self.__encoder_b.get())

        self.__current_angle = self.__limiter.evaluate(2 - repeat(self.__crt.get_turn(max_turn=2) + 1.83, 2.))

    def update(self):
        self.__heading_motor.set(self.__speed_ratio * self.__speed_input.get())

        self.__crt_encoder_a.set_angle01(self.__encoder_a.get())
        self.__crt_encoder_b.set_angle01(self.__encoder_b.get())

        self.__current_angle = self.__limiter.evaluate(2 - repeat(self.__crt.get_turn(max_turn=2) + 1.83, 2.))

    def recenter(self, angle=0):
        pass

    def current_heading(self) -> float:
        return self.__current_angle

    def set_heading(self, angle: float):
        pass

    def initSendable(self, builder: SendableBuilder):
        def set_speed_ratio(val: float):
            self.__speed_ratio = val

        builder.addDoubleProperty("encoderA", self.__encoder_a.get, lambda v: None)
        builder.addDoubleProperty("encoderA_raw", self.__encoder_a.get_raw, lambda v: None)

        builder.addDoubleProperty("encoderB", self.__encoder_b.get, lambda v: None)
        builder.addDoubleProperty("encoderB_raw", self.__encoder_b.get_raw, lambda v: None)

        builder.addDoubleProperty("heading", self.current_heading, lambda v: None)

        builder.addDoubleProperty("speedRatio", lambda: self.__speed_ratio, set_speed_ratio)
