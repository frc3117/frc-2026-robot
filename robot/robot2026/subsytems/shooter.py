from collections import deque

import math


from frctools import Component, WPI_CANSparkMax, WPI_CANSparkFlex, Timer, CoroutineOrder
from frctools.sensor import Encoder
from frctools.frcmath import clamp, repeat, SlewRateLimiter, remap, Vector2 as v2
from frctools.controll import PID
from frctools.drivetrain import SwerveDrive


from frc_ballistic_solver import CRT, CRTEncoder, Vector3, Vector2


from ..pose import RobotPoseEstimator

from wpiutil import SendableBuilder
from wpilib import SmartDashboard
from wpimath.filter import LinearFilter
from wpimath.geometry import Pose2d


# ((2 * pi * r) / 60) * 2
# RPM = v /
SHOOTER_WHEEL_RADIUS = 0.047625
MPS_TO_RPM = ((math.pi * SHOOTER_WHEEL_RADIUS) / 60) * 0.9

# 0.25 Hard-Min
# 1.66 -> Hard-Max

# 0 rad -> -0.7632 turn
# 2pi rad -> 0.7632 turn
# 4pi rad -> 1.7632 turn


# Arrow on bolt angle 1.012


MIN_HARDSTOP_TURN = 0.29
MAX_HARDSTOP_TURN = 1.73

MIN_TURN = -0.7632
MAX_TURN = 1.7632
RANGE_TURN = MAX_TURN - MIN_TURN

MIN_ANGLE = 0.
MAX_ANGLE = 4. * math.pi
RANGE_ANGLE = MAX_ANGLE - MIN_ANGLE

TURRET_LOCAL_POS = Vector3(-0.12, 0.19, 0.4)
#TURRET_LOCAL_POS = Vector3(0.19, -0.12, 0.4)


ELEVATION_MIN = 0.372 - 0.122
ELEVATION_MAX = 0.829 - 0.122


ELEVATION_MIN_HARDSTOP = 0.385 - 0.122
ELEVATION_MAX_HARDSTOP = 0.82 - 0.122


class AngleFilter:
    def __init__(self, count: int):
        self.__buffer = deque(maxlen=count)
        self.__value = 0.

    def reset(self):
        self.__buffer.clear()
        self.__value = 0.

    def evaluate(self, value: float) -> float:
        vec = v2(math.cos(value), math.sin(value))
        self.__buffer.append(vec)

        vec_sum = v2()
        for v in self.__buffer:
            vec_sum += v

        vec_avg = vec_sum / float(len(self.__buffer))
        self.__value = math.atan2(vec_avg.y, vec_avg.x)

        return self.__value

    def value(self) -> float:
        return self.__value


def __elevation_encoder_to_angle__(val: float) -> float:
    return 0.372 + ((val - 0.01) * math.tau) - 0.122


def __normalize_angle__(angle: float) -> float:
    return repeat(angle - MIN_ANGLE, RANGE_ANGLE) + MIN_ANGLE


def __normalize_turn__(turn: float) -> float:
    return repeat(turn - MIN_TURN, RANGE_TURN) + MIN_TURN


def __angle_to_turn__(angle: float) -> float:
    #angle = __normalize_angle__(angle)
    return remap(MIN_ANGLE, MAX_ANGLE, MIN_TURN, MAX_TURN, angle)


def __turn_to_angle__(turn: float) -> float:
    #turn = __normalize_turn__(turn)
    return remap(MIN_TURN, MAX_TURN, MIN_ANGLE, MAX_ANGLE, turn)


class Shooter(Component):
    # Heading
    __encoder_a: Encoder
    __encoder_b: Encoder
    __heading_motor: WPI_CANSparkMax
    __heading_pid: PID

    __crt_heading_encoder_a: CRTEncoder
    __crt_heading_encoder_b: CRTEncoder
    __crt_heading: CRT

    __current_heading_raw: float = math.nan
    __current_heading: float = 0.
    __current_heading_world: float = 0.
    __target_heading: float = 0.
    __target_heading_field_relative: bool = False

    # Elevation
    __elevation_encoder: Encoder
    __elevation_motor: WPI_CANSparkMax
    __elevation_pid: PID

    __current_elevation_raw: float = 0.
    __current_elevation: float = 0.
    __target_elevation: float = 0.

    # Speed
    __speed_motor_a: WPI_CANSparkFlex
    __speed_motor_b: WPI_CANSparkFlex
    __speed_pid: PID

    __current_speed_raw: float = 0.
    __current_speed: float = 0.
    __target_speed: float = 0.

    # __speed_input: Input
    # __speed_ratio: float = 0.1

    __is_shooting: bool = False

    __limiter: SlewRateLimiter
    __robot_heading_filter: AngleFilter

    __control_loop = None

    __swerve: SwerveDrive
    __pose: RobotPoseEstimator

    def __init__(self,
                 heading_encoder_a: Encoder,
                 heading_encoder_b: Encoder,
                 heading_motor: WPI_CANSparkMax,
                 heading_pid: PID,
                 elevation_encoder: Encoder,
                 elevation_motor: WPI_CANSparkMax,
                 elevation_pid: PID,
                 speed_motor_a: WPI_CANSparkFlex,
                 speed_motor_b: WPI_CANSparkFlex,
                 speed_pid: PID):
        super().__init__()

        # Heading
        self.__heading_encoder_a = heading_encoder_a
        self.__heading_encoder_b = heading_encoder_b
        self.__heading_motor = heading_motor
        self.__heading_pid = heading_pid

        self.__crt_heading_encoder_a = CRTEncoder(18, 207, 1.)
        self.__crt_heading_encoder_b = CRTEncoder(23, 207, 1.)
        self.__crt_heading = CRT(self.__crt_heading_encoder_a, self.__crt_heading_encoder_b, 207)

        self.__limiter = SlewRateLimiter(4)
        self.__robot_heading_filter = AngleFilter(1)

        # Elevation
        self.__elevation_encoder = elevation_encoder
        self.__elevation_motor = elevation_motor
        self.__elevation_pid = elevation_pid

        # Speed
        self.__speed_motor_a = speed_motor_a
        self.__speed_motor_b = speed_motor_b
        self.__speed_pid = speed_pid

        SmartDashboard.putData('Shooter/heading/pid', self.__heading_pid)
        SmartDashboard.putData('Shooter/elevation/pid', self.__elevation_pid)
        SmartDashboard.putData('Shooter/speed/pid', self.__speed_pid)

        #SmartDashboard.putData('Shooter/elevation/pid', self.__elevation_pid)


    def init(self):
        self.__swerve = self.robot.get_component('Swerve')
        self.__pose = self.robot.get_component('Pose_Estimator')

        self.__robot_heading_filter.reset()

        self.__current_heading_raw = math.nan
        self.__evaluate_current_heading(False)

        self.set_target_heading(self.__current_heading, False)
        self.__limiter.last_value = self.__current_heading

        self.__evaluate_current_elevation()
        self.set_target_elevation(self.__current_elevation)

    def update_disabled(self):
        self.__evaluate_current_heading(False)
        self.__evaluate_current_elevation()
        self.__evaluate_current_speed()

    def update(self):
        self.__evaluate_current_heading(True)
        self.__evaluate_current_elevation()
        self.__evaluate_current_speed()

        self.__control_loop = Timer.start_coroutine_if_stopped(self.__control_loop__, self.__control_loop, CoroutineOrder.LATE)

    def __control_loop__(self):
        while True:
            yield False

            #self.set_target_elevation(remap(-1, 1, ELEVATION_MIN_HARDSTOP, ELEVATION_MAX_HARDSTOP, math.sin(7.5 * Timer.get_current_time())))

            # Heading
            if self.__target_heading_field_relative:
                current_heading_norm = self.__current_heading_world
            else:
                current_heading_norm = self.__current_heading

            # else:
            # current_heading_norm = 0.7632

            if current_heading_norm > self.__target_heading:
                error_down = current_heading_norm - self.__target_heading
                error_up = 1 - error_down
            else:
                error_up = self.__target_heading - current_heading_norm
                error_down = 1 - error_up

            next_up = self.__current_heading + error_up
            next_down = self.__current_heading - error_down

            heading_error = 0.
            if error_up <= error_down:
                if next_up <= MAX_HARDSTOP_TURN:
                    heading_error = error_up
                elif next_down >= MIN_HARDSTOP_TURN:
                    heading_error = -error_down
            elif error_down <= error_up:
                if next_down >= MIN_HARDSTOP_TURN:
                    heading_error = -error_down
                elif next_up <= MAX_HARDSTOP_TURN:
                    heading_error = error_up

            # heading_error = self.__target_heading - self.__current_heading
            #heading_error = abs(heading_error) * (heading_error / (abs(heading_error) + 0.015))
            heading_output = self.__heading_pid.evaluate(heading_error)
            self.__heading_motor.set(clamp(heading_output, -0.55, 0.55))

            # Elevation
            elevation_error = self.__target_elevation - self.__current_elevation

            elevation_output = self.__elevation_pid.evaluate(elevation_error)
            self.__elevation_motor.set(clamp(elevation_output, -0.6, 0.6))

            # Speed
            if self.__is_shooting:
                speed_error = self.__target_speed - self.__current_speed

                speed_output = self.__speed_pid.evaluate(speed_error, feed_forward=self.__target_speed)
                speed_output = max(-0.25, speed_output)

                self.__speed_motor_a.set_voltage(speed_output)
                self.__speed_motor_b.set_voltage(speed_output)
            else:
                self.__speed_motor_a.set(0.)
                self.__speed_motor_b.set(0.)
                self.__speed_pid.reset_integral()

    def __evaluate_current_heading(self, with_limiter: bool = True):
        self.__crt_heading_encoder_a.set_angle01(self.__heading_encoder_a.get())
        self.__crt_heading_encoder_b.set_angle01(self.__heading_encoder_b.get())

        try:
            self.__current_heading_raw = self.__crt_heading.get_turn(max_turn=2.,
                                                                     previous_turn=self.__current_heading_raw,
                                                                     proximity_weight=-0.01)
        except TypeError:
            # Compatibility path for older frc_ballistic_solver builds
            self.__current_heading_raw = self.__crt_heading.get_turn(max_turn=2.)

        instant_heading = repeat(self.__current_heading_raw - 0.61, 2.)
        if with_limiter:
            self.__current_heading = self.__limiter.evaluate(instant_heading)
        else:
            self.__current_heading = instant_heading

        self.__current_heading_world = repeat(self.__current_heading + (-self.__swerve.get_heading() / math.tau), 1)

        self.__current_heading_world = self.__robot_heading_filter.evaluate((self.__current_heading * math.tau) - 0.1 + self.__pose.get_current_pose().rotation().radians())
        self.__current_heading_world = repeat(self.__current_heading_world / math.tau + 0.5, 1.)

    def __evaluate_current_elevation(self):
        self.__current_elevation_raw = self.__elevation_encoder.get_raw()
        self.__current_elevation = __elevation_encoder_to_angle__(self.__elevation_encoder.get())

    def __evaluate_current_speed(self):
        self.__current_speed_raw = self.__speed_motor_a.get()
        self.__current_speed = self.__current_speed_raw

    def current_heading(self) -> float:
        return self.__current_heading
    def current_heading_raw(self) -> float:
        return self.__current_heading_raw

    def current_heading_world(self) -> float:
        return self.__current_heading_world

    def current_heading_world_angle(self) -> float:
        return __turn_to_angle__(self.__current_heading_world)

    def hold_heading_(self):
        self.set_target_field_heading(self.__current_heading_world)

    def set_target_field_heading(self, heading: float):
        self.set_target_heading(heading, True)

    def set_target_heading_angle(self, heading: float, field_relative: bool = True):
        self.__target_heading_field_relative = field_relative
        self.__set_target_heading_internal__(__angle_to_turn__(heading))
    def set_target_heading(self, heading: float, field_relative: bool = True):
        self.__target_heading_field_relative = field_relative
        #turn = __normalize_turn__(heading)

        self.__set_target_heading_internal__(heading)
    def __set_target_heading_internal__(self, heading: float):
        self.__target_heading = repeat(heading, 1)
    def get_target_heading(self) -> float:
        return self.__target_heading

    def current_elevation(self) -> float:
        return self.__current_elevation
    def current_elevation_raw(self) -> float:
        return self.__current_elevation_raw

    def set_target_elevation(self, elevation: float):
        self.__target_elevation = clamp(elevation, ELEVATION_MIN_HARDSTOP, ELEVATION_MAX_HARDSTOP)
    def get_target_elevation(self) -> float:
        return self.__target_elevation

    def current_speed(self) -> float:
        return self.__current_speed
    def current_speed_raw(self) -> float:
        return self.__current_speed_raw

    def set_shooter_speed(self, speed: float):
        self.__target_speed = speed / MPS_TO_RPM
    def get_target_speed(self) -> float:
        return self.__target_speed

    def start_shooting(self):
        if not self.__is_shooting:
            self.__is_shooting = True
            self.set_target_heading(self.__current_heading, False)

    def stop_shooting(self):
        if self.__is_shooting:
            self.__is_shooting = False
            self.set_target_heading(self.__current_heading, False)

    def set_shooting(self, state: bool):
        if state:
            self.start_shooting()
        else:
            self.stop_shooting()

    def get_shooter_pose(self, robot_pos: Vector2, robot_heading: float) -> Vector3:
        rotated_turret_pos = Vector3(TURRET_LOCAL_POS.x*math.cos(robot_heading) - TURRET_LOCAL_POS.y*math.sin(robot_heading),
                                     TURRET_LOCAL_POS.x*math.sin(robot_heading) + TURRET_LOCAL_POS.y*math.cos(robot_heading),
                                     TURRET_LOCAL_POS.z)
        return Vector3(robot_pos.x + rotated_turret_pos.x, robot_pos.y + rotated_turret_pos.y, rotated_turret_pos.z)

    def initSendable(self, builder: SendableBuilder):
        # Heading
        builder.addDoubleProperty("encoderA", self.__heading_encoder_a.get, lambda v: None)
        builder.addDoubleProperty("encoderA_raw", self.__heading_encoder_a.get_raw, lambda v: None)

        builder.addDoubleProperty("encoderB", self.__heading_encoder_b.get, lambda v: None)
        builder.addDoubleProperty("encoderB_raw", self.__heading_encoder_b.get_raw, lambda v: None)

        builder.addDoubleProperty("heading_raw", self.current_heading_raw, lambda v: None)
        builder.addDoubleProperty("heading", self.current_heading, lambda v: None)
        builder.addDoubleProperty("heading_world", self.current_heading_world, lambda v: None)
        builder.addDoubleProperty("target_heading", self.get_target_heading, lambda v: self.set_target_heading(v))

        # Elevation
        builder.addDoubleProperty("elevation/encoder", self.current_elevation, lambda v: None)
        builder.addDoubleProperty("elevation/encoder_raw", self.current_elevation_raw, lambda v: None)
        builder.addDoubleProperty("elevation/target", self.get_target_elevation, self.set_target_elevation)

        # Speed
        builder.addDoubleProperty("speed/encoder", self.current_speed, lambda v: None)
        #builder.addDoubleProperty("speed/encoder_raw", self.current_elevation_raw, lambda v: None)
        builder.addDoubleProperty("speed/target", self.get_target_speed, self.set_shooter_speed)

        #builder.addDoubleProperty("speedRatio", lambda: self.__speed_ratio, set_speed_ratio)
