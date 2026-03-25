import math

from frctools import Component, WPI_CANSparkMax, WPI_CANSparkFlex, Timer, CoroutineOrder
from frctools.sensor import Encoder
from frctools.frcmath import clamp, repeat, SlewRateLimiter
from frctools.controll import PID


from frc_ballistic_solver import CRT, CRTEncoder


from wpiutil import SendableBuilder
from wpilib import SmartDashboard


RPM_2_MPS = 0.001


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
    __target_heading: float = 0.

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

    __limiter: SlewRateLimiter

    __control_loop = None

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

    def init(self):
        self.__current_heading_raw = math.nan
        self.__evaluate_current_heading(False)

        self.set_target_heading(self.__current_heading)
        self.__limiter.last_value = self.__current_heading

        self.__current_elevation_raw = math.nan
        self.__evaluate_current_elevation()

    def update_disabled(self):
        self.__evaluate_current_heading(False)
        #self.__evaluate_current_elevation()
        #self.__evaluate_current_speed()

    def update(self):
        self.__evaluate_current_heading()
        #self.__evaluate_current_elevation()
        #self.__evaluate_current_speed()

        self.__control_loop = Timer.start_coroutine_if_stopped(self.__control_loop, self.__control_loop__, CoroutineOrder.LATE)

    def __control_loop__(self):
        while True:
            # Heading
            heading_error = self.__target_heading - self.__current_heading
            heading_output = self.__heading_pid.evaluate(heading_error)
            self.__heading_motor.set(clamp(heading_output, -0.35, 0.35))

            yield False

    def __evaluate_current_heading(self, with_limiter: bool = True):
        self.__crt_heading_encoder_a.set_angle01(self.__heading_encoder_a.get())
        self.__crt_heading_encoder_b.set_angle01(self.__heading_encoder_b.get())

        self.__current_heading_raw = self.__crt_heading.get_turn(max_turn=2.,
                                                                 previous_turn=self.__current_heading_raw,
                                                                 proximity_weight=-0.0005)

        instant_heading = repeat(self.__current_heading_raw - 1.05, 2.)
        if with_limiter:
            self.__current_heading = self.__limiter.evaluate(instant_heading)
        else:
            self.__current_heading = instant_heading

    def __evaluate_current_elevation(self):
        self.__current_elevation_raw = self.__elevation_encoder.get_raw()
        self.__current_elevation = self.__elevation_encoder.get()

    def __evaluate_current_speed(self):
        self.__current_speed_raw = (self.__speed_motor_a.get() + self.__speed_motor_b.get() / 2.)
        self.__current_speed = self.__current_speed_raw * RPM_2_MPS

    def recenter(self, angle=0):
        pass

    def current_heading(self) -> float:
        return self.__current_heading
    def current_heading_raw(self) -> float:
        return self.__current_heading_raw

    def set_target_heading(self, heading: float, field_relative: bool = True):
        # TODO: Add field relative
        self.__target_heading = heading
    def get_target_heading(self) -> float:
        return self.__target_heading

    def current_elevation(self) -> float:
        return self.__current_elevation
    def current_elevation_raw(self) -> float:
        return self.__current_elevation_raw

    def set_target_elevation(self, elevation: float):
        self.__target_elevation = elevation
    def get_target_elevation(self) -> float:
        return self.__target_elevation

    def current_speed(self) -> float:
        return self.__current_speed
    def current_speed_raw(self) -> float:
        return self.__current_speed_raw

    def set_target_speed(self, speed: float):
        self.__target_speed = speed
    def get_target_speed(self) -> float:
        return self.__target_speed

    def initSendable(self, builder: SendableBuilder):
        # Heading
        builder.addDoubleProperty("encoderA", self.__heading_encoder_a.get, lambda v: None)
        builder.addDoubleProperty("encoderA_raw", self.__heading_encoder_a.get_raw, lambda v: None)

        builder.addDoubleProperty("encoderB", self.__heading_encoder_b.get, lambda v: None)
        builder.addDoubleProperty("encoderB_raw", self.__heading_encoder_b.get_raw, lambda v: None)

        builder.addDoubleProperty("heading_raw", self.current_heading_raw, lambda v: None)
        builder.addDoubleProperty("heading", self.current_heading, lambda v: None)
        builder.addDoubleProperty("target_heading", self.get_target_heading, lambda v: self.set_target_heading(v))

        # Elevation
        builder.addDoubleProperty("elevation/encoder", self.current_elevation, lambda v: None)
        builder.addDoubleProperty("elevation/encoder_raw", self.current_elevation_raw, lambda v: None)
        builder.addDoubleProperty("elevation/target", self.get_target_elevation, self.set_target_elevation)

        # Speed
        builder.addDoubleProperty("speed/encoder", self.current_speed, lambda v: None)
        builder.addDoubleProperty("speed/encoder_raw", self.current_elevation_raw, lambda v: None)
        builder.addDoubleProperty("speed/target", self.get_target_speed, self.set_target_speed)

        # builder.addDoubleProperty("speedRatio", lambda: self.__speed_ratio, set_speed_ratio)
