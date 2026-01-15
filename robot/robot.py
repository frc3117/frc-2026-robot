from frctools import (RobotBase,
                      WPI_CANSparkMax,
                      WPI_CANSparkFlex)
from frctools.input import (Input,
                            XboxControllerInput,
                            PowerTransform)
from frctools.drivetrain import (SwerveDrive,
                                 SwerveModule,
                                 SwerveDriveMode)
from frctools.sensor import Encoder
from frctools.controll import PID
from frctools.frcmath import Vector2

from robot2026 import ShiftUtils
from robot2026.subsytems import (Climber,
                                 Feeder,
                                 Shooter)


from wpilib import (ADIS16448_IMU,
                    SPI,
                    DutyCycleEncoder)


class Robot(RobotBase):
    def register_inputs(self):
        # Swerve Inputs
        Input.add_axis('horizontal',
                       0,
                       XboxControllerInput.LEFT_JOYSTICK_Y,
                       deadzone=0.02,
                       # axis_filter=SlewRateLimiter(600),
                       axis_transform=PowerTransform(1))
        Input.add_axis('vertical',
                       0,
                       XboxControllerInput.LEFT_JOYSTICK_X,
                       inverted=True,
                       deadzone=0.02,
                       # axis_filter=SlewRateLimiter(600),
                       axis_transform=PowerTransform(1))

        Input.create_composite_axis('rotation',
                                    positive=Input.add_axis('rotation_pos',
                                                            0,
                                                            XboxControllerInput.RIGHT_TRIGGER,
                                                            deadzone=0.02,
                                                            axis_transform=PowerTransform(1)),
                                    negative=Input.add_axis('rotation_neg',
                                                            0,
                                                            XboxControllerInput.LEFT_TRIGGER,
                                                            deadzone=0.02,
                                                            axis_transform=PowerTransform(1)),
                                    )

        # Subsystems Inputs

    def register_subsystems(self):
        # Swerve

        #  FL   FR
        #   0---1
        #   |   |
        #   3---2
        #  RL   RR

        swerve_modules = [
            SwerveModule(drive_motor=WPI_CANSparkFlex(1, True, brake=True, inverted=True),
                         steering_motor=WPI_CANSparkMax(2, True, brake=True),
                         steering_encoder=Encoder(DutyCycleEncoder(0), 0, False),
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         position=Vector2(-1., 1.)),

            SwerveModule(drive_motor=WPI_CANSparkFlex(3, True, brake=True, inverted=True),
                         steering_motor=WPI_CANSparkMax(4, True, brake=True),
                         steering_encoder=Encoder(DutyCycleEncoder(1), 0, False),
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         position=Vector2(1., 1.)),

            SwerveModule(drive_motor=WPI_CANSparkFlex(5, True, brake=True, inverted=True),
                         steering_motor=WPI_CANSparkMax(6, True, brake=True),
                         steering_encoder=Encoder(DutyCycleEncoder(2), 0, False),
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         position=Vector2(1., -1.)),

            SwerveModule(drive_motor=WPI_CANSparkFlex(7, True, brake=True, inverted=True),
                         steering_motor=WPI_CANSparkMax(8, True, brake=True),
                         steering_encoder=Encoder(DutyCycleEncoder(3), 0, False),
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         position=Vector2(-1., -1.))
        ]

        swerve = SwerveDrive(swerve_modules, imu=ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s), start_heading=0)
        swerve.set_drive_mode(SwerveDriveMode.FIELD_CENTRIC)
        swerve.set_cosine_compensation(True)
        self.add_component('Swerve', swerve)

        # Subsystems
        climber = Climber()
        self.add_component('Climber', climber)

        feeder = Feeder()
        self.add_component('Feeder', feeder)

        shooter = Shooter()
        self.add_component('Shooter', shooter)

    def robotInit(self):
        super().robotInit()

        self.register_inputs()
        self.register_subsystems()

    def robotPeriodic(self):
        ShiftUtils.refresh_shift()

        super().robotPeriodic()

    def disabledInit(self):
        ShiftUtils.reset()

        super().disabledInit()
