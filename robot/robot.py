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
from frctools.frcmath import Vector2, Vector3

import time
import math


from wpilib import (ADIS16448_IMU,
                    SPI, AnalogEncoder,
                    DutyCycleEncoder,
                    DigitalInput)


class Robot(RobotBase):
    test_input: Input

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

    def register_subsystems(self):
        # Swerve

        #  FL      FR
        #   3------0
        #   |      |
        #   2------1
        #  RL      RR

        swerve_modules = [
                SwerveModule(drive_motor=Robot.WPI_TalonFX(8, brake=True),
                             steering_motor=Robot.WPI_CANSparkMax(7, brushless=True, brake=True),
                             steering_encoder=AnalogEncoder(3),
                             steering_controller=PID(1, 0, 0),
                             steering_offset=0.964010-0.25,
                             position=Vector2(0.256, 0.312)),

                SwerveModule(drive_motor=Robot.WPI_TalonFX(6, brake=True),
                             steering_motor=Robot.WPI_CANSparkMax(5, brushless=True, brake=True),
                             steering_encoder=AnalogEncoder(0),
                             steering_controller=PID(1, 0, 0),
                             steering_offset=0.6943,
                             position=Vector2(0.256, -0.312)),

                SwerveModule(drive_motor=Robot.WPI_TalonFX(4, inverted=True, brake=True),
                             steering_motor=Robot.WPI_CANSparkMax(3, brushless=True, brake=True),
                             steering_encoder=AnalogEncoder(2),
                             steering_controller=PID(1, 0, 0),
                             steering_offset=0.3196,
                             position=Vector2(-0.256, -0.312)),

                SwerveModule(drive_motor=Robot.WPI_TalonFX(2, inverted=True, brake=True),
                             steering_motor=Robot.WPI_CANSparkMax(1, brushless=True, brake=True),
                             steering_encoder=AnalogEncoder(1),
                             steering_controller=PID(1, 0, 0),
                             steering_offset=0.9142,
                             position=Vector2(-0.256, 0.312)),
            ]

        swerve = SwerveDrive(modules=swerve_modules,
                             imu=ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s),
                             start_heading=math.pi)

        swerve = SwerveDrive(swerve_modules, imu=ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s), start_heading=0)
        swerve.set_drive_mode(SwerveDriveMode.FIELD_CENTRIC)
        swerve.set_cosine_compensation(True)
        self.add_component('Swerve', swerve)

    def robotInit(self):
        super().robotInit()

        self.register_inputs()
        self.register_subsystems()

    def disabledInit(self):
        # ShiftUtils.reset()

        super().disabledInit()