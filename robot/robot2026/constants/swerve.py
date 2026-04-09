from .comon import ControllerType, MotorSettings

from dataclasses import dataclass

import math


@dataclass
class SwerveModuleSettings:
    drive_motor: MotorSettings
    dir_motor: MotorSettings



SWERVE_FRONT_LEFT_DRIVE_MOTOR = MotorSettings(controller_type=ControllerType.CAN_SPARK_FLEX,
                                              can_id=2,
                                              brushless=True,
                                              brake=False,
                                              inverted=True)
#SWERVE_FRONT_LEFT_MODULE = SwerveModuleSettings(drive_motor=)

SWERVE_WHEEL_DIAMETER = 0.0952
SWERVE_DRIVE_GEAR_RATIO = 7.23
SWERVE_DRIVE_ENCODER_CONVERSION_FACTOR = (SWERVE_WHEEL_DIAMETER * math.pi) / SWERVE_DRIVE_GEAR_RATIO


__all__ = [
    'SWERVE_WHEEL_DIAMETER',
    'SWERVE_DRIVE_GEAR_RATIO',
    'SWERVE_DRIVE_ENCODER_CONVERSION_FACTOR'
]
