from dataclasses import dataclass
from enum import StrEnum
from typing import Union


from frctools import WPI_CANSparkMax, WPI_CANSparkFlex


class ControllerType(StrEnum):
    CAN_SPARK_MAX = 'can_spark_max'
    CAN_SPARK_FLEX = 'can_spark_flex'


@dataclass
class MotorSettings:
    controller_type: ControllerType
    can_id: int
    brushless: bool
    brake: bool
    inverted: bool

    @property
    def motor(self) -> Union[WPI_CANSparkMax, WPI_CANSparkFlex, None]:
        match self.controller_type:
            case ControllerType.CAN_SPARK_MAX:
                return WPI_CANSparkMax(self.can_id,
                                       self.brushless,
                                       self.brake,
                                       self.inverted)

            case ControllerType.CAN_SPARK_FLEX:
                return WPI_CANSparkFlex(self.can_id,
                                        self.brushless,
                                        self.brake,
                                        self.inverted)

        return None
