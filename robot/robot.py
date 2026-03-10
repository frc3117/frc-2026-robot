from frctools import (RobotBase, WPI_CANSparkMax)
from frctools.sensor import Encoder
from frctools.input import (Input,
                            XboxControllerInput,
                            PowerTransform)

from robot2026.subsytems import Shooter

#from robot2026 import ShiftUtils
#from robot2026.subsytems import (Climber,
#                                 Feeder,
#                                 Shooter)


from wpilib import (ADIS16448_IMU,
                    SPI,
                    DutyCycleEncoder)


class Robot(RobotBase):
    test_input: Input

    shooter: Shooter

    def register_inputs(self):
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
        # Subsystems
        #climber = Climber()
        #self.add_component('Climber', climber)

        #feeder = Feeder()
        #self.add_component('Feeder', feeder)

        self.shooter = Shooter(Encoder(DutyCycleEncoder(0), 0.), Encoder(DutyCycleEncoder(1), 0.), WPI_CANSparkMax(4, True, True))
        self.add_component('Shooter', self.shooter)

    def robotInit(self):
        super().robotInit()

        self.register_inputs()
        self.register_subsystems()

    def robotPeriodic(self):
        super().robotPeriodic()

    def disabledInit(self):
        #ShiftUtils.reset()

        super().disabledInit()
