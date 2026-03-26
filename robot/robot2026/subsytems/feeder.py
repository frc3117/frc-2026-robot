from frctools import Component, WPI_CANSparkMax, WPI_CANSparkFlex


class Feeder(Component):
    __speed_motor: WPI_CANSparkMax
    __winch_motor: WPI_CANSparkFlex

    def __init__(self, speed_motor: WPI_CANSparkMax, winch_motor: WPI_CANSparkFlex):
        super().__init__()

        self.__speed_motor = speed_motor
        self.__winch_motor = winch_motor
