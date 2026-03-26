from frctools import Component, WPI_CANSparkFlex, RobotBase


class Indexer(Component):
    __cassette_motor: WPI_CANSparkFlex

    def __init__(self, cassette_motor: WPI_CANSparkFlex):
        super().__init__()

        self.__cassette_motor = cassette_motor
