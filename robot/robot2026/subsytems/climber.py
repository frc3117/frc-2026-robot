from frctools import Component
from frctools.sensor import Encoder


import wpiutil


class Climber(Component):
    __height_encoder: Encoder = None
    __height_motor = None
    __target_height: float = 0.

    def __init__(self, height_encoder: Encoder, height_motor):
        super().__init__()

        self.height_encoder = height_encoder
        self.height_motor = height_motor

        self.__target_height = self.height_encoder.get()

    def get_current_height(self) -> float:
        return self.height_encoder.get()

    def get_target_height(self) -> float:
        return self.__target_height

    def set_target_height(self, height: float):
        self.__target_height = height

    def is_at_target(self) -> bool:
        return False

    def wait_until_target(self):
        yield from ()
        while not self.is_at_target():
            yield None

    def initSendable(self, builder: wpiutil.SendableBuilder):
        builder.addDoubleProperty("height", self.get_current_height, lambda v: None)
        builder.addDoubleProperty("target_height", self.get_target_height, self.set_target_height)
        builder.addBooleanProperty("")
