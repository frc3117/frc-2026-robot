from frctools import Component, WPI_CANSparkMax, WPI_CANSparkFlex, Coroutine, CoroutineOrder, Timer
from frctools.input import Input
from frctools.frcmath import clamp


from wpilib import DigitalInput


import wpiutil


class Feeder(Component):
    __speed_motor: WPI_CANSparkMax

    __winch_bottom_limit: DigitalInput
    __winch_top_limit: DigitalInput
    __winch_motor: WPI_CANSparkFlex

    __intake_motor_speed: float = 0.8

    __should_be_retracted = True
    __is_feeding: bool = False

    __controll_loop: Coroutine = None

    def __init__(self,
                 speed_motor: WPI_CANSparkMax,
                 winch_motor: WPI_CANSparkFlex,
                 top_limit: DigitalInput,
                 bot_limit: DigitalInput):
        super().__init__()

        self.__speed_motor = speed_motor
        self.__winch_motor = winch_motor
        self.__winch_top_limit = top_limit
        self.__winch_bottom_limit = bot_limit

    def update(self):
        self.__controll_loop = Timer.start_coroutine_if_stopped(self.__control_loop__, self.__controll_loop, CoroutineOrder.LATE)

        winch_output = 0.
        #if self.__temp_input.get():
        #    winch_output += self.__speed_motor_output
        #if self.__temp_input_neg.get():
        #    winch_output -= self.__speed_motor_output

        #self.__winch_motor.set(winch_output)

    def __control_loop__(self):
        yield from ()

        while True:
            yield False

            min_winch = 0. if self.is_retracted() else -1.
            max_winch = 0. if self.is_expended() else 1.

            if self.__should_be_retracted:
                winch_target = -1
            else:
                winch_target = 1

            self.__winch_motor.set(clamp(winch_target, min_winch, max_winch) * 0.6)

            if self.__is_feeding and self.is_expended():
                self.__speed_motor.set(self.__intake_motor_speed)
            else:
                self.__speed_motor.set(0.)

    def __logic_loop__(self):
        yield from ()

        while True:
            yield False

    def set_intake_speed(self, speed: float):
        self.__intake_motor_speed = speed

    def start_feeding(self):
        self.__is_feeding = True

    def stop_feeding(self):
        self.__is_feeding = False

    def set_feeding(self, state: bool):
        if state:
            self.start_feeding()
        else:
            self.stop_feeding()

    def set_retracted(self):
        self.__should_be_retracted = True

    def set_expended(self):
        self.__should_be_retracted = False

    def set_retracted_state(self, state: bool):
        self.__should_be_retracted = state

    def is_retracted(self) -> bool:
        return not self.__winch_top_limit.get()
    def wait_until_retracted(self) -> bool:
        yield from ()
        while not self.is_retracted():
            yield False

    def is_expended(self) -> bool:
        return not self.__winch_bottom_limit.get()
    def wait_until_expended(self):
        yield from ()
        while not self.is_expended():
            yield False

    def initSendable(self, builder: wpiutil.SendableBuilder):
        def set_speed_motor_output(v: float):
            self.__speed_motor_output = v

        builder.addDoubleProperty('speed/motor_output', lambda: self.__intake_motor_speed, set_speed_motor_output)

        builder.addBooleanProperty('winch/top_limit', self.is_retracted, lambda v: None)
        builder.addBooleanProperty('winch/bot_limit', self.is_expended, lambda v: None)
