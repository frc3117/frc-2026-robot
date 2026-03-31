from frctools import Component, WPI_CANSparkFlex, RobotBase, Coroutine, CoroutineOrder, Timer


class Indexer(Component):
    __cassette_motor: WPI_CANSparkFlex
    __pre_feeder_motor: WPI_CANSparkFlex

    __is_indexing: bool = False

    __control_loop: Coroutine = None

    def __init__(self, cassette_motor: WPI_CANSparkFlex, pre_feeder_motor: WPI_CANSparkFlex):
        super().__init__()

        self.__cassette_motor = cassette_motor
        self.__pre_feeder_motor = pre_feeder_motor

    def update(self):
        self.__control_loop = Timer.start_coroutine_if_stopped(self.__control_loop__, self.__control_loop, CoroutineOrder.LATE)

    def __control_loop__(self):
        yield from ()

        while True:
            yield False

            if self.__is_indexing:
                self.__cassette_motor.set(1.)
                self.__pre_feeder_motor.set(0.65)
            else:
                self.__cassette_motor.set(0.)
                self.__pre_feeder_motor.set(0.)

    def start_indexing(self):
        self.__is_indexing = True

    def stop_indexing(self):
        self.__is_indexing = False

    def set_indexing(self, state: bool):
        if state:
            self.start_indexing()
        else:
            self.stop_indexing()
