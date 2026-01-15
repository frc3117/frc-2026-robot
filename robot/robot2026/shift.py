from frctools import Alliance, Timer


from typing import Dict
from enum import IntFlag, IntEnum


from wpilib import DriverStation


class ShiftPeriod(IntEnum):
    NONE: int = -1
    AUTO: int = 0
    TRANSITION: int = 1
    FIRST: int = 2
    SECOND: int = 3
    THIRD: int = 4
    FOURTH: int = 5
    ENDGAME: int = 6

    @staticmethod
    def get_current():
        Timer.get_current_field_time()


class ShiftState(IntFlag):
    RED: int = 0b01
    BLUE: int = 0b10

    NONE: int = 0b00
    BOTH: int = 0b11

    @staticmethod
    def get_current() -> 'ShiftState':
        data = DriverStation.getGameSpecificMessage()
        match data:
            case 'R':
                return ShiftState.RED

            case 'B':
                return ShiftState.BLUE

            case _:
                return ShiftState.NONE

class Shift:
    __IS_INITIALIZED__: bool = False
    __SHIFTS__: Dict[ShiftPeriod, 'Shift']
    __SHIFT_ORDER__: list[ShiftPeriod] = [
        ShiftPeriod.TRANSITION,
        ShiftPeriod.FIRST,
        ShiftPeriod.SECOND,
        ShiftPeriod.THIRD,
        ShiftPeriod.FOURTH,
        ShiftPeriod.ENDGAME
    ]

    period: ShiftPeriod
    start_time: float
    end_time: float

    def __init__(self, period: ShiftPeriod, start_time: float, end_time: float):
        self.period = period
        self.start_time = start_time
        self.end_time = end_time

    @staticmethod
    def get_shift(time: float, is_auto: bool) -> 'Shift':
        if not Shift.__IS_INITIALIZED__:
            Shift.__initialize__()

        if is_auto:
            return Shift.__SHIFTS__[ShiftPeriod.AUTO]

        for period in Shift.__SHIFT_ORDER__:
            shift = Shift.__SHIFTS__.get(period)
            if shift.start_time <= time:
                return shift

        return Shift.__SHIFTS__[ShiftPeriod.ENDGAME]

    @staticmethod
    def from_period(period: ShiftPeriod):
        if not Shift.__IS_INITIALIZED__:
            Shift.__initialize__()

        return Shift.__SHIFTS__.get(period)

    @staticmethod
    def __initialize__():
        Shift.__IS_INITIALIZED__ = True

        Shift.__SHIFTS__ = {
            ShiftPeriod.NONE: Shift(ShiftPeriod.NONE, 0., 0.),
            ShiftPeriod.AUTO: Shift(ShiftPeriod.AUTO, 20., 0.),
            ShiftPeriod.TRANSITION: Shift(ShiftPeriod.TRANSITION, 140., 130.),
            ShiftPeriod.FIRST: Shift(ShiftPeriod.FIRST, 130., 105.),
            ShiftPeriod.SECOND: Shift(ShiftPeriod.SECOND, 105., 80.),
            ShiftPeriod.THIRD: Shift(ShiftPeriod.THIRD, 80., 55.),
            ShiftPeriod.FOURTH: Shift(ShiftPeriod.FOURTH, 55., 30.),
            ShiftPeriod.ENDGAME: Shift(ShiftPeriod.ENDGAME, 30., 0.)
        }

class CurrentShiftInfo:
    current_shift: Shift
    shift_state: ShiftState
    remaining_time: float

    def __init__(self, current_shift: Shift, shift_state: ShiftState):
        self.current_shift = current_shift
        self.shift_state = shift_state

        self.remaining_time = Timer.get_current_field_time() - self.current_shift.end_time


class ShiftUtils:
    __FIRST_SHIFT_ALLIANCE__: Alliance = Alliance.UNDEFINED
    __CURRENT_SHIFT__INFO__: CurrentShiftInfo

    @staticmethod
    def get_current_shift() -> CurrentShiftInfo:
        return ShiftUtils.__CURRENT_SHIFT__INFO__

    @staticmethod
    def refresh_shift():
        # If the robot is disabled. Do not check for shifts
        if not Timer.is_enabled():
            ShiftUtils.reset()
            return

        # If the alliance with the first shift is not defined yet, check if we can find it (Only in teleop)
        if not Timer.is_auto() and ShiftUtils.__FIRST_SHIFT_ALLIANCE__ == Alliance.UNDEFINED:
            match ShiftState.get_current():
                case ShiftState.RED:
                    ShiftUtils.__FIRST_SHIFT_ALLIANCE__ = Alliance.RED

                case ShiftState.BLUE:
                    ShiftUtils.__FIRST_SHIFT_ALLIANCE__ = Alliance.BLUE

        # Get which alliance can score based on the current shift period
        shift = Shift.get_shift(Timer.get_current_field_time(), Timer.is_auto())
        match shift.period:
            # For those shifts, both alliance can score in their hub
            case ShiftPeriod.AUTO | ShiftPeriod.TRANSITION | ShiftPeriod.ENDGAME:
                state = ShiftState.BOTH

            # Those shift, only the alliance that scored the least fuel in auto can score in their hub
            case ShiftPeriod.FIRST | ShiftPeriod.THIRD:
                if ShiftUtils.__FIRST_SHIFT_ALLIANCE__ == Alliance.RED:
                    state = ShiftState.RED
                elif ShiftUtils.__FIRST_SHIFT_ALLIANCE__ == Alliance.BLUE:
                    state = ShiftState.BLUE
                else:
                    state = ShiftState.get_current()

            # Those shift, only the alliance that scored the most fuel in auto can score in their hub
            case ShiftPeriod.SECOND | ShiftPeriod.FOURTH:
                if ShiftUtils.__FIRST_SHIFT_ALLIANCE__ == Alliance.RED:
                    state = ShiftState.BLUE
                elif ShiftUtils.__FIRST_SHIFT_ALLIANCE__ == Alliance.BLUE:
                    state = ShiftState.RED
                else:
                    state = ShiftState.get_current()

            # Unknown shift likely due to not being run with the FMS
            case _:
                state = ShiftState.NONE

        # Update the current shift info with the latest shift info
        ShiftUtils.__CURRENT_SHIFT__INFO__ = CurrentShiftInfo(shift, state)

    @staticmethod
    def reset():
        __FIRST_SHIFT_ALLIANCE__ = Alliance.UNDEFINED
        __CURRENT_SHIFT__INFO__ = CurrentShiftInfo(Shift.from_period(ShiftPeriod.NONE), ShiftState.NONE)
