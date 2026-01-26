from typing import overload


class Range:
    min: float
    max: float

    @overload
    def __init__(self): ...
    @overload
    def __init__(self, min: float, max: float): ...


class Vector3:
    x: float
    y: float
    z: float

    @overload
    def __init__(self): ...
    @overload
    def __init__(self, x: float, y: float, z: float): ...

    def sqr_magnitude(self) -> float: ...
    def magnitude(self) -> float: ...

    def dot(self, other: 'Vector3') -> float: ...

    @overload
    def __add__(self, other: Vector3) -> Vector3: ...
    @overload
    def __add__(self, other: float) -> Vector3: ...
    @overload
    def __add__(self, other: int) -> Vector3: ...

    @overload
    def __sub__(self, other: Vector3) -> Vector3: ...
    @overload
    def __sub__(self, other: float) -> Vector3: ...
    @overload
    def __sub__(self, other: int) -> Vector3: ...

    @overload
    def __mul__(self, other: Vector3) -> Vector3: ...
    @overload
    def __mul__(self, other: float) -> Vector3: ...
    @overload
    def __mul__(self, other: int) -> Vector3: ...

    @overload
    def __div__(self, other: Vector3) -> Vector3: ...
    @overload
    def __div__(self, other: float) -> Vector3: ...
    @overload
    def __div__(self, other: int) -> Vector3: ...


class BallisticSolution:
    valid: bool
    time: float
    speed: float
    aim_dir: Vector3
    impact_vel: Vector3
    impact_dir: Vector3
    heading: float
    elevation: float
    score: float


class BallisticSolver:
    def __init__(self,
                 target_pos: Vector3,
                 speed_range: Range,
                 airtime_range: Range,
                 impact_cone_axis: Vector3 = Vector3(0., 0., -1),
                 impact_cone_tolerance: float = 0.785,
                 gravity: Vector3 = Vector3(0., 0., -9.81),
                 sample_count: int=15,
                 prefer_high_arc: bool=True): ...

    def solve(self, shooter_pos: Vector3, shooter_world_vel: Vector3) -> BallisticSolution: ...
