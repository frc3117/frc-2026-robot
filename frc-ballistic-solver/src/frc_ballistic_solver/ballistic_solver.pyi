from typing import overload, List


class Range:
    min: float
    max: float

    @overload
    def __init__(self): ...
    @overload
    def __init__(self, min: float, max: float): ...

    def lerp(self, t: float) -> float: ...
    def between(self, v: float) -> bool: ...
    def clamp(self, v: float) -> float: ...
    def center(self) -> float: ...
    def width(self) -> float: ...


class Vector2:
    x: float
    y: float

    @overload
    def __init__(self): ...
    @overload
    def __init__(self, x: float, y: float): ...


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


class Projectile:
    OBJ_AREA: float
    OBJ_MASS: float
    OBJ_MOMENT_INERTIA: float
    AIR_DENSITY: float
    DRAG_COEF: float
    MAGNUS_COEF: float

    def __init__(self,
                 obj_area: float,
                 obj_mass: float,
                 obj_moment_inertia: float,
                 air_density: float = 1.293,
                 drag_coef: float = 0.47,
                 magnus_coef: float = 0.000791): ...


class BallisticSimState:
    time: float

    position: Vector3
    rotation: Vector3

    linear_velocity: Vector3
    angular_velocity: Vector3


class BallisticSimulator:
    def __init__(self,
                 projectile: Projectile,
                 initial_state: BallisticSimState,
                 dt: float,
                 gravity: Vector3 = Vector3(0., 0., -9.81)): ...

    def do_step(self) -> BallisticSimState: ...


class BallisticSolution:
    valid: bool
    time: float
    speed: float
    ang_speed: float
    aim_dir: Vector3
    impact_vel: Vector3
    impact_dir: Vector3
    heading: float
    elevation: float
    score: float

    def get_angular_velocity(self) -> Vector3: ...


class HybridBallisticSolver:
    def __init__(self,
                 target_pos: Vector3,
                 speed_range: Range,
                 airtime_range: Range,
                 projectile: Projectile,
                 impact_cone_axis: Vector3 = Vector3(0., 0., -1.),
                 impact_cone_tolerance: float = 0.785,
                 gravity: Vector3 = Vector3(0., 0., -9.81),
                 sample_count: int = 15,
                 prefer_high_arc: bool = True,
                 refinement_passes: int = 3,
                 convergence_threshold: float = 0.02,
                 dt: float = 0.01,
                 magnus_ratio: float = 1.,
                 ball_radius: float = 0.072,
                 bottom_wheel_radius: float = 0.0048,
                 top_wheel_radius: float = 1.): ...

    def solve(self, shooter_pos: Vector3, shooter_world_vel) -> BallisticSolution: ...


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


class FieldZone:
    ID: int

    def __init__(self, id: int, vertices: List[Vector2]): ...

    def is_point_in_zone(self, p: Vector2) -> bool: ...


class Field:
    def __init__(self, zones: List[FieldZone]): ...

    def point_zone(self, p: Vector2) -> int: ...