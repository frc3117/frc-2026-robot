from frc_ballistic_solver import Range, Vector3, BallisticSolution, BallisticSolver, HybridBallisticSolver


import time


def from_unit(v_unity: Vector3) -> Vector3:
    return Vector3(
        v_unity.z,
        -v_unity.x,
        v_unity.y
    )


solver = HybridBallisticSolver(
    target_pos=from_unit(Vector3(3.661, 1.486, 0.)),
    speed_range=Range(0.1, 25),
    airtime_range=Range(0.5, 3.),
    impact_cone_tolerance=1.57,
    sample_count=25
)

pos = from_unit(Vector3(-0.015, 0.437, 1.34))
velocity = Vector3(0., 0., 0.)

start_time = time.perf_counter()
for i in range(50):
    sol = solver.solve(pos, velocity)
end_time = time.perf_counter()

elapsed = end_time - start_time
print(elapsed)
print(sol)

#print(ballistic_solver.add(1, 1))