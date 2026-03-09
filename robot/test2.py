from frc_ballistic_solver import (Vector3,
                                  Range,
                                  BallisticSimState,
                                  BallisticSimulator,
                                  Projectile,
                                  HybridBallisticSolver,
                                  BallisticSolver,
                                  BallisticSolution)


import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

import numpy as np

import time


#img = plt.imread('FRC_2026_Field.png')


X_MIN = 0.
X_MAX = 16.54
X_RANGE = Range(X_MIN, X_MAX)

Y_MIN = 0.
Y_MAX = 8.07
Y_RANGE = Range(Y_MIN, Y_MAX)

Z_MIN = 0.
Z_MAX = 2.54
Z_RANGE = Range(Z_MIN, Z_MAX)

TOTAL_STEP = 10000
DT = 0.01

ball = Projectile(0.0176, 0.227, 0.0008625)

def from_unit(v_unity: Vector3) -> Vector3:
    return Vector3(
        v_unity.z,
        -v_unity.x,
        v_unity.y
    )


field_center = Vector3(X_RANGE.center(), Y_RANGE.center(), 0)

target_pos = from_unit(Vector3(3.661, 1.486, 0.)) + field_center
pos = from_unit(Vector3(-0.015, 0.437, 1.34)) + field_center
velocity = Vector3()

print(target_pos)
print(pos)

fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')
ax.set_aspect('equal', adjustable='box')

ax.set_box_aspect((X_MAX-X_MIN, Y_MAX-Y_MIN, Z_MAX-Z_MIN))


def solve_and_simulate(solver, plot_color, euler: bool):
    start_time = time.perf_counter()
    solution: BallisticSolution = solver.solve(pos, velocity)
    print(solution)
    end_time = time.perf_counter()

    initial_state = BallisticSimState()
    initial_state.position = pos
    initial_state.linear_velocity = velocity + solution.aim_dir * solution.speed
    initial_state.angular_velocity = Vector3(0., 0., 0.)
    initial_state.time = 0.

    x = [initial_state.position.x]
    y = [initial_state.position.y]
    z = [initial_state.position.z]

    simulator = BallisticSimulator(ball, initial_state, DT)
    for i in range(TOTAL_STEP):
        if euler:
            step = simulator.do_step_euler()
        else:
            step = simulator.do_step()

        x.append(step.position.x)
        y.append(step.position.y)
        z.append(step.position.z)

        if step.position.z < 0:
            break

    elapsed = end_time - start_time
    print(elapsed)

    ax.plot(x, y, z, color=plot_color)


#solver = HybridBallisticSolver(
#    target_pos=target_pos,
#    speed_range=Range(0.1, 25),
#    airtime_range=Range(0.5, 3.),
#    impact_cone_tolerance=1.57,
#    sample_count=25,
#    projectile=ball,
#    refinement_passes=3
#)

simple_solver = BallisticSolver(
    target_pos=target_pos,
    speed_range=Range(0.1, 25),
    airtime_range=Range(0.5, 3.),
    impact_cone_tolerance=1.57,
    sample_count=25,
)
solve_and_simulate(simple_solver, (0, 1, 1), True)

hybrid_solver = HybridBallisticSolver(
    target_pos=target_pos,
    speed_range=Range(0.1, 25),
    airtime_range=Range(0.5, 3.),
    impact_cone_tolerance=1.57,
    sample_count=25,
    projectile=ball,
    refinement_passes=3,
    dt=DT,
    convergence_threshold=0.03
)
solve_and_simulate(hybrid_solver, (0, 0, 1), True)

ax.scatter3D([pos.x, target_pos.x], [pos.y, target_pos.y], [pos.z, target_pos.z], c=[(1, 0, 0), (0, 1, 0)])
#ax.imshow(img)

ax.set_xlim(X_MIN, X_MAX)
ax.set_ylim(Y_MIN, Y_MAX)
ax.set_zlim(Z_MIN, Z_MAX)
plt.show()
