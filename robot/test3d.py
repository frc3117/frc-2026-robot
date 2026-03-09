from threading import Thread


import open3d as o3d
import numpy as np


import frc_ballistic_solver as bal


TOTAL_STEP = 10000
DT = 0.01

PROJECTILE = bal.Projectile(0.5, 0.227, 0.0008625)

INITIAL_STATE = bal.BallisticSimState()
INITIAL_STATE.position = bal.Vector3(0., 0., 0.)
INITIAL_STATE.rotation = bal.Vector3(0., 0., 0.)
INITIAL_STATE.linear_velocity = bal.Vector3(5., 1., 25.)
INITIAL_STATE.angular_velocity = bal.Vector3(0., 0., 1.)

SIMULATOR = bal.BallisticSimulator(PROJECTILE, INITIAL_STATE, DT)


def get_trajectory_line():
    points = [[
        INITIAL_STATE.position.x,
        INITIAL_STATE.position.y,
        INITIAL_STATE.position.z
    ]]
    lines = []

    def add_point(p: bal.Vector3):
        p_i = len(points)

        points.append([p.x, p.y, p.z])
        lines.append([p_i-1, p_i])

    for i in range(TOTAL_STEP):
        step = SIMULATOR.do_step()
        add_point(step.position)

    points = np.array(points)
    lines = np.array(lines)

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)

    line_set.paint_uniform_color([0, 1, 0])

    return "Trajectory", line_set


class O3DBallisticApp:
    def __init__(self):
        self.__is_done = False

        self.app = o3d.visualization.gui.Application.instance
        self.app.initialize()

        self.main_vis = o3d.visualization.O3DVisualizer()
        self.app.add_window(self.main_vis)

        self.main_vis.set_on_close(self.__on_close__)

    def run(self):
        self.__is_done = False

        Thread(target=self.__thread__).start()
        self.app.run()

    def stop(self):
        self.__is_done = True

    def __on_close__(self):
        self.stop()
        return True

    def __thread__(self):
        trajectory_name, trajectory_geom = get_trajectory_line()

        def add_init():
            self.main_vis.line_width = 1
            self.main_vis.add_geometry(trajectory_name, trajectory_geom)

            self.main_vis.reset_camera_to_default()
            #self.main_vis.setup_camera(60, [0, 0, 0],
            #                           [0, 0, 0] + [0, 0, -3],
            #                           [0, -1, 0])

        o3d.visualization.gui.Application.instance.post_to_main_thread(
            self.main_vis, add_init)

        while not self.__is_done:
            def update_app():
                pass

            if self.__is_done:  # might have changed while sleeping
                break

            o3d.visualization.gui.Application.instance.post_to_main_thread(
                self.main_vis, update_app)


app = O3DBallisticApp()
app.run()
