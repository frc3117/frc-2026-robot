import math

from frctools import Component, CoroutineOrder, Coroutine, Timer, Alliance
from frctools.input import Input
from frctools.drivetrain import SwerveDrive

import frc_ballistic_solver as bal

from .feeder import Feeder
from .indexer import Indexer
from .shooter import Shooter

from ..pose import RobotPoseEstimator, RebuiltField, RebuiltFieldZone


from wpilib import SmartDashboard, FieldObject2d
from wpimath.geometry import Pose2d, Pose3d, Rotation3d


from enum import IntEnum


SOLVER_SAMPLE_COUNT = 25
SOLVER_DT = 0.01
SOLVER_REFINEMENT = 0
SOLVER_CONVERGENCE_THRESHOLD = 0.03
SOLVER_MAGNUS_RATIO = -1.2


FUEL_PROJECTILE = bal.Projectile(0.0176, 0.227, 0.0008625, magnus_coef=0.000791)


TOP_POS = 8.082
RIGHT_POS = 16.535


def mirror_pose(ref: Pose3d, mirror_x: bool, mirror_y: bool) -> Pose3d:
    if mirror_x:
        x = RIGHT_POS - ref.x
    else:
        x = ref.x

    if mirror_y:
        y = TOP_POS - ref.y
    else:
        y = ref.y

    return Pose3d(x, y, 0, Rotation3d(0, 0, 0))


# Blue Target Pose
BLUE_HUB_TARGET_POSE = Pose3d(4.6228, 4.041, 1.588, Rotation3d(0, 0, 0))
BLUE_BOT_PASS_TARGET_POSE = Pose3d(1.5, 1.5, 0, Rotation3d(0, 0, 0))
BLUE_TOP_PASS_TARGET_POSE = mirror_pose(BLUE_BOT_PASS_TARGET_POSE, False, True)
BLUE_BOT_MID_PASS_TARGET_POSE = Pose3d(9.5, 1., 0, Rotation3d(0, 0, 0))
BLUE_TOP_MID_PASS_TARGET_POSE = mirror_pose(BLUE_BOT_MID_PASS_TARGET_POSE, False, True)

# Red Target Pose
RED_HUB_TARGET_POSE = Pose3d(11.913, 4.041, 1.588, Rotation3d(0, 0, 0))
RED_TOP_PASS_TARGET_POSE = mirror_pose(BLUE_BOT_PASS_TARGET_POSE, True, False)
RED_BOT_PASS_TARGET_POSE = mirror_pose(BLUE_BOT_PASS_TARGET_POSE, True, True)
RED_TOP_MID_PASS_TARGET_POSE = mirror_pose(BLUE_BOT_MID_PASS_TARGET_POSE, True, False)
RED_BOT_MID_PASS_TARGET_POSE = mirror_pose(BLUE_BOT_MID_PASS_TARGET_POSE, True, True)


class AimTarget:
    __pose: Pose2d
    __solver: bal.HybridBallisticSolver

    def __init__(self, pose: Pose3d):
        self.__pose = pose.toPose2d()
        try:
            self.__solver = bal.HybridBallisticSolver(
                target_pos=bal.Vector3(pose.x, pose.y, pose.z),
                speed_range=bal.Range(3, 10),
                airtime_range=bal.Range(0.15, 3),
                impact_cone_tolerance=1.57,
                sample_count=SOLVER_SAMPLE_COUNT,
                projectile=FUEL_PROJECTILE,
                refinement_passes=SOLVER_REFINEMENT,
                dt=SOLVER_DT,
                convergence_threshold=SOLVER_CONVERGENCE_THRESHOLD,
                magnus_ratio=SOLVER_MAGNUS_RATIO
            )
        except TypeError:
            # Compatibility path for older frc_ballistic_solver builds
            self.__solver = bal.HybridBallisticSolver(
                target_pos=bal.Vector3(pose.x, pose.y, pose.z),
                speed_range=bal.Range(3, 10),
                airtime_range=bal.Range(0.15, 3),
                impact_cone_tolerance=1.57,
                sample_count=SOLVER_SAMPLE_COUNT,
                projectile=FUEL_PROJECTILE,
                refinement_passes=SOLVER_REFINEMENT,
                dt=SOLVER_DT,
                convergence_threshold=SOLVER_CONVERGENCE_THRESHOLD,
            )

    @property
    def pose(self) -> Pose2d:
        return self.__pose

    @property
    def solver(self) -> bal.HybridBallisticSolver:
        return self.__solver


class RobotMode(IntEnum):
    NONE = -1
    IDLE = 0
    SOUFFLEUSE = 1
    CLIMB = 2


class RobotController(Component):
    __current_mode: RobotMode = RobotMode.IDLE
    __requested_mode: RobotMode = RobotMode.NONE

    __idle_button: Input
    __souffleuse_button: Input
    __climb_button: Input

    __pose_estimator: RobotPoseEstimator
    __feeder: Feeder
    __indexer: Indexer
    __shooter: Shooter
    __swerve: SwerveDrive

    __input_loop: Coroutine = None
    __logic_loop: Coroutine = None

    __field: RebuiltField
    __previous_side_is_top = False

    __blue_hub_target: AimTarget
    __blue_top_pass_target: AimTarget
    __blue_bot_pass_target: AimTarget
    __blue_top_mid_pass_target: AimTarget
    __blue_bot_mid_pass_target: AimTarget

    __red_hub_target: AimTarget
    __red_top_pass_target: AimTarget
    __red_bot_pass_target: AimTarget
    __red_top_mid_pass_target: AimTarget
    __red_bot_mid_pass_target: AimTarget

    __turret_object: FieldObject2d

    def __init__(self):
        super().__init__()

        self.__field = RebuiltField()

        self.__blue_hub_target = AimTarget(BLUE_HUB_TARGET_POSE)
        self.__blue_top_pass_target = AimTarget(BLUE_TOP_PASS_TARGET_POSE)
        self.__blue_bot_pass_target = AimTarget(BLUE_BOT_PASS_TARGET_POSE)
        self.__blue_top_mid_pass_target = AimTarget(BLUE_TOP_MID_PASS_TARGET_POSE)
        self.__blue_bot_mid_pass_target = AimTarget(BLUE_BOT_MID_PASS_TARGET_POSE)

        self.__red_hub_target = AimTarget(RED_HUB_TARGET_POSE)
        self.__red_top_pass_target = AimTarget(RED_TOP_PASS_TARGET_POSE)
        self.__red_bot_pass_target = AimTarget(RED_BOT_PASS_TARGET_POSE)
        self.__red_top_mid_pass_target = AimTarget(RED_TOP_MID_PASS_TARGET_POSE)
        self.__red_bot_mid_pass_target = AimTarget(RED_BOT_MID_PASS_TARGET_POSE)

    def init(self):
        self.__idle_button = Input.get_input('idle')
        self.__souffleuse_button = Input.get_input('souffleuse')
        self.__climb_button = Input.get_input('climb')

        self.__pose_estimator = self.robot.get_component('Pose_Estimator')
        self.__feeder = self.robot.get_component('Feeder')
        self.__indexer = self.robot.get_component('Indexer')
        self.__shooter = self.robot.get_component('Shooter')
        self.__swerve = self.robot.get_component('Swerve')

        self.__swerve.set_current_heading(-self.__pose_estimator.get_current_pose().rotation().radians())

        self.__turret_object = self.__pose_estimator.get_object('turret')

        if Alliance.get_alliance() == Alliance.RED:
            Input.get_input('horizontal').set_inverted(True)
            Input.get_input('vertical').set_inverted(True)
        else:
            Input.get_input('horizontal').set_inverted(False)
            Input.get_input('vertical').set_inverted(False)

    def update(self):
        self.__input_loop = Timer.start_coroutine_if_stopped(self.__input_loop__, self.__input_loop, CoroutineOrder.EARLY, False)
        self.__logic_loop = Timer.start_coroutine_if_stopped(self.__logic_loop__, self.__logic_loop, CoroutineOrder.NORMAL, True)

    def __input_loop__(self):
        yield from ()
        while True:
            yield False

            # Idle Mode
            if self.__idle_button.get_button_up() and self.__current_mode != RobotMode.IDLE:
                self.__requested_mode = RobotMode.IDLE
            # Souffleuse Mode
            elif self.__souffleuse_button.get_button_down():
                # Go into Idle if already in Souffleuse
                if self.__current_mode != RobotMode.SOUFFLEUSE:
                    self.__requested_mode = RobotMode.SOUFFLEUSE
            # Climb Mode
            #elif self.__climb_button.get_button_up():
                # Go into Idle if already in Climb
            #    if self.__current_mode == RobotMode.CLIMB:
            #        self.__requested_mode = RobotMode.IDLE
            #    else:
            #        self.__requested_mode = RobotMode.CLIMB

    def __logic_loop__(self):
        yield from ()

        while True:
            yield False

            match self.__current_mode:
                case RobotMode.IDLE:
                    yield from self.__do_idle_mode__()

                case RobotMode.SOUFFLEUSE:
                    yield from self.__do_souffleuse_mode__()
                    pass

                case RobotMode.CLIMB:
                    pass

            if self.__requested_mode != RobotMode.NONE:
                self.__current_mode = self.__requested_mode


    def __do_idle_mode__(self):
        def idle_to_souffleuse_transition():
            yield from ()

            # Extend and Start Feeder
            #self.__feeder.set_expended()
            #yield from self.__feeder.wait_until_expended()

            #self.__feeder.start_feeding()
            #self.__indexer.start_indexing()
            #self.__shooter.start_shooting()

            self.__shooter.hold_heading_()

            #self.__shooter.set_target_heading_angle(0., True)
            print('Done transition')

        def idle_to_climb_transition():
            yield from ()

            # Maybe nothing

        yield from ()
        while True:
            yield False

            match self.__requested_mode:
                case RobotMode.SOUFFLEUSE:
                    yield from idle_to_souffleuse_transition()
                    return

    def __do_souffleuse_mode__(self):
        def souffleuse_to_idle_transition():
            yield from ()

            # Stop and Retract Feeder
            self.__shooter.set_target_heading_angle(0., False)

            self.__feeder.stop_feeding()
            self.__indexer.stop_indexing()
            self.__shooter.stop_shooting()

            self.__swerve.set_speed(1)

            #self.__feeder.set_retracted()
            #yield from self.__feeder.wait_until_retracted()

        def souffleuse_to_climb_transition():
            yield from ()

            # Stop and Retract Feeder
            self.__shooter.set_target_heading_angle(0., False)

            self.__feeder.stop_feeding()
            self.__indexer.stop_indexing()
            self.__shooter.stop_shooting()

            self.__swerve.set_speed(1)

            #self.__feeder.set_retracted()
            #yield from self.__feeder.wait_until_retracted()

        def try_apply_turret_target(target: AimTarget, robot_pos, robot_vel):
            if target is None:
                return

            self.__pose_estimator.set_aim_target(target.pose)

            solver = target.solver
            sol = solver.solve(robot_pos, robot_vel)

            if sol.valid:
                aim_dir = sol.aim_dir
                heading = (math.atan2(aim_dir.y, aim_dir.x) + 0.5 * math.pi) / math.tau

                elevation = (0.5 * math.pi) - sol.elevation
                if Timer.get_frame_count() % 100:
                    print(elevation)

                self.__shooter.set_target_heading(heading)
                SmartDashboard.putNumber('target_heading', heading)
                SmartDashboard.putNumber('target_speed', sol.speed)
                self.__shooter.set_shooter_speed(math.pow(sol.speed, 1.02))
                self.__shooter.set_target_elevation(math.pow(elevation, 0.9))

        yield from ()

        while True:
            yield False

            # Estimate new target
            wpi_pose = self.__pose_estimator.get_current_pose()
            wpi_vel = self.__pose_estimator.get_current_velocity()

            pos = bal.Vector2(wpi_pose.x, wpi_pose.y)

            turret_pos = self.__shooter.get_shooter_pose(pos, wpi_pose.rotation().radians())
            #turret_vel = bal.Vector3(wpi_vel.x, wpi_vel.y, 0.) * 0.2
            #turret_vel = bal.Vector3(0., 0., 0.)

            vel = self.__swerve.get_previous_translation().rotate(-self.__swerve.get_heading())
            turret_vel = bal.Vector3(vel.x, vel.y, 0.) * -2.5

            self.__turret_object.setPose(Pose2d(turret_pos.x, turret_pos.y, self.__shooter.current_heading_world_angle()))

            alliance = Alliance.get_alliance()

            curr_zone = self.__field.get_zone(pos, alliance)
            #curr_zone = RebuiltFieldZone.SHOOT_IN_HUB
            SmartDashboard.putString('Zone', curr_zone.name)
            curr_target = None
            match curr_zone:
                case RebuiltFieldZone.SHOOT_IN_HUB:
                    if alliance == Alliance.RED:
                        curr_target = self.__red_hub_target
                    else:
                        curr_target = self.__blue_hub_target

                case RebuiltFieldZone.PASS_LEFT:
                    self.__previous_side_is_top = True
                    if alliance == Alliance.RED:
                        curr_target = self.__red_top_pass_target
                    else:
                        curr_target = self.__blue_top_pass_target

                case RebuiltFieldZone.PASS_RIGHT:
                    self.__previous_side_is_top = False
                    if alliance == Alliance.RED:
                        curr_target = self.__red_bot_pass_target
                    else:
                        curr_target = self.__blue_bot_pass_target

                case RebuiltFieldZone.PASS_PREVIOUS:
                    if self.__previous_side_is_top:
                        if alliance == Alliance.RED:
                            curr_target = self.__red_top_pass_target
                        else:
                            curr_target = self.__blue_top_pass_target
                    else:
                        if alliance == Alliance.RED:
                            curr_target = self.__red_bot_pass_target
                        else:
                            curr_target = self.__blue_bot_pass_target

                case RebuiltFieldZone.PASS_CENTER_PREVIOUS:
                    if self.__previous_side_is_top:
                        if alliance == Alliance.RED:
                            curr_target = self.__red_top_mid_pass_target
                        else:
                            curr_target = self.__blue_top_mid_pass_target
                    else:
                        if alliance == Alliance.RED:
                            curr_target = self.__red_bot_mid_pass_target
                        else:
                            curr_target = self.__blue_bot_mid_pass_target

            try_apply_turret_target(curr_target, turret_pos, turret_vel)

            souffleuse_state = self.__souffleuse_button.get()
            self.__feeder.set_feeding(souffleuse_state)
            self.__indexer.set_indexing(souffleuse_state)
            self.__shooter.set_shooting(souffleuse_state)

            #self.__set_output_speed__(1)

            if souffleuse_state:
                self.__swerve.set_speed(0.5)
            else:
                self.__swerve.set_speed(1)

            # Check if robot want to switch mode
            match self.__requested_mode:
                case RobotMode.NONE:
                    pass

                case RobotMode.IDLE:
                    # Do transition to Idle
                    yield from souffleuse_to_idle_transition()
                    return

                case RobotMode.CLIMB:
                    # Do transition to Climb
                    yield from souffleuse_to_climb_transition()
                    return

                case RobotMode.SOUFFLEUSE:
                    self.__requested_mode = RobotMode.NONE

    def set_mode(self, mode: RobotMode):
        self.__requested_mode = mode

    def set_idle_mode(self):
        self.__requested_mode = RobotMode.IDLE

    def set_souffleuse_mode(self):
        self.__requested_mode = RobotMode.SOUFFLEUSE

    def set_climbing_mode(self):
        self.__requested_mode = RobotMode.CLIMB

    def __set_output_speed__(self, speed: float):
        intake_speed = 0.8
        cassette_speed = 1.
        pre_feeder_speed = 0.65
        shooter_speed = 0.5

        self.__feeder.set_intake_speed(intake_speed)
        self.__indexer.set_cassette_speed(cassette_speed)
        self.__indexer.set_pre_feeder_speed(pre_feeder_speed)
        self.__shooter.set_shooter_speed(shooter_speed)
