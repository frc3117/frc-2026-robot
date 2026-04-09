from frctools import Component, Timer, Coroutine, CoroutineOrder, Alliance
from frctools.drivetrain import SwerveDrive
from frctools.frcmath import Vector2, Vector3


import frc_ballistic_solver as ball


from photonlibpy import PhotonPoseEstimator, PhotonCamera


from wpilib import Field2d, FieldObject2d, SmartDashboard, RobotBase
from wpimath.geometry import Translation2d, Rotation3d, Pose2d, Rotation2d, Transform3d, Transform2d, Pose3d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField


from typing import List
from enum import IntEnum
from collections import deque

from .constants.camera import CameraSettings

# http://photonvision-3117-{0..2}.local:5800

# Cam 0: Rear right
# pos(0.0984, 0.3215, 0.4882) rot()

# Cam 1: Front left
# pos(-0.2699, -0.1913, 0.3612) rot()


class PoseEstimationCamera:
    __camera: PhotonCamera
    __camera_transform: Transform3d

    __pose_estimator: PhotonPoseEstimator
    __obj: FieldObject2d

    def __init__(self, field_layout: AprilTagFieldLayout, settings: CameraSettings):
        self.__camera = PhotonCamera(settings.name)
        self.__camera_transform = settings.transform

        self.__pose_estimator = PhotonPoseEstimator(field_layout, self.__camera_transform)

    def generate_object(self, field: Field2d):
        self.__obj = field.getObject(self.__camera.getName())

    def apply_vision_pose_estimate(self, estimator):
        if RobotBase.isSimulation():
            return

        try:
            results = self.__camera.getAllUnreadResults()
        except Exception:
            # In sim/offline runs, PhotonVision may not be connected.
            return

        for result in results:
            estimated_pose = self.__pose_estimator.estimateCoprocMultiTagPose(result)
            if estimated_pose is None:
                estimated_pose = self.__pose_estimator.estimateLowestAmbiguityPose(result)

            if estimated_pose is not None:
                estimator.addVisionMeasurement(estimated_pose.estimatedPose.toPose2d(), estimated_pose.timestampSeconds)

    def update_object(self, robot_pose: Pose2d):
        new_pose = (Pose3d(robot_pose) + self.__camera_transform).toPose2d()
        self.__obj.setPose(new_pose)


class RobotPoseEstimator(Component):
    __swerve: SwerveDrive

    __pose_camera: List[PoseEstimationCamera]
    __swerve_pose_estimator: SwerveDrive4PoseEstimator

    __field: Field2d
    __aim_target: FieldObject2d

    __initial_pose: Pose2d
    __current_velocity: Vector2

    __current_pose: Pose2d
    __current_pose_time: float

    __previous_pose: Pose2d
    __previous_pose_time: float

    __velocity: Transform2d
    __vel_buff: deque

    __pose_estimator_logic_coroutine: Coroutine = None

    def __init__(self, field_layout: AprilTagFieldLayout, settings: List[CameraSettings]):
        super().__init__()

        self.__initial_pose = Pose2d(Translation2d(), Rotation2d())
        self.__pose_camera = [PoseEstimationCamera(field_layout, s) for s in settings]

        self.__field = Field2d()
        self.__aim_target = self.__field.getObject("aimTarget")

        for cam in self.__pose_camera:
            cam.generate_object(self.__field)

        self.__vel_buff = deque(maxlen=10)

        self.__swerve = self.robot.get_component('Swerve')
        self.__swerve_pose_estimator = SwerveDrive4PoseEstimator(self.__swerve.get_swerve_4kinematics(),
                                                                 self.__swerve.get_gyro_angle2d() * -1,
                                                                 self.__swerve.get_modules_positions4(),
                                                                 self.__initial_pose)

        SmartDashboard.putData('Pose_Estimator/field', self.__field)

    def init(self):
        self.__previous_pose = self.__swerve_pose_estimator.getEstimatedPosition()
        self.__previous_pose_time = Timer.get_current_time()

        self.__current_pose = self.__previous_pose
        self.__current_pose_time = Timer.get_current_time()

        self.__velocity = Transform2d()
        self.__vel_buff.clear()

    def update_disabled(self):
        self.refresh_pose()

    def update(self):
        self.__pose_estimator_logic_coroutine = Timer.start_coroutine_if_stopped(self.__pose_estimator_logic_loop__, self.__pose_estimator_logic_coroutine, CoroutineOrder.EARLY, ignore_stop_all=True)

    def __pose_estimator_logic_loop__(self):
        yield from ()

        while True:
            yield False

            self.refresh_pose()

    def refresh_pose(self):
        for camera in self.__pose_camera:
            camera.apply_vision_pose_estimate(self.__swerve_pose_estimator)

        self.__previous_pose = self.__current_pose
        self.__previous_pose_time = Timer.get_current_time()

        self.__current_pose = self.__swerve_pose_estimator.update(self.__swerve.get_gyro_angle2d() * -1,
                                                                  self.__swerve.get_modules_positions4())

        if self.__current_pose is not None:
            self.__field.setRobotPose(self.__current_pose)

            for camera in self.__pose_camera:
                camera.update_object(self.__current_pose)

        vel = (self.__current_pose - self.__previous_pose) / (Timer.get_current_time() - self.__previous_pose_time)
        self.__velocity = vel
        #self.__vel_buff.append(vel)

        #vel_avg = Transform2d()
        #for v in self.__vel_buff:
        #    vel_avg = vel_avg + v

        #self.__velocity = vel_avg / len(self.__vel_buff)


    def get_current_pose(self) -> Pose2d:
        return self.__current_pose
    def set_current_pose(self, pose: Pose2d):
        self.__swerve_pose_estimator.resetPosition(self.__swerve.get_gyro_angle2d(), self.__swerve.get_modules_positions4(), pose)

    def get_current_velocity(self) -> Transform2d:
        return self.__velocity

    def set_aim_target(self, pose: Pose2d):
        self.__aim_target.setPose(pose)

    def get_object(self, name: str) -> FieldObject2d:
        return self.__field.getObject(name)


class RebuiltFieldZone(IntEnum):
    DONT_SHOOT = -1
    SHOOT_IN_HUB = 0
    PASS_PREVIOUS = 1
    PASS_LEFT = 2
    PASS_RIGHT = 3,
    PASS_CENTER_PREVIOUS = 4,


class RebuiltField:
    __BOTTOM__ = 0.
    __TOP__ = 8.069326

    __LEFT__ = 0.
    __RIGHT__ = 16.540988

    def __init__(self):
        # Our side of the field
        shoot_in_hub_zone = ball.FieldZone(RebuiltFieldZone.SHOOT_IN_HUB, [
            ball.Vector2(self.__LEFT__ - 1., self.__BOTTOM__ - 1.),
            ball.Vector2(self.__LEFT__ - 1., self.__TOP__ + 1.),
            ball.Vector2(self.__LEFT__ + 4.3, self.__TOP__ + 1.),
            ball.Vector2(self.__LEFT__ + 4.3, self.__BOTTOM__ - 1.)
        ])

        # Center of the field
        dont_shoot_center_zone = ball.FieldZone(RebuiltFieldZone.DONT_SHOOT, [])
        pass_previous_zone = ball.FieldZone(RebuiltFieldZone.PASS_PREVIOUS, [
            ball.Vector2(self.__LEFT__ + 4.95, self.__BOTTOM__ + 3.45),
            ball.Vector2(self.__LEFT__ + 4.95, self.__TOP__ - 3.45),
            ball.Vector2(self.__RIGHT__ - 4.95, self.__TOP__ - 3.45),
            ball.Vector2(self.__RIGHT__ - 4.95, self.__BOTTOM__ + 3.45)
        ])
        pass_left_zone = ball.FieldZone(RebuiltFieldZone.PASS_LEFT, [
            ball.Vector2(self.__LEFT__ + 4.95, self.__TOP__ + 1),
            ball.Vector2(self.__LEFT__ + 4.95, self.__TOP__ - 3.45),
            ball.Vector2(self.__RIGHT__ - 4.95, self.__TOP__ - 3.45),
            ball.Vector2(self.__RIGHT__ - 4.95, self.__TOP__ + 1)
        ])
        pass_right_zone = ball.FieldZone(RebuiltFieldZone.PASS_RIGHT, [
            ball.Vector2(self.__LEFT__ + 4.95, self.__BOTTOM__ - 1),
            ball.Vector2(self.__LEFT__ + 4.95, self.__BOTTOM__ + 3.45),
            ball.Vector2(self.__RIGHT__ - 4.95, self.__BOTTOM__ + 3.45),
            ball.Vector2(self.__RIGHT__ - 4.95, self.__BOTTOM__ - 1)
        ])

        # Other side of the field
        dont_shoot_enemy_zone = ball.FieldZone(RebuiltFieldZone.DONT_SHOOT, [])
        pass_center_previous_zone = ball.FieldZone(RebuiltFieldZone.PASS_CENTER_PREVIOUS, [
            ball.Vector2(self.__RIGHT__ - 4.3, self.__BOTTOM__ + 3.45),
            ball.Vector2(self.__RIGHT__ - 4.3, self.__TOP__ - 3.45),
            ball.Vector2(self.__RIGHT__ + 1., self.__TOP__ - 3.45),
            ball.Vector2(self.__RIGHT__ + 1., self.__BOTTOM__ + 3.45)
        ])
        pass_left_full_field_zone = ball.FieldZone(RebuiltFieldZone.PASS_LEFT, [
            ball.Vector2(self.__RIGHT__ - 4.3, self.__TOP__ + 1),
            ball.Vector2(self.__RIGHT__ - 4.3, self.__TOP__ - 3.45),
            ball.Vector2(self.__RIGHT__ + 1., self.__TOP__ - 3.45),
            ball.Vector2(self.__RIGHT__ + 1., self.__TOP__ + 1)
        ])
        pass_right_full_field_zone = ball.FieldZone(RebuiltFieldZone.PASS_RIGHT, [
            ball.Vector2(self.__RIGHT__ - 4.3, self.__BOTTOM__ - 1),
            ball.Vector2(self.__RIGHT__ - 4.3, self.__BOTTOM__ + 3.45),
            ball.Vector2(self.__RIGHT__ + 1., self.__BOTTOM__ + 3.45),
            ball.Vector2(self.__RIGHT__ + 1., self.__BOTTOM__ - 1)
        ])

        # Order of priority is from top to bottom
        self.__field = ball.Field([
            dont_shoot_center_zone,
            dont_shoot_enemy_zone,
            shoot_in_hub_zone,
            pass_previous_zone,
            pass_left_zone,
            pass_right_zone,
            pass_center_previous_zone,
            pass_left_full_field_zone,
            pass_right_full_field_zone
        ])

    def get_zone(self, pos: ball.Vector2, alliance: Alliance = Alliance.BLUE) -> RebuiltFieldZone:
        if alliance == Alliance.RED:
            pos.x = self.__RIGHT__ - pos.x
            pos.y = self.__TOP__ - pos.y

        zone_id = self.__field.point_zone(pos)
        return RebuiltFieldZone(zone_id)

    @staticmethod
    def from_baked(file: str = None):
        pass
