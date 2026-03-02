from frctools import Component, Timer, Coroutine, CoroutineOrder
from frctools.drivetrain import SwerveDrive
from frctools.frcmath import Vector2, Vector3


import frc_ballistic_solver as ball


from photonlibpy import PhotonPoseEstimator, PhotonCamera


from wpimath.geometry import Translation3d, Rotation3d, Pose2d, Rotation2d, Transform3d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField


from typing import List
from enum import IntEnum


class PoseEstimationCamera:
    __camera: PhotonCamera
    __camera_transform: Transform3d

    __pose_estimator: PhotonPoseEstimator

    def __init__(self, field: AprilTagFieldLayout, camera: PhotonCamera, camera_position: Vector3, camera_rotation: Vector3):
        self.__camera = camera
        self.__camera_transform = Transform3d(camera_position.to_translation3d(),
                                              Rotation3d.fromDegrees(camera_rotation.x, camera_rotation.y, camera_rotation.z))

        self.__pose_estimator = PhotonPoseEstimator(field, self.__camera_transform)

    def apply_vision_pose_estimate(self, estimator):
        for result in self.__camera.getAllUnreadResults():
            estimated_pose = self.__pose_estimator.estimateCoprocMultiTagPose(result)
            if estimated_pose is not None:
                estimated_pose = self.__pose_estimator.estimateLowestAmbiguityPose(result)

            estimator.addVisionMeasurement(estimated_pose, estimated_pose.timestampSeconds)


class RobotPoseEstimator(Component):
    __swerve: SwerveDrive

    __pose_camera: List[PoseEstimationCamera]
    __swerve_pose_estimator: SwerveDrive4PoseEstimator

    __initial_pose: Pose2d
    __current_pose: Pose2d
    __current_velocity: Vector2

    __pose_estimator_logic_coroutine: Coroutine

    def __init__(self, initial_position: Vector2, initial_heading: float, pose_camera: List[PoseEstimationCamera]):
        super().__init__()

        self.__initial_pose = Pose2d(initial_position.to_translation2d(), Rotation2d.fromDegrees(initial_heading))
        self.__pose_camera = pose_camera

    def init(self):
        self.__swerve = self.robot.get_component('Swerve')
        self.__swerve_pose_estimator = SwerveDrive4PoseEstimator(self.__swerve.get_swerve_4kinematics(),
                                                                 self.__swerve.get_gyro_angle2d(),
                                                                 self.__swerve.get_modules_positions4(),
                                                                 self.__initial_pose)

    def update(self):
        Timer.start_coroutine_if_stopped(self.__pose_estimator_logic_loop__, self.__pose_estimator_logic_coroutine, CoroutineOrder.EARLY, ignore_stop_all=True)

    def __pose_estimator_logic_loop__(self):
        while True:
            for camera in self.__pose_camera:
                camera.apply_vision_pose_estimate(self.__swerve_pose_estimator)

            self.__current_pose = self.__swerve_pose_estimator.update(self.__swerve.get_gyro_angle2d(), self.__swerve.get_modules_positions4())

    def get_current_pose(self) -> Pose2d:
        return self.__current_pose
    def set_current_pose(self, pose: Pose2d):
        self.__swerve_pose_estimator.resetPosition(self.__swerve.get_gyro_angle2d(), self.__swerve.get_modules_positions4(), pose)


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
            ball.Vector2(self.__LEFT__, self.__BOTTOM__),
            ball.Vector2(self.__LEFT__, self.__TOP__),
            ball.Vector2(self.__LEFT__ + 4.3, self.__TOP__),
            ball.Vector2(self.__LEFT__ + 4.3, self.__BOTTOM__)
        ])

        # Center of the field
        dont_shoot_center_zone = ball.FieldZone(RebuiltFieldZone.DONT_SHOOT, [])
        pass_previous_zone = ball.FieldZone(RebuiltFieldZone.PASS_PREVIOUS, [])
        pass_left_zone = ball.FieldZone(RebuiltFieldZone.PASS_LEFT, [])
        pass_right_zone = ball.FieldZone(RebuiltFieldZone.PASS_RIGHT, [])

        # Other side of the field
        dont_shoot_enemy_zone = ball.FieldZone(RebuiltFieldZone.DONT_SHOOT, [])
        pass_center_previous_zone = ball.FieldZone(RebuiltFieldZone.PASS_CENTER_PREVIOUS, [])
        pass_left_full_field_zone = ball.FieldZone(RebuiltFieldZone.PASS_LEFT, [])
        pass_right_full_field_zone = ball.FieldZone(RebuiltFieldZone.PASS_RIGHT, [])

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

    def get_zone(self, pos: ball.Vector2) -> RebuiltFieldZone:
        zone_id = self.__field.point_zone(pos)
        return RebuiltFieldZone(zone_id)

    @staticmethod
    def from_baked(file: str = None):
        pass