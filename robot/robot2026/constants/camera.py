__all__ = [
    'CameraSettings',
    'APRILTAGS_FIELD_LAYOUT',
    'CAM_FRONT_LEFT',
    'CAM_FRONT_RIGHT',
    'CAM_REAR_RIGHT',
    'CAMERA_SETTINGS'
]


from frctools.frcmath import Vector3

from wpimath.geometry import Rotation3d, Translation3d, Transform3d
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from dataclasses import dataclass


@dataclass
class CameraSettings:
    name: str
    yaw: float
    pitch: float
    roll: float
    position: Vector3

    @property
    def translation(self) -> Translation3d:
        return self.position.to_translation3d()

    @property
    def rotation(self) -> Rotation3d:
        rot = Rotation3d.fromDegrees(self.roll, 0., 0.)
        rot = rot.rotateBy(Rotation3d.fromDegrees(0., self.pitch, 0.))
        rot = rot.rotateBy(Rotation3d.fromDegrees(0., 0., self.yaw))

        return rot

    @property
    def transform(self):
        return Transform3d(self.translation, self.rotation)


APRILTAGS_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

CAM_FRONT_LEFT = CameraSettings('Front_Left', 90., -29.5, 181., Vector3(0.0889, 0.2985, 0.4926))
#CAM_FRONT_RIGHT = CameraSettings('Front_Right', 270, -31., 182.5, Vector3(0.88, -0.301, 0.485))
CAM_REAR_RIGHT = CameraSettings('Rear_Right', 180., -33.5, 180.5, Vector3(-0.2699, -0.1913, 0.3612))

CAMERA_SETTINGS = [
    CAM_FRONT_LEFT,
    #CAM_FRONT_RIGHT,
    CAM_REAR_RIGHT
]
