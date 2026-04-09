from frctools import (RobotBase,
                      WPI_CANSparkMax,
                      WPI_CANSparkFlex)
from frctools.input import (Input,
                            XboxControllerInput,
                            PowerTransform)
from frctools.drivetrain import (SwerveDrive,
                                 SwerveModule,
                                 SwerveDriveMode)
from frctools.sensor import Encoder
from frctools.controll import PID
from frctools.frcmath import Vector2, Vector3

import frc_ballistic_solver as ball
import time
import math

# from robot2026 import ShiftUtils
from robot2026 import constants as const
from robot2026.pose import PoseEstimationCamera, RobotPoseEstimator
from robot2026.subsytems import (Climber,
                                 Feeder,
                                 Shooter,
                                 Indexer,
                                 RobotController)



from wpilib import (ADIS16448_IMU,
                    SPI,
                    DutyCycleEncoder,
                    DigitalInput)
from wpimath.geometry import Rotation3d
from navx import AHRS

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from photonlibpy import PhotonCamera


WHEEL_DIAMETER = 0.0952
DRIVE_GEAR_RATIO = 7.23
DRIVE_CONVERSION_FACTOR = (WHEEL_DIAMETER * math.pi) / DRIVE_GEAR_RATIO

CAM_FRONT_LEFT_YAW = 90
CAM_FRONT_LEFT_PITCH = -29.5
CAM_FRONT_LEFT_ROLL = 181.
CAM_FRONT_LEFT_NAME = ''

CAM_REAR_RIGHT_YAW = 180.
CAM_REAR_RIGHT_PITCH = -33.5
CAM_REAR_RIGHT_ROLL = 0.5 + 180
CAM_REAR_RIGHT_NAME = ''

CAM_FRONT_RIGHT_YAW = 270
CAM_FRONT_RIGHT_PITCH = 0.
CAM_FRONT_RIGHT_ROLL = 0.
CAM_FRONT_RIGHT_NAME = 0.


class TestIMU:
    def __init__(self, imu: AHRS):
        self.__imu = imu

    def getAngle(self) -> float:
        return self.__imu.getAngle()


class Robot(RobotBase):
    test_input: Input

    def register_inputs(self):
        # Swerve Inputs
        Input.add_axis('horizontal',
                       0,
                       XboxControllerInput.LEFT_JOYSTICK_Y,
                       inverted=False,
                       deadzone=0.02,
                       # axis_filter=SlewRateLimiter(600),
                       axis_transform=PowerTransform(1))
        Input.add_axis('vertical',
                       0,
                       XboxControllerInput.LEFT_JOYSTICK_X,
                       inverted=False,
                       deadzone=0.02,
                       # axis_filter=SlewRateLimiter(600),
                       axis_transform=PowerTransform(1))

        Input.create_composite_axis('rotation',
                                    positive=Input.add_axis('rotation_pos',
                                                            0,
                                                            XboxControllerInput.RIGHT_TRIGGER,
                                                            deadzone=0.02,
                                                            axis_transform=PowerTransform(1)),
                                    negative=Input.add_axis('rotation_neg',
                                                            0,
                                                            XboxControllerInput.LEFT_TRIGGER,
                                                            deadzone=0.02,
                                                            axis_transform=PowerTransform(1)),
                                    )
        self.test_input = Input.add_button("test", 0, XboxControllerInput.A)

        # Subsystems Inputs

        Input.add_button('idle', 0, XboxControllerInput.B)
        Input.add_button('souffleuse', 0, XboxControllerInput.A)
        Input.add_button('climb', 0, XboxControllerInput.Y)

    def register_subsystems(self):
        # Swerve

        # !!!!!!! TOP VIEW !!!!!!!
        #  FL      FR
        #   3------0
        #   |      |
        #   2------1
        #  RL      RR

        drive_motor_0 = WPI_CANSparkFlex(2, True, False, True)
        drive_motor_0.set_position_conversion_factor(DRIVE_CONVERSION_FACTOR)
        drive_motor_0.set_velocity_conversion_factor(DRIVE_CONVERSION_FACTOR)
        dir_motor_0 = WPI_CANSparkMax(1, True, False, True)
        dir_motor_0.set_absolute_encoder_inverted(True)
        dir_encoder_0 = Encoder(dir_motor_0.get_absolute_encoder(), 0.198, False)
        #dir_encoder_0 = Encoder(dir_motor_0.get_absolute_encoder(), 0.803, False)

        drive_motor_1 = WPI_CANSparkFlex(4, True, False, True)
        drive_motor_1.set_position_conversion_factor(DRIVE_CONVERSION_FACTOR)
        drive_motor_1.set_velocity_conversion_factor(DRIVE_CONVERSION_FACTOR)
        dir_motor_1 = WPI_CANSparkMax(3, True, False, True)
        dir_motor_1.set_absolute_encoder_inverted(True)
        dir_encoder_1 = Encoder(dir_motor_1.get_absolute_encoder(), 0.085, False)
        #dir_encoder_1 = Encoder(dir_motor_1.get_absolute_encoder(), 0.918, False)

        drive_motor_2 = WPI_CANSparkFlex(6, True, False, True)
        drive_motor_2.set_position_conversion_factor(DRIVE_CONVERSION_FACTOR)
        drive_motor_2.set_velocity_conversion_factor(DRIVE_CONVERSION_FACTOR)
        dir_motor_2 = WPI_CANSparkMax(5, True, False, True)
        dir_motor_2.set_absolute_encoder_inverted(True)
        dir_encoder_2 = Encoder(dir_motor_2.get_absolute_encoder(), 0.0243, False)
        #dir_encoder_2 = Encoder(dir_motor_2.get_absolute_encoder(), 0.975, False)

        drive_motor_3 = WPI_CANSparkFlex(8, True, False, True)
        drive_motor_3.set_position_conversion_factor(DRIVE_CONVERSION_FACTOR)
        drive_motor_3.set_velocity_conversion_factor(DRIVE_CONVERSION_FACTOR)
        dir_motor_3 = WPI_CANSparkMax(7, True, False, True)
        dir_motor_3.set_absolute_encoder_inverted(True)
        dir_encoder_3 = Encoder(dir_motor_3.get_absolute_encoder(), 0.426 + 0.5, False)
        #dir_encoder_3 = Encoder(dir_motor_3.get_absolute_encoder(), 0.071, False)

        swerve_modules = [
            SwerveModule(drive_motor=drive_motor_0,
                         steering_motor=dir_motor_0,
                         steering_encoder=dir_encoder_0,
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         position=Vector2(0.251, -0.302),
                         rotation_inverted=False),
                         #position=Vector2(11.8717, -9.8750)),

            SwerveModule(drive_motor=drive_motor_1,
                         steering_motor=dir_motor_1,
                         steering_encoder=dir_encoder_1,
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         #position=Vector2(-11.8717, 9.8750)),
                         position=Vector2(-0.251, -0.302),
                         rotation_inverted=False),

            SwerveModule(drive_motor=drive_motor_2,
                         steering_motor=dir_motor_2,
                         steering_encoder=dir_encoder_2,
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         position=Vector2(-0.251, 0.302),
                         rotation_inverted=False),

            SwerveModule(drive_motor=drive_motor_3,
                         steering_motor=dir_motor_3,
                         steering_encoder=dir_encoder_3,
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         #position=Vector2(11.8717, -9.8750)),
                         position=Vector2(0.251, 0.302),
                         rotation_inverted=False)
        ]

        #imu = ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s)
        imu = AHRS.create_spi()

        #imu.setAngleAdjustment(0.)
        #imu.reset()
        #imu.reset()
        #imu.getAngle()
        #imu.

        while imu.isCalibrating():
            time.sleep(1)
            print('Sleep')
        print('Done')
        imu.zeroYaw()

        swerve = SwerveDrive(swerve_modules, imu=TestIMU(imu), start_heading=0., imu_inverted=False)
        swerve.set_drive_mode(SwerveDriveMode.FIELD_CENTRIC)
        #swerve.set_current_heading(0.)
        swerve.set_cosine_compensation(True)
        self.add_component('Swerve', swerve)


        # 30 Cassettte
        # 20 Feeder Speed
        # 22 Feeder Winch

        # Subsystems

        # Pose Estimator
        pose_estimator = RobotPoseEstimator(const.camera.APRILTAGS_FIELD_LAYOUT, const.camera.CAMERA_SETTINGS)
        self.add_component('Pose_Estimator', pose_estimator)

        # Feeder
        # Top 3 Limit
        # Bot 4 Limit

        top_limit = DigitalInput(4)
        bot_limit = DigitalInput(3)

        speed_motor = WPI_CANSparkMax(20, True, False, True)
        winch_motor = WPI_CANSparkFlex(22, True, False, False)

        feeder = Feeder(speed_motor=speed_motor,
                        winch_motor=winch_motor,
                        top_limit=top_limit,
                        bot_limit=bot_limit)
        self.add_component('Feeder', feeder)

        # Indexer
        cassette_motor = WPI_CANSparkFlex(30, True, False, False)
        pre_feeder_motor = WPI_CANSparkFlex(31, True, False, True)

        indexer = Indexer(cassette_motor=cassette_motor, pre_feeder_motor=pre_feeder_motor)
        self.add_component('Indexer', indexer)

        # Shooter
        heading_encoder_a = Encoder(DutyCycleEncoder(1), 0.)
        heading_encoder_b = Encoder(DutyCycleEncoder(0), 0.)
        heading_motor = WPI_CANSparkMax(40, True, False, True)
        heading_pid = PID(5, 0, 0, integral_range=(-1, 1), reset_integral_on_flip=True)

        elevation_motor = WPI_CANSparkMax(43, True, False, True)
        elevation_encoder = Encoder(elevation_motor.get_absolute_encoder(), 0.24, False)
        elevation_pid = PID(2.5, 0, 0, integral_range=(-1, 1), reset_integral_on_flip=True)

        speed_motor_a = WPI_CANSparkFlex(41, True, False, False)
        speed_motor_b = WPI_CANSparkFlex(42, True, False, True)
        speed_pid = PID(0.0, 0.0012, 0, kf=0.001806, integral_range=(-12, 12), reset_integral_on_flip=True)

        shooter = Shooter(heading_encoder_a=heading_encoder_a,
                          heading_encoder_b=heading_encoder_b,
                          heading_motor=heading_motor,
                          heading_pid=heading_pid,
                          elevation_encoder=elevation_encoder,
                          elevation_motor=elevation_motor,
                          elevation_pid=elevation_pid,
                          speed_motor_a=speed_motor_a,
                          speed_motor_b=speed_motor_b,
                          speed_pid=speed_pid)
        self.add_component('Shooter', shooter)

        # Climber


        #climber = Climber()
        #self.add_component('Climber', climber)

        # Controller
        controller = RobotController()
        self.add_component('Controller', controller)

    def robotInit(self):
        super().robotInit()

        self.register_inputs()
        self.register_subsystems()

    def robotPeriodic(self):
        # ShiftUtils.refresh_shift()

        super().robotPeriodic()
        '''
        if self.test_input.get_button_down():
            proj = ball.Projectile(0.0176, 0.227, 0.0008625)

            target_pos = ball.Vector3(8.27, 0.374, 1.486)
            robot_pos = ball.Vector3(9.61, 4.05, 0.437)
            robot_velocity = ball.Vector3(0., 0., 0.)

            solver = ball.HybridBallisticSolver(target_pos=target_pos,
                                                speed_range=ball.Range(0.1, 25),
                                                airtime_range=ball.Range(0.5, 3.),
                                                impact_cone_tolerance=1.57,
                                                sample_count=25,
                                                projectile=proj,
                                                refinement_passes=3,
                                                dt=0.01,
                                                convergence_threshold=0.03)

            start_time = time.perf_counter()
            solution = solver.solve(robot_pos, robot_velocity)
            # print(solution)
            end_time = time.perf_counter()

            elapsed = end_time - start_time
        '''

    def disabledInit(self):
        # ShiftUtils.reset()

        super().disabledInit()
