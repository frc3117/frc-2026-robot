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

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from photonlibpy import PhotonCamera


class Robot(RobotBase):
    test_input: Input

    def register_inputs(self):
        # Swerve Inputs
        Input.add_axis('horizontal',
                       0,
                       XboxControllerInput.LEFT_JOYSTICK_Y,
                       deadzone=0.02,
                       # axis_filter=SlewRateLimiter(600),
                       axis_transform=PowerTransform(1))
        Input.add_axis('vertical',
                       0,
                       XboxControllerInput.LEFT_JOYSTICK_X,
                       inverted=True,
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

        # Second Driver Inputs 

        Input.add_button('ToggleManuel', 1, XboxControllerInput.Y)
        Input.add_button('ManuelShooterPreFeeder', 1, XboxControllerInput.A)
        Input.add_button('ManuelSouffleurCassette', 1, XboxControllerInput.B)

        Input.add_axis('ManuelShooterSpeed',
                       1,
                       XboxControllerInput.LEFT_JOYSTICK_Y,
                       inverted=True,
                       deadzone=0.02,
                       # axis_filter=SlewRateLimiter(600),
                       axis_transform=PowerTransform(1))
        
        Input.create_composite_axis('ManuelSpinningTurret',
                                    positive=Input.add_axis('ManuelTurnRight',
                                                            1,
                                                            XboxControllerInput.RIGHT_TRIGGER,
                                                            deadzone=0.02,
                                                            axis_transform=PowerTransform(1)),
                                    negative=Input.add_axis('ManuelTurnLeft',
                                                            1,
                                                            XboxControllerInput.LEFT_TRIGGER,
                                                            deadzone=0.02,
                                                            axis_transform=PowerTransform(1)),
                                    )

    def register_subsystems(self):
        # Swerve

        #  FL      FR
        #   3------0
        #   |      |
        #   2------1
        #  RL      RR

        drive_motor_0 = WPI_CANSparkFlex(2, True, False, True)
        dir_motor_0 = WPI_CANSparkMax(1, True, False, False)
        dir_encoder_0 = Encoder(dir_motor_0.get_absolute_encoder(), 0.802, False)

        drive_motor_1 = WPI_CANSparkFlex(4, True, False, True)
        dir_motor_1 = WPI_CANSparkMax(3, True, False, False)
        dir_encoder_1 = Encoder(dir_motor_1.get_absolute_encoder(), 0.916, False)

        drive_motor_2 = WPI_CANSparkFlex(6, True, False, False)
        dir_motor_2 = WPI_CANSparkMax(5, True, False, False)
        dir_encoder_2 = Encoder(dir_motor_2.get_absolute_encoder(), 0.46, False)

        drive_motor_3 = WPI_CANSparkFlex(8, True, False, False)
        dir_motor_3 = WPI_CANSparkMax(7, True, False, False)
        dir_encoder_3 = Encoder(dir_motor_3.get_absolute_encoder(), 0.072, False)

        swerve_modules = [
            SwerveModule(drive_motor=drive_motor_0,
                         steering_motor=dir_motor_0,
                         steering_encoder=dir_encoder_0,
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         position=Vector2(11.8717, 9.8750)),

            SwerveModule(drive_motor=drive_motor_1,
                         steering_motor=dir_motor_1,
                         steering_encoder=dir_encoder_1,
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         position=Vector2(-11.8717, 9.8750)),
                         #position=Vector2(11.8717, -9.8750)),

            SwerveModule(drive_motor=drive_motor_2,
                         steering_motor=dir_motor_2,
                         steering_encoder=dir_encoder_2,
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         position=Vector2(-11.8717, -9.8750)),

            SwerveModule(drive_motor=drive_motor_3,
                         steering_motor=dir_motor_3,
                         steering_encoder=dir_encoder_3,
                         steering_controller=PID(0.3, 0, 0),
                         steering_offset=0.,
                         position=Vector2(11.8717, -9.8750)),
                         #position=Vector2(-11.8717, 9.8750))
        ]

        swerve = SwerveDrive(swerve_modules, imu=ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s), start_heading=0)
        swerve.set_drive_mode(SwerveDriveMode.FIELD_CENTRIC)
        swerve.set_cosine_compensation(True)
        self.add_component('Swerve', swerve)


        # 30 Cassettte
        # 20 Feeder Speed
        # 22 Feeder Winch

        # Subsystems

        # Pose Estimator
        april_tag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)

        front_left_camera = PoseEstimationCamera(april_tag_layout,
                                                 PhotonCamera("Front_Left"),
                                                 Vector3(-0.2699, -0.1913, 0.3612),
                                                 Vector3(0.0175, 5.768, 0.5 * math.pi))
        rear_right_camera = PoseEstimationCamera(april_tag_layout,
                                                 PhotonCamera("Rear_Right"),
                                                 Vector3(-0.2699, -0.1913, 0.3612),
                                                 Vector3(0.00873, 5.821, math.pi))

        pose_estimator = RobotPoseEstimator(Vector2(), 0., [
            front_left_camera,
            rear_right_camera
        ])

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

        #elevation_encoder = Encoder(DutyCycleEncoder(2), 0.)
        #elevation_motor = WPI_CANSparkMax(43, True, False, True)
        #elevation_pid = PID(1, 0, 0, integral_range=(-1, 1), reset_integral_on_flip=True)

        speed_motor_a = WPI_CANSparkFlex(41, True, False, False)
        speed_motor_b = WPI_CANSparkFlex(42, True, False, True)
        #speed_pid = PID(1, 0, 0, integral_range=(-1, 1), reset_integral_on_flip=True)

        shooter = Shooter(heading_encoder_a=heading_encoder_a,
                          heading_encoder_b=heading_encoder_b,
                          heading_motor=heading_motor,
                          heading_pid=heading_pid,
                          elevation_encoder=None,#elevation_encoder,
                          elevation_motor=None,#elevation_motor,
                          elevation_pid=None,#elevation_pid,
                          speed_motor_a=speed_motor_a,
                          speed_motor_b=speed_motor_b,
                          speed_pid=None)#speed_pid)
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
            print(elapsed)

    def disabledInit(self):
        # ShiftUtils.reset()

        super().disabledInit()
