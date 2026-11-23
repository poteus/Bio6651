from wpilib import Field2d, SmartDashboard, DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from phoenix6.hardware import Pigeon2
from phoenix6 import utils
import ntcore
import commands2

class Vision(commands2.Subsystem):
    def __init__(self, limelight_names, drivetrain):
        super().__init__()

        self.mt2_init = False

        self.drivetrain = drivetrain

        self.instance = ntcore.NetworkTableInstance.getDefault()
        self.limelight_names = limelight_names

        self.mt1_subscribers = {}
        self.mt2_subscribers = {}

        self.imu_publishers = {}
        self.orientation_publishers = {}

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        for name in self.limelight_names:
            table = self.instance.getTable(name)
            self.mt1_subscribers[name] = table.getFloatArrayTopic("botpose_wpiblue").subscribe([])
            self.mt2_subscribers[name] = table.getFloatArrayTopic("botpose_orb_wpilib").subscribe([])

            self.imu_publishers[name] = table.getIntegerTopic("imumode_set").publish()
            self.orientation_publishers[name] = table.getFloatArrayTopic("robot_orientation_set").publish()

    def set_imu_mode(self, limelight_name, mode = 1):
        """
        0: EXTERNAL_ONLY (Uses external gyro only)
        1: EXTERNAL_SEED (Seeds internal IMU to match external yaw)
        2: INTERNAL_ONLY (Uses internal IMU only)
        3: INTERNAL_MT1_ASSIST (Internal IMU + MT1 vision yaw)
        4: INTERNAL_EXTERNAL_ASSIST (Internal IMU + external gyro)
        """
        self.imu_publishers[limelight_name].set(mode)

    def periodic(self):
        pose = self.drivetrain.get_state().pose
        yaw_deg = pose.rotation().degrees()
        yaw_rate_deg_per_sec = self.drivetrain.pigeon2.get_angular_velocity_z_world().value

        self.field.setRobotPose(pose)
        SmartDashboard.putNumber("Pose X", round(pose.X(), 3))
        SmartDashboard.putNumber("Pose Y", round(pose.Y(), 3))
        SmartDashboard.putNumber("Pose Yaw", round(yaw_deg, 1))

        imu_mode = 1 if DriverStation.isDisabled() else 4

        for name in self.limelight_names:
            self.imu_publishers[name].set(imu_mode)

            self.orientation_publishers[name].set(
                [yaw_deg, yaw_rate_deg_per_sec, 0.0, 0.0, 0.0, 0.0]
            )

            self._process_mt2(name, yaw_rate_deg_per_sec)

    def _process_mt2(self, name, yaw_rate_deg):
        raw = self.mt2_subscribers[name].get()

        if len(raw) < 11: return

        x, y, _, _, _, pose_yaw_deg, latency_ms, tag_count, _, avg_dist, _ = raw

        if tag_count < 1 or abs(yaw_rate_deg) > 360.0:
            return

        timestamp = utils.get_current_time_seconds() - (latency_ms / 1000.0)
        vision_pose = Pose2d(Translation2d(x,y), Rotation2d.fromDegrees(pose_yaw_deg))

        self.drivetrain.add_vision_measurement(vision_pose, timestamp, (.7, .7, 99999999.0))