from wpilib import Field2d, SmartDashboard, DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from phoenix6.hardware import Pigeon2
from phoenix6 import utils
import ntcore
import commands2

class Vision(commands2.Subsystem):
    def __init__(self, limelight_names, drivetrain):
        super().__init__()

        self.mt2_seeded = False
        self.pigeon_array = []

        self.drivetrain = drivetrain

        self.instance = ntcore.NetworkTableInstance.getDefault()
        self.limelight_names = limelight_names

        self.mt1_subscribers = {}
        self.mt2_subscribers = {}

        self.imu_publishers = {}
        self.orientation_publishers = {}
        self.drivetrain_publishers = {}

        self.elastic_publishers = {}

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        for name in self.limelight_names:
            table = self.instance.getTable(name)
            self.mt1_subscribers[name] = table.getFloatArrayTopic("botpose_wpiblue").subscribe([])
            self.mt2_subscribers[name] = table.getFloatArrayTopic("botpose_orb_wpiblue").subscribe([])

            self.imu_publishers[name] = table.getIntegerTopic("imumode_set").publish()
            self.orientation_publishers[name] = table.getFloatArrayTopic("robot_orientation_set").publish()

            self.elastic_publishers["botpose_wpiblue_rounded"] = table.getDoubleArrayTopic("botpose_wpiblue_rounded").publish()
            self.elastic_publishers["botpose_orb_wpiblue_rounded"] = table.getDoubleArrayTopic("botpose_orb_wpiblue_rounded").publish()

            self.imu_publishers[name].set(4)

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
        pigeon_yaw = self.drivetrain.pigeon2.get_yaw()

        self.field.setRobotPose(pose)
        SmartDashboard.putNumber("Pose X", round(pose.X(), 1))
        SmartDashboard.putNumber("Pose Y", round(pose.Y(), 1))
        SmartDashboard.putNumber("Pose Yaw", round(yaw_deg, 1))

        SmartDashboard.putNumber("PigeonYaw", round(pigeon_yaw.value, 2))

        for name in self.limelight_names:
            self.orientation_publishers[name].set(
                [yaw_deg, yaw_rate_deg_per_sec, 0.0, 0.0, 0.0, 0.0]
            )

            mt1_new_array = []
            mt1 = self.mt1_subscribers[name].get()

            mt2_new_array = []
            mt2 = self.mt2_subscribers[name].get()

            for i in range(len(mt1)):
                mt1_new_array.append(round(mt1[i], 1)) 

            for i in range(len(mt2)):
                mt2_new_array.append(round(mt2[i], 1)) 

            self.elastic_publishers["botpose_wpiblue_rounded"].set(mt1_new_array)
            self.elastic_publishers["botpose_orb_wpiblue_rounded"].set(mt2_new_array)

            self._process_mt2(name, yaw_rate_deg_per_sec)

            if self.mt2_seeded == False:
                self.pigeon_array.append(pigeon_yaw)
                max = 0
                if len(self.pigeon_array) == 10:
                    mean = sum(self.pigeon_array)/len(self.pigeon_array)

                    for value in self.pigeon_array:
                        if abs(value - mean) > max:
                            max = abs(value - mean)
                    if max < 0.2:
                        self.mt2_seeded = True
                        self.drivetrain.pigeon2.set_yaw(mean)
                        self.pigeon_array = []
                    else:
                        self.pigeon_array.pop(0)




    def _process_mt2(self, name, yaw_rate_deg):
        raw = self.mt2_subscribers[name].get()

        if len(raw) != 11: return

        x, y, _, _, _, pose_yaw_deg, latency_ms, tag_count, _, avg_dist, _ = raw

        if tag_count < 1 or abs(yaw_rate_deg) > 360.0:
            return

        timestamp = utils.get_current_time_seconds() - (latency_ms / 1000.0)
        vision_pose = Pose2d(Translation2d(x,y), Rotation2d.fromDegrees(pose_yaw_deg))

        self.drivetrain.add_vision_measurement(vision_pose, timestamp, (.7, .7, 99999999.0))