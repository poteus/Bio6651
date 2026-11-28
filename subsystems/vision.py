from wpilib import Field2d, SmartDashboard, DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from phoenix6.hardware import Pigeon2
from phoenix6 import utils
import ntcore
import commands2
import wpilib

class Vision(commands2.Subsystem):
    def __init__(self, limelight_names, drivetrain):
        super().__init__()

        self.mt2_seeded = False
        self.pigeon_array = []

        self.seed_timer = wpilib.Timer()
        self.seed_timer.start()

        self.drivetrain = drivetrain

        self.instance = ntcore.NetworkTableInstance.getDefault()
        self.limelight_names = limelight_names

        self.mt1_subscribers = {}
        self.mt2_subscribers = {}

        self.drivetrain_publishers = {}
        self.imu_publishers = {}
        self.orientation_publishers = {}

        self.elastic_publishers = {}

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        SmartDashboard.putBoolean("mt2_seeded",False)

        for name in self.limelight_names:
            table = self.instance.getTable(name)
            self.mt1_subscribers[name] = table.getDoubleArrayTopic("botpose_wpiblue").subscribe([])
            self.mt2_subscribers[name] = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe([])

            self.imu_publishers[name] = table.getIntegerTopic("imumode_set").publish()
            self.set_imu_mode(name,1)

            self.orientation_publishers[name] = table.getDoubleArrayTopic("robot_orientation_set").publish()

            self.elastic_publishers[f"{name}/botpose_wpiblue_rounded"] = table.getDoubleArrayTopic("botpose_wpiblue_rounded").publish()
            self.elastic_publishers[f"{name}/botpose_orb_wpiblue_rounded"] = table.getDoubleArrayTopic("botpose_orb_wpiblue_rounded").publish()

        posetable = self.instance.getTable("Drivetrain")

        self.drivetrain_publisher = posetable.getStructTopic("Pose2d", Pose2d).publish()            

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
        pigeon_yaw = self.drivetrain.pigeon2.get_yaw().value

        self.field.setRobotPose(pose)
        self.drivetrain_publisher.set(pose)
        SmartDashboard.putNumber("Pose X", round(pose.X(), 1))
        SmartDashboard.putNumber("Pose Y", round(pose.Y(), 1))
        SmartDashboard.putNumber("Pose Yaw", round(yaw_deg, 1))

        SmartDashboard.putNumber("PigeonYaw", round(pigeon_yaw, 2))

        for name in self.limelight_names:
            self.orientation_publishers[name].set(
                [pigeon_yaw, yaw_rate_deg_per_sec, 0.0, 0.0, 0.0, 0.0]
            )

            mt1_new_array = []
            mt1 = self.mt1_subscribers[name].get()

            mt2_new_array = []
            mt2 = self.mt2_subscribers[name].get()

            for i in range(len(mt1)):
                mt1_new_array.append(round(mt1[i], 1)) 

            for i in range(len(mt2)):
                mt2_new_array.append(round(mt2[i], 1)) 

            self.elastic_publishers[f"{name}/botpose_wpiblue_rounded"].set(mt1_new_array)
            self.elastic_publishers[f"{name}/botpose_orb_wpiblue_rounded"].set(mt2_new_array)

            self._process_mt2(name, yaw_rate_deg_per_sec)

            if self.mt2_seeded == False and len(mt1) >= 11 and mt1[0] != 0 and self.seed_timer.hasElapsed(20.0):
                self.pigeon_array.append(mt1[5])

                if len(self.pigeon_array) == 10:
                    max_difference = 0
                    mean = sum(self.pigeon_array)/len(self.pigeon_array)

                    for value in self.pigeon_array:
                        if abs(value - mean) > max_difference:
                            max_difference = abs(value - mean)
                    if max_difference < 0.2:
                        self.mt2_seeded = True
                        self.pigeon_array = []
                        self.seed_pigeon(mean)
                        SmartDashboard.putBoolean("mt2_seeded",True)
                    else:
                        self.pigeon_array.pop(0)


    def seed_pigeon(self, yaw_deg):
        self.drivetrain.pigeon2.set_yaw(yaw_deg)
        current_pose = self.drivetrain.get_state().pose
        seeded_pose = Pose2d(current_pose.translation(), Rotation2d.fromDegrees(yaw_deg))
        self.drivetrain.reset_pose(seeded_pose)
        for name in self.limelight_names:
            self.set_imu_mode(name, 4)

    def _process_mt2(self, name, yaw_rate_deg):
        raw = self.mt2_subscribers[name].get()

        if len(raw) < 11 : return

        x, y, _, _, _, pose_yaw_deg, latency_ms, tag_count, _, avg_dist, _ = raw[:11]

        if tag_count < 1 or abs(yaw_rate_deg) > 360.0:
            return

        now = utils.get_current_time_seconds()
        timestamp = now - (latency_ms / 1000.0) 
        vision_pose = Pose2d(Translation2d(x,y), Rotation2d.fromDegrees(pose_yaw_deg))

        SmartDashboard.putNumberArray("Vision Pose", [x,y,pose_yaw_deg])

        SmartDashboard.putNumber("timestamp", timestamp)
        SmartDashboard.putNumber("vision_age_sec", now - timestamp)

        self.drivetrain.add_vision_measurement(vision_pose, timestamp, (.7, .7, 99999999.0))

