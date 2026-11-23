from wpilib import Field2d, SmartDashboard
from wpimath.geometry import Pose2d
from phoenix6.hardware import Pigeon2
import ntcore

class Vision:
    def __init__(self, limelight_names, drivetrain):
        self.mt2_init = False

        self.drivetrain = drivetrain

        self.instance = ntcore.NetworkTableInstance.getDefault()
        self.limelight_names = limelight_names

        self.mt1_subscribers = {}

        self.imu_publishers = {}
        self.orientation_publishers = {}

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        for name in self.limelight_names:
            table = self.instance.getTable(name)
            self.mt1_subscribers[name] = table.getFloatArrayTopic("botpose_wpiblue").subscribe([])

            self.imu_publishers[name] = table.getIntegerTopic("imumode_set").publish()
            self.orientation_publishers[name] = table.getDoubleArrayTopic("robot_orientation_set").publish()

    def set_imu_mode(self, limelight_name, mode = 1):
        """
        0: EXTERNAL_ONLY (Uses external gyro only)
        1: EXTERNAL_SEED (Seeds internal IMU to match external yaw)
        2: INTERNAL_ONLY (Uses internal IMU only)
        3: INTERNAL_MT1_ASSIST (Internal IMU + MT1 vision yaw)
        4: INTERNAL_EXTERNAL_ASSIST (Internal IMU + external gyro)
        """
        self.imu_publishers[limelight_name].set(mode)

    def get_mt1(self, limelight_name):
        array = self.mt1_subscribers[limelight_name].get()

    def feed(self, orientation_data):
        pass

    def periodic(self):
        pose = self.drivetrain.get_state().pose()
        rotation_in_degrees = pose.rotation().degrees()

        SmartDashboard.putNumber("Pose X", round(pose.X(), 3))
        SmartDashboard.putNumber("Pose Y", round(pose.Y(), 3))
        SmartDashboard.putNumber("Pose Yaw", round(rotation_in_degrees, 1))

        # pigeon = Pigeon2(25)

        # self.feed([pigeon.get_yaw(), pigeon.])