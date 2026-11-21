#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math

import commands2
from commands2 import cmd
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants

from phoenix6 import swerve, SignalLogger
from wpilib import DriverStation
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from wpimath import applyDeadband

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """
    def __init__(self) -> None:
        self._max_speed = (
            1.0 * TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(0.0)
            .with_rotational_deadband(0.0)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            )
        )

        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._joystick = CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()

        # Configure the button bindings
        self.configureButtonBindings()

    @staticmethod
    def _shape_axis(value: float, deadband: float = 0.08) -> float:
        deadbanded = applyDeadband(value, deadband)
        return math.copysign(deadbanded * deadbanded, deadbanded)

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        self._shape_axis(-self._joystick.getLeftY()) * self._max_speed
                    )
                    .with_velocity_y(
                        self._shape_axis(-self._joystick.getLeftX()) * self._max_speed
                    )
                    .with_rotational_rate(
                        self._shape_axis(-self._joystick.getRightX()) * self._max_angular_rate
                    )
                )
            )
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(self.drivetrain.seed_field_centric)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # Simple drive forward auton
        idle = swerve.requests.Idle()
        return cmd.sequence(
            # Reset our field centric heading to match the robot
            # facing away from our alliance station wall (0 deg).
            self.drivetrain.runOnce(
                lambda: self.drivetrain.seed_field_centric(Rotation2d.fromDegrees(0))
            ),
            # Then slowly drive forward (away from us) for 5 seconds.
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(0.5)
                    .with_velocity_y(0)
                    .with_rotational_rate(0)
                )
            )
            .withTimeout(5.0),
            # Finally idle for the rest of auton
            self.drivetrain.apply_request(lambda: idle)
        )
