// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.RobotConfig;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Autos extends LifecycleSubsystem {
  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final ImuSubsystem imu;
  private final RobotCommands actions;
  private final AutoChooser autoChooser;

  public Autos(
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      ImuSubsystem imu,
      RobotCommands actions) {
    super(SubsystemPriority.AUTOS);
    this.swerve = swerve;
    this.localization = localization;
    this.imu = imu;
    this.actions = actions;

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        localization::getOdometryPose,
        localization::resetPose,
        swerve::getRobotRelativeSpeeds,
        (robotRelativeSpeeds) -> {
          robotRelativeSpeeds.omegaRadiansPerSecond *= -1.0;
          swerve.setRobotRelativeSpeeds(robotRelativeSpeeds, true);
        },
        new HolonomicPathFollowerConfig(
            new PIDConstants(4.0, 0.0, 0.0),
            new PIDConstants(4.0, 0.0, 0.0),
            SwerveSubsystem.MaxSpeed,
            0.387,
            new ReplanningConfig()),
        () -> false,
        swerve);

    NamedCommands.registerCommand("preloadNote", actions.preloadNoteCommand());
    NamedCommands.registerCommand(
        "eagerSpeakerShot", actions.waitForIdle().andThen(actions.speakerShotCommand()));
    NamedCommands.registerCommand(
        "speakerShot",
        actions
            .waitForIdle()
            .andThen(actions.speakerShotCommand().withTimeout(3))
            .andThen(actions.outtakeShooterCommand().withTimeout(1)));
    NamedCommands.registerCommand(
        "subwooferShot",
        actions
            .subwooferShotCommand()
            .withTimeout(3)
            .andThen(actions.outtakeShooterCommand().withTimeout(1)));
    NamedCommands.registerCommand("intakeFloor", actions.intakeCommand());
     NamedCommands.registerCommand("outtakeShooter", actions.outtakeShooterCommand());

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Autos/Trajectory/ActivePath", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Autos/Trajectory/TargetPose", targetPose);
        });

    if (!RobotConfig.IS_DEVELOPMENT) {
      PPLibTelemetry.enableCompetitionMode();
    }

    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

    autoChooser = new AutoChooser();
  }

  public Command getAutoCommand() {
    return autoChooser.getAutoCommand();
  }

  private Optional<Rotation2d> getRotationTargetOverride() {
    // Some condition that should decide if we want to override rotation
    if (swerve.snapsEnabled()) {
      // Return an optional containing the rotation override (this should be a field relative
      // rotation)
      return Optional.of(swerve.snapAngle());
    } else {
      return Optional.empty();
    }
  }

  @Override
  public void disabledPeriodic() {
    // Constantly load the selected auto to avoid lag on auto init
    getAutoCommand();
  }
}
