// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Autos extends LifecycleSubsystem {
  private static Command wrapAutoEvent(String commandName, Command command) {
    return Commands.sequence(
            Commands.print("[COMMANDS] Starting auto event " + commandName),
            command.deadlineWith(
                Commands.waitSeconds(5)
                    .andThen(
                        Commands.print(
                            "[COMMANDS] Auto event "
                                + commandName
                                + " has been running for 5+ seconds!"))),
            Commands.print("[COMMANDS] Finished auto event " + commandName))
        .handleInterrupt(() -> System.out.println("[COMMANDS] Cancelled auto event " + commandName))
        .withName(commandName);
  }

  private static void registerCommand(String eventName, Command command) {
    NamedCommands.registerCommand(eventName, wrapAutoEvent("Auto_" + eventName, command));
  }

  private final SwerveSubsystem swerve;
  private final AutoChooser autoChooser;

  public Autos(SwerveSubsystem swerve, LocalizationSubsystem localization, RobotCommands actions) {
    super(SubsystemPriority.AUTOS);
    this.swerve = swerve;
    var autoCommands = new AutoCommands(actions, null);

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        localization::getOdometryPose,
        localization::resetPose,
        swerve::getRobotRelativeSpeeds,
        (robotRelativeSpeeds) -> {
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

    registerCommand("preloadNote", actions.preloadNoteCommand());
    registerCommand("speakerShotNoTimeout", actions.speakerShotCommand());
    registerCommand("speakerShot", autoCommands.speakerShotWithTimeout());
    registerCommand("subwooferShot", autoCommands.subwooferShotWithTimeout());
    registerCommand("intakeFloor", actions.intakeCommand());
    registerCommand("outtakeShooter", actions.outtakeShooterCommand());
    registerCommand("homeClimber", actions.homeCommand());
    registerCommand("stow", actions.stowCommand());
    registerCommand("midlineNotesFromAmp", autoCommands.getMidlineNotesAmpCommand());

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          // Logger.recordOutput(
          //     "Autos/Trajectory/ActivePath", activePath.toArray(new Pose2d[activePath.size()]));
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

    FollowPathCommand.warmupCommand().schedule();
    PathfindingCommand.warmupCommand().schedule();
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
