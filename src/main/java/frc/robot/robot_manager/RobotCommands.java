// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotCommands {
  private final RobotManager robot;

  public RobotCommands(RobotManager robot) {
    this.robot = robot;
  }

  public Command stowCommand() {
    return Commands.runOnce(() -> robot.stowRequest(), robot.wrist, robot.elevator)
        .andThen(
            Commands.race(
                robot.waitForStateCommand(RobotState.IDLE_NO_GP),
                robot.waitForStateCommand(RobotState.IDLE_WITH_GP),
                robot.waitForStateCommand(RobotState.WAITING_AMP_SHOT)));
  }

  public Command stowAfterIntakeCommand() {
    return Commands.runOnce(
            () -> robot.stowAfterIntakeRequest(),
            robot.wrist,
            robot.intake,
            robot.conveyor,
            robot.queuer)
        .andThen(
            Commands.race(
                robot.waitForStateCommand(RobotState.IDLE_NO_GP),
                robot.waitForStateCommand(RobotState.IDLE_WITH_GP)));
  }

  public Command intakeFloorCommand() {
    return Commands.runOnce(
            () -> robot.groundIntakeRequest(), robot.intake, robot.queuer, robot.conveyor)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_WITH_GP));
  }

  public Command outtakeIntakeCommand() {
    return Commands.runOnce(
            () -> robot.outtakeIntakeRequest(), robot.intake, robot.queuer, robot.conveyor)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP));
  }

  public Command outtakeShooterCommand() {
    return Commands.runOnce(() -> robot.outtakeShooterRequest(), robot.shooter, robot.queuer)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP));
  }

  public Command homeCommand() {
    return Commands.runOnce(() -> robot.homingRequest(), robot.wrist, robot.elevator, robot.climber)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP));
  }

  public Command waitForSpeakerShotCommand() {
    return Commands.runOnce(
        () -> robot.waitSpeakerShotRequest(),
        robot.wrist,
        robot.shooter,
        robot.queuer,
        robot.localization,
        robot.snaps,
        robot.vision,
        robot.swerve);
  }

  public Command passToConveyorForAmpShotCommand() {
    return Commands.runOnce(
        () -> robot.passNoteToConveyorRequest(), robot.intake, robot.queuer, robot.conveyor);
  }

  public Command waitForFloorShotCommand() {
    return Commands.runOnce(
        () -> robot.waitFloorShotRequest(),
        robot.wrist,
        robot.shooter,
        robot.queuer,
        robot.localization,
        robot.snaps,
        robot.vision,
        robot.swerve);
  }

  public Command confirmShotCommand() {
    return Commands.runOnce(
        () -> robot.confirmShotRequest(),
        robot.wrist,
        robot.shooter,
        robot.elevator,
        robot.queuer,
        robot.localization,
        robot.snaps,
        robot.vision,
        robot.swerve);
  }

  public Command speakerShotCommand() {
    return Commands.runOnce(
            () -> robot.speakerShotRequest(),
            robot.wrist,
            robot.shooter,
            robot.queuer,
            robot.localization,
            robot.snaps,
            robot.vision,
            robot.swerve)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP))
        .finallyDo(
            () -> {
              robot.snaps.setEnabled(false);
            });
  }

  public Command ampShotCommand() {
    return Commands.runOnce(() -> robot.ampShotRequest(), robot.elevator, robot.conveyor)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP));
  }

  public Command trapShotCommand() {
    return Commands.runOnce(
            () -> robot.trapShotRequest(), robot.elevator, robot.climber, robot.conveyor)
        .andThen(robot.waitForStateCommand(RobotState.CLIMBER_HANGING));
  }

  public Command waitSubwooferShotCommand() {
    return Commands.runOnce(
        () -> robot.waitSubwooferShotRequest(), robot.wrist, robot.shooter, robot.queuer);
  }

  public Command subwooferShotCommand() {
    return Commands.runOnce(
            () -> robot.subwooferShotRequest(), robot.wrist, robot.queuer, robot.shooter)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP));
  }

  public Command getClimberCommand() {
    return Commands.runOnce(
        () -> robot.getClimberRequest(), robot.wrist, robot.intake, robot.shooter, robot.climber);
  }

  public Command preloadNoteCommand() {
    return Commands.runOnce(
        () -> robot.preloadNoteRequest(), robot.wrist, robot.intake, robot.shooter, robot.climber);
  }

  public Command waitForIdle() {
    return Commands.waitUntil(
        () ->
            robot.getState() == RobotState.IDLE_NO_GP
                || robot.getState() == RobotState.IDLE_WITH_GP);
  }
}
