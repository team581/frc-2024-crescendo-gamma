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

  public Command stowUpCommand() {
    return Commands.runOnce(
            () -> robot.stowUpRequest(), robot.wrist, robot.intake, robot.shooter, robot.climber)
        .andThen(
            Commands.race(
                robot.waitForStateCommand(RobotState.IDLE_UP_NO_GP),
                robot.waitForStateCommand(RobotState.IDLE_UP_WITH_GP)));
  }

  public Command stowDownCommand() {
    return Commands.runOnce(
            () -> robot.stowDownRequest(), robot.wrist, robot.intake, robot.shooter, robot.climber)
        .andThen(
            Commands.race(
                robot.waitForStateCommand(RobotState.IDLE_DOWN_NO_GP),
                robot.waitForStateCommand(RobotState.IDLE_DOWN_WITH_GP)));
  }

  public Command stowUpAfterIntakeCommand() {
    return Commands.runOnce(
            () -> robot.stowUpAfterIntakeRequest(),
            robot.wrist,
            robot.intake,
            robot.shooter,
            robot.climber)
        .andThen(
            Commands.race(
                robot.waitForStateCommand(RobotState.IDLE_UP_NO_GP),
                robot.waitForStateCommand(RobotState.IDLE_UP_WITH_GP)));
  }

  public Command stowDownAfterIntakeCommand() {
    return Commands.runOnce(
            () -> robot.stowDownAfterIntakeRequest(),
            robot.wrist,
            robot.intake,
            robot.shooter,
            robot.climber)
        .andThen(
            Commands.race(
                robot.waitForStateCommand(RobotState.IDLE_DOWN_NO_GP),
                robot.waitForStateCommand(RobotState.IDLE_DOWN_WITH_GP)));
  }

  public Command intakeFloorCommand() {
    return Commands.runOnce(
            () -> robot.groundIntakeRequest(),
            robot.wrist,
            robot.intake,
            robot.shooter,
            robot.climber)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_DOWN_WITH_GP));
  }

  public Command sourceIntakeCommand() {
    return Commands.runOnce(
            () -> robot.sourceIntakeRequest(),
            robot.wrist,
            robot.intake,
            robot.shooter,
            robot.climber)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_DOWN_WITH_GP));
  }

  public Command outtakeCommand() {
    return Commands.runOnce(
            () -> robot.outtakeRequest(), robot.wrist, robot.intake, robot.shooter, robot.climber)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_DOWN_NO_GP));
  }

  public Command homeCommand() {
    return Commands.runOnce(
            () -> robot.homingRequest(), robot.wrist, robot.intake, robot.shooter, robot.climber)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_DOWN_NO_GP));
  }

  public Command waitForSpeakerShotCommand() {
    return Commands.runOnce(
        () -> robot.waitSpeakerShotRequest(),
        robot.wrist,
        robot.intake,
        robot.shooter,
        robot.climber);
  }

  public Command waitForAmpShotCommand() {
    return Commands.runOnce(
        () -> robot.waitAmpShotRequest(), robot.wrist, robot.intake, robot.shooter, robot.climber);
  }

  public Command waitForFloorShotCommand() {
    return Commands.runOnce(
        () -> robot.waitFloorShotRequest(),
        robot.wrist,
        robot.intake,
        robot.shooter,
        robot.climber);
  }

  public Command confirmShotCommand() {
    return Commands.runOnce(
        () -> robot.confirmShotRequest(), robot.wrist, robot.intake, robot.shooter, robot.climber);
  }

  public Command speakerShotCommand() {
    return Commands.runOnce(
            () -> robot.speakerShotRequest(),
            robot.wrist,
            robot.intake,
            robot.shooter,
            robot.climber)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_DOWN_NO_GP))
        .finallyDo(
            () -> {
              robot.snaps.setEnabled(false);
            });
  }

  public Command ampShotCommand() {
    return Commands.runOnce(
            () -> robot.ampShotRequest(), robot.wrist, robot.intake, robot.shooter, robot.climber)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_DOWN_NO_GP));
  }

  public Command trapShotCommand() {
    return Commands.runOnce(
            () -> robot.trapShotRequest(), robot.wrist, robot.intake, robot.shooter, robot.climber)
        .andThen(robot.waitForStateCommand(RobotState.CLIMBER_HANGING));
  }

  public Command waitSubwooferShotCommand() {
    return Commands.runOnce(
        () -> robot.waitSubwooferShotRequest(),
        robot.wrist,
        robot.intake,
        robot.shooter,
        robot.climber);
  }

  public Command subwooferShotCommand() {
    return Commands.runOnce(
            () -> robot.subwooferShotRequest(),
            robot.wrist,
            robot.intake,
            robot.shooter,
            robot.climber)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_DOWN_NO_GP));
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
            robot.getState() == RobotState.IDLE_DOWN_NO_GP
                || robot.getState() == RobotState.IDLE_DOWN_WITH_GP
                || robot.getState() == RobotState.IDLE_UP_NO_GP
                || robot.getState() == RobotState.IDLE_UP_WITH_GP);
  }
}
