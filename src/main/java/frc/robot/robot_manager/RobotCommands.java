// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;

  public RobotCommands(RobotManager robot) {
    this.robot = robot;
    var requirementsList =
        List.of(
            robot.wrist,
            robot.elevator,
            robot.climber,
            robot.noteManager.conveyor,
            robot.noteManager.intake,
            robot.noteManager.queuer,
            robot.shooter);
    requirements = requirementsList.toArray(new Subsystem[requirementsList.size()]);
  }

  public Command stowCommand() {
    return Commands.runOnce(() -> robot.stowRequest(), requirements)
        .andThen(
            Commands.race(
                robot.waitForStateCommand(RobotState.IDLE_NO_GP),
                robot.waitForStateCommand(RobotState.IDLE_WITH_GP)));
  }

  public Command intakeCommand() {
    return Commands.runOnce(() -> robot.intakeRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_WITH_GP));
  }

  public Command outtakeCommand() {
    return Commands.runOnce(() -> robot.outtakeRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP));
  }

  public Command outtakeShooterCommand() {
    return Commands.runOnce(() -> robot.outtakeShooterRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP));
  }

  public Command homeCommand() {
    return Commands.runOnce(() -> robot.homingRequest(), requirements)
        .andThen(
            Commands.race(
                robot.waitForStateCommand(RobotState.IDLE_NO_GP),
                robot.waitForStateCommand(RobotState.IDLE_WITH_GP)));
  }

  public Command waitForSpeakerShotCommand() {
    return Commands.runOnce(() -> robot.waitSpeakerShotRequest(), requirements);
  }

  public Command waitForFloorShotCommand() {
    return Commands.runOnce(() -> robot.waitFloorShotRequest(), requirements);
  }

  public Command confirmShotCommand() {
    return Commands.runOnce(() -> robot.confirmShotRequest(), requirements);
  }

  public Command waitForAmpShotCommand() {
    return Commands.runOnce(() -> robot.waitAmpShotRequest(), requirements);
  }

  public Command waitSubwooferShotCommand() {
    return Commands.runOnce(() -> robot.waitSubwooferShotRequest(), requirements);
  }

  public Command speakerShotCommand() {
    return Commands.runOnce(() -> robot.speakerShotRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP));
  }

  public Command ampShotCommand() {
    return Commands.runOnce(() -> robot.ampShotRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP));
  }

  public Command subwooferShotCommand() {
    return Commands.runOnce(() -> robot.subwooferShotRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP));
  }

  public Command trapShotCommand() {
    return Commands.runOnce(() -> robot.trapShotRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.CLIMBER_HANGING));
  }

  public Command getClimberForwardCommand() {
    return Commands.runOnce(() -> robot.getClimberForwardRequest(), requirements);
  }

  public Command getClimberBackwardCommand() {
    return Commands.runOnce(() -> robot.getClimberBackwardRequest(), requirements);
  }

  public Command preloadNoteCommand() {
    return Commands.runOnce(() -> robot.preloadNoteRequest(), requirements);
  }

  public Command waitForIdle() {
    return Commands.waitUntil(
        () ->
            robot.getState() == RobotState.IDLE_NO_GP
                || robot.getState() == RobotState.IDLE_WITH_GP);
  }
}
