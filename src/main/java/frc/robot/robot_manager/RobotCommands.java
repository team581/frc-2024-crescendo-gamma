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
                robot.waitForStateCommand(RobotState.IDLE_WITH_GP)))
        .withName("StowCommand");
  }

  public Command stopIntakingCommand() {
    return Commands.runOnce(() -> robot.stopIntakingRequest(), requirements)
        .andThen(
            Commands.race(
                robot.waitForStateCommand(RobotState.IDLE_NO_GP),
                robot.waitForStateCommand(RobotState.IDLE_WITH_GP)))
        .withName("StopIntakingCommand");
  }

  public Command intakeCommand() {
    return Commands.runOnce(() -> robot.intakeRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_WITH_GP))
        .withName("IntakeCommand");
  }

  public Command shooterOuttakeCommand() {
    return Commands.runOnce(() -> robot.outtakeShooterRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP))
        .withName("ShooterOuttakeCommand");
  }

  public Command outtakeCommand() {
    return Commands.runOnce(() -> robot.outtakeRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP))
        .withName("OuttakeCommand");
  }

  public Command outtakeShooterCommand() {
    return Commands.runOnce(() -> robot.outtakeShooterRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP))
        .withName("OuttakeShooterCommand");
  }

  public Command shooterAmpCommand() {
    return Commands.runOnce(() -> robot.shooterAmpRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP))
        .withName("shooterAmpCommand");
  }

  public Command homeCommand() {
    return Commands.runOnce(() -> robot.climber.startHoming(), robot.climber)
        .withName("HomeCommand");
  }

  public Command waitForSpeakerShotCommand() {
    return Commands.runOnce(() -> robot.waitSpeakerShotRequest(), requirements)
        .withName("WaitForSpeakerShotCommand");
  }

  public Command waitForFloorShotCommand() {
    return Commands.runOnce(() -> robot.waitFloorShotRequest(), requirements)
        .withName("WaitForFloorShotCommand");
  }

  public Command confirmShotCommand() {
    return Commands.runOnce(() -> robot.confirmShotRequest(), requirements)
        .withName("ConfirmShotCommand");
  }

  public Command waitForAmpShotCommand() {
    return Commands.runOnce(() -> robot.waitAmpShotRequest(), requirements)
        .withName("WaitForAmpShotCommand");
  }

  public Command waitPodiumShotCommand() {
    return Commands.runOnce(() -> robot.waitPodiumShotRequest(), requirements)
        .withName("WaitForPodiumShotCommand");
  }

  public Command waitSubwooferShotCommand() {
    return Commands.runOnce(() -> robot.waitSubwooferShotRequest(), requirements)
        .withName("WaitSubwooferShotCommand");
  }

  public Command speakerShotCommand() {
    return Commands.runOnce(() -> robot.speakerShotRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP))
        .finallyDo(
            () -> {
              robot.snaps.setEnabled(false);
            })
        .withName("SpeakerShotCommand");
  }

  public Command ampShotCommand() {
    return Commands.runOnce(() -> robot.ampShotRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP))
        .withName("AmpShotCommand");
  }

  public Command subwooferShotCommand() {
    return Commands.runOnce(() -> robot.subwooferShotRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP))
        .withName("SubwooferShotCommand");
  }

  public Command podiumShotCommand() {
    return Commands.runOnce(() -> robot.podiumShotRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.IDLE_NO_GP))
        .withName("PodiumShotCommand");
  }

  public Command trapShotCommand() {
    return Commands.runOnce(() -> robot.climb5HangingTrapScoreRequest(), requirements)
        .andThen(robot.waitForStateCommand(RobotState.CLIMB_4_HANGING))
        .withName("TrapShotCommand");
  }

  public Command getClimberForwardCommand() {
    return Commands.runOnce(() -> robot.getClimberForwardRequest(), requirements)
        .withName("ClimberForwardCommand");
  }

  public Command getClimberBackwardCommand() {
    return Commands.runOnce(() -> robot.getClimberBackwardRequest(), requirements)
        .withName("ClimberBackwardCommand");
  }

  public Command preloadNoteCommand() {
    return Commands.runOnce(() -> robot.preloadNoteRequest(), requirements)
        .withName("PreloadNoteCommand");
  }

  public Command stopShootingCommand() {
    return Commands.runOnce(() -> robot.stopShootingRequest(), requirements)
        .withName("StopShootingCommand");
  }
}
