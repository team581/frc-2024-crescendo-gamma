// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;

public class AutoCommands {
  private final RobotCommands actions;
  private final RobotManager robotManager;

  public AutoCommands(RobotCommands actions, RobotManager robotManager) {
    this.actions = actions;
    this.robotManager = robotManager;
  }

  public Command subwooferShotWithTimeout() {
    return actions
        .subwooferShotCommand()
        .withTimeout(2)
        .andThen(actions.outtakeShooterCommand().withTimeout(1))
        .withName("SubwooferShotWithTimeout");
  }

  public Command speakerShotWithTimeout() {
    return actions
        .speakerShotCommand()
        .withTimeout(2)
        .andThen(actions.outtakeShooterCommand().withTimeout(1))
        .withName("SpeakerShotWithTimeout");
  }

  public Command getMidlineNotesAmpCommand() {
    var red4ToRightWingShot = PathPlannerPath.fromPathFile("Red 4 to RWS");
    var red4To5 = PathPlannerPath.fromPathFile("Red 4 to 5");
    var rightWingShotTo5 = PathPlannerPath.fromPathFile("Red RWS to 5");
    var red5ToCenterWingShot = PathPlannerPath.fromPathFile("Red 5 to CWS");
    var red5To6 = PathPlannerPath.fromPathFile("Red 5 to 6");
    var centerWingShotTo6 = PathPlannerPath.fromPathFile("Red CWS to 6");
    ///    var red6ToStageWingShot = PathPlannerPath.fromPathFile("Red 6 to SWS");
    ///    var red6To7 = PathPlannerPath.fromPathFile("Red 6 to 7");
    ///    var stageWingShotTo7 = PathPlannerPath.fromPathFile("SWS to Red 7");
    ///    var red7ToLeftWingShot= PathPlannerPath.fromPathFile("Red 7 to Left Wing Shot");

    return Commands.sequence(
        Commands.either(
            AutoBuilder.followPath(red4ToRightWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(AutoBuilder.followPath(rightWingShotTo5)),
            AutoBuilder.followPath(red4To5),
            () -> robotManager.getState().hasNote),
        Commands.either(
            AutoBuilder.followPath(red5ToCenterWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(AutoBuilder.followPath(centerWingShotTo6)),
            AutoBuilder.followPath(red5To6),
            () -> robotManager.getState().hasNote));
  }
}
