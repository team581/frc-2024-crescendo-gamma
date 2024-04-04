// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import java.util.function.BooleanSupplier;

public class AutoCommands {
  private static Command followPathForAlliance(PathPlannerPath redPath, PathPlannerPath bluePath) {
    return Commands.either(
        AutoBuilder.followPath(redPath),
        AutoBuilder.followPath(bluePath),
        FmsSubsystem::isRedAlliance);
  }

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
        .andThen(actions.forceSpeakerShotCommand().withTimeout(1))
        .withName("SubwooferShotWithTimeout");
  }

  public Command speakerShotWithTimeout() {
    return actions
        .speakerShotCommand()
        .withTimeout(2)
        .andThen(actions.forceSpeakerShotCommand().withTimeout(1))
        .withName("SpeakerShotWithTimeout");
  }

  public Command getMidlineNotesAmpCommand() {
    var red4ToRightWingShot = PathPlannerPath.fromPathFile("Red 4 to RWS");
    var red4To5 = PathPlannerPath.fromPathFile("Red 4 to 5");
    var redRightWingShotTo5 = PathPlannerPath.fromPathFile("Red RWS to 5");
    var red5ToCenterWingShot = PathPlannerPath.fromPathFile("Red 5 to CWS");
    var red5To6 = PathPlannerPath.fromPathFile("Red 5 to 6");
    var redCenterWingShotTo6 = PathPlannerPath.fromPathFile("Red CWS to 6");
    var red6ToStageWingShot = PathPlannerPath.fromPathFile("Red 6 to SWS");
    var redStageWingShotTo6 = PathPlannerPath.fromPathFile("Red SWS to 6");
    ///    var red6To7 = PathPlannerPath.fromPathFile("Red 6 to 7");
    ///    var redStageWingShotTo7 = PathPlannerPath.fromPathFile("SWS to Red 7");
    ///    var red7ToLeftWingShot= PathPlannerPath.fromPathFile("Red 7 to Left Wing Shot");

    var blue4ToRightWingShot = PathPlannerPath.fromPathFile("Red 4 to RWS");
    var blue4To5 = PathPlannerPath.fromPathFile("Red 4 to 5");
    var blueRightWingShotTo5 = PathPlannerPath.fromPathFile("Red RWS to 5");
    var blue5ToCenterWingShot = PathPlannerPath.fromPathFile("Red 5 to CWS");
    var blue5To6 = PathPlannerPath.fromPathFile("Red 5 to 6");
    var blueCenterWingShotTo6 = PathPlannerPath.fromPathFile("Red CWS to 6");
    var blue6ToStageWingShot = PathPlannerPath.fromPathFile("Red 6 to SWS");
    var blueStageWingShotTo6 = PathPlannerPath.fromPathFile("Red SWS to 6");
    ///    var blue6To7 = PathPlannerPath.fromPathFile("Red 6 to 7");
    ///    var blueStageWingShotTo7 = PathPlannerPath.fromPathFile("SWS to Red 7");
    ///    var blue7ToLeftWingShot= PathPlannerPath.fromPathFile("Red 7 to Left Wing Shot");

    BooleanSupplier hasNote =
        () ->
            robotManager.getState().hasNote
                || robotManager.getState() == RobotState.FINISH_INTAKING;
    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red4ToRightWingShot, blue4ToRightWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redRightWingShotTo5, blueRightWingShotTo5)),
            followPathForAlliance(red4To5, blue4To5),
            hasNote),
        Commands.either(
                followPathForAlliance(red5ToCenterWingShot, blue5ToCenterWingShot)
                    .andThen(speakerShotWithTimeout())
                    .andThen(followPathForAlliance(redCenterWingShotTo6, blueCenterWingShotTo6)),
                followPathForAlliance(red5To6, blue5To6),
                hasNote)
            .andThen(
                followPathForAlliance(red6ToStageWingShot, blue6ToStageWingShot)
                    .andThen(speakerShotWithTimeout())
                    .andThen(followPathForAlliance(redStageWingShotTo6, blueStageWingShotTo6))));
  }

  public Command getMidlineNotesSourceCommand() {
    var red6To5 = PathPlannerPath.fromPathFile("Red 6 to 5");
    var red5To4 = PathPlannerPath.fromPathFile("Red 5 to 4");
    var red6ToStageWingShot = PathPlannerPath.fromPathFile("Red 6 to SWS");
    var redStageWingShotTo5 = PathPlannerPath.fromPathFile("Red SWS to 5");
    var red5ToCenterWingShot = PathPlannerPath.fromPathFile("Red 5 to CWS");
    var redCenterWingShotTo4 = PathPlannerPath.fromPathFile("Red CWS to 4");
    var red4ToRightWingShot = PathPlannerPath.fromPathFile("Red 4 to RWS");
    var redRightWingShotTo4 = PathPlannerPath.fromPathFile("Red RWS to 4");

    var blue6To5 = PathPlannerPath.fromPathFile("Blue 6 to 5");
    var blue5To4 = PathPlannerPath.fromPathFile("Blue 5 to 4");
    var blue6ToStageWingShot = PathPlannerPath.fromPathFile("Blue 6 to SWS");
    var blueStageWingShotTo5 = PathPlannerPath.fromPathFile("Blue SWS to 5");
    var blue5ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 5 to CWS");
    var blueCenterWingShotTo4 = PathPlannerPath.fromPathFile("Blue CWS to 4");
    var blue4ToRightWingShot = PathPlannerPath.fromPathFile("Blue 4 to RWS");
    var blueRightWingShotTo4 = PathPlannerPath.fromPathFile("Blue RWS to 4");

    BooleanSupplier hasNote =
        () ->
            robotManager.getState().hasNote
                || robotManager.getState() == RobotState.FINISH_INTAKING;
    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red6ToStageWingShot, blue6ToStageWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redStageWingShotTo5, blueStageWingShotTo5)),
            followPathForAlliance(red6To5, blue6To5),
            hasNote),
        Commands.either(
                followPathForAlliance(red5ToCenterWingShot, blue5ToCenterWingShot)
                    .andThen(speakerShotWithTimeout())
                    .andThen(followPathForAlliance(redCenterWingShotTo4, blueCenterWingShotTo4)),
                followPathForAlliance(red5To4, blue5To4),
                hasNote)
            .andThen(
                followPathForAlliance(red4ToRightWingShot, blue4ToRightWingShot)
                    .andThen(speakerShotWithTimeout())
                    .andThen(followPathForAlliance(redRightWingShotTo4, blueRightWingShotTo4))));
  }
}
