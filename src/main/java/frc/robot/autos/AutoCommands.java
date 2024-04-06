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

public class AutoCommands {
  private static final boolean USE_DYNAMIC_AUTOS = false;

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

  private boolean hasNote() {
    if (!USE_DYNAMIC_AUTOS) {
      return true;
    }

    return robotManager.noteManager.intake.sensorHasNote()
        || robotManager.noteManager.queuer.sensorHasNote()
        || robotManager.getState().hasNote
        || robotManager.getState() == RobotState.FINISH_INTAKING;
  }

  public Command getMidlineNotesAmp456Command() {
    var red4ToRightWingShot = PathPlannerPath.fromPathFile("Red 4 to RWS");
    var red4To5 = PathPlannerPath.fromPathFile("Red 4 to 5");
    var redRightWingShotTo5 = PathPlannerPath.fromPathFile("Red RWS to 5");
    var red5ToCenterWingShot = PathPlannerPath.fromPathFile("Red 5 to CWS");
    var red5To6 = PathPlannerPath.fromPathFile("Red 5 to 6");
    var redCenterWingShotTo6 = PathPlannerPath.fromPathFile("Red CWS to 6");
    var red6ToStageWingShot = PathPlannerPath.fromPathFile("Red 6 to SWS");

    var blue4ToRightWingShot = PathPlannerPath.fromPathFile("Blue 4 to RWS");
    var blue4To5 = PathPlannerPath.fromPathFile("Blue 4 to 5");
    var blueRightWingShotTo5 = PathPlannerPath.fromPathFile("Blue RWS to 5");
    var blue5ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 5 to CWS");
    var blue5To6 = PathPlannerPath.fromPathFile("Blue 5 to 6");
    var blueCenterWingShotTo6 = PathPlannerPath.fromPathFile("Blue CWS to 6");
    var blue6ToStageWingShot = PathPlannerPath.fromPathFile("Blue 6 to SWS");

    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red4ToRightWingShot, blue4ToRightWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redRightWingShotTo5, blueRightWingShotTo5)),
            followPathForAlliance(red4To5, blue4To5),
            this::hasNote),
        Commands.either(
                followPathForAlliance(red5ToCenterWingShot, blue5ToCenterWingShot)
                    .andThen(speakerShotWithTimeout())
                    .andThen(followPathForAlliance(redCenterWingShotTo6, blueCenterWingShotTo6)),
                followPathForAlliance(red5To6, blue5To6),
                this::hasNote)
            .andThen(
                followPathForAlliance(red6ToStageWingShot, blue6ToStageWingShot)
                    .andThen(speakerShotWithTimeout())));
  }

   public Command getMidlineNotesAmp45Command() {
    var red4ToRightWingShot = PathPlannerPath.fromPathFile("Red 4 to RWS");
    var red4To5 = PathPlannerPath.fromPathFile("Red 4 to 5");
    var redRightWingShotTo5 = PathPlannerPath.fromPathFile("Red RWS to 5");
    var red5ToCenterWingShot = PathPlannerPath.fromPathFile("Red 5 to CWS");
    var red5To6 = PathPlannerPath.fromPathFile("Red 5 to 6");

    var blue4ToRightWingShot = PathPlannerPath.fromPathFile("Blue 4 to RWS");
    var blue4To5 = PathPlannerPath.fromPathFile("Blue 4 to 5");
    var blueRightWingShotTo5 = PathPlannerPath.fromPathFile("Blue RWS to 5");
    var blue5ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 5 to CWS");
    var blue5To6 = PathPlannerPath.fromPathFile("Blue 5 to 6");

    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red4ToRightWingShot, blue4ToRightWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redRightWingShotTo5, blueRightWingShotTo5)),
            followPathForAlliance(red4To5, blue4To5),
            this::hasNote),
        Commands.either(
                followPathForAlliance(red5ToCenterWingShot, blue5ToCenterWingShot)
                    .andThen(speakerShotWithTimeout()),
            followPathForAlliance(red5To6, blue5To6),
            this::hasNote));
  }

  public Command getMidlineNotesAltAmpCommand() {
    var red5To6 = PathPlannerPath.fromPathFile("Red 5 to 6");
    var red6To4 = PathPlannerPath.fromPathFile("Red 6 to 4");
    var red5ToCenterWingShot = PathPlannerPath.fromPathFile("Red 5 to CWS");
    var redCenterWingShotTo6 = PathPlannerPath.fromPathFile("Red CWS to 6");
    var red6ToStageWingShot = PathPlannerPath.fromPathFile("Red 6 to SWS");
    var redStageWingShotTo4 = PathPlannerPath.fromPathFile("Red SWS to 4");
    var red4ToRightWingShot = PathPlannerPath.fromPathFile("Red 4 to RWS");

    var blue5To6 = PathPlannerPath.fromPathFile("Blue 5 to 6");
    var blue6To4 = PathPlannerPath.fromPathFile("Blue 6 to 4");
    var blue5ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 5 to CWS");
    var blueCenterWingShotTo6 = PathPlannerPath.fromPathFile("Blue CWS to 6");
    var blue6ToStageWingShot = PathPlannerPath.fromPathFile("Blue 6 to SWS");
    var blueStageWingShotTo4 = PathPlannerPath.fromPathFile("Blue SWS to 4");
    var blue4ToRightWingShot = PathPlannerPath.fromPathFile("Blue 4 to RWS");

    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red5ToCenterWingShot, blue5ToCenterWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redCenterWingShotTo6, blueCenterWingShotTo6)),
            followPathForAlliance(red5To6, blue5To6),
            this::hasNote),
        Commands.either(
                followPathForAlliance(red6ToStageWingShot, blue6ToStageWingShot)
                    .andThen(speakerShotWithTimeout())
                    .andThen(followPathForAlliance(redStageWingShotTo4, blueStageWingShotTo4)),
                followPathForAlliance(red6To4, blue6To4),
                this::hasNote)
            .andThen(
                followPathForAlliance(red4ToRightWingShot, blue4ToRightWingShot)
                    .andThen(speakerShotWithTimeout())));
  }

  public Command getMidlineNotesSourceCommand() {
    var red8ToLeftWingShot = PathPlannerPath.fromPathFile("Red 8 to LWS");
    var redLeftWingShotTo7 = PathPlannerPath.fromPathFile("Red LWS to 7");
    var red7ToStageWingShot = PathPlannerPath.fromPathFile("Red 7 to SWS");
    var redStageWingShotTo6 = PathPlannerPath.fromPathFile("Red SWS to 6");
    var red6ToStageWingShot = PathPlannerPath.fromPathFile("Red 6 to SWS");
    var red8To7 = PathPlannerPath.fromPathFile("Red 8 to 7");
    var red7To6 = PathPlannerPath.fromPathFile("Red 7 to 6");

    var blue8ToLeftWingShot = PathPlannerPath.fromPathFile("Blue 8 to LWS");
    var blueLeftWingShotTo7 = PathPlannerPath.fromPathFile("Blue LWS to 7");
    var blue7ToStageWingShot = PathPlannerPath.fromPathFile("Blue 7 to SWS");
    var blueStageWingShotTo6 = PathPlannerPath.fromPathFile("Blue SWS to 6");
    var blue6ToStageWingShot = PathPlannerPath.fromPathFile("Blue 6 to SWS");
    var blue8To7 = PathPlannerPath.fromPathFile("Blue 8 to 7");
    var blue7To6 = PathPlannerPath.fromPathFile("Blue 7 to 6");

    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red8ToLeftWingShot, blue8ToLeftWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redLeftWingShotTo7, blueLeftWingShotTo7)),
            followPathForAlliance(red8To7, blue8To7),
            this::hasNote),
        Commands.either(
                followPathForAlliance(red7ToStageWingShot, blue7ToStageWingShot)
                    .andThen(speakerShotWithTimeout())
                    .andThen(followPathForAlliance(redStageWingShotTo6, blueStageWingShotTo6)),
                followPathForAlliance(red7To6, blue7To6),
                this::hasNote)
            .andThen(
                followPathForAlliance(red6ToStageWingShot, blue6ToStageWingShot)
                    .andThen(speakerShotWithTimeout())));
  }
}
