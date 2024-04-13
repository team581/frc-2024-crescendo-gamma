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
  private static final boolean USE_DYNAMIC_AUTOS = true;

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

  public Command speakerSnapCommand() {
    return Commands.runOnce(
        () -> {
          robotManager.snaps.setAngle(robotManager.vision.getDistanceAngleSpeaker().targetAngle());
          robotManager.snaps.setEnabled(true);
        });
  }

  public Command subwooferShotWithTimeout() {
    return actions
        .subwooferShotCommand()
        .withTimeout(2)
        .andThen(actions.forceSpeakerShotCommand().withTimeout(1))
        .withName("SubwooferShotWithTimeout");
  }

  public Command presetLeftShot() {
    return actions
        .presetLeftShotCommand()
        .andThen(actions.forceSpeakerShotCommand().withTimeout(1))
        .withName("PresetLeftShot");
  }

  public Command presetMiddleShot() {
    return actions
        .presetMiddleShotCommand()
        .andThen(actions.forceSpeakerShotCommand().withTimeout(1))
        .withName("PresetMiddleShot");
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

    var blue5To6 = PathPlannerPath.fromPathFile("Blue 5 to 6");
    var blue6To4 = PathPlannerPath.fromPathFile("Blue 6 to 4");
    var blue5ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 5 to CWS");
    var blueCenterWingShotTo6 = PathPlannerPath.fromPathFile("Blue CWS to 6");
    var blue6ToStageWingShot = PathPlannerPath.fromPathFile("Blue 6 to SWS");

    return Commands.sequence(
        Commands.either(
            followPathForAlliance(red5ToCenterWingShot, blue5ToCenterWingShot)
                .andThen(speakerShotWithTimeout())
                .andThen(followPathForAlliance(redCenterWingShotTo6, blueCenterWingShotTo6)),
            followPathForAlliance(red5To6, blue5To6),
            this::hasNote),
        Commands.either(
            followPathForAlliance(red6ToStageWingShot, blue6ToStageWingShot)
                .andThen(speakerShotWithTimeout()),
            followPathForAlliance(red6To4, blue6To4),
            this::hasNote));
  }

  public Command getMidlineNotesOP4Command() {
    var red5ToRightWingShot = PathPlannerPath.fromPathFile("Red 5 to RWS");
    var redRightWingShotTo6 = PathPlannerPath.fromPathFile("Red RWS to 6");
    var red6ToCenterWingShot = PathPlannerPath.fromPathFile("Red 6 to CWS");
    var redAltCenterWingShotTo4 = PathPlannerPath.fromPathFile("Red Alt CWS to 4");

    var blue5ToRightWingShot = PathPlannerPath.fromPathFile("Blue 5 to RWS");
    var blueRightWingShotTo6 = PathPlannerPath.fromPathFile("Blue RWS to 6");
    var blue6ToCenterWingShot = PathPlannerPath.fromPathFile("Blue 6 to CWS");
    var blueAltCenterWingShotTo4 = PathPlannerPath.fromPathFile("Blue Alt CWS to 4");

    return Commands.sequence(
        followPathForAlliance(red5ToRightWingShot, blue5ToRightWingShot)
            .andThen(speakerShotWithTimeout())
            .andThen(followPathForAlliance(redRightWingShotTo6, blueRightWingShotTo6)),
        followPathForAlliance(red6ToCenterWingShot, blue6ToCenterWingShot)
            .andThen(speakerShotWithTimeout())
            .andThen(followPathForAlliance(redAltCenterWingShotTo4, blueAltCenterWingShotTo4)));
  }

  public Command getMidlineNotes64Command() {
    var red6ToAltCenterWingShot = PathPlannerPath.fromPathFile("Red 6 to Alt CWS");
    var red4ToRightWingShot = PathPlannerPath.fromPathFile("Red 4 to RWS");
    var redAltCenterWingShotTo4 = PathPlannerPath.fromPathFile("Red Alt CWS to 4");

    var blue6ToAltCenterWingShot = PathPlannerPath.fromPathFile("Blue 6 to Alt CWS");
    var blue4ToRightWingShot = PathPlannerPath.fromPathFile("Blue 4 to RWS");
    var blueAltCenterWingShotTo4 = PathPlannerPath.fromPathFile("Blue Alt CWS to 4");

    return Commands.sequence(
        followPathForAlliance(red6ToAltCenterWingShot, blue6ToAltCenterWingShot)
            .andThen(speakerShotWithTimeout())
            .andThen(followPathForAlliance(redAltCenterWingShotTo4, blueAltCenterWingShotTo4)),
        followPathForAlliance(red4ToRightWingShot, blue4ToRightWingShot)
            .andThen(speakerShotWithTimeout()));
  }

  public Command getMidlineNotesSourceCommand() {
    var red8ToLeftWingShot = PathPlannerPath.fromPathFile("Red 8 to LWS");
    var redLeftWingShotTo7 = PathPlannerPath.fromPathFile("Red LWS to 7");
    var red7ToAltCenterWingShot = PathPlannerPath.fromPathFile("Red 7 to Alt CWS");
    var redAltCenterWingShotTo6 = PathPlannerPath.fromPathFile("Red Alt CWS to 6");
    var red6ToStageWingShot = PathPlannerPath.fromPathFile("Red 6 to SWS");
    var red8To7 = PathPlannerPath.fromPathFile("Red 8 to 7");
    var red7To6 = PathPlannerPath.fromPathFile("Red 7 to 6");

    var blue8ToLeftWingShot = PathPlannerPath.fromPathFile("Blue 8 to LWS");
    var blueLeftWingShotTo7 = PathPlannerPath.fromPathFile("Blue LWS to 7");
    var blue7ToAltCenterWingShot = PathPlannerPath.fromPathFile("Blue 7 to Alt CWS");
    var blueAltCenterWingShotTo6 = PathPlannerPath.fromPathFile("Blue Alt CWS to 6");
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
                followPathForAlliance(red7ToAltCenterWingShot, blue7ToAltCenterWingShot)
                    .andThen(speakerShotWithTimeout())
                    .andThen(
                        followPathForAlliance(redAltCenterWingShotTo6, blueAltCenterWingShotTo6)),
                followPathForAlliance(red7To6, blue7To6),
                this::hasNote)
            .andThen(
                followPathForAlliance(red6ToStageWingShot, blue6ToStageWingShot)
                    .andThen(speakerShotWithTimeout())));
  }
}
