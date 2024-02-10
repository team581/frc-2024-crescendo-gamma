// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.climber.ClimberMode;
import frc.robot.climber.IClimberSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.shooter.ShooterMode;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.shoulder.ShoulderPositions;
import frc.robot.shoulder.ShoulderSubsystem;
import frc.robot.snaps.SnapManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.DistanceAngle;
import frc.robot.vision.VisionSubsystem;
import java.util.EnumSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class RobotManager extends LifecycleSubsystem {
  private static final Set<RobotState> HAS_GP_STATES =
      EnumSet.of(
          RobotState.WAITING_SUBWOOFER_SHOT,
          RobotState.PREPARE_SUBWOOFER_SHOT,
          RobotState.SUBWOOFER_SHOOT,
          RobotState.TRAP_SHOOT,
          RobotState.WAITING_SPEAKER_SHOT,
          RobotState.PREPARE_SPEAKER_SHOT,
          RobotState.SPEAKER_SHOOT,
          RobotState.PREPARE_AMP_SHOT,
          RobotState.AMP_SHOOT,
          RobotState.IDLE_UP_WITH_GP,
          RobotState.IDLE_DOWN_WITH_GP,
          RobotState.SOURCE_INTAKING_SETTLING,
          RobotState.GROUND_INTAKING_SETTLING);
  private static final Set<RobotState> HOMED_STATES = EnumSet.allOf(RobotState.class);

  {
    HOMED_STATES.remove(RobotState.HOMING);
    HOMED_STATES.remove(RobotState.UNHOMED);
  }

  public final ShoulderSubsystem shoulder;
  public final IntakeSubsystem intake;
  public final ShooterSubsystem shooter;
  public final LocalizationSubsystem localization;
  public final VisionSubsystem vision;
  public final IClimberSubsystem climber;
  public final SwerveSubsystem swerve;
  public final SnapManager snaps;
  private final ImuSubsystem imu;

  private RobotState state = RobotState.UNHOMED;

  // State request flags
  private boolean homingFlag = false;
  private boolean waitSpeakerShotFlag = false;
  private boolean speakerShotFlag = false;
  private boolean waitAmpShotFlag = false;
  private boolean ampShotFlag = false;
  private boolean trapShotFlag = false;
  private boolean waitSubwooferShotFlag = false;
  private boolean subwooferShotFlag = false;
  private boolean groundIntakeFlag = false;
  private boolean sourceIntakeFlag = false;
  private boolean outtakeFlag = false;
  private boolean stowUpFlag = false;
  private boolean stowDownFlag = false;
  private boolean stowUpAfterIntakeFlag = false;
  private boolean stowDownAfterIntakeFlag = false;
  private boolean waitingClimberRaisedFlag = false;
  private boolean raiseClimberFlag = false;
  private boolean hangClimberFlag = false;
  private boolean preloadNoteRequest = false;
  private boolean waitFloorShotFlag = false;
  private boolean floorShotFlag = false;

  public RobotManager(
      ShoulderSubsystem shoulder,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      LocalizationSubsystem localization,
      VisionSubsystem vision,
      IClimberSubsystem climber,
      SwerveSubsystem swerve,
      SnapManager snaps,
      ImuSubsystem imu) {
    super(SubsystemPriority.ROBOT_MANAGER);
    this.shoulder = shoulder;
    this.intake = intake;
    this.shooter = shooter;
    this.localization = localization;
    this.vision = vision;
    this.climber = climber;
    this.swerve = swerve;
    this.snaps = snaps;
    this.imu = imu;
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("RobotManager/State", state);

    Logger.recordOutput("RobotManager/HomingFlag", homingFlag);
    Logger.recordOutput("RobotManager/Flags/WaitSpeakerShotFlag", waitSpeakerShotFlag);
    Logger.recordOutput("RobotManager/Flags/SpeakerShotFlag", speakerShotFlag);
    Logger.recordOutput("RobotManager/Flags/WaitAmpShotFlag", waitAmpShotFlag);
    Logger.recordOutput("RobotManager/Flags/AmpShotFlag", ampShotFlag);
    Logger.recordOutput("RobotManager/Flags/TrapShotFlag", trapShotFlag);
    Logger.recordOutput("RobotManager/Flags/WaitSubwooferShotFlag", waitSubwooferShotFlag);
    Logger.recordOutput("RobotManager/Flags/SubwooferShotFlag", subwooferShotFlag);
    Logger.recordOutput("RobotManager/Flags/StowDownAfterIntakeFlag", stowDownAfterIntakeFlag);
    Logger.recordOutput("RobotManager/Flags/StowUpAfterIntakeFlag", stowUpAfterIntakeFlag);
    Logger.recordOutput("RobotManager/Flags/GroundIntakeFlag", groundIntakeFlag);
    Logger.recordOutput("RobotManager/Flags/SourceIntakeFlag", sourceIntakeFlag);
    Logger.recordOutput("RobotManager/Flags/OuttakeFlag", outtakeFlag);
    Logger.recordOutput("RobotManager/Flags/StowUpFlag", stowUpFlag);
    Logger.recordOutput("RobotManager/Flags/StowDownFlag", stowDownFlag);
    Logger.recordOutput("RobotManager/Flags/WaitingClimberRaisedFlag", waitingClimberRaisedFlag);
    Logger.recordOutput("RobotManager/Flags/RaiseClimberFlag", raiseClimberFlag);
    Logger.recordOutput("RobotManager/Flags/HangClimberFlag", hangClimberFlag);
    Logger.recordOutput("RobotManager/Flags/PreloadNoteRequest", preloadNoteRequest);
    Logger.recordOutput("RobotManager/Flags/WaitFloorShotFlag", waitFloorShotFlag);
    Logger.recordOutput("RobotManager/Flags/FloorShotFlag", floorShotFlag);

    DistanceAngle speakerVisionTargets = vision.getDistanceAngleSpeaker();
    DistanceAngle floorShotVisionTargets = vision.getDistanceAngleFloorShot();
    double speakerDistance = speakerVisionTargets.distance();
    double floorSpotDistance = floorShotVisionTargets.distance();
    Rotation2d shoulderAngleForSpeaker = shoulder.getAngleFromDistanceToSpeaker(speakerDistance);
    Rotation2d shoulderAngleForFloorSpot =
        shoulder.getAngleFromDistanceToFloorSpot(floorSpotDistance);
    Rotation2d robotAngleToSpeaker =
        Rotation2d.fromDegrees(
            imu.getRobotHeading().getDegrees() + speakerVisionTargets.angle().getDegrees());
    shooter.setSpeakerDistance(speakerDistance);
    Rotation2d robotAngleToFloorSpot =
        Rotation2d.fromDegrees(
            imu.getRobotHeading().getDegrees() + floorShotVisionTargets.angle().getDegrees());
    shooter.setFloorSpotDistance(floorSpotDistance);

    // State transitions from requests
    if (homingFlag) {
      shoulder.resetHoming();
      state = RobotState.HOMING;
    }
    if (groundIntakeFlag) {
      intake.setState(IntakeState.IDLE);
      if (HOMED_STATES.contains(state)) {
        state = RobotState.GROUND_INTAKING;
      }
    }
    if (stowUpFlag) {
      if (HOMED_STATES.contains(state)) {
        if (HAS_GP_STATES.contains(state)) {
          state = RobotState.IDLE_UP_WITH_GP;
        } else {
          state = RobotState.IDLE_UP_NO_GP;
        }
      }
    }
    if (stowDownFlag) {
      if (HOMED_STATES.contains(state)) {
        if (HAS_GP_STATES.contains(state)) {
          state = RobotState.IDLE_DOWN_WITH_GP;
        } else {
          state = RobotState.IDLE_DOWN_NO_GP;
        }
      }
    }
    if (stowDownAfterIntakeFlag) {
      if (HOMED_STATES.contains(state) && state != RobotState.GROUND_INTAKING_SETTLING) {
        if (HAS_GP_STATES.contains(state)) {
          state = RobotState.IDLE_DOWN_WITH_GP;
        } else {
          state = RobotState.IDLE_DOWN_NO_GP;
        }
      }
    }
    if (stowUpAfterIntakeFlag) {
      if (HOMED_STATES.contains(state) && state != RobotState.GROUND_INTAKING_SETTLING) {
        if (HAS_GP_STATES.contains(state)) {
          state = RobotState.IDLE_UP_WITH_GP;
        } else {
          state = RobotState.IDLE_UP_NO_GP;
        }
      }
    }
    if (waitingClimberRaisedFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.WAITING_CLIMBER_RAISED;
      }
    }
    if (raiseClimberFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.PREPARE_CLIMBER_RAISED;
      }
    }
    if (hangClimberFlag) {
      if (state == RobotState.CLIMBER_RAISED) {
        state = RobotState.PREPARE_CLIMBER_HANGING;
      }
    }
    if (waitSpeakerShotFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.WAITING_SPEAKER_SHOT;
      }
    }
    if (waitSubwooferShotFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.WAITING_SUBWOOFER_SHOT;
      }
    }
    if (waitAmpShotFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.WAITING_AMP_SHOT;
      }
    }
    if (sourceIntakeFlag) {
      intake.setState(IntakeState.IDLE);
      if (HOMED_STATES.contains(state)) {
        state = RobotState.SOURCE_INTAKING;
      }
    }
    if (outtakeFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.OUTTAKING;
      }
    }
    if (speakerShotFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.PREPARE_SPEAKER_SHOT;
      }
    }
    if (ampShotFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.PREPARE_AMP_SHOT;
      }
    }
    if (trapShotFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.TRAP_SHOOT;
      }
    }
    if (subwooferShotFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.PREPARE_SUBWOOFER_SHOT;
      }
    }
    if (preloadNoteRequest) {
      if (HOMED_STATES.contains(state) && !HAS_GP_STATES.contains(state)) {
        state = RobotState.IDLE_UP_WITH_GP;
      }
    }
    if (waitFloorShotFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.WAITING_FLOOR_SHOT;
      }
    }
    if (floorShotFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.PREPARE_FLOOR_SHOT;
      }
    }

    // Automatic state transitions
    switch (state) {
      case UNHOMED:
        if (shoulder.getHomingState() == HomingState.HOMED) {
          state = RobotState.IDLE_DOWN_NO_GP;
        }
        break;
      case IDLE_UP_NO_GP:
      case IDLE_DOWN_NO_GP:
      case IDLE_UP_WITH_GP:
      case IDLE_DOWN_WITH_GP:
      case WAITING_SPEAKER_SHOT:
      case WAITING_AMP_SHOT:
      case WAITING_FLOOR_SHOT:
      case WAITING_SUBWOOFER_SHOT:
      case WAITING_CLIMBER_RAISED:
      case CLIMBER_RAISED:
      case CLIMBER_HANGING:
        // Do nothing
        break;
      case HOMING:
        if (shoulder.getHomingState() == HomingState.HOMED) {
          state = RobotState.IDLE_DOWN_NO_GP;
        }
        break;
      case GROUND_INTAKING:
        if (intake.atGoal(IntakeState.INTAKING)) {
          state = RobotState.GROUND_INTAKING_SETTLING;
        }
        break;
      case GROUND_INTAKING_SETTLING:
        if (intake.atGoal(IntakeState.IDLE)) {
          state = RobotState.IDLE_DOWN_WITH_GP;
        }
        break;
      case SOURCE_INTAKING:
        if (intake.atGoal(IntakeState.INTAKING)) {
          state = RobotState.SOURCE_INTAKING_SETTLING;
        }
        break;
      case SOURCE_INTAKING_SETTLING:
        if (intake.atGoal(IntakeState.IDLE)) {
          state = RobotState.IDLE_UP_WITH_GP;
        }
        break;
      case PREPARE_FLOOR_SHOT:
        if (shoulder.atAngle(shoulderAngleForFloorSpot)
            && shooter.atGoal(ShooterMode.FLOOR_SHOT)
            && (Math.abs(floorShotVisionTargets.angle().getDegrees()) < 2.5)
            && swerve.movingSlowEnoughForSpeakerShot()
            && imu.getRobotAngularVelocity().getDegrees() < 2.5) {
          state = RobotState.FLOOR_SHOOT;
        }
        break;
      case PREPARE_SUBWOOFER_SHOT:
        if (shoulder.atAngle(ShoulderPositions.SUBWOOFER_SHOT)
            && shooter.atGoal(ShooterMode.SUBWOOFER_SHOT)) {
          state = RobotState.SUBWOOFER_SHOOT;
        }
        break;
      case PREPARE_AMP_SHOT:
        if (shoulder.atAngle(ShoulderPositions.AMP_SHOT) && shooter.atGoal(ShooterMode.AMP_SHOT)) {
          state = RobotState.AMP_SHOOT;
        }
        break;
      case PREPARE_SPEAKER_SHOT:
        double distance = speakerDistance;
        imu.setTolerance(imu.getAngleToleranceFromDistanceToSpeaker(distance));
        if (shoulder.atAngle(shoulderAngleForSpeaker, distance)
            && shooter.atGoal(ShooterMode.SPEAKER_SHOT)
            && (Math.abs(speakerVisionTargets.angle().getDegrees())
                < imu.getTolerance().getDegrees())
            && swerve.movingSlowEnoughForSpeakerShot()
            && imu.getRobotAngularVelocity().getDegrees() < imu.getTolerance().getDegrees()
            && imu.atAngle(robotAngleToSpeaker, distance)) {
          state = RobotState.SPEAKER_SHOOT;
        }
        break;
      case OUTTAKING:
      case FLOOR_SHOOT:
      case AMP_SHOOT:
      case SUBWOOFER_SHOOT:
      case SPEAKER_SHOOT:
        if (intake.atGoal(IntakeState.IDLE)) {
          state = RobotState.IDLE_DOWN_NO_GP;
        }
        break;
      case TRAP_SHOOT:
        if (intake.atGoal(IntakeState.IDLE)) {
          state = RobotState.CLIMBER_HANGING;
        }
        break;
      case PREPARE_CLIMBER_RAISED:
        if (climber.atGoal(ClimberMode.RAISED)) {
          state = RobotState.CLIMBER_RAISED;
        }
        break;
      case PREPARE_CLIMBER_HANGING:
        if (climber.atGoal(ClimberMode.HANGING)) {
          state = RobotState.CLIMBER_HANGING;
        }
        break;
      default:
        // Should never happen
        break;
    }

    // State actions
    switch (state) {
      case UNHOMED:
        shoulder.startPreMatchHoming();
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.IDLE);
        break;
      case HOMING:
        shoulder.startMidMatchHoming();
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.IDLE);
        break;
      case IDLE_UP_NO_GP:
        shoulder.setAngle(ShoulderPositions.STOWED_UP);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.IDLE);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case IDLE_UP_WITH_GP:
        shoulder.setAngle(ShoulderPositions.STOWED_UP);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.IDLE);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case IDLE_DOWN_NO_GP:
        shoulder.setAngle(ShoulderPositions.STOWED_DOWN);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.IDLE);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case IDLE_DOWN_WITH_GP:
        shoulder.setAngle(ShoulderPositions.STOWED_DOWN);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.IDLE);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case GROUND_INTAKING:
        shoulder.setAngle(ShoulderPositions.GROUND_INTAKING);
        intake.setState(IntakeState.INTAKING);
        shooter.setMode(ShooterMode.INTAKE);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case GROUND_INTAKING_SETTLING:
        shoulder.setAngle(ShoulderPositions.STOWED_DOWN);
        intake.setState(IntakeState.INTAKING);
        shooter.setMode(ShooterMode.INTAKE);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case SOURCE_INTAKING:
        shoulder.setAngle(ShoulderPositions.SOURCE_INTAKING);
        intake.setState(IntakeState.INTAKING);
        shooter.setMode(ShooterMode.INTAKE);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case SOURCE_INTAKING_SETTLING:
        shoulder.setAngle(ShoulderPositions.STOWED_UP);
        intake.setState(IntakeState.INTAKING);
        shooter.setMode(ShooterMode.INTAKE);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case OUTTAKING:
        shoulder.setAngle(ShoulderPositions.OUTTAKING);
        intake.setState(IntakeState.OUTTAKING);
        shooter.setMode(ShooterMode.IDLE);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case WAITING_FLOOR_SHOT:
      case PREPARE_FLOOR_SHOT:
        shoulder.setAngle(shoulderAngleForFloorSpot);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.FLOOR_SHOT);
        climber.setGoal(ClimberMode.IDLE);
        snaps.setAngle(robotAngleToFloorSpot);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case FLOOR_SHOOT:
        shoulder.setAngle(shoulderAngleForFloorSpot);
        intake.setState(IntakeState.PASS_TO_QUEUER);
        shooter.setMode(ShooterMode.FLOOR_SHOT);
        climber.setGoal(ClimberMode.IDLE);
        snaps.setAngle(robotAngleToFloorSpot);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case WAITING_SUBWOOFER_SHOT:
      case PREPARE_SUBWOOFER_SHOT:
        shoulder.setAngle(ShoulderPositions.SUBWOOFER_SHOT);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case SUBWOOFER_SHOOT:
        shoulder.setAngle(ShoulderPositions.SUBWOOFER_SHOT);
        intake.setState(IntakeState.PASS_TO_QUEUER);
        shooter.setMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case WAITING_SPEAKER_SHOT:
      case PREPARE_SPEAKER_SHOT:
        shoulder.setAngle(shoulderAngleForSpeaker);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoal(ClimberMode.IDLE);
        snaps.setAngle(robotAngleToSpeaker);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case SPEAKER_SHOOT:
        shoulder.setAngle(shoulderAngleForSpeaker);
        intake.setState(IntakeState.PASS_TO_QUEUER);
        shooter.setMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoal(ClimberMode.IDLE);
        snaps.setAngle(robotAngleToSpeaker);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case WAITING_AMP_SHOT:
      case PREPARE_AMP_SHOT:
        shoulder.setAngle(ShoulderPositions.AMP_SHOT);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.AMP_SHOT);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case AMP_SHOOT:
        shoulder.setAngle(ShoulderPositions.AMP_SHOT);
        intake.setState(IntakeState.PASS_TO_CONVEYOR);
        shooter.setMode(ShooterMode.AMP_SHOT);
        climber.setGoal(ClimberMode.IDLE);
        break;
      case WAITING_CLIMBER_RAISED:
        shoulder.setAngle(ShoulderPositions.WAITING_CLIMBER_RAISED);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.IDLE);
        climber.setGoal(ClimberMode.RAISED);
        break;
      case PREPARE_CLIMBER_RAISED:
      case CLIMBER_RAISED:
        shoulder.setAngle(ShoulderPositions.TRAP_SHOT);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.IDLE);
        climber.setGoal(ClimberMode.RAISED);
        break;
      case PREPARE_CLIMBER_HANGING:
        shoulder.setAngle(ShoulderPositions.TRAP_SHOT);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.IDLE);
        climber.setGoal(ClimberMode.HANGING);
        break;
      case CLIMBER_HANGING:
        shoulder.setAngle(ShoulderPositions.TRAP_SHOT);
        intake.setState(IntakeState.IDLE);
        shooter.setMode(ShooterMode.IDLE);
        climber.setGoal(ClimberMode.HANGING);
        break;
      case TRAP_SHOOT:
        shoulder.setAngle(ShoulderPositions.TRAP_SHOT);
        intake.setState(IntakeState.PASS_TO_CONVEYOR);
        shooter.setMode(ShooterMode.IDLE);
        climber.setGoal(ClimberMode.HANGING);
        break;
      default:
        // Should never happen
        break;
    }

    // Reset all flags
    homingFlag = false;
    waitSpeakerShotFlag = false;
    waitSubwooferShotFlag = false;
    speakerShotFlag = false;
    ampShotFlag = false;
    waitAmpShotFlag = false;
    trapShotFlag = false;
    subwooferShotFlag = false;
    groundIntakeFlag = false;
    stowDownAfterIntakeFlag = false;
    stowUpAfterIntakeFlag = false;
    sourceIntakeFlag = false;
    outtakeFlag = false;
    stowUpFlag = false;
    stowDownFlag = false;
    waitingClimberRaisedFlag = false;
    raiseClimberFlag = false;
    hangClimberFlag = false;
    preloadNoteRequest = false;
    waitFloorShotFlag = false;
    floorShotFlag = false;
  }

  public void homingRequest() {
    homingFlag = true;
  }

  public void waitSpeakerShotRequest() {
    waitSpeakerShotFlag = true;
  }

  public void waitSubwooferShotRequest() {
    waitSubwooferShotFlag = true;
  }

  public void speakerShotRequest() {
    speakerShotFlag = true;
  }

  public void ampShotRequest() {
    ampShotFlag = true;
  }

  public void trapShotRequest() {
    trapShotFlag = true;
  }

  public void subwooferShotRequest() {
    subwooferShotFlag = true;
  }

  public void groundIntakeRequest() {
    groundIntakeFlag = true;
  }

  public void sourceIntakeRequest() {
    sourceIntakeFlag = true;
  }

  public void outtakeRequest() {
    outtakeFlag = true;
  }

  public void stowUpRequest() {
    stowUpFlag = true;
  }

  public void stowDownRequest() {
    stowDownFlag = true;
  }

  public void waitingClimberRaisedRequest() {
    waitingClimberRaisedFlag = true;
  }

  public void raiseClimberRequest() {
    raiseClimberFlag = true;
  }

  public void hangClimberRequest() {
    hangClimberFlag = true;
  }

  public void waitAmpShotRequest() {
    waitAmpShotFlag = true;
  }

  public void preloadNoteRequest() {
    preloadNoteRequest = true;
  }

  public void waitFloorShotRequest() {
    waitFloorShotFlag = true;
  }

  public void floorShotRequest() {
    floorShotFlag = true;
  }

  public void confirmShotRequest() {
    if (state == RobotState.WAITING_SUBWOOFER_SHOT) {
      subwooferShotRequest();
    } else if (state == RobotState.WAITING_AMP_SHOT) {
      ampShotRequest();
    } else if (state == RobotState.WAITING_FLOOR_SHOT) {
      floorShotRequest();
    } else {
      speakerShotRequest();
    }
  }

  public void getClimberRequest() {
    if (state == RobotState.WAITING_CLIMBER_RAISED) {
      raiseClimberRequest();
    } else if (state == RobotState.CLIMBER_RAISED) {
      hangClimberRequest();
    } else if (state == RobotState.CLIMBER_HANGING) {
      trapShotRequest();
    } else {
      // Start climb sequence
      waitingClimberRaisedRequest();
    }
  }

  public void stowDownAfterIntakeRequest() {
    stowDownAfterIntakeFlag = true;
  }

  public void stowUpAfterIntakeRequest() {
    stowUpAfterIntakeFlag = true;
  }

  public Command waitForStateCommand(RobotState goalState) {
    return Commands.waitUntil(() -> this.state == goalState);
  }

  public RobotState getState() {
    return state;
  }
}
