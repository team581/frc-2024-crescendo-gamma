// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.climber.ClimberMode;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.elevator.ElevatorPositions;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.note_manager.NoteManager;
import frc.robot.note_manager.NoteState;
import frc.robot.shooter.ShooterMode;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.snaps.SnapManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;
import frc.robot.wrist.WristPositions;
import frc.robot.wrist.WristSubsystem;

public class RobotManager extends StateMachine<RobotState, RobotFlag> {
  public final WristSubsystem wrist;
  public final ElevatorSubsystem elevator;
  public final ShooterSubsystem shooter;
  public final LocalizationSubsystem localization;
  public final VisionSubsystem vision;
  public final ClimberSubsystem climber;
  public final SwerveSubsystem swerve;
  public final SnapManager snaps;
  private final ImuSubsystem imu;
  public final NoteManager noteManager;

  private double speakerDistance = 0;
  private double floorSpotDistance = 0;
  private Rotation2d robotAngleToSpeaker = new Rotation2d();
  private Rotation2d wristAngleForSpeaker = new Rotation2d();
  private Rotation2d robotAngleToFloorSpot = new Rotation2d();
  private Rotation2d wristAngleForFloorSpot = new Rotation2d();

  public RobotManager(
      WristSubsystem wrist,
      ElevatorSubsystem elevator,
      ShooterSubsystem shooter,
      LocalizationSubsystem localization,
      VisionSubsystem vision,
      ClimberSubsystem climber,
      SwerveSubsystem swerve,
      SnapManager snaps,
      ImuSubsystem imu,
      NoteManager noteManager) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotFlag.class, RobotState.IDLE_NO_GP);
    this.wrist = wrist;
    this.elevator = elevator;
    this.shooter = shooter;
    this.localization = localization;
    this.vision = vision;
    this.climber = climber;
    this.swerve = swerve;
    this.snaps = snaps;
    this.imu = imu;
    this.noteManager = noteManager;
  }

  @Override
  protected void collectInputs() {
    var speakerVisionTargets = vision.getDistanceAngleSpeaker();
    var floorSpotVisionTargets = vision.getDistanceAngleFloorShot();
    speakerDistance = speakerVisionTargets.distance();
    floorSpotDistance = floorSpotVisionTargets.distance();
    wristAngleForSpeaker = wrist.getAngleFromDistanceToSpeaker(speakerDistance);
    wristAngleForFloorSpot = wrist.getAngleFromDistanceToFloorSpot(floorSpotDistance);
    robotAngleToSpeaker =
        Rotation2d.fromDegrees(
            imu.getRobotHeading().getDegrees() + speakerVisionTargets.angle().getDegrees());
    robotAngleToFloorSpot =
        Rotation2d.fromDegrees(
            imu.getRobotHeading().getDegrees() + floorSpotVisionTargets.angle().getDegrees());
  }

  @Override
  protected RobotState processFlag(RobotState state, RobotFlag flag) {
    switch (flag) {
      case STOW:
        if (state.hasNote) {
          if (state == RobotState.WAITING_AMP_SHOT) {
            return RobotState.PREPARE_IDLE_WITH_GP_FROM_CONVEYOR;
          }

          return RobotState.IDLE_WITH_GP;
        }

        return RobotState.IDLE_NO_GP;
      case INTAKE:
        return RobotState.INTAKING;
      case INTAKE_SLOW:
        if (state != RobotState.IDLE_WITH_GP) {
          return RobotState.INTAKING_SLOW;
        }

        return state;
      case CLIMB_1_LINEUP_OUTER:
        return RobotState.CLIMB_1_LINEUP_OUTER;
      case CLIMB_2_LINEUP_INNER:
        if (state == RobotState.CLIMB_1_LINEUP_OUTER || state == RobotState.CLIMB_3_LINEUP_FINAL) {
          return RobotState.CLIMB_2_LINEUP_INNER;
        }

        return state;
      case CLIMB_3_LINEUP_FINAL:
        if (state == RobotState.CLIMB_2_LINEUP_INNER || state == RobotState.CLIMB_4_HANGING) {
          return RobotState.CLIMB_3_LINEUP_FINAL;
        }

        return state;
      case CLIMB_4_HANGING:
        if (state == RobotState.CLIMB_3_LINEUP_FINAL
            || state == RobotState.CLIMB_5_HANGING_TRAP_SCORE) {
          return RobotState.PREPARE_CLIMB_4_HANGING;
        }

        return state;
      case CLIMB_5_HANGING_TRAP_SCORE:
        if (state == RobotState.CLIMB_4_HANGING
            || state == RobotState.CLIMB_6_HANGING_ELEVATOR_SHAKE) {
          return RobotState.CLIMB_5_HANGING_TRAP_SCORE;
        }

        return state;
      case CLIMB_6_HANGING_ELEVATOR_SHAKE:
        if (state == RobotState.CLIMB_5_HANGING_TRAP_SCORE) {
          return RobotState.CLIMB_6_HANGING_ELEVATOR_SHAKE;
        }
        return state;
      case WAIT_SPEAKER_SHOT:
        return RobotState.WAITING_SPEAKER_SHOT;
      case WAIT_SUBWOOFER_SHOT:
        return RobotState.WAITING_SUBWOOFER_SHOT;
      case OUTTAKE:
        return RobotState.OUTTAKING;
      case OUTTAKE_SHOOTER:
        return RobotState.OUTTAKING_SHOOTER;
      case SPEAKER_SHOT:
        return RobotState.PREPARE_SPEAKER_SHOT;
      case WAIT_AMP_SHOT:
        return RobotState.PREPARE_WAITING_AMP_SHOT;
      case AMP_SHOT:
        return RobotState.AMP_SHOT;
      case SUBWOOFER_SHOT:
        return RobotState.PREPARE_SUBWOOFER_SHOT;
      case PRELOAD_NOTE:
        return RobotState.IDLE_WITH_GP;
      case WAIT_FLOOR_SHOT:
        return RobotState.WAITING_FLOOR_SHOT;
      case FLOOR_SHOT:
        return RobotState.PREPARE_FLOOR_SHOT;
      default:
        // Should never happen
        return state;
    }
  }

  @Override
  protected void afterFlagTransition(RobotState state) {
    if (state == RobotState.INTAKING) {
      // Reset note manager state so that we don't instantly think we're done intaking
      // Need to force set the state, rather than doing a state request, due to order of
      // subsystems executing
      noteManager.evilStateOverride(NoteState.IDLE_NO_GP);
    }
  }

  @Override
  protected RobotState processTransitions(RobotState state) {
    switch (state) {
      case IDLE_NO_GP:
      case IDLE_WITH_GP:
      case WAITING_SPEAKER_SHOT:
      case WAITING_AMP_SHOT:
      case WAITING_FLOOR_SHOT:
      case WAITING_SUBWOOFER_SHOT:
      case OUTTAKING:
        // Do nothing
        return state;
      case PREPARE_IDLE_WITH_GP_FROM_CONVEYOR:
        if (elevator.atPosition(ElevatorPositions.STOWED)) {
          return RobotState.IDLE_WITH_GP;
        }
        return state;
      case PREPARE_WAITING_AMP_SHOT:
        if (noteManager.getState() == NoteState.IDLE_IN_CONVEYOR
            && wrist.atAngle(WristPositions.STOWED)) {
          return RobotState.WAITING_AMP_SHOT;
        }
        return state;
      case INTAKING:
      case INTAKING_SLOW:
        if (noteManager.getState() == NoteState.IDLE_IN_QUEUER) {
          return RobotState.IDLE_WITH_GP;
        }
        return state;
      case PREPARE_FLOOR_SHOT:
        if (wrist.atAngle(wristAngleForFloorSpot)
            && shooter.atGoal(ShooterMode.FLOOR_SHOT)
            && noteManager.getState() == NoteState.IDLE_IN_QUEUER
            && Math.abs(robotAngleToFloorSpot.getDegrees()) < 2.5
            && swerve.movingSlowEnoughForSpeakerShot()
            && Math.abs(imu.getRobotAngularVelocity().getDegrees()) < 2.5) {
          return RobotState.FLOOR_SHOOT;
        }
        return state;
      case PREPARE_SUBWOOFER_SHOT:
        if (wrist.atAngle(WristPositions.SUBWOOFER_SHOT)
            && shooter.atGoal(ShooterMode.SUBWOOFER_SHOT)
            && noteManager.getState() == NoteState.IDLE_IN_QUEUER) {
          return RobotState.SUBWOOFER_SHOOT;
        }
        return state;
      case PREPARE_AMP_SHOT:
        if (noteManager.getState() == NoteState.IDLE_IN_CONVEYOR
            && wrist.atAngle(WristPositions.STOWED)) {
          return RobotState.AMP_SHOT;
        }
        return state;
      case PREPARE_SPEAKER_SHOT:
        if (wrist.atAngleForSpeaker(wristAngleForSpeaker, speakerDistance)
            && shooter.atGoal(ShooterMode.SPEAKER_SHOT)
            && noteManager.getState() == NoteState.IDLE_IN_QUEUER
            && swerve.movingSlowEnoughForSpeakerShot()
            && imu.belowVelocityForSpeaker(speakerDistance)
            && imu.atAngleForSpeaker(robotAngleToSpeaker, speakerDistance)) {
          return RobotState.SPEAKER_SHOOT;
        }
        return state;
      case OUTTAKING_SHOOTER:
      case FLOOR_SHOOT:
      case SUBWOOFER_SHOOT:
      case SPEAKER_SHOOT:
      case AMP_SHOT:
        if (noteManager.getState() == NoteState.IDLE_NO_GP) {
          return RobotState.IDLE_NO_GP;
        }
        return state;
      case CLIMB_1_LINEUP_OUTER:
      case CLIMB_2_LINEUP_INNER:
      case CLIMB_3_LINEUP_FINAL:
      case CLIMB_4_HANGING:
      case CLIMB_6_HANGING_ELEVATOR_SHAKE:
        return state;
      case PREPARE_CLIMB_4_HANGING:
        if (climber.atGoal(ClimberMode.HANGING)
            && elevator.atPosition(ElevatorPositions.CLIMBING)) {
          return RobotState.CLIMB_4_HANGING;
        }
        return state;
      case PREPARE_CLIMB_5_HANGING_TRAP_SCORE:
        if (noteManager.getState() == NoteState.IDLE_IN_CONVEYOR
            && elevator.atPosition(ElevatorPositions.TRAP_SHOT)) {
          return RobotState.CLIMB_5_HANGING_TRAP_SCORE;
        }
        return state;
      case CLIMB_5_HANGING_TRAP_SCORE:
        if (noteManager.getState() == NoteState.IDLE_NO_GP) {
          return RobotState.CLIMB_4_HANGING;
        }
        return state;
      default:
        // Should never happen
        return state;
    }
  }

  @Override
  protected void stateActions(RobotState state) {
    shooter.setSpeakerDistance(speakerDistance);
    shooter.setFloorSpotDistance(floorSpotDistance);

    switch (state) {
      case IDLE_NO_GP:
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.idleNoGPRequest();
        break;
      case IDLE_WITH_GP:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.idleInQueuerRequest();
        break;
      case INTAKING:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.intakeRequest();
        break;
      case INTAKING_SLOW:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.intakeSlowRequest();
        break;
      case OUTTAKING:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.outtakeRequest();
        break;
      case OUTTAKING_SHOOTER:
        wrist.setAngle(WristPositions.OUTTAKING_SHOOTER);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.OUTTAKE);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.shooterOuttakeRequest();
        break;
      case WAITING_FLOOR_SHOT:
      case PREPARE_FLOOR_SHOT:
        wrist.setAngle(wristAngleForFloorSpot);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FLOOR_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.idleInQueuerRequest();
        snaps.setAngle(robotAngleToFloorSpot);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case FLOOR_SHOOT:
        wrist.setAngle(wristAngleForFloorSpot);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FLOOR_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.shooterScoreRequest();
        snaps.setAngle(robotAngleToFloorSpot);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case WAITING_SUBWOOFER_SHOT:
      case PREPARE_SUBWOOFER_SHOT:
        wrist.setAngle(WristPositions.SUBWOOFER_SHOT);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.idleInQueuerRequest();
        break;
      case SUBWOOFER_SHOOT:
        wrist.setAngle(WristPositions.SUBWOOFER_SHOT);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.shooterScoreRequest();
        break;
      case WAITING_SPEAKER_SHOT:
      case PREPARE_SPEAKER_SHOT:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.idleInQueuerRequest();
        snaps.setAngle(robotAngleToSpeaker);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case SPEAKER_SHOOT:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.shooterScoreRequest();
        snaps.setAngle(robotAngleToSpeaker);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case PREPARE_WAITING_AMP_SHOT:
      case PREPARE_IDLE_WITH_GP_FROM_CONVEYOR:
        wrist.setAngle(WristPositions.CLIMBING);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.ampWaitRequest();
        break;
      case WAITING_AMP_SHOT:
      case PREPARE_AMP_SHOT:
        wrist.setAngle(WristPositions.CLIMBING);
        elevator.setGoalHeight(ElevatorPositions.AMP_OUTTAKE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.ampWaitRequest();
        break;
      case AMP_SHOT:
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.AMP_OUTTAKE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.ampScoreRequest();
        break;
      case CLIMB_1_LINEUP_OUTER:
        wrist.setAngle(WristPositions.CLIMBING);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.LINEUP_OUTER);
        noteManager.trapWaitRequest();
        break;
      case CLIMB_2_LINEUP_INNER:
        wrist.setAngle(WristPositions.CLIMBING);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.LINEUP_INNER);
        noteManager.trapWaitRequest();
        break;
      case CLIMB_3_LINEUP_FINAL:
        wrist.setAngle(WristPositions.CLIMBING);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.LINEUP_INNER);
        noteManager.trapWaitRequest();
        break;
      case PREPARE_CLIMB_4_HANGING:
      case CLIMB_4_HANGING:
        wrist.setAngle(WristPositions.CLIMBING);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.trapWaitRequest();
        break;
      case PREPARE_CLIMB_5_HANGING_TRAP_SCORE:
        wrist.setAngle(WristPositions.CLIMBING);
        elevator.setGoalHeight(ElevatorPositions.TRAP_SHOT);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.trapWaitRequest();
        break;
      case CLIMB_5_HANGING_TRAP_SCORE:
        wrist.setAngle(WristPositions.CLIMBING);
        elevator.setGoalHeight(ElevatorPositions.TRAP_SHOT);
        elevator.setPulsing(false);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.trapShotRequest();
        break;
      case CLIMB_6_HANGING_ELEVATOR_SHAKE:
        wrist.setAngle(WristPositions.CLIMBING);
        elevator.setGoalHeight(ElevatorPositions.TRAP_SHOT);
        elevator.setPulsing(true);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.trapShotRequest();
        break;
      default:
        // Should never happen
        break;
    }

    swerve.setShootingMode(state.shootingMode);
  }

  public void waitSubwooferShotRequest() {
    checkFlag(RobotFlag.WAIT_SUBWOOFER_SHOT);
  }

  public void subwooferShotRequest() {
    checkFlag(RobotFlag.SUBWOOFER_SHOT);
  }

  public void waitSpeakerShotRequest() {
    checkFlag(RobotFlag.WAIT_SPEAKER_SHOT);
  }

  public void speakerShotRequest() {
    checkFlag(RobotFlag.SPEAKER_SHOT);
  }

  public void waitAmpShotRequest() {
    checkFlag(RobotFlag.WAIT_AMP_SHOT);
  }

  public void ampShotRequest() {
    checkFlag(RobotFlag.AMP_SHOT);
  }

  public void waitFloorShotRequest() {
    checkFlag(RobotFlag.WAIT_FLOOR_SHOT);
  }

  public void floorShotRequest() {
    checkFlag(RobotFlag.FLOOR_SHOT);
  }

  public void intakeRequest() {
    checkFlag(RobotFlag.INTAKE);
  }

  public void intakeSlowRequest() {
    checkFlag(RobotFlag.INTAKE_SLOW);
  }

  public void outtakeRequest() {
    checkFlag(RobotFlag.OUTTAKE);
  }

  public void outtakeShooterRequest() {
    checkFlag(RobotFlag.OUTTAKE_SHOOTER);
  }

  public void stowRequest() {
    checkFlag(RobotFlag.STOW);
  }

  public void preloadNoteRequest() {
    checkFlag(RobotFlag.PRELOAD_NOTE);
  }

  public void confirmShotRequest() {
    var state = getState();

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

  public void climb1LineupOutterRequest() {
    checkFlag(RobotFlag.CLIMB_1_LINEUP_OUTER);
  }

  public void climb2LineupInnerRequest() {
    checkFlag(RobotFlag.CLIMB_2_LINEUP_INNER);
  }

  public void climb3LineupFinalRequest() {
    checkFlag(RobotFlag.CLIMB_3_LINEUP_FINAL);
  }

  public void climb4HangingRequest() {
    checkFlag(RobotFlag.CLIMB_4_HANGING);
  }

  public void climb5HangingTrapScoreRequest() {
    checkFlag(RobotFlag.CLIMB_5_HANGING_TRAP_SCORE);
  }

  public void climb6HangingElevatorShakeRequest() {
    checkFlag(RobotFlag.CLIMB_6_HANGING_ELEVATOR_SHAKE);
  }

  public void getClimberForwardRequest() {
    var state = getState();

    switch (state) {
      case CLIMB_1_LINEUP_OUTER:
        climb2LineupInnerRequest();
        break;
      case CLIMB_2_LINEUP_INNER:
        climb3LineupFinalRequest();
        break;
      case CLIMB_3_LINEUP_FINAL:
        climb4HangingRequest();
        break;
      case PREPARE_CLIMB_4_HANGING:
      case CLIMB_4_HANGING:
        climb5HangingTrapScoreRequest();
        break;
      case CLIMB_5_HANGING_TRAP_SCORE:
        climb6HangingElevatorShakeRequest();
        break;
      default:
        // Start climb sequence
        climb1LineupOutterRequest();
        break;
    }
  }

  public void getClimberBackwardRequest() {
    var state = getState();

    switch (state) {
      case CLIMB_1_LINEUP_OUTER:
        stowRequest();
        break;
      case CLIMB_2_LINEUP_INNER:
        climb1LineupOutterRequest();
        break;
      case CLIMB_3_LINEUP_FINAL:
        climb2LineupInnerRequest();
        break;
      case CLIMB_4_HANGING:
        climb3LineupFinalRequest();
        break;
      case CLIMB_5_HANGING_TRAP_SCORE:
        climb4HangingRequest();
        break;
      case CLIMB_6_HANGING_ELEVATOR_SHAKE:
        climb5HangingTrapScoreRequest();
        break;
      default:
        // Do nothing if climb sequence isn't started
        break;
    }
  }
}
