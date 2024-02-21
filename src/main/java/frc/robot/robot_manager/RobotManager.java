// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.util.FlagManager;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.DistanceAngle;
import frc.robot.vision.VisionSubsystem;
import frc.robot.wrist.WristPositions;
import frc.robot.wrist.WristSubsystem;
import org.littletonrobotics.junction.Logger;

public class RobotManager extends LifecycleSubsystem {
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

  // TODO: Climber homing should be separate from robot manager homing. The robot manager should not
  // really care whether we're homed.
  private RobotState state = RobotState.IDLE_NO_GP;

  private final FlagManager<RobotFlag> flags = new FlagManager<>(RobotFlag.class);

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
    super(SubsystemPriority.ROBOT_MANAGER);
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
  public void robotPeriodic() {
    Logger.recordOutput("RobotManager/State", state);
    flags.log();

    DistanceAngle speakerVisionTargets = vision.getDistanceAngleSpeaker();
    DistanceAngle floorSpotVisionTargets = vision.getDistanceAngleFloorShot();
    double speakerDistance = speakerVisionTargets.distance();
    double floorSpotDistance = floorSpotVisionTargets.distance();
    Rotation2d wristAngleForSpeaker = wrist.getAngleFromDistanceToSpeaker(speakerDistance);
    Rotation2d wristAngleForFloorSpot = wrist.getAngleFromDistanceToFloorSpot(floorSpotDistance);
    Rotation2d robotAngleToSpeaker =
        Rotation2d.fromDegrees(
            imu.getRobotHeading().getDegrees() + speakerVisionTargets.angle().getDegrees());
    shooter.setSpeakerDistance(speakerDistance);
    Rotation2d robotAngleToFloorSpot =
        Rotation2d.fromDegrees(
            imu.getRobotHeading().getDegrees() + floorSpotVisionTargets.angle().getDegrees());
    shooter.setFloorSpotDistance(floorSpotDistance);

    // State transitions from requests
    for (RobotFlag flag : flags.getChecked()) {
      switch (flag) {
        case HOMING:
          climber.resetHoming();
          state = RobotState.HOMING;
          break;
        case STOW:
          if (state.homed) {
            if (state.hasNote) {
              state = RobotState.IDLE_WITH_GP;
            } else {
              state = RobotState.IDLE_NO_GP;
            }
          }
          break;
        case INTAKE:
          if (state.homed) {
            state = RobotState.GROUND_INTAKING;
          }
          break;
        case WAITING_CLIMBER_RAISED:
          if (state.homed) {
            state = RobotState.PREPARE_WAITING_CLIMBER_RAISED;
          }
          break;
        case RAISE_CLIMBER:
          if (state == RobotState.WAITING_CLIMBER_RAISED) {
            state = RobotState.PREPARE_CLIMBER_RAISED;
          }
          break;
        case HANG_CLIMBER:
          if (state == RobotState.CLIMBER_RAISED) {
            state = RobotState.PREPARE_CLIMBER_HANGING;
          }
          break;
        case WAIT_SPEAKER_SHOT:
          if (state.homed) {
            state = RobotState.WAITING_SPEAKER_SHOT;
          }
          break;
        case WAIT_SUBWOOFER_SHOT:
          if (state.homed) {
            state = RobotState.WAITING_SUBWOOFER_SHOT;
          }
          break;
        case OUTTAKE:
          if (state.homed) {
            state = RobotState.OUTTAKING;
          }
          break;
        case OUTTAKE_SHOOTER:
          if (state.homed) {
            state = RobotState.OUTTAKING_SHOOTER;
          }
          break;
        case SPEAKER_SHOT:
          if (state.homed) {
            state = RobotState.PREPARE_SPEAKER_SHOT;
          }
          break;
        case WAIT_AMP_SHOT:
          if (state.homed) {
            state = RobotState.PREPARE_WAITING_AMP_SHOT;
          }
          break;
        case AMP_SHOT:
          if (state.homed) {
            state = RobotState.AMP_SHOT;
          }
          break;
        case TRAP_SHOT:
          if (state.homed) {
            state = RobotState.TRAP_OUTTAKE;
          }
          break;
        case SUBWOOFER_SHOT:
          if (state.homed) {
            state = RobotState.PREPARE_SUBWOOFER_SHOT;
          }
          break;
        case PRELOAD_NOTE:
          if (state.homed) {
            state = RobotState.IDLE_WITH_GP;
          }
          break;
        case WAIT_FLOOR_SHOT:
          if (state.homed) {
            state = RobotState.WAITING_FLOOR_SHOT;
          }
          break;
        case FLOOR_SHOT:
          if (state.homed) {
            state = RobotState.PREPARE_FLOOR_SHOT;
          }
          break;
      }
    }

    // Automatic state transitions
    switch (state) {
      case UNHOMED:
      case IDLE_NO_GP:
      case IDLE_WITH_GP:
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
        if (climber.getHomingState() == HomingState.HOMED) {
          state = RobotState.IDLE_NO_GP;
        }
        break;
      case PREPARE_WAITING_AMP_SHOT:
        if (noteManager.getState() == NoteState.IDLE_IN_CONVEYOR
            && wrist.atAngle(WristPositions.STOWED)) {
          state = RobotState.WAITING_AMP_SHOT;
        }
        break;
      case GROUND_INTAKING:
        if (noteManager.getState() == NoteState.IDLE_IN_QUEUER) {
          state = RobotState.IDLE_WITH_GP;
        }
        break;
      case PREPARE_FLOOR_SHOT:
        if (wrist.atAngle(wristAngleForFloorSpot)
            && shooter.atGoal(ShooterMode.FLOOR_SHOT)
            && noteManager.getState() == NoteState.IDLE_IN_QUEUER
            && Math.abs(robotAngleToFloorSpot.getDegrees()) < 2.5
            && swerve.movingSlowEnoughForSpeakerShot()
            && Math.abs(imu.getRobotAngularVelocity().getDegrees()) < 2.5) {
          state = RobotState.FLOOR_SHOOT;
        }
        break;
      case PREPARE_SUBWOOFER_SHOT:
        if (wrist.atAngle(WristPositions.SUBWOOFER_SHOT)
            && shooter.atGoal(ShooterMode.SUBWOOFER_SHOT)
            && noteManager.getState() == NoteState.IDLE_IN_QUEUER) {
          state = RobotState.SUBWOOFER_SHOOT;
        }
        break;
      case PREPARE_AMP_SHOT:
        if (noteManager.getState() == NoteState.IDLE_IN_CONVEYOR
            && wrist.atAngle(WristPositions.STOWED)) {
          state = RobotState.AMP_SHOT;
        }
        break;
      case PREPARE_SPEAKER_SHOT:
        if (wrist.atAngleForSpeaker(wristAngleForSpeaker, speakerDistance)
            && shooter.atGoal(ShooterMode.SPEAKER_SHOT)
            && noteManager.getState() == NoteState.IDLE_IN_QUEUER
            && swerve.movingSlowEnoughForSpeakerShot()
            && imu.belowVelocityForSpeaker(speakerDistance)
            && imu.atAngleForSpeaker(robotAngleToSpeaker, speakerDistance)) {
          state = RobotState.SPEAKER_SHOOT;
        }
        break;
      case OUTTAKING_SHOOTER:
      case FLOOR_SHOOT:
      case SUBWOOFER_SHOOT:
      case SPEAKER_SHOOT:
      case AMP_SHOT:
        if (noteManager.getState() == NoteState.IDLE_NO_GP
            && wrist.atAngle(WristPositions.STOWED)) {
          state = RobotState.IDLE_NO_GP;
        }
        break;
      case PREPARE_TRAP_OUTTAKE:
        if (noteManager.getState() == NoteState.IDLE_IN_CONVEYOR
            && elevator.atPosition(ElevatorPositions.TRAP_SHOT)
            && wrist.atAngle(WristPositions.STOWED)) {
          state = RobotState.TRAP_OUTTAKE;
        }
        break;
      case OUTTAKING:
        if (noteManager.getState() == NoteState.IDLE_NO_GP) {
          state = RobotState.IDLE_NO_GP;
        }
        break;
      case TRAP_OUTTAKE:
        if (noteManager.getState() == NoteState.IDLE_NO_GP
            && elevator.atPosition(ElevatorPositions.TRAP_SHOT)
            && wrist.atAngle(WristPositions.STOWED)) {
          state = RobotState.CLIMBER_HANGING;
        }
        break;
      case PREPARE_WAITING_CLIMBER_RAISED:
        if (wrist.atAngle(WristPositions.STOWED)) {
          state = RobotState.WAITING_CLIMBER_RAISED;
        }
        break;
      case PREPARE_CLIMBER_RAISED:
        if (climber.atGoal(ClimberMode.RAISED)
            && elevator.atPosition(ElevatorPositions.CLIMBING)
            && wrist.atAngle(WristPositions.STOWED)) {
          state = RobotState.CLIMBER_RAISED;
        }
        break;
      case PREPARE_CLIMBER_HANGING:
        if (climber.atGoal(ClimberMode.HANGING)
            && elevator.atPosition(ElevatorPositions.CLIMBING)
            && wrist.atAngle(WristPositions.STOWED)) {
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
        wrist.setAngle(WristPositions.STOWED);
        climber.startHoming();
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        noteManager.idleNoGPRequest();
        break;
      case HOMING:
        wrist.setAngle(WristPositions.STOWED);
        climber.startHoming();
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        noteManager.idleNoGPRequest();
        break;
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
      case GROUND_INTAKING:
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.intakeRequest();
        break;
      case OUTTAKING:
        wrist.setAngle(WristPositions.STOWED);
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
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        noteManager.ampWaitRequest();
        break;
      case WAITING_AMP_SHOT:
      case PREPARE_AMP_SHOT:
        wrist.setAngle(WristPositions.STOWED);
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
      case PREPARE_WAITING_CLIMBER_RAISED:
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.RAISED);
        noteManager.ampWaitRequest();
        break;
      case WAITING_CLIMBER_RAISED:
      case PREPARE_CLIMBER_RAISED:
      case CLIMBER_RAISED:
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.RAISED);
        noteManager.ampWaitRequest();
        break;
      case PREPARE_CLIMBER_HANGING:
      case CLIMBER_HANGING:
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.ampWaitRequest();
        break;
      case PREPARE_TRAP_OUTTAKE:
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.TRAP_SHOT);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.ampWaitRequest();
        break;
      case TRAP_OUTTAKE:
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.TRAP_SHOT);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.ampScoreRequest();
        break;
      default:
        // Should never happen
        break;
    }

    swerve.setShootingMode(state.shootingMode);

    // Reset all flags
    flags.clear();
  }

  public void homingRequest() {
    flags.check(RobotFlag.HOMING);
  }

  public void waitSubwooferShotRequest() {
    flags.check(RobotFlag.WAIT_SUBWOOFER_SHOT);
  }

  public void subwooferShotRequest() {
    flags.check(RobotFlag.SUBWOOFER_SHOT);
  }

  public void waitSpeakerShotRequest() {
    flags.check(RobotFlag.WAIT_SPEAKER_SHOT);
  }

  public void speakerShotRequest() {
    flags.check(RobotFlag.SPEAKER_SHOT);
  }

  public void waitAmpShotRequest() {
    flags.check(RobotFlag.WAIT_AMP_SHOT);
  }

  public void ampShotRequest() {
    flags.check(RobotFlag.AMP_SHOT);
  }

  public void waitFloorShotRequest() {
    flags.check(RobotFlag.WAIT_FLOOR_SHOT);
  }

  public void floorShotRequest() {
    flags.check(RobotFlag.FLOOR_SHOT);
  }

  public void trapShotRequest() {
    flags.check(RobotFlag.TRAP_SHOT);
  }

  public void intakeRequest() {
    flags.check(RobotFlag.INTAKE);
  }

  public void outtakeRequest() {
    flags.check(RobotFlag.OUTTAKE);
  }

  public void outtakeShooterRequest() {
    flags.check(RobotFlag.OUTTAKE_SHOOTER);
  }

  public void stowRequest() {
    flags.check(RobotFlag.STOW);
  }

  public void waitingRaiseClimberRequest() {
    flags.check(RobotFlag.WAITING_CLIMBER_RAISED);
  }

  public void raiseClimberRequest() {
    flags.check(RobotFlag.RAISE_CLIMBER);
  }

  public void hangClimberRequest() {
    flags.check(RobotFlag.HANG_CLIMBER);
  }

  public void preloadNoteRequest() {
    flags.check(RobotFlag.PRELOAD_NOTE);
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

  public void getClimberForwardRequest() {
    switch (state) {
      case WAITING_CLIMBER_RAISED:
        raiseClimberRequest();
        break;
      case CLIMBER_RAISED:
        hangClimberRequest();
        break;
      case CLIMBER_HANGING:
        trapShotRequest();
        break;
      default:
        // Start climb sequence
        waitingRaiseClimberRequest();
        break;
    }
  }

  public void getClimberBackwardRequest() {
    switch (state) {
      case WAITING_CLIMBER_RAISED:
        stowRequest();
        break;
      case CLIMBER_RAISED:
        waitingRaiseClimberRequest();
        break;
      case CLIMBER_HANGING:
        raiseClimberRequest();
        break;
      default:
        // Do nothing if climb sequence isn't already started
        break;
    }
  }

  public Command waitForStateCommand(RobotState goalState) {
    return Commands.waitUntil(() -> this.state == goalState);
  }

  public RobotState getState() {
    return state;
  }
}
