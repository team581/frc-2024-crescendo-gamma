// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.climber.ClimberMode;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.conveyor.ConveyorState;
import frc.robot.elevator.ElevatorPositions;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.note_manager.NoteManager;
import frc.robot.queuer.QueuerState;
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
  public final NoteManager noteManager;
  public final ElevatorSubsystem elevator;
  public final ShooterSubsystem shooter;
  public final LocalizationSubsystem localization;
  public final VisionSubsystem vision;
  public final ClimberSubsystem climber;
  public final SwerveSubsystem swerve;
  public final SnapManager snaps;
  private final ImuSubsystem imu;

  private RobotState state = RobotState.UNHOMED;

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
    DistanceAngle floorSpotVisionTargets = vision.getDistanceAngleFloorSpot();
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
          wrist.resetHoming();
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
          intake.setState(IntakeState.IDLE);
          if (state.homed) {
            state = RobotState.GROUND_INTAKING;
          }
          break;
        case WAITING_CLIMBER_RAISED:
          if (state.homed) {
            state = RobotState.WAITING_CLIMBER_RAISED;
          }
        case RAISE_CLIMBER:
          if (state == RobotState.WAITING_CLIMBER_RAISED) {
            state = RobotState.PREPARE_CLIMBER_RAISED;
          }
        case HANG_CLIMBER:
          if (state == RobotState.CLIMBER_RAISED) {
            state = RobotState.PREPARE_CLIMBER_HANGING;
          }
        case WAIT_SPEAKER_SHOT:
          if (state.homed) {
            state = RobotState.WAITING_SPEAKER_SHOT;
          }
        case WAIT_SUBWOOFER_SHOT:
          if (state.homed) {
            state = RobotState.WAITING_SUBWOOFER_SHOT;
          }
        case OUTTAKE:
          if (state.homed && state.noteLocation != RobotState.OUTTAKING.noteLocation) {
            state = RobotState.OUTTAKING;
          }
        case OUTTAKE_SHOOTER:
          if (state.homed) {
            state = RobotState.OUTTAKING_SHOOTER;
          }
        case SPEAKER_SHOT:
          if (state.homed) {
            state = RobotState.PREPARE_SPEAKER_SHOT;
          }
        case WAIT_AMP_SHOT:
          if (state.homed) {
            state = RobotState.WAITING_AMP_SHOT;
          }
        case AMP_SHOT:
          if (state.homed) {
            state = RobotState.AMP_SHOT;
          }
        case TRAP_SHOT:
          if (state.homed) {
            state = RobotState.TRAP_OUTTAKE;
          }
        case SUBWOOFER_SHOT:
          if (state.homed) {
            state = RobotState.PREPARE_SUBWOOFER_SHOT;
          }
        case PRELOAD_NOTE:
          if (state.homed) {
            state = RobotState.IDLE_WITH_GP;
          }
        case WAIT_FLOOR_SHOT:
          if (state.homed) {
            state = RobotState.WAITING_FLOOR_SHOT;
          }
        case FLOOR_SHOT:
          if (state.homed) {
            state = RobotState.PREPARE_FLOOR_SHOT;
          }
      }
    }

    // Automatic state transitions
    switch (state) {
      case UNHOMED:
        if (wrist.getHomingState() == HomingState.HOMED
            && climber.getHomingState() == HomingState.HOMED
            && elevator.getHomingState() == HomingState.HOMED) {
          state = RobotState.IDLE_NO_GP;
        }
        break;
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
        if (wrist.getHomingState() == HomingState.HOMED
            && elevator.getHomingState() == HomingState.HOMED
            && climber.getHomingState() == HomingState.HOMED) {
          state = RobotState.IDLE_NO_GP;
        }
        break;
      case GROUND_INTAKING:
        if (intake.atGoal(IntakeState.INTAKING) && queuer.hasNote()) {
          state = RobotState.IDLE_WITH_GP;
        }
        break;
      case PREPARE_FLOOR_SHOT:
        if (wrist.atAngle(wristAngleForFloorSpot)
            && shooter.atGoal(ShooterMode.FLOOR_SHOT)
            && (Math.abs(floorSpotVisionTargets.angle().getDegrees()) < 2.5)
            && swerve.movingSlowEnoughForSpeakerShot()
            && imu.getRobotAngularVelocity().getDegrees() < 2.5) {
          state = RobotState.FLOOR_SHOOT;
        }
        break;
      case PREPARE_SUBWOOFER_SHOT:
        if (wrist.atAngle(WristPositions.SUBWOOFER_SHOT)
            && shooter.atGoal(ShooterMode.SUBWOOFER_SHOT)) {
          state = RobotState.SUBWOOFER_SHOOT;
        }
        break;
      case AMP_SHOT:
        if (conveyor.atGoal(ConveyorState.AMP_SHOT)) {
          state = RobotState.IDLE_NO_GP;
        }
        break;
      case PREPARE_SPEAKER_SHOT:
        double distance = speakerDistance;
        imu.setTolerance(imu.getAngleToleranceFromDistanceToSpeaker(distance));
        if (wrist.atAngle(wristAngleForSpeaker, distance)
            && shooter.atGoal(ShooterMode.SPEAKER_SHOT)
            && queuer.atGoal(QueuerState.IDLE)
            && (Math.abs(speakerVisionTargets.angle().getDegrees())
                < imu.getTolerance().getDegrees())
            && swerve.movingSlowEnoughForSpeakerShot()
            && imu.getRobotAngularVelocity().getDegrees() < imu.getTolerance().getDegrees()
            && imu.atAngle(robotAngleToSpeaker, distance)) {
          state = RobotState.SPEAKER_SHOOT;
        }
        break;
      case OUTTAKING_SHOOTER:
      case FLOOR_SHOOT:
      case SUBWOOFER_SHOOT:
      case SPEAKER_SHOOT:
        if (queuer.atGoal(QueuerState.PASS_TO_SHOOTER) && !state.hasNote) {
          state = RobotState.IDLE_NO_GP;
        }
        break;
      case PREPARE_TRAP_OUTTAKE:
        if (((intake.atGoal(IntakeState.PASS_NOTE_OUTTAKE)
                    && queuer.atGoal(QueuerState.PASS_TO_INTAKE)
                    && conveyor.atGoal(ConveyorState.INTAKE_TO_SELF))
                || conveyor.atGoal(ConveyorState.WAITING_AMP_SHOT) && conveyor.hasNote())
            && elevator.atGoal(ElevatorPositions.TRAP_SHOT)) {
          state = RobotState.TRAP_OUTTAKE;
        }
        break;
        // TODO: What happens if the note is not in the intake at start? (ex. in the conveyor)
        // I believe it would instantly think it's done
        // We need a PREPARE_OUTTAKING_INTAKE state, which passes the note to the intake/queuer/idk,
        // so that the end condition here is correct
      case OUTTAKING:
        if (intake.atGoal(IntakeState.OUTTAKING)) {
          state = RobotState.IDLE_NO_GP;
        }
        break;
      case TRAP_OUTTAKE:
        if (conveyor.atGoal(ConveyorState.AMP_SHOT)
            && elevator.atGoal(ElevatorPositions.TRAP_SHOT)) {
          state = RobotState.CLIMBER_HANGING;
        }
        break;
      case PREPARE_CLIMBER_RAISED:
        if (climber.atGoal(ClimberMode.RAISED) && elevator.atGoal(ElevatorPositions.CLIMBING)) {
          state = RobotState.CLIMBER_RAISED;
        }
        break;
      case PREPARE_CLIMBER_HANGING:
        if (climber.atGoal(ClimberMode.HANGING) && elevator.atGoal(ElevatorPositions.CLIMBING)) {
          state = RobotState.CLIMBER_HANGING;
        }
        break;
      case CONVEYOR_TO_INTAKE_FOR_QUEUER:
        if (conveyor.atGoal(ConveyorState.PASS_TO_INTAKE)
            && intake.atGoal(IntakeState.PASS_NOTE_OUTTAKE)) {
          state = RobotState.INTAKE_TO_QUEUER;
        }
      case INTAKE_TO_QUEUER:
        if (conveyor.atGoal(ConveyorState.INTAKE_TO_QUEUER)
            && intake.atGoal(IntakeState.INTAKING)
            && queuer.atGoal(QueuerState.IDLE)) {
          state = RobotState.IDLE_WITH_GP;
        }
      case QUEUER_TO_INTAKE_FOR_CONVEYOR:
        if (queuer.atGoal(QueuerState.PASS_TO_INTAKE)
            && conveyor.atGoal(ConveyorState.INTAKE_TO_SELF)
            && intake.atGoal(IntakeState.PASS_NOTE_OUTTAKE)) {
          state = RobotState.INTAKE_TO_CONVEYOR;
        }
      case INTAKE_TO_CONVEYOR:
        if (conveyor.atGoal(ConveyorState.INTAKE_TO_SELF) && intake.atGoal(IntakeState.INTAKING)) {
          state = RobotState.WAITING_AMP_SHOT;
        }
      default:
        // Should never happen
        break;
    }

    // State actions
    switch (state) {
      case UNHOMED:
        wrist.startPreMatchHoming();
        climber.startHoming();
        elevator.startPreMatchHoming();
        conveyor.setState(ConveyorState.IDLE);
        queuer.setState(QueuerState.IDLE);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        break;
      case HOMING:
        wrist.startMidMatchHoming();
        climber.startHoming();
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        conveyor.setState(ConveyorState.IDLE);
        queuer.setState(QueuerState.IDLE);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        break;
      case IDLE_NO_GP:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case IDLE_WITH_GP:
        wrist.setAngle(wristAngleForSpeaker);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case GROUND_INTAKING:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.PASS_TO_INTAKE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.INTAKING);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case OUTTAKING:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.PASS_TO_INTAKE);
        conveyor.setState(ConveyorState.INTAKE_TO_SELF);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.OUTTAKING);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case OUTTAKING_SHOOTER:
        wrist.setAngle(WristPositions.OUTTAKING_SHOOTER);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        conveyor.setState(ConveyorState.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.OUTTAKE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case CONVEYOR_TO_INTAKE_FOR_QUEUER:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.PASS_TO_INTAKE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.PASS_NOTE_OUTTAKE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case INTAKE_TO_CONVEYOR:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.INTAKE_TO_SELF);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.INTAKING);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case QUEUER_TO_INTAKE_FOR_CONVEYOR:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.PASS_TO_INTAKE);
        conveyor.setState(ConveyorState.INTAKE_TO_SELF);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.PASS_NOTE_OUTTAKE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case INTAKE_TO_QUEUER:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.INTAKE_TO_SELF);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.INTAKING);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case WAITING_FLOOR_SHOT:
      case PREPARE_FLOOR_SHOT:
        wrist.setAngle(wristAngleForFloorSpot);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.FLOOR_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        snaps.setAngle(robotAngleToFloorSpot);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case FLOOR_SHOOT:
        wrist.setAngle(wristAngleForFloorSpot);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        conveyor.setState(ConveyorState.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.FLOOR_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        snaps.setAngle(robotAngleToFloorSpot);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case WAITING_SUBWOOFER_SHOT:
      case PREPARE_SUBWOOFER_SHOT:
        wrist.setAngle(WristPositions.SUBWOOFER_SHOT);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case SUBWOOFER_SHOOT:
        wrist.setAngle(WristPositions.SUBWOOFER_SHOT);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        conveyor.setState(ConveyorState.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case WAITING_SPEAKER_SHOT:
      case PREPARE_SPEAKER_SHOT:
        wrist.setAngle(wristAngleForSpeaker);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        snaps.setAngle(robotAngleToSpeaker);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case SPEAKER_SHOOT:
        wrist.setAngle(wristAngleForSpeaker);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        conveyor.setState(ConveyorState.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        snaps.setAngle(robotAngleToSpeaker);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case WAITING_AMP_SHOT:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.WAITING_AMP_SHOT);
        elevator.setGoalHeight(ElevatorPositions.AMP_OUTTAKE);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case AMP_SHOT:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.AMP_SHOT);
        elevator.setGoalHeight(ElevatorPositions.AMP_OUTTAKE);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case WAITING_CLIMBER_RAISED:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.WAITING_AMP_SHOT);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.RAISED);
        break;
      case PREPARE_CLIMBER_RAISED:
      case CLIMBER_RAISED:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.AMP_SHOT);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.RAISED);
        break;
      case PREPARE_CLIMBER_HANGING:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.IDLE);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.HANGING);
        break;
      case CLIMBER_HANGING:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.IDLE);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.HANGING);
        break;
      case PREPARE_TRAP_OUTTAKE:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.AMP_SHOT);
        elevator.setGoalHeight(ElevatorPositions.TRAP_SHOT);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.HANGING);
      case TRAP_OUTTAKE:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setState(ConveyorState.AMP_SHOT);
        elevator.setGoalHeight(ElevatorPositions.TRAP_SHOT);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.HANGING);
        break;
      default:
        // Should never happen
        break;
    }

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
