// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.climber.ClimberMode;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.conveyor.ConveyorMode;
import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.elevator.ElevatorPositions;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.queuer.QueuerState;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.shooter.ShooterMode;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.snaps.SnapManager;
import frc.robot.swerve.SwerveSubsystem;
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
  public final IntakeSubsystem intake;
  public final ElevatorSubsystem elevator;
  public final QueuerSubsystem queuer;
  public final ConveyorSubsystem conveyor;
  public final ShooterSubsystem shooter;
  public final LocalizationSubsystem localization;
  public final VisionSubsystem vision;
  public final ClimberSubsystem climber;
  public final SwerveSubsystem swerve;
  public final SnapManager snaps;
  private final ImuSubsystem imu;

  private RobotState state = RobotState.UNHOMED;

  // State request flags
  private boolean homingFlag = false;
  private boolean waitSpeakerShotFlag = false;
  private boolean speakerShotFlag = false;
  private boolean ampShotFlag = false;
  private boolean passNoteToConveyorFlag = false;
  private boolean passNoteToQueuerFlag = false;
  private boolean trapShotFlag = false;
  private boolean waitSubwooferShotFlag = false;
  private boolean subwooferShotFlag = false;
  private boolean groundIntakeFlag = false;
  private boolean outtakeShooterFlag = false;
  private boolean outtakeIntakeFlag = false;
  private boolean stowFlag = false;
  private boolean stowAfterIntakeFlag = false;
  private boolean waitingClimberRaisedFlag = false;
  private boolean raiseClimberFlag = false;
  private boolean hangClimberFlag = false;
  private boolean preloadNoteRequest = false;
  private boolean waitFloorShotFlag = false;
  private boolean floorShotFlag = false;

  public RobotManager(
      WristSubsystem wrist,
      IntakeSubsystem intake,
      ElevatorSubsystem elevator,
      QueuerSubsystem queuer,
      ConveyorSubsystem conveyor,
      ShooterSubsystem shooter,
      LocalizationSubsystem localization,
      VisionSubsystem vision,
      ClimberSubsystem climber,
      SwerveSubsystem swerve,
      SnapManager snaps,
      ImuSubsystem imu) {
    super(SubsystemPriority.ROBOT_MANAGER);
    this.wrist = wrist;
    this.intake = intake;
    this.elevator = elevator;
    this.queuer = queuer;
    this.conveyor = conveyor;
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
    Logger.recordOutput("RobotManager/Flags/AmpShotFlag", ampShotFlag);
    Logger.recordOutput("RobotManager/Flags/TrapShotFlag", trapShotFlag);
    Logger.recordOutput("RobotManager/Flags/WaitSubwooferShotFlag", waitSubwooferShotFlag);
    Logger.recordOutput("RobotManager/Flags/SubwooferShotFlag", subwooferShotFlag);
    Logger.recordOutput("RobotManager/Flags/StowAfterIntakeFlag", stowAfterIntakeFlag);
    Logger.recordOutput("RobotManager/Flags/IntakeFlag", groundIntakeFlag);
    Logger.recordOutput("RobotManager/Flags/OuttakeIntakeFlag", outtakeIntakeFlag);
    Logger.recordOutput("RobotManager/Flags/OuttakeShooterFlag", outtakeShooterFlag);
    Logger.recordOutput("RobotManager/Flags/StowFlag", stowFlag);
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
    Rotation2d wristAngleForSpeaker = wrist.getAngleFromDistanceToSpeaker(speakerDistance);
    Rotation2d wristAngleForFloorSpot = wrist.getAngleFromDistanceToFloorSpot(floorSpotDistance);
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
      wrist.resetHoming();
      state = RobotState.HOMING;
    }
    if (groundIntakeFlag) {
      intake.setState(IntakeState.IDLE);
      if (state.homed) {
        state = RobotState.GROUND_INTAKING;
      }
    }
    if (stowFlag) {
      if (state.homed) {
        if (state.hasNote) {
          state = RobotState.IDLE_WITH_GP;
        } else {
          state = RobotState.IDLE_NO_GP;
        }
      }
    }
    if (stowAfterIntakeFlag) {
      if (state.homed && state != RobotState.GROUND_INTAKING) {
        if (state.hasNote) {
          state = RobotState.IDLE_WITH_GP;
        } else {
          state = RobotState.IDLE_NO_GP;
        }
      }
    }
    if (waitingClimberRaisedFlag) {
      if (state.homed) {
        state = RobotState.WAITING_CLIMBER_RAISED;
      }
    }
    if (raiseClimberFlag) {
      if (state.homed) {
        state = RobotState.PREPARE_CLIMBER_RAISED;
      }
    }
    if (hangClimberFlag) {
      if (state == RobotState.CLIMBER_RAISED) {
        state = RobotState.PREPARE_CLIMBER_HANGING;
      }
    }
    if (waitSpeakerShotFlag) {
      if (state.homed) {
        state = RobotState.WAITING_SPEAKER_SHOT;
      }
    }
    if (waitSubwooferShotFlag) {
      if (state.homed) {
        state = RobotState.WAITING_SUBWOOFER_SHOT;
      }
    }
    if (outtakeIntakeFlag) {
      if (state.homed) {
        state = RobotState.OUTTAKING_INTAKE;
      }
    }
    if (outtakeShooterFlag) {
      if (state.homed) {
        state = RobotState.OUTTAKING_SHOOTER;
      }
    }
    if (speakerShotFlag) {
      if (state.homed) {
        state = RobotState.PREPARE_SPEAKER_SHOT;
      }
    }
    if (ampShotFlag) {
      if (state.homed) {
        state = RobotState.AMP_SHOT;
      }
    }
    if (passNoteToQueuerFlag) {
      if (state.homed) {
        state = RobotState.CONVEYOR_TO_INTAKE_FOR_SHOOTER;
      }
    }
    if (passNoteToConveyorFlag) {
      if (state.homed) {
        state = RobotState.QUEUER_TO_INTAKE_FOR_AMP;
      }
    }
    if (trapShotFlag) {
      if (state.homed) {
        state = RobotState.TRAP_OUTTAKE;
      }
    }
    if (subwooferShotFlag) {
      if (state.homed) {
        state = RobotState.PREPARE_SUBWOOFER_SHOT;
      }
    }
    if (preloadNoteRequest) {
      if (state.homed && !state.hasNote) {
        state = RobotState.IDLE_WITH_GP;
      }
    }
    if (waitFloorShotFlag) {
      if (state.homed) {
        state = RobotState.WAITING_FLOOR_SHOT;
      }
    }
    if (floorShotFlag) {
      if (state.homed) {
        state = RobotState.PREPARE_FLOOR_SHOT;
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
        if (intake.atGoal(IntakeState.INTAKING) && queuer.getHasNote()) {
          state = RobotState.IDLE_WITH_GP;
        }
        break;
      case PREPARE_FLOOR_SHOT:
        if (wrist.atAngle(wristAngleForFloorSpot)
            && shooter.atGoal(ShooterMode.FLOOR_SHOT)
            && (Math.abs(floorShotVisionTargets.angle().getDegrees()) < 2.5)
            && swerve.movingSlowEnoughForSpeakerShot()
            && imu.getRobotAngularVelocity().getDegrees() < 2.5
            && queuer.atGoal(QueuerState.IDLE)) {
          state = RobotState.FLOOR_SHOOT;
        }
        break;
      case PREPARE_SUBWOOFER_SHOT:
        if (wrist.atAngle(WristPositions.SUBWOOFER_SHOT)
            && shooter.atGoal(ShooterMode.SUBWOOFER_SHOT)
            && queuer.atGoal(QueuerState.IDLE)) {
          state = RobotState.SUBWOOFER_SHOOT;
        }
        break;
      case AMP_SHOT:
        if (conveyor.atGoal(ConveyorMode.AMP_SHOT)) {
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
                    && conveyor.atGoal(ConveyorMode.PASS_TO_CONVEYOR))
                || conveyor.atGoal(ConveyorMode.WAITING_AMP_SHOT) && conveyor.hasNote())
            && elevator.atGoal(ElevatorPositions.TRAP_SHOT)) {
          state = RobotState.TRAP_OUTTAKE;
        }
        break;
      case TRAP_OUTTAKE:
        if (conveyor.atGoal(ConveyorMode.AMP_SHOT)
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
      case CONVEYOR_TO_INTAKE_FOR_SHOOTER:
        if (conveyor.atGoal(ConveyorMode.PASS_TO_INTAKE)
            && intake.atGoal(IntakeState.PASS_NOTE_OUTTAKE)) {
          state = RobotState.INTAKE_TO_QUEUER_FOR_SHOOTER;
        }
      case INTAKE_TO_QUEUER_FOR_SHOOTER:
        if (conveyor.atGoal(ConveyorMode.PASS_TO_SHOOTER)
            && intake.atGoal(IntakeState.INTAKING)
            && queuer.atGoal(QueuerState.IDLE)) {
          state = RobotState.IDLE_WITH_GP;
        }
      case QUEUER_TO_INTAKE_FOR_AMP:
        if (queuer.atGoal(QueuerState.PASS_TO_INTAKE)
            && conveyor.atGoal(ConveyorMode.PASS_TO_CONVEYOR)
            && intake.atGoal(IntakeState.PASS_NOTE_OUTTAKE)) {
          state = RobotState.INTAKE_TO_CONVEYOR_FOR_AMP;
        }
      case INTAKE_TO_CONVEYOR_FOR_AMP:
        if (conveyor.atGoal(ConveyorMode.PASS_TO_CONVEYOR) && intake.atGoal(IntakeState.INTAKING)) {
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
        conveyor.setMode(ConveyorMode.IDLE);
        queuer.setState(QueuerState.IDLE);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        break;
      case HOMING:
        wrist.startMidMatchHoming();
        climber.startHoming();
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        conveyor.setMode(ConveyorMode.IDLE);
        queuer.setState(QueuerState.IDLE);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        break;
      case IDLE_NO_GP:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case IDLE_WITH_GP:
        wrist.setAngle(wristAngleForSpeaker);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case GROUND_INTAKING:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.PASS_TO_INTAKE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.INTAKING);
        shooter.setGoalMode(ShooterMode.INTAKING);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case OUTTAKING_INTAKE:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.PASS_TO_INTAKE);
        conveyor.setMode(ConveyorMode.PASS_TO_CONVEYOR);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.OUTTAKING);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case OUTTAKING_SHOOTER:
        wrist.setAngle(WristPositions.OUTTAKING_SHOOTER);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.OUTTAKE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case CONVEYOR_TO_INTAKE_FOR_SHOOTER:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.PASS_TO_INTAKE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.PASS_NOTE_OUTTAKE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case INTAKE_TO_CONVEYOR_FOR_AMP:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.PASS_TO_CONVEYOR);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.INTAKING);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case QUEUER_TO_INTAKE_FOR_AMP:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.PASS_TO_INTAKE);
        conveyor.setMode(ConveyorMode.PASS_TO_CONVEYOR);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.PASS_NOTE_OUTTAKE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case INTAKE_TO_QUEUER_FOR_SHOOTER:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.PASS_TO_CONVEYOR);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.INTAKING);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case WAITING_FLOOR_SHOT:
      case PREPARE_FLOOR_SHOT:
        wrist.setAngle(wristAngleForFloorSpot);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
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
        conveyor.setMode(ConveyorMode.IDLE);
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
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case SUBWOOFER_SHOOT:
        wrist.setAngle(WristPositions.SUBWOOFER_SHOT);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case WAITING_SPEAKER_SHOT:
      case PREPARE_SPEAKER_SHOT:
        wrist.setAngle(wristAngleForSpeaker);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
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
        conveyor.setMode(ConveyorMode.IDLE);
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
        conveyor.setMode(ConveyorMode.WAITING_AMP_SHOT);
        elevator.setGoalHeight(ElevatorPositions.AMP_OUTTAKE);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case AMP_SHOT:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.AMP_SHOT);
        elevator.setGoalHeight(ElevatorPositions.AMP_OUTTAKE);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case WAITING_CLIMBER_RAISED:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.WAITING_AMP_SHOT);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.RAISED);
        break;
      case PREPARE_CLIMBER_RAISED:
      case CLIMBER_RAISED:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.AMP_SHOT);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.RAISED);
        break;
      case PREPARE_CLIMBER_HANGING:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.HANGING);
        break;
      case CLIMBER_HANGING:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.HANGING);
        break;
      case TRAP_OUTTAKE:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.AMP_SHOT);
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
    homingFlag = false;
    waitSpeakerShotFlag = false;
    speakerShotFlag = false;
    ampShotFlag = false;
    passNoteToConveyorFlag = false;
    passNoteToQueuerFlag = false;
    trapShotFlag = false;
    waitSubwooferShotFlag = false;
    subwooferShotFlag = false;
    groundIntakeFlag = false;
    outtakeShooterFlag = false;
    outtakeIntakeFlag = false;
    stowFlag = false;
    stowAfterIntakeFlag = false;
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

  public void passNoteToConveyorRequest() {
    passNoteToConveyorFlag = true;
  }

  public void passNoteToQueuerRequest() {
    passNoteToQueuerFlag = true;
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

  public void outtakeIntakeRequest() {
    outtakeIntakeFlag = true;
  }

  public void outtakeShooterRequest() {
    outtakeShooterFlag = true;
  }

  public void stowRequest() {
    stowFlag = true;
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

  public void stowAfterIntakeRequest() {
    stowAfterIntakeFlag = true;
  }

  public Command waitForStateCommand(RobotState goalState) {
    return Commands.waitUntil(() -> this.state == goalState);
  }

  public RobotState getState() {
    return state;
  }
}
