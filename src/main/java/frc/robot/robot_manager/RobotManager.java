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
import java.util.EnumSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class RobotManager extends LifecycleSubsystem {
  private static final Set<RobotState> HAS_GP_STATES =
      EnumSet.of(
          RobotState.WAITING_FLOOR_SHOT,
          RobotState.PREPARE_FLOOR_SHOT,
          RobotState.FLOOR_SHOOT,
          RobotState.WAITING_SUBWOOFER_SHOT,
          RobotState.PREPARE_SUBWOOFER_SHOT,
          RobotState.SUBWOOFER_SHOOT,
          RobotState.TRAP_SHOOT,
          RobotState.WAITING_SPEAKER_SHOT,
          RobotState.PREPARE_SPEAKER_SHOT,
          RobotState.SPEAKER_SHOOT,
          RobotState.WAITING_AMP_SHOT,
          RobotState.PREPARE_AMP_SHOT,
          RobotState.AMP_SHOOT,
          RobotState.IDLE_WITH_GP,
          RobotState.WAITING_CLIMBER_RAISED,
          RobotState.PREPARE_CLIMBER_RAISED,
          RobotState.CLIMBER_RAISED,
          RobotState.PREPARE_CLIMBER_HANGING,
          RobotState.CLIMBER_HANGING);
  private static final Set<RobotState> HOMED_STATES = EnumSet.allOf(RobotState.class);

  {
    HOMED_STATES.remove(RobotState.HOMING);
    HOMED_STATES.remove(RobotState.UNHOMED);
  }

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
  private boolean waitAmpShotFlag = false;
  private boolean ampShotFlag = false;
  private boolean trapShotFlag = false;
  private boolean waitSubwooferShotFlag = false;
  private boolean subwooferShotFlag = false;
  private boolean IntakeFlag = false;
  private boolean outtakeShooterFlag = false;
  private boolean outtakeConveyorFlag = false;
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
    Logger.recordOutput("RobotManager/Flags/WaitAmpShotFlag", waitAmpShotFlag);
    Logger.recordOutput("RobotManager/Flags/AmpShotFlag", ampShotFlag);
    Logger.recordOutput("RobotManager/Flags/TrapShotFlag", trapShotFlag);
    Logger.recordOutput("RobotManager/Flags/WaitSubwooferShotFlag", waitSubwooferShotFlag);
    Logger.recordOutput("RobotManager/Flags/SubwooferShotFlag", subwooferShotFlag);
    Logger.recordOutput("RobotManager/Flags/StowAfterIntakeFlag", stowAfterIntakeFlag);
    Logger.recordOutput("RobotManager/Flags/IntakeFlag", IntakeFlag);
    Logger.recordOutput("RobotManager/Flags/OuttakeIntakeFlag", outtakeIntakeFlag);
    Logger.recordOutput("RobotManager/Flags/OuttakeShooterFlag", outtakeShooterFlag);
    Logger.recordOutput("RobotManager/Flags/OuttakeConveyorFlag", outtakeConveyorFlag);
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
    if (IntakeFlag) {
      intake.setState(IntakeState.IDLE);
      if (HOMED_STATES.contains(state)) {
        state = RobotState.GROUND_INTAKING;
      }
    }
    if (stowFlag) {
      if (HOMED_STATES.contains(state)) {
        if (HAS_GP_STATES.contains(state)) {
          state = RobotState.IDLE_WITH_GP;
        } else {
          state = RobotState.IDLE_NO_GP;
        }
      }
    }
    if (stowAfterIntakeFlag) {
      if (HOMED_STATES.contains(state) && state != RobotState.GROUND_INTAKING) {
        if (HAS_GP_STATES.contains(state)) {
          state = RobotState.IDLE_WITH_GP;
        } else {
          state = RobotState.IDLE_NO_GP;
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
    if (outtakeIntakeFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.OUTTAKING_INTAKE;
      }
    }
    if (outtakeConveyorFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.OUTTAKING_CONVEYOR;
      }
    }
    if (outtakeShooterFlag) {
      if (HOMED_STATES.contains(state)) {
        state = RobotState.OUTTAKING_SHOOTER;
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
        state = RobotState.IDLE_WITH_GP;
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
        if (wrist.getHomingState() == HomingState.HOMED) {
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
        if (intake.atGoal(IntakeState.INTAKING)) {
          state = RobotState.IDLE_WITH_GP;
        }
        break;
      case PREPARE_FLOOR_SHOT:
        if (wrist.atAngle(wristAngleForFloorSpot)
            && intake.atGoal(IntakeState.PASS_TO_QUEUER)
            && shooter.atGoal(ShooterMode.FLOOR_SHOT)
            && (Math.abs(floorShotVisionTargets.angle().getDegrees()) < 2.5)
            && swerve.movingSlowEnoughForSpeakerShot()
            && imu.getRobotAngularVelocity().getDegrees() < 2.5) {
          state = RobotState.FLOOR_SHOOT;
        }
        break;
      case PREPARE_SUBWOOFER_SHOT:
        if (wrist.atAngle(WristPositions.SUBWOOFER_SHOT)
            && shooter.atGoal(ShooterMode.SUBWOOFER_SHOT)
            && intake.atGoal(IntakeState.PASS_TO_QUEUER)) {
          state = RobotState.SUBWOOFER_SHOOT;
        }
        break;
      case PREPARE_AMP_SHOT:
        if (wrist.atAngle(WristPositions.AMP_SHOT)
            && intake.atGoal(IntakeState.PASS_TO_CONVEYOR)
            && conveyor.atGoal(ConveyorMode.CONVEYORING)
            && conveyor.getHasNote()) {
          state = RobotState.AMP_SHOOT;
        }
        break;
      case AMP_SHOOT:
        if (conveyor.atGoal(ConveyorMode.OUTTAKE) && !getCollectiveHasNote()) {
          state = RobotState.IDLE_NO_GP;
        }
        break;
      case PREPARE_SPEAKER_SHOT:
        double distance = speakerDistance;
        imu.setTolerance(imu.getAngleToleranceFromDistanceToSpeaker(distance));
        if (wrist.atAngle(wristAngleForSpeaker, distance)
            && shooter.atGoal(ShooterMode.SPEAKER_SHOT)
            && (Math.abs(speakerVisionTargets.angle().getDegrees())
                < imu.getTolerance().getDegrees())
            && swerve.movingSlowEnoughForSpeakerShot()
            && imu.getRobotAngularVelocity().getDegrees() < imu.getTolerance().getDegrees()
            && imu.atAngle(robotAngleToSpeaker, distance)
            && intake.atGoal(IntakeState.PASS_TO_QUEUER)) {
          state = RobotState.SPEAKER_SHOOT;
        }
        break;
      case FLOOR_SHOOT:
      case SUBWOOFER_SHOOT:
      case SPEAKER_SHOOT:
        if (queuer.atGoal(QueuerState.IDLE) && !getCollectiveHasNote()) {
          state = RobotState.IDLE_NO_GP;
        }
        break;
      case PREPARE_TRAP_SHOT:
        if (intake.atGoal(IntakeState.PASS_TO_CONVEYOR)
            && conveyor.atGoal(ConveyorMode.CONVEYORING)
            && conveyor.getHasNote()) {
          state = RobotState.TRAP_SHOOT;
        }
        break;
      case TRAP_SHOOT:
        if (conveyor.atGoal(ConveyorMode.OUTTAKE) && !getCollectiveHasNote()) {
          state = RobotState.CLIMBER_HANGING;
        }
        break;
      case PREPARE_CLIMBER_RAISED:
        if (climber.atGoal(ClimberMode.RAISED) && wrist.atAngle(WristPositions.CLIMBING)) {
          state = RobotState.CLIMBER_RAISED;
        }
        break;
      case PREPARE_CLIMBER_HANGING:
        if (climber.atGoal(ClimberMode.HANGING) && wrist.atAngle(WristPositions.CLIMBING)) {
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
        wrist.startPreMatchHoming();
        climber.startPreMatchHoming();
        elevator.startPreMatchHoming();
        conveyor.setMode(ConveyorMode.IDLE);
        queuer.setState(QueuerState.IDLE);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        break;
      case HOMING:
        wrist.startMidMatchHoming();
        climber.startMidMatchHoming();
        elevator.startMidMatchHoming();
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
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case GROUND_INTAKING:
        wrist.setAngle(WristPositions.STOWED);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.INTAKING);
        shooter.setGoalMode(ShooterMode.INTAKING);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case OUTTAKING_INTAKE:
        wrist.setAngle(WristPositions.OUTTAKING);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.OUTTAKING);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case OUTTAKING_SHOOTER:
        wrist.setAngle(WristPositions.OUTTAKING);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.OUTTAKE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case OUTTAKING_CONVEYOR:
        wrist.setAngle(WristPositions.OUTTAKING);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.OUTTAKE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case WAITING_FLOOR_SHOT:
      case PREPARE_FLOOR_SHOT:
        wrist.setAngle(wristAngleForFloorSpot);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.FLOOR_SHOT);
        intake.setState(IntakeState.PASS_TO_QUEUER);
        shooter.setGoalMode(ShooterMode.FLOOR_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        snaps.setAngle(robotAngleToFloorSpot);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case FLOOR_SHOOT:
        wrist.setAngle(wristAngleForFloorSpot);
        queuer.setState(QueuerState.PASSING_NOTE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.FLOOR_SHOT);
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
        elevator.setGoalHeight(ElevatorPositions.SUBWOOFER_SHOT);
        intake.setState(IntakeState.PASS_TO_QUEUER);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case SUBWOOFER_SHOOT:
        wrist.setAngle(WristPositions.SUBWOOFER_SHOT);
        queuer.setState(QueuerState.PASSING_NOTE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.SUBWOOFER_SHOT);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case WAITING_SPEAKER_SHOT:
      case PREPARE_SPEAKER_SHOT:
        wrist.setAngle(wristAngleForSpeaker);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.SPEAKER_SHOOT);
        intake.setState(IntakeState.PASS_TO_QUEUER);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        snaps.setAngle(robotAngleToSpeaker);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case SPEAKER_SHOOT:
        wrist.setAngle(wristAngleForSpeaker);
        queuer.setState(QueuerState.PASSING_NOTE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.SPEAKER_SHOOT);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.IDLE);
        snaps.setAngle(robotAngleToSpeaker);
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case WAITING_AMP_SHOT:
      case PREPARE_AMP_SHOT:
        wrist.setAngle(WristPositions.AMP_SHOT);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.CONVEYORING);
        elevator.setGoalHeight(ElevatorPositions.AMP_SHOT);
        intake.setState(IntakeState.PASS_TO_CONVEYOR);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case AMP_SHOOT:
        wrist.setAngle(WristPositions.AMP_SHOT);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.OUTTAKE);
        elevator.setGoalHeight(ElevatorPositions.AMP_SHOT);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.IDLE);
        break;
      case WAITING_CLIMBER_RAISED:
        wrist.setAngle(WristPositions.CLIMBING);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.RAISED);
        break;
      case PREPARE_CLIMBER_RAISED:
      case CLIMBER_RAISED:
        wrist.setAngle(WristPositions.TRAP_SHOT);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.RAISED);
        break;
      case PREPARE_CLIMBER_HANGING:
        wrist.setAngle(WristPositions.TRAP_SHOT);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.CONVEYORING);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.PASS_TO_CONVEYOR);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.HANGING);
        break;
      case CLIMBER_HANGING:
        wrist.setAngle(WristPositions.TRAP_SHOT);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.IDLE);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        intake.setState(IntakeState.IDLE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.HANGING);
        break;
      case TRAP_SHOOT:
        wrist.setAngle(WristPositions.TRAP_SHOT);
        queuer.setState(QueuerState.IDLE);
        conveyor.setMode(ConveyorMode.TRAP_OUTTAKE);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
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
    waitAmpShotFlag = false;
    ampShotFlag = false;
    trapShotFlag = false;
    waitSubwooferShotFlag = false;
    subwooferShotFlag = false;
    IntakeFlag = false;
    outtakeShooterFlag = false;
    outtakeConveyorFlag = false;
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

  public void trapShotRequest() {
    trapShotFlag = true;
  }

  public void subwooferShotRequest() {
    subwooferShotFlag = true;
  }

  public void groundIntakeRequest() {
    IntakeFlag = true;
  }

  public void outtakeIntakeRequest() {
    outtakeIntakeFlag = true;
  }

  public void outtakeShooterRequest() {
    outtakeShooterFlag = true;
  }

  public void outtakeCpnveyorRequest() {
    outtakeConveyorFlag = true;
  }

  public void stowUpRequest() {
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

  public void stowAfterIntakeRequest() {
    stowAfterIntakeFlag = true;
  }

  public Command waitForStateCommand(RobotState goalState) {
    return Commands.waitUntil(() -> this.state == goalState);
  }

  public boolean getCollectiveHasNote() {
    return intake.getHasNote() || queuer.getHasNote() || conveyor.getHasNote();
  }

  public RobotState getState() {
    return state;
  }
}
