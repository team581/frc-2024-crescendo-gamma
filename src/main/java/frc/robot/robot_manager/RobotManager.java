// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.climber.ClimberMode;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.RobotConfig;
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
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.DistanceAngle;
import frc.robot.vision.VisionState;
import frc.robot.vision.VisionStrategy;
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

  private RobotState state = RobotState.IDLE_NO_GP;

  private final FlagManager<RobotFlag> flags = new FlagManager<>("RobotManager", RobotFlag.class);

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
    DistanceAngle speakerDistanceAngle = vision.getDistanceAngleSpeaker();

    // change to Speaker or MovedSpeaker
    DistanceAngle polarSpeakerCoordinate = vision.getDistanceAngleSpeaker();
    DistanceAngle polarFloorShotCoordinate = vision.getDistanceAngleFloorShot();
    DistanceAngle floorSpotVisionTargets = vision.getDistanceAngleFloorShot();
    double speakerDistance = speakerDistanceAngle.distance();
    double floorSpotDistance = floorSpotVisionTargets.distance();
    Rotation2d wristAngleForSpeaker = wrist.getAngleFromDistanceToSpeaker(speakerDistance);
    Rotation2d wristAngleForFloorSpot = wrist.getAngleFromDistanceToFloorSpot(floorSpotDistance);
    shooter.setSpeakerDistance(speakerDistance);
    shooter.setFloorSpotDistance(floorSpotDistance);

    // State transitions from requests
    for (RobotFlag flag : flags.getChecked()) {
      switch (flag) {
        case STOW:
          if (state.hasNote) {
            if (state == RobotState.WAITING_AMP_SHOT) {
              state = RobotState.PREPARE_IDLE_WITH_GP_FROM_CONVEYOR;
            } else {
              state = RobotState.IDLE_WITH_GP;
            }
          } else {
            state = RobotState.IDLE_NO_GP;
          }
          break;
        case INTAKE:
          if (!state.climbing) {
            // Reset note manager state so that we don't instantly think we're done intaking
            // Need to force set the state, rather than doing a state request, due to order of
            // subsystems executing
            noteManager.evilStateOverride(NoteState.IDLE_NO_GP);
            state = RobotState.INTAKING;
          }
          break;
        case STOP_INTAKING:
          if (!state.climbing) {
            if (state == RobotState.FINISH_INTAKING) {
              // Ignore the request, we should finish intaking the note fully
            } else if (state == RobotState.IDLE_WITH_GP) {
              // You released the button after fully intaking the game piece
            } else {
              // The note might be partially in the intake, but hasn't triggered the sensor, so we
              // enter LAZY_INTAKING to ensure it's fully intaked
              state = RobotState.LAZY_INTAKING;
            }
          }
          break;
        case CLIMB_1_LINEUP_OUTER:
          state = RobotState.CLIMB_1_LINEUP_OUTER;
          break;
        case CLIMB_2_LINEUP_INNER:
          if (state == RobotState.CLIMB_1_LINEUP_OUTER
              || state == RobotState.CLIMB_3_LINEUP_FINAL) {
            state = RobotState.CLIMB_2_LINEUP_INNER;
          }
          break;
        case CLIMB_3_LINEUP_FINAL:
          if (state == RobotState.CLIMB_2_LINEUP_INNER || state == RobotState.CLIMB_4_HANGING) {
            state = RobotState.CLIMB_3_LINEUP_FINAL;
          }
          break;
        case CLIMB_4_HANGING:
          if (state == RobotState.CLIMB_3_LINEUP_FINAL
              || state == RobotState.CLIMB_5_HANGING_TRAP_SCORE) {
            state = RobotState.PREPARE_CLIMB_4_HANGING;
          }
          break;
        case CLIMB_5_HANGING_TRAP_SCORE:
          if (state == RobotState.CLIMB_4_HANGING || state == RobotState.CLIMB_6_HANGING_FINISHED) {
            state = RobotState.CLIMB_5_HANGING_TRAP_SCORE;
          }
          break;
        case CLIMB_6_HANGING_ELEVATOR_FINISH:
          if (state == RobotState.CLIMB_5_HANGING_TRAP_SCORE) {
            state = RobotState.CLIMB_6_HANGING_FINISHED;
          }
          break;
        case WAIT_SPEAKER_SHOT:
          if (!state.climbing) {
            state = RobotState.WAITING_SPEAKER_SHOT;
          }
          break;
        case WAIT_SUBWOOFER_SHOT:
          if (!state.climbing) {
            state = RobotState.WAITING_SUBWOOFER_SHOT;
          }
          break;
        case WAIT_PODIUM_SHOT:
          if (!state.climbing) {
            state = RobotState.WAITING_PODIUM_SHOT;
          }
          break;
        case OUTTAKE:
          if (!state.climbing) {
            state = RobotState.OUTTAKING;
          }
          break;
        case OUTTAKE_SHOOTER:
          if (!state.climbing) {
            state = RobotState.OUTTAKING_SHOOTER;
          }
          break;
        case WAIT_SHOOTER_AMP:
          if (!state.climbing) {
            state = RobotState.WAIT_SHOOTER_AMP;
          }
          break;
        case SHOOTER_AMP:
          if (!state.climbing) {
            state = RobotState.PREPARE_SHOOTER_AMP;
          }
          break;
        case SPEAKER_SHOT:
          if (!state.climbing) {
            state = RobotState.PREPARE_SPEAKER_SHOT;
          }
          break;
        case FORCE_SPEAKER_SHOT:
          if (!state.climbing) {
            state = RobotState.SPEAKER_SHOOT;
          }
          break;
        case WAIT_AMP_SHOT:
          if (!state.climbing) {
            state = RobotState.PREPARE_WAITING_AMP_SHOT;
          }
          break;
        case AMP_SHOT:
          if (!state.climbing) {
            state = RobotState.AMP_SHOT;
          }
          break;
        case SUBWOOFER_SHOT:
          if (!state.climbing) {
            state = RobotState.PREPARE_SUBWOOFER_SHOT;
          }
          break;
        case PODIUM_SHOT:
          if (!state.climbing) {
            state = RobotState.PREPARE_PODIUM_SHOT;
          }
          break;
        case PRELOAD_NOTE:
          if (!state.climbing) {
            state = RobotState.IDLE_WITH_GP;
          }
          break;
        case WAIT_FLOOR_SHOT:
          if (!state.climbing) {
            state = RobotState.WAITING_FLOOR_SHOT;
          }
          break;
        case FLOOR_SHOT:
          if (!state.climbing) {
            state = RobotState.PREPARE_FLOOR_SHOT;
          }
          break;
        case STOP_SHOOTING:
          if (!state.climbing
              && state != RobotState.IDLE_NO_GP
              && state != RobotState.WAITING_PODIUM_SHOT
              && state != RobotState.WAITING_SUBWOOFER_SHOT) {
            state = RobotState.IDLE_WITH_GP;
          }
          break;
        case UNJAM:
          if (!state.climbing) {
            state = RobotState.UNJAM;
          }
      }
    }

    Logger.recordOutput("RobotManager/StateAfterFlags", state);

    // Automatic state transitions
    switch (state) {
      case IDLE_NO_GP:
      case IDLE_WITH_GP:
      case WAITING_SPEAKER_SHOT:
      case WAITING_AMP_SHOT:
      case WAITING_FLOOR_SHOT:
      case WAITING_SUBWOOFER_SHOT:
      case WAITING_PODIUM_SHOT:
      case WAIT_SHOOTER_AMP:
      case OUTTAKING:
        // Do nothing
        break;
      case PREPARE_IDLE_WITH_GP_FROM_CONVEYOR:
        if (elevator.atPosition(ElevatorPositions.STOWED)) {
          state = RobotState.IDLE_WITH_GP;
        }
        break;
      case PREPARE_WAITING_AMP_SHOT:
        if (noteManager.getState() == NoteState.IDLE_IN_CONVEYOR
            && wrist.atAngle(WristPositions.FULLY_STOWED)) {
          state = RobotState.WAITING_AMP_SHOT;
        }
        break;
      case INTAKING:
        if (noteManager.getState() == NoteState.INTAKE_TO_QUEUER) {
          state = RobotState.FINISH_INTAKING;
        }
        break;
      case FINISH_INTAKING:
      case LAZY_INTAKING:
        if (noteManager.getState() == NoteState.IDLE_IN_QUEUER) {
          state = RobotState.IDLE_WITH_GP;
        }
        break;
      case PREPARE_FLOOR_SHOT:
        {
          var wristAtGoal = wrist.atAngle(wristAngleForFloorSpot);
          var shooterAtGoal = shooter.atGoal(ShooterMode.FLOOR_SHOT);
          var headingAtGoal = imu.atAngleForFloorSpot(floorSpotVisionTargets.targetAngle());
          // If using TX TY, then don't care about pose jitter
          var jitterAtGoal = localization.atSafeJitter() || RobotConfig.get().vision().strategy() == VisionStrategy.TX_TY_AND_MEGATAG;
          var swerveAtGoal = swerve.movingSlowEnoughForFloorShot();
          var angularVelocityAtGoal = Math.abs(imu.getRobotAngularVelocity().getDegrees()) < 360.0;
          DogLog.log("RobotManager/FloorShot/WristAtGoal", wristAtGoal);
          DogLog.log("RobotManager/FloorShot/ShooterAtGoal", shooterAtGoal);
          DogLog.log("RobotManager/FloorShot/HeadingAtGoal", headingAtGoal);
          DogLog.log("RobotManager/FloorShot/JitterAtGoal", jitterAtGoal);
          DogLog.log("RobotManager/FloorShot/SwerveAtGoal", swerveAtGoal);
          DogLog.log("RobotManager/FloorShot/AngularVelocityAtGoal", angularVelocityAtGoal);

          if (wristAtGoal
              && shooterAtGoal
              && headingAtGoal
              // && jitterAtGoal
              && swerveAtGoal
              && angularVelocityAtGoal) {
            state = RobotState.FLOOR_SHOOT;
          }
          break;
        }
      case PREPARE_PODIUM_SHOT:
        if (wrist.atAngle(WristPositions.PODIUM_SHOT)
            && shooter.atGoal(ShooterMode.PODIUM_SHOT)
            && noteManager.getState() == NoteState.IDLE_IN_QUEUER) {
          state = RobotState.PODIUM_SHOOT;
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
      case PREPARE_SHOOTER_AMP:
        if (noteManager.getState() == NoteState.IDLE_IN_QUEUER
            && wrist.atAngle(WristPositions.SHOOTER_AMP)
            && shooter.atGoal(ShooterMode.SHOOTER_AMP)
            && elevator.atPosition(ElevatorPositions.STOWED)) {
          state = RobotState.SHOOTER_AMP;
        }
        break;
      case PREPARE_SPEAKER_SHOT:
        {
          boolean wristAtGoal = wrist.atAngleForSpeaker(speakerDistance);
          boolean shooterAtGoal = shooter.atGoal(ShooterMode.SPEAKER_SHOT);
          boolean poseJitterSafe = localization.atSafeJitter();
          boolean swerveSlowEnough = swerve.movingSlowEnoughForSpeakerShot();
          boolean angularVelocitySlowEnough = imu.belowVelocityForSpeaker(speakerDistance);
          boolean robotHeadingAtGoal =
              imu.atAngleForSpeaker(speakerDistanceAngle.targetAngle(), speakerDistance);
          boolean limelightWorking =
              RobotConfig.get().vision().strategy() == VisionStrategy.TX_TY_AND_MEGATAG
                  ? speakerDistanceAngle.seesSpeakerTag()
                  : vision.getState() == VisionState.SEES_TAGS;

          Logger.recordOutput("RobotManager/SpeakerShot/LimelightWorking", limelightWorking);
          Logger.recordOutput("RobotManager/SpeakerShot/WristAtGoal", wristAtGoal);
          Logger.recordOutput("RobotManager/SpeakerShot/ShooterAtGoal", shooterAtGoal);
          Logger.recordOutput("RobotManager/SpeakerShot/PoseJitterSafe", poseJitterSafe);
          Logger.recordOutput("RobotManager/SpeakerShot/SwerveSlowEnough", swerveSlowEnough);
          Logger.recordOutput(
              "RobotManager/SpeakerShot/AngularVelocitySlowEnough", angularVelocitySlowEnough);
          Logger.recordOutput("RobotManager/SpeakerShot/RobotHeadingAtGoal", robotHeadingAtGoal);
          if ((limelightWorking || DriverStation.isAutonomous())
              && wristAtGoal
              && shooterAtGoal
              // If using TX TY, then don't care about pose jitter
              && (poseJitterSafe || DriverStation.isAutonomous() || RobotConfig.get().vision().strategy() == VisionStrategy.TX_TY_AND_MEGATAG)
              && swerveSlowEnough
              && angularVelocitySlowEnough
              && robotHeadingAtGoal) {
            state = RobotState.SPEAKER_SHOOT;
          }
        }
        break;
      case OUTTAKING_SHOOTER:
      case SHOOTER_AMP:
      case FLOOR_SHOOT:
      case SUBWOOFER_SHOOT:
      case PODIUM_SHOOT:
      case SPEAKER_SHOOT:
      case AMP_SHOT:
        if (noteManager.getState() == NoteState.IDLE_NO_GP) {
          state = RobotState.IDLE_NO_GP;
        }
        break;
      case UNJAM:
      case CLIMB_1_LINEUP_OUTER:
      case CLIMB_2_LINEUP_INNER:
      case CLIMB_3_LINEUP_FINAL:
      case CLIMB_4_HANGING:
      case CLIMB_6_HANGING_FINISHED:
        break;
      case PREPARE_CLIMB_4_HANGING:
        if (climber.atGoal(ClimberMode.HANGING)
            && elevator.atPosition(ElevatorPositions.CLIMBING)) {
          state = RobotState.CLIMB_4_HANGING;
        }
        break;
      case PREPARE_CLIMB_5_HANGING_TRAP_SCORE:
        if (elevator.atPosition(ElevatorPositions.TRAP_SHOT)
            && climber.atGoal(ClimberMode.HANGING)) {
          state = RobotState.CLIMB_5_HANGING_TRAP_SCORE;
        }
        break;
      case CLIMB_5_HANGING_TRAP_SCORE:
        if (noteManager.getState() == NoteState.IDLE_NO_GP) {
          state = RobotState.CLIMB_4_HANGING;
        }
        break;
      default:
        // Should never happen
        break;
    }

    Logger.recordOutput("RobotManager/StateAfterTransitions", state);

    // State actions
    switch (state) {
      case IDLE_NO_GP:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        if (DriverStation.isAutonomous()) {
          shooter.setGoalMode(ShooterMode.IDLE);
        } else {
          shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        }
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleNoGPRequest();
        break;
      case IDLE_WITH_GP:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
        break;
      case LAZY_INTAKING:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.lazyIntakeRequest();
        break;
      case FINISH_INTAKING:
      case INTAKING:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.intakeRequest();
        break;
      case OUTTAKING:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.outtakeRequest();
        break;
      case OUTTAKING_SHOOTER:
        wrist.setAngle(WristPositions.OUTTAKING_SHOOTER);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.OUTTAKE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterOuttakeRequest();
        break;
      case PREPARE_SHOOTER_AMP:
      case WAIT_SHOOTER_AMP:
        wrist.setAngle(WristPositions.SHOOTER_AMP);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SHOOTER_AMP);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
        break;
      case SHOOTER_AMP:
        wrist.setAngle(WristPositions.SHOOTER_AMP);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SHOOTER_AMP);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
        break;
      case WAITING_FLOOR_SHOT:
      case PREPARE_FLOOR_SHOT:
        wrist.setAngle(wristAngleForFloorSpot);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FLOOR_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
        snaps.setAngle(polarFloorShotCoordinate.targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case FLOOR_SHOOT:
        wrist.setAngle(wristAngleForFloorSpot);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FLOOR_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
        snaps.setAngle(polarFloorShotCoordinate.targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case WAITING_PODIUM_SHOT:
      case PREPARE_PODIUM_SHOT:
        wrist.setAngle(WristPositions.PODIUM_SHOT);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PODIUM_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
        snaps.setAngle(SnapManager.getPodiumAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case PODIUM_SHOOT:
        wrist.setAngle(WristPositions.PODIUM_SHOT);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.PODIUM_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
        snaps.setAngle(SnapManager.getPodiumAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case WAITING_SUBWOOFER_SHOT:
      case PREPARE_SUBWOOFER_SHOT:
        wrist.setAngle(WristPositions.SUBWOOFER_SHOT);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
        break;
      case SUBWOOFER_SHOOT:
        wrist.setAngle(WristPositions.SUBWOOFER_SHOT);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SUBWOOFER_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
        break;
      case WAITING_SPEAKER_SHOT:
      case PREPARE_SPEAKER_SHOT:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.idleInQueuerRequest();
        snaps.setAngle(polarSpeakerCoordinate.targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case SPEAKER_SHOOT:
        wrist.setAngle(wristAngleForSpeaker);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.SPEAKER_SHOT);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.shooterScoreRequest();
        snaps.setAngle(polarSpeakerCoordinate.targetAngle());
        snaps.setEnabled(true);
        snaps.cancelCurrentCommand();
        break;
      case PREPARE_WAITING_AMP_SHOT:
      case PREPARE_IDLE_WITH_GP_FROM_CONVEYOR:
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.ampWaitRequest();
        break;
      case WAITING_AMP_SHOT:
      case PREPARE_AMP_SHOT:
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.AMP_OUTTAKE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.ampWaitRequest();
        break;
      case AMP_SHOT:
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.AMP_OUTTAKE);
        shooter.setGoalMode(ShooterMode.IDLE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.ampScoreRequest();
        break;
      case UNJAM:
        wrist.setAngle(WristPositions.STOWED);
        elevator.setGoalHeight(ElevatorPositions.ANTI_JAM);
        shooter.setGoalMode(ShooterMode.OUTTAKE);
        climber.setGoalMode(ClimberMode.STOWED);
        noteManager.outtakeRequest();
        break;
      case CLIMB_1_LINEUP_OUTER:
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.LINEUP_OUTER);
        noteManager.trapWaitRequest();
        break;
      case CLIMB_2_LINEUP_INNER:
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.STOWED);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.LINEUP_INNER);
        noteManager.trapWaitRequest();
        break;
      case CLIMB_3_LINEUP_FINAL:
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.LINEUP_INNER);
        noteManager.trapWaitRequest();
        break;
      case PREPARE_CLIMB_4_HANGING:
      case CLIMB_4_HANGING:
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.trapWaitRequest();
        break;
      case PREPARE_CLIMB_5_HANGING_TRAP_SCORE:
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.TRAP_SHOT);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.trapWaitRequest();
        break;
      case CLIMB_5_HANGING_TRAP_SCORE:
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.TRAP_SHOT);
        elevator.setPulsing(false);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.trapShotRequest();
        // No matter what, try doing the trap score
        // If the handoff isn't completed at this point, there's no way to finish it while the
        // elevator is up
        // So, just try scoring regardless of robot state
        noteManager.evilStateOverride(NoteState.TRAP_SCORING);
        break;
      case CLIMB_6_HANGING_FINISHED:
        wrist.setAngle(WristPositions.FULLY_STOWED);
        elevator.setGoalHeight(ElevatorPositions.CLIMBING_FINISHED);
        elevator.setPulsing(false);
        shooter.setGoalMode(ShooterMode.FULLY_STOPPED);
        climber.setGoalMode(ClimberMode.HANGING);
        noteManager.idleNoGPRequest();
        break;
      default:
        // Should never happen
        break;
    }

    swerve.setShootingMode(state.shootingMode);

    // Reset all flags
    flags.clear();

    SmartDashboard.putBoolean("HasNote", getState().hasNote);
  }

  public void waitPodiumShotRequest() {
    flags.check(RobotFlag.WAIT_PODIUM_SHOT);
  }

  public void podiumShotRequest() {
    flags.check(RobotFlag.PODIUM_SHOT);
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

  public void forceSpeakerShotRequest() {
    flags.check(RobotFlag.FORCE_SPEAKER_SHOT);
  }

  public void waitAmpShotRequest() {
    flags.check(RobotFlag.WAIT_AMP_SHOT);
  }

  public void ampShotRequest() {
    flags.check(RobotFlag.AMP_SHOT);
  }

  public void shooterAmpRequest() {
    flags.check(RobotFlag.SHOOTER_AMP);
  }

  public void waitShooterAmpRequest() {
    flags.check(RobotFlag.WAIT_SHOOTER_AMP);
  }

  public void waitFloorShotRequest() {
    flags.check(RobotFlag.WAIT_FLOOR_SHOT);
  }

  public void floorShotRequest() {
    flags.check(RobotFlag.FLOOR_SHOT);
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

  public void unjamRequest() {
    flags.check(RobotFlag.UNJAM);
  }

  public void stopShootingRequest() {
    flags.check(RobotFlag.STOP_SHOOTING);
  }

  public void preloadNoteRequest() {
    flags.check(RobotFlag.PRELOAD_NOTE);
  }

  public void confirmShotRequest() {
    if (state == RobotState.WAITING_SUBWOOFER_SHOT) {
      subwooferShotRequest();
    } else if (state == RobotState.WAITING_PODIUM_SHOT) {
      podiumShotRequest();
    } else if (state == RobotState.WAITING_AMP_SHOT) {
      ampShotRequest();
    } else if (state == RobotState.WAITING_FLOOR_SHOT) {
      floorShotRequest();
    } else if (state == RobotState.WAIT_SHOOTER_AMP) {
      shooterAmpRequest();
    } else {
      speakerShotRequest();
    }
  }

  public void stopIntakingRequest() {
    flags.check(RobotFlag.STOP_INTAKING);
  }

  public void climb1LineupOutterRequest() {
    flags.check(RobotFlag.CLIMB_1_LINEUP_OUTER);
  }

  public void climb2LineupInnerRequest() {
    flags.check(RobotFlag.CLIMB_2_LINEUP_INNER);
  }

  public void climb3LineupFinalRequest() {
    flags.check(RobotFlag.CLIMB_3_LINEUP_FINAL);
  }

  public void climb4HangingRequest() {
    flags.check(RobotFlag.CLIMB_4_HANGING);
  }

  public void climb5HangingTrapScoreRequest() {
    flags.check(RobotFlag.CLIMB_5_HANGING_TRAP_SCORE);
  }

  public void climb6HangingElevatorFinishRequest() {
    flags.check(RobotFlag.CLIMB_6_HANGING_ELEVATOR_FINISH);
  }

  public void getClimberForwardRequest() {
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
        climb6HangingElevatorFinishRequest();
        break;
      case CLIMB_6_HANGING_FINISHED:
        // Do nothing, already at end of climb sequence
        break;
      default:
        // Start climb sequence
        climb1LineupOutterRequest();
        break;
    }
  }

  public void getClimberBackwardRequest() {
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
      case CLIMB_6_HANGING_FINISHED:
        climb5HangingTrapScoreRequest();
        break;
      default:
        // Do nothing if climb sequence isn't started
        break;
    }
  }

  public Command waitForStateCommand(RobotState goalState) {
    return Commands.waitUntil(() -> this.state == goalState);
  }

  public RobotState getState() {
    return state;
  }

  @Override
  public void teleopInit() {
    if (RobotConfig.IS_DEVELOPMENT) {
      // Stow when entering teleop, but only when we're in dev mode
      // Helps to avoid evil stuff happening after exiting auto
      stowRequest();
    }
  }
}
