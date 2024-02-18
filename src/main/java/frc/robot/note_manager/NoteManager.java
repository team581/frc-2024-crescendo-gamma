// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_manager;

import frc.robot.conveyor.ConveyorState;
import frc.robot.conveyor.ConveyorSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.queuer.QueuerState;
import frc.robot.queuer.QueuerSubsystem;
import frc.robot.util.FlagManager;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class NoteManager extends LifecycleSubsystem {
  private final FlagManager<NoteFlag> flags = new FlagManager<>(NoteFlag.class);

  private final QueuerSubsystem queuer;
  private final IntakeSubsystem intake;
  private final ConveyorSubsystem conveyor;

  private NoteState state = NoteState.IDLE_NO_GP;

  public NoteManager(QueuerSubsystem queuer, IntakeSubsystem intake, ConveyorSubsystem conveyor) {
    super(SubsystemPriority.ROBOT_MANAGER);
    this.queuer = queuer;
    this.intake = intake;
    this.conveyor = conveyor;
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("NoteManager/State", state);
    flags.log();

    // State transitions from requests
    for (NoteFlag flag : flags.getChecked()) {
      switch (flag) {
        case AMP_SCORE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.AMP_SCORING;
          }
          break;
        case AMP_WAIT:
          if (state == NoteState.IDLE_IN_QUEUER) {
            state = NoteState.QUEUER_TO_INTAKE_FOR_CONVEYOR;
          } else if (state == NoteState.IDLE_IN_CONVEYOR) {
            // Do nothing, you are already idling with the note in the conveyor
          }
          break;
        case IDLE_IN_QUEUER:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_INTAKE_FOR_QUEUER_IDLE;
          }
          break;
        case IDLE_NO_GP:
          state = NoteState.IDLE_NO_GP;
          break;
        case INTAKE:
          state = NoteState.INTAKE_TO_QUEUER;
          break;
        case OUTTAKE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_INTAKE_FOR_OUTTAKING;
          } else if (state == NoteState.IDLE_IN_QUEUER) {
            state = NoteState.QUEUER_TO_INTAKE_FOR_OUTTAKING;
          }
          break;
        case SHOOTER_OUTTAKE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_INTAKE_FOR_SHOOTER_OUTTAKE;
          } else if (state == NoteState.IDLE_IN_QUEUER) {
            state = NoteState.SHOOTER_OUTTAKING;
          }
          break;
        case SHOOTER_SCORE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_INTAKE_FOR_SHOOTER_SCORE;
          } else if (state == NoteState.IDLE_IN_QUEUER) {
            state = NoteState.SHOOTING;
          }
      }
    }

    // Automatic state transitions
    switch (state) {
      case IDLE_NO_GP:
      case IDLE_IN_CONVEYOR:
      case IDLE_IN_QUEUER:
        // Do nothing
        break;
      case AMP_SCORING:
        if (!conveyor.hasNote()) {
          state = NoteState.IDLE_NO_GP;
        }
        break;
      case SHOOTING:
      case OUTTAKING:
      case SHOOTER_OUTTAKING:
        if (!queuer.hasNote() && !intake.hasNote()) {
          state = NoteState.IDLE_NO_GP;
        }
        break;
      case INTAKE_TO_QUEUER:
        if (queuer.hasNote() && !intake.hasNote()) {
          state = NoteState.IDLE_IN_QUEUER;
        }
        break;
      case INTAKE_TO_CONVEYOR:
        if (conveyor.hasNote() && !intake.hasNote()) {
          state = NoteState.IDLE_IN_CONVEYOR;
        }
        break;
      case QUEUER_TO_INTAKE_FOR_CONVEYOR:
        if (intake.hasNote() && !queuer.hasNote()) {
          state = NoteState.INTAKE_TO_CONVEYOR;
        }
        break;
      case CONVEYOR_TO_INTAKE_FOR_OUTTAKING:
        if (!conveyor.hasNote() && intake.hasNote()) {
          state = NoteState.OUTTAKING;
        }
        break;
      case CONVEYOR_TO_INTAKE_FOR_QUEUER_IDLE:
        if (!conveyor.hasNote() && intake.hasNote()) {
          state = NoteState.INTAKE_TO_QUEUER;
        }
        break;
      case QUEUER_TO_INTAKE_FOR_OUTTAKING:
        if (!queuer.hasNote() && intake.hasNote()) {
          state = NoteState.OUTTAKING;
        }
        break;
      default:
        break;
    }

    // State actions
    switch (state) {
      case IDLE_NO_GP:
      case IDLE_IN_CONVEYOR:
      case IDLE_IN_QUEUER:
        intake.setState(IntakeState.IDLE);
        conveyor.setState(ConveyorState.IDLE);
        queuer.setState(QueuerState.IDLE);
        break;
      case INTAKE_TO_QUEUER:
        intake.setState(IntakeState.TO_QUEUER);
        conveyor.setState(ConveyorState.PASS_TO_SHOOTER);
        queuer.setState(QueuerState.INTAKING);
        break;
      case AMP_SCORING:
        intake.setState(IntakeState.IDLE);
        conveyor.setState(ConveyorState.AMP_SHOT);
        queuer.setState(QueuerState.IDLE);
        break;
      case CONVEYOR_TO_INTAKE_FOR_OUTTAKING:
      case CONVEYOR_TO_INTAKE_FOR_QUEUER_IDLE:
        intake.setState(IntakeState.FROM_CONVEYOR);
        conveyor.setState(ConveyorState.PASS_TO_INTAKE);
        queuer.setState(QueuerState.IDLE);
        break;
      case OUTTAKING:
        intake.setState(IntakeState.OUTTAKING);
        conveyor.setState(ConveyorState.PASS_TO_INTAKE);
        queuer.setState(QueuerState.PASS_TO_INTAKE);
        break;
      case QUEUER_TO_INTAKE_FOR_CONVEYOR:
        intake.setState(IntakeState.FROM_QUEUER);
        conveyor.setState(ConveyorState.PASS_TO_INTAKE);
        queuer.setState(QueuerState.PASS_TO_INTAKE);
        break;
      case INTAKE_TO_CONVEYOR:
        intake.setState(IntakeState.TO_CONVEYOR);
        conveyor.setState(ConveyorState.PASS_TO_CONVEYOR);
        queuer.setState(QueuerState.IDLE);
        break;
      case QUEUER_TO_INTAKE_FOR_OUTTAKING:
        intake.setState(IntakeState.OUTTAKING);
        conveyor.setState(ConveyorState.PASS_TO_INTAKE);
        queuer.setState(QueuerState.PASS_TO_INTAKE);
        break;
      case SHOOTER_OUTTAKING:
        intake.setState(IntakeState.TO_QUEUER);
        conveyor.setState(ConveyorState.PASS_TO_SHOOTER);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        break;
      case SHOOTING:
        intake.setState(IntakeState.TO_QUEUER_SHOOTING);
        conveyor.setState(ConveyorState.PASS_TO_SHOOTER);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        break;
      default:
        break;
    }

    // Reset all flags
    flags.clear();
  }

  public void intakeRequest() {
    flags.check(NoteFlag.INTAKE);
  }

  public void shooterScoreRequest() {
    flags.check(NoteFlag.SHOOTER_SCORE);
  }

  public void shooterOuttakeRequest() {
    flags.check(NoteFlag.SHOOTER_OUTTAKE);
  }

  public void idleInQueuerRequest() {
    flags.check(NoteFlag.IDLE_IN_QUEUER);
  }

  public void ampScoreRequest() {
    flags.check(NoteFlag.AMP_SCORE);
  }

  public void ampWaitRequest() {
    flags.check(NoteFlag.AMP_WAIT);
  }

  public void outtakeRequest() {
    flags.check(NoteFlag.OUTTAKE);
  }
}