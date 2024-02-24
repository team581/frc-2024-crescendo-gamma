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
  private final FlagManager<NoteFlag> flags = new FlagManager<>("NoteManager", NoteFlag.class);

  public final QueuerSubsystem queuer;
  public final IntakeSubsystem intake;
  public final ConveyorSubsystem conveyor;

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
        case TRAP_WAIT:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            // Do nothing, you are already idling with the note in the conveyor
          } else if (state == NoteState.QUEUER_TO_CONVEYOR_FOR_IDLE) {
            // Do nothing, we are already in the handoff process
          } else {
            state = NoteState.QUEUER_TO_CONVEYOR_FOR_IDLE;
          }
          break;
        case TRAP_SCORE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.TRAP_SCORING;
          }
          break;
        case IDLE_IN_QUEUER:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_QUEUER_FOR_IDLE;
          } else if (state == NoteState.CONVEYOR_TO_QUEUER_FOR_IDLE) {
            // Do nothing, we are already in the handoff process
          } else {
            state = NoteState.CONVEYOR_TO_QUEUER_FOR_IDLE;
          }
          break;
        case IDLE_NO_GP:
          state = NoteState.IDLE_NO_GP;
          break;
        case INTAKE:
          state = NoteState.INTAKE_TO_QUEUER;
          break;
        case INTAKE_SLOW:
          state = NoteState.INTAKE_SLOW_TO_QUEUER;
          break;
        case OUTTAKE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_INTAKE_FOR_OUTTAKING;
          } else {
            state = NoteState.QUEUER_TO_INTAKE_FOR_OUTTAKING;
          }
          break;
        case SHOOTER_OUTTAKE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_QUEUER_FOR_SHOOTER_OUTTAKE;
          } else {
            state = NoteState.SHOOTER_OUTTAKING;
          }
          break;
        case SHOOTER_SCORE:
          if (state == NoteState.IDLE_IN_CONVEYOR) {
            state = NoteState.CONVEYOR_TO_QUEUER_FOR_SHOOTER_SCORE;
          } else {
            state = NoteState.SHOOTING;
          }
      }
    }

    Logger.recordOutput("NoteManager/StateAfterFlags", state);

    // Automatic state transitions
    switch (state) {
      case OUTTAKING:
      case IDLE_NO_GP:
      case IDLE_IN_CONVEYOR:
      case IDLE_IN_QUEUER:
      case TRAP_SCORING:
        // Do nothing
        break;
      case AMP_SCORING:
      case SHOOTING:
      case SHOOTER_OUTTAKING:
        if (!queuer.hasNote() && !intake.hasNote()) {
          state = NoteState.IDLE_NO_GP;
        }
        break;
      case INTAKE_TO_QUEUER:
      case INTAKE_SLOW_TO_QUEUER:
        if (queuer.hasNote() && !intake.hasNote()) {
          state = NoteState.IDLE_IN_QUEUER;
        }
        break;
      case CONVEYOR_TO_INTAKE_FOR_OUTTAKING:
        if (!conveyor.hasNote() && intake.hasNote()) {
          state = NoteState.OUTTAKING;
        }
        break;
      case QUEUER_TO_INTAKE_FOR_OUTTAKING:
        if (!queuer.hasNote() && intake.hasNote()) {
          state = NoteState.OUTTAKING;
        }
        break;
      case CONVEYOR_TO_QUEUER_FOR_IDLE:
        if (!conveyor.hasNote() && queuer.hasNote()) {
          state = NoteState.IDLE_IN_QUEUER;
        }
        break;
      case CONVEYOR_TO_QUEUER_FOR_SHOOTER_OUTTAKE:
        if (!conveyor.hasNote() && queuer.hasNote()) {
          state = NoteState.SHOOTER_OUTTAKING;
        }
        break;
      case CONVEYOR_TO_QUEUER_FOR_SHOOTER_SCORE:
        if (!conveyor.hasNote() && queuer.hasNote()) {
          state = NoteState.SHOOTING;
        }
        break;
      case QUEUER_TO_CONVEYOR_FOR_IDLE:
        if (!queuer.hasNote() && conveyor.hasNote()) {
          state = NoteState.IDLE_IN_CONVEYOR;
        }
        break;
      default:
        break;
    }

    Logger.recordOutput("NoteManager/StateAfterTransitions", state);

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
        conveyor.setState(ConveyorState.INTAKE_TO_QUEUER);
        queuer.setState(QueuerState.INTAKING);
        break;
      case INTAKE_SLOW_TO_QUEUER:
        intake.setState(IntakeState.TO_QUEUER_SLOW);
        conveyor.setState(ConveyorState.INTAKE_TO_QUEUER);
        queuer.setState(QueuerState.INTAKING);
        break;
      case AMP_SCORING:
        intake.setState(IntakeState.IDLE);
        conveyor.setState(ConveyorState.AMP_SHOT);
        queuer.setState(QueuerState.IDLE);
        break;
      case TRAP_SCORING:
        intake.setState(IntakeState.IDLE);
        conveyor.setState(ConveyorState.TRAP_SHOT_PULSE);
        queuer.setState(QueuerState.IDLE);
        break;
      case CONVEYOR_TO_INTAKE_FOR_OUTTAKING:
        intake.setState(IntakeState.FROM_CONVEYOR);
        conveyor.setState(ConveyorState.CONVEYOR_TO_INTAKE);
        queuer.setState(QueuerState.IDLE);
        break;
      case OUTTAKING:
      case QUEUER_TO_INTAKE_FOR_OUTTAKING:
        intake.setState(IntakeState.OUTTAKING);
        conveyor.setState(ConveyorState.QUEUER_TO_INTAKE);
        queuer.setState(QueuerState.PASS_TO_INTAKE);
        break;
      case SHOOTER_OUTTAKING:
        intake.setState(IntakeState.TO_QUEUER);
        conveyor.setState(ConveyorState.INTAKE_TO_QUEUER);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        break;
      case SHOOTING:
        intake.setState(IntakeState.TO_QUEUER_SHOOTING);
        conveyor.setState(ConveyorState.INTAKE_TO_QUEUER);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        break;
      case CONVEYOR_TO_QUEUER_FOR_IDLE:
        intake.setState(IntakeState.IDLE);
        conveyor.setState(ConveyorState.CONVEYOR_TO_QUEUER);
        queuer.setState(QueuerState.INTAKING);
        break;
      case CONVEYOR_TO_QUEUER_FOR_SHOOTER_OUTTAKE:
      case CONVEYOR_TO_QUEUER_FOR_SHOOTER_SCORE:
        intake.setState(IntakeState.IDLE);
        conveyor.setState(ConveyorState.CONVEYOR_TO_QUEUER);
        queuer.setState(QueuerState.PASS_TO_SHOOTER);
        break;
      case QUEUER_TO_CONVEYOR_FOR_IDLE:
        intake.setState(IntakeState.QUEUER_TO_CONVEYOR);
        conveyor.setState(ConveyorState.INTAKE_TO_SELF);
        queuer.setState(QueuerState.PASS_TO_CONVEYOR);
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

  public void intakeSlowRequest() {
    flags.check(NoteFlag.INTAKE_SLOW);
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

  public void idleNoGPRequest() {
    flags.check(NoteFlag.IDLE_NO_GP);
  }

  public void ampScoreRequest() {
    flags.check(NoteFlag.AMP_SCORE);
  }

  public void trapShotRequest() {
    flags.check(NoteFlag.TRAP_SCORE);
  }

  public void trapWaitRequest() {
    flags.check(NoteFlag.TRAP_WAIT);
  }

  public void ampWaitRequest() {
    flags.check(NoteFlag.AMP_WAIT);
  }

  public void outtakeRequest() {
    flags.check(NoteFlag.OUTTAKE);
  }

  public NoteState getState() {
    return state;
  }

  public void evilStateOverride(NoteState newState) {
    state = newState;
  }
}
