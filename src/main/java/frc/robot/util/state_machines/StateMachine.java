// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.state_machines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public abstract class StateMachine<S extends Enum<S>, F extends Enum<F>>
    extends LifecycleSubsystem {
  private final FlagManager<F> flags;
  private S state;

  protected StateMachine(SubsystemPriority priority, Class<F> flagClass, S initialState) {
    super(priority);
    flags = new FlagManager<>(subsystemName, flagClass);
    state = initialState;
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput(subsystemName + "/State", state);
    flags.log();

    collectInputs();

    var stateBeforeFlags = state;
    for (F flag : flags.getChecked()) {
      state = processFlag(state, flag);
    }

    if (state != stateBeforeFlags) {
      afterFlagTransition(state);
      Logger.recordOutput(subsystemName + "/StateAfterFlags", state);
    } else {
      Logger.recordOutput(subsystemName + "/StateAfterFlags", "(no change)");
    }

    var stateBeforeTransitions = state;
    state = processTransitions(state);

    if (state != stateBeforeTransitions) {
      Logger.recordOutput(subsystemName + "/StateAfterTransition", state);
    } else {
      Logger.recordOutput(subsystemName + "/StateAfterTransition", "(no change)");
    }

    stateActions(state);

    flags.clear();
  }

  protected void collectInputs() {}

  protected S processFlag(S state, F flag) {
    return state;
  }

  protected void afterFlagTransition(S state) {}

  protected S processTransitions(S state) {
    return state;
  }

  protected void stateActions(S state) {}

  protected void checkFlag(F flag) {
    flags.check(flag);
  }

  public S getState() {
    return state;
  }

  public Command waitForStateCommand(S goalState) {
    return Commands.waitUntil(() -> this.state == goalState);
  }
}
