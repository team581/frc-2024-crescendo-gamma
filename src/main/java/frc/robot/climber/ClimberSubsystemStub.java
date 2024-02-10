// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystemStub extends LifecycleSubsystem implements IClimberSubsystem {
  private ClimberMode goalMode = ClimberMode.IDLE;
  private double goalPosition = 0;

  public ClimberSubsystemStub() {
    super(SubsystemPriority.CLIMBER);
  }

  public void setGoal(ClimberMode newMode) {
    goalMode = newMode;
  }

  public boolean atGoal(ClimberMode mode) {
    if (mode != goalMode) {
      return false;
    }

    if (goalMode == ClimberMode.IDLE) {
      return true;
    }

    return false;
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("Climber/GoalPosition", goalPosition);
    Logger.recordOutput("Climber/GoalMode", goalMode);

    if (goalMode == ClimberMode.IDLE) {
      goalPosition = 0;
    } else if (goalMode == ClimberMode.RAISED) {
      goalPosition = 20;
    } else if (goalMode == ClimberMode.HANGING) {
      goalPosition = 10;
    }
  }
}
