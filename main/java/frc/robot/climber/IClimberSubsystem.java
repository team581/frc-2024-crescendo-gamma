// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IClimberSubsystem extends Subsystem {
  public void setGoal(ClimberMode newMode);

  public boolean atGoal(ClimberMode mode);
}
