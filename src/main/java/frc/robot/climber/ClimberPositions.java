// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import frc.robot.config.RobotConfig;

public class ClimberPositions {
  public static final double IDLE = RobotConfig.get().climber().minDistance();
  public static final double RAISED = 0;
  public static final double HANGING = 0;
}
