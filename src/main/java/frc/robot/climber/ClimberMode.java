// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

public enum ClimberMode {
  STOWED(0.2),
  LINEUP_OUTER(22.75),
  LINEUP_INNER(16),
  HANGING(1.2);

  final double position;

  ClimberMode(double position) {
    this.position = position;
  }
}
