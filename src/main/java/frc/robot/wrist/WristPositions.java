// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public class WristPositions {
  public static final Rotation2d STOWED = Rotation2d.fromDegrees(-5);

  public static final Rotation2d OUTTAKING_SHOOTER = Rotation2d.fromDegrees(-15);
  public static final Rotation2d FLOOR_SHOT = Rotation2d.fromDegrees(60);
  public static final Rotation2d SUBWOOFER_SHOT = Rotation2d.fromDegrees(5);

  private WristPositions() {}
}
