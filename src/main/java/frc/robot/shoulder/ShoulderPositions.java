// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shoulder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.RobotConfig;

public class ShoulderPositions {
  public static final Rotation2d STOWED_UP = Rotation2d.fromDegrees(60);
  public static final Rotation2d STOWED_DOWN = Rotation2d.fromDegrees(-5);

  public static final Rotation2d GROUND_INTAKING = RobotConfig.get().shoulder().minAngle();
  public static final Rotation2d SOURCE_INTAKING = Rotation2d.fromDegrees(54.5);

  public static final Rotation2d OUTTAKING = Rotation2d.fromDegrees(-15);

  public static final Rotation2d FLOOR_SHOT = Rotation2d.fromDegrees(60);

  public static final Rotation2d SUBWOOFER_SHOT = Rotation2d.fromDegrees(5);
  public static final Rotation2d AMP_SHOT = Rotation2d.fromDegrees(85);

  public static final Rotation2d WAITING_CLIMBER_RAISED = Rotation2d.fromDegrees(0);
  public static final Rotation2d TRAP_SHOT = Rotation2d.fromDegrees(95);

  private ShoulderPositions() {}
}
