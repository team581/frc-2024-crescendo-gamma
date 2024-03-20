// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

public class ShooterRPMs {
  public static final double FULLY_STOPPED = 0;
  public static final double IDLE = 1000;
  public static final double OUTTAKE = 1000;
  public static final double SUBWOOFER = 3000;
  public static final double PODIUM = 4000;
  public static final double SHOOTER_AMP = 600;

  public static final double AMP_TOLERANCE = 100;
  public static final double TOLERANCE = 250;
  public static final double SPIN_RATIO = 3.5 / 5.0;

  private ShooterRPMs() {}
}
