// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.scheduling;

public enum SubsystemPriority {
  AUTOS(31),
  ROBOT_MANAGER(30),

  SNAPS(20),

  CLIMBER(10),
  SWERVE(10),
  IMU(10),
  SHOOTER(10),
  SHOULDER(10),
  ELEVATOR(10),//Maybe needs to be different priority
  VISION(10),
  LOCALIZATION(10),
  LIGHTS(10),
  INTAKE(10),

  FMS(0),
  RUMBLE_CONTROLLER(0);

  final int value;

  private SubsystemPriority(int priority) {
    this.value = priority;
  }
}
