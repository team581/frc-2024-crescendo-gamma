// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.scheduling;

public enum SubsystemPriority {
  // Vision must run before **everything** to ensure that the cached data is fresh
  VISION(50),

  NOTE_TRACKING(41),
  AUTOS(40),

  NOTE_MANAGER(31),
  ROBOT_MANAGER(30),

  SNAPS(20),

  CLIMBER(10),
  SWERVE(10),
  IMU(10),
  SHOOTER(10),
  ELEVATOR(10),
  WRIST(10),
  LOCALIZATION(10),
  LIGHTS(10),
  INTAKE(10),
  QUEUER(10),
  CONVEYOR(10),

  FMS(0),
  RUMBLE_CONTROLLER(0);

  final int value;

  private SubsystemPriority(int priority) {
    this.value = priority;
  }
}
