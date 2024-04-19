// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

public enum IntakeState {
  IDLE,
  OUTTAKING,
  FROM_QUEUER,
  FROM_CONVEYOR,
  TO_QUEUER,
  TO_QUEUER_SLOW,
  SHUFFLE_ASSIST_WHEN_QUEUER_SENSOR_TURNS_OFF,
  SHUFFLE,
  TO_CONVEYOR,
  TO_QUEUER_SHOOTING;
}
