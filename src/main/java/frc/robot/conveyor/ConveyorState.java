// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.conveyor;

public enum ConveyorState {
  IDLE,
  INTAKE_TO_QUEUER,
  QUEUER_TO_INTAKE,
  WAITING_AMP_SHOT,
  AMP_SHOT,
  TO_INTAKE,
  /** Turns on at 12V and then off every n seconds. */
  TRAP_SHOT_PULSE;
}
