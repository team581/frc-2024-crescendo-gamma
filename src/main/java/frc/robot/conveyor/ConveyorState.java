// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.conveyor;

public enum ConveyorState {
  IDLE,
  PASS_TO_SHOOTER,
  PASS_TO_INTAKE,
  PASS_TO_CONVEYOR,
  WAITING_AMP_SHOT,
  AMP_SHOT;
}
