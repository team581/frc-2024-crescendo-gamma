// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_manager;

public enum NoteState {
  IDLE_NO_GP,

  INTAKE_TO_QUEUER,
  INTAKE_SLOW_TO_QUEUER,
  IDLE_IN_QUEUER,

  CONVEYOR_TO_QUEUER_FOR_IDLE,
  CONVEYOR_TO_QUEUER_FOR_SHOOTER_OUTTAKE,
  CONVEYOR_TO_QUEUER_FOR_SHOOTER_SCORE,

  SHOOTING,
  SHOOTER_OUTTAKING,

  QUEUER_TO_CONVEYOR_FOR_IDLE,
  IDLE_IN_CONVEYOR,

  AMP_SCORING,
  TRAP_SCORING,

  QUEUER_TO_INTAKE_FOR_OUTTAKING,
  CONVEYOR_TO_INTAKE_FOR_OUTTAKING,
  OUTTAKING;
}
