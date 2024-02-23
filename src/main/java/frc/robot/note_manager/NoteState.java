// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_manager;

public enum NoteState {
  IDLE_NO_GP,

  INTAKE_TO_QUEUER,
  IDLE_IN_QUEUER,

  CONVEYOR_TO_INTAKE_FOR_QUEUER_IDLE,
  CONVEYOR_TO_INTAKE_FOR_SHOOTER_OUTTAKE,
  CONVEYOR_TO_INTAKE_FOR_SHOOTER_SCORE,

  SHOOTING,
  SHOOTER_OUTTAKING,

  QUEUER_TO_INTAKE_FOR_CONVEYOR,
  QUEUER_TO_INTAKE_FOR_CONVEYOR_FINAL,
  INTAKE_TO_CONVEYOR,
  IDLE_IN_CONVEYOR,

  AMP_SCORING,

  QUEUER_TO_INTAKE_FOR_OUTTAKING,
  CONVEYOR_TO_INTAKE_FOR_OUTTAKING,
  OUTTAKING;
}
