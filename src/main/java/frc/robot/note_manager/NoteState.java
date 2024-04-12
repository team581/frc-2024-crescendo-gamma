// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.note_manager;

public enum NoteState {
  IDLE_NO_GP(false),

  /** Note is in intake, going to be sent to queuer for idle. */
  INTAKE_TO_QUEUER(false),
  INTAKE_TO_QUEUER_FOR_SHOOTING(false),
  LAZY_INTAKE_TO_QUEUER(false),
  IDLE_IN_QUEUER_SHUFFLE(false),
  IDLE_IN_QUEUER(false),
  GROUND_NOTE_TO_INTAKE(false),

  SHOOTING(false),
  SHOOTER_OUTTAKING(false),

  QUEUER_TO_INTAKE_FOR_CONVEYOR(false),
  QUEUER_TO_INTAKE_FOR_CONVEYOR_FINAL(false),
  INTAKE_TO_CONVEYOR(true),
  IDLE_IN_CONVEYOR(true),

  AMP_SCORING(true),
  TRAP_SCORING(true),

  UNJAM(true),

  QUEUER_TO_INTAKE_FOR_OUTTAKING(false),
  OUTTAKING(false);

  public final boolean inConveyor;

  NoteState(boolean inConveyor) {
    this.inConveyor = inConveyor;
  }
}
