// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

import frc.robot.note_manager.NoteLocation;

public enum RobotState {
  /** Not homed. */
  UNHOMED(false, NoteLocation.NONE, false),
  /** Homing. */
  HOMING(false, NoteLocation.NONE, false),

  /** Idling without a note. */
  IDLE_NO_GP(false, NoteLocation.NONE),
  /** Idling with a note in the queuer. */
  IDLE_WITH_GP(true, NoteLocation.QUEUER),

  /** Intaking a game piece. Transition to INTAKE_TO_QUEUER when done. */
  GROUND_INTAKING(false, NoteLocation.NONE),

  /** Outtaking via the shooter. Game piece should be in queuer at start. */
  OUTTAKING_SHOOTER(true, NoteLocation.QUEUER),
  /** Outtaking via the intake. Game piece should be in queuer at start. */
  OUTTAKING(true, NoteLocation.INTAKE),

  /** Preparing for floor shot, waiting for driver to commit. */
  WAITING_FLOOR_SHOT(true, NoteLocation.QUEUER),
  /** Preparing for floor shot, should shoot when ready. */
  PREPARE_FLOOR_SHOT(true, NoteLocation.QUEUER),
  /** Actively doing the floor shot. */
  FLOOR_SHOOT(true, NoteLocation.QUEUER),

  /**
   * Get ready for subwoofer shot, wait for driver/operator (?) to confirm, then go to
   * PREPARE_SUBWOOFER_SHOT.
   */
  WAITING_SUBWOOFER_SHOT(true, NoteLocation.QUEUER),
  /** Get ready for subwoofer shot, automatically go to SUBWOOFER_SHOOT when ready. */
  PREPARE_SUBWOOFER_SHOT(true, NoteLocation.QUEUER),
  /** Actively doing the subwoofer shot. */
  SUBWOOFER_SHOOT(true, NoteLocation.QUEUER),

  PREPARE_TRAP_OUTTAKE(true, NoteLocation.CONVEYOR),
  TRAP_OUTTAKE(true, NoteLocation.CONVEYOR),

  /** Get ready for speaker shot, wait for driver to confirm, then go to PREPARE_SPEAKER_SHOT. */
  WAITING_SPEAKER_SHOT(true, NoteLocation.QUEUER),
  /** Get ready for speaker shot, automatically go to SPEAKER_SHOOT when ready. */
  PREPARE_SPEAKER_SHOT(true, NoteLocation.QUEUER),
  SPEAKER_SHOOT(true, NoteLocation.QUEUER),

  /** Note in conveyor, waiting for driver to commit to amp score. */
  WAITING_AMP_SHOT(true, NoteLocation.CONVEYOR),
  /** Get ready for amp shot, automatically go to AMP_SHOT when ready. */
  PREPARE_AMP_SHOT(true, NoteLocation.CONVEYOR),
  /** Actively scoring in the amp. */
  AMP_SHOT(true, NoteLocation.CONVEYOR),

  /** Arm moves to chain height, climber hooks are touching the chain */
  WAITING_CLIMBER_RAISED(true, NoteLocation.CONVEYOR),

  /** On the ground, arm goes up */
  PREPARE_CLIMBER_RAISED(true, NoteLocation.CONVEYOR),
  CLIMBER_RAISED(true, NoteLocation.CONVEYOR),

  /** Actually climbing then hanging */
  PREPARE_CLIMBER_HANGING(true, NoteLocation.CONVEYOR),
  CLIMBER_HANGING(true, NoteLocation.CONVEYOR);

  public final boolean hasNote;
  public final boolean homed;
  public final NoteLocation noteLocation;

  RobotState(boolean hasNote, NoteLocation noteLocation) {
    this(hasNote, noteLocation, true);
  }

  RobotState(boolean hasNote, NoteLocation noteLocation, boolean homed) {
    this.hasNote = hasNote;
    this.noteLocation = noteLocation;
    this.homed = homed;
  }
}
