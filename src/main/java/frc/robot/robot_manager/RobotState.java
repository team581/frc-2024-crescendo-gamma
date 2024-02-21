// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robot_manager;

public enum RobotState {
  /** Not homed. */
  UNHOMED(false, false, false),
  /** Homing. */
  HOMING(false, false, false),

  /** Idling without a note. */
  IDLE_NO_GP(false, false),
  /** Idling with a note in the queuer. */
  IDLE_WITH_GP(true, false),

  /** Intaking a game piece. Transition to INTAKE_TO_QUEUER when done. */
  GROUND_INTAKING(false, false),

  /** Outtaking via the shooter. Game piece should be in queuer at start. */
  OUTTAKING_SHOOTER(true, false),
  /** Outtaking via the intake. Game piece should be in queuer at start. */
  OUTTAKING(true, false),

  /** Preparing for floor shot, waiting for driver to commit. */
  WAITING_FLOOR_SHOT(true, false),
  /** Preparing for floor shot, should shoot when ready. */
  PREPARE_FLOOR_SHOT(true,true),
  /** Actively doing the floor shot. */
  FLOOR_SHOOT(true, true),

  /**
   * Get ready for subwoofer shot, wait for driver/operator (?) to confirm, then go to
   * PREPARE_SUBWOOFER_SHOT.
   */
  WAITING_SUBWOOFER_SHOT(true, false),
  /** Get ready for subwoofer shot, automatically go to SUBWOOFER_SHOOT when ready. */
  PREPARE_SUBWOOFER_SHOT(true, true),
  /** Actively doing the subwoofer shot. */
  SUBWOOFER_SHOOT(true, true),

  PREPARE_TRAP_OUTTAKE(true, false),
  TRAP_OUTTAKE(true, false),

  /** Get ready for speaker shot, wait for driver to confirm, then go to PREPARE_SPEAKER_SHOT. */
  WAITING_SPEAKER_SHOT(true,false),
  /** Get ready for speaker shot, automatically go to SPEAKER_SHOOT when ready. */
  PREPARE_SPEAKER_SHOT(true,true),
  SPEAKER_SHOOT(true, true),

  /** Note maybe in queuer, need to move it to conveyor, and then transition to WAITING_AMP_SHOT. */
  PREPARE_WAITING_AMP_SHOT(true, false),
  /** Note in conveyor, waiting for driver to commit to amp score. */
  WAITING_AMP_SHOT(true, false),
  /** Get ready for amp shot, automatically go to AMP_SHOT when ready. */
  PREPARE_AMP_SHOT(true, false),
  /** Actively scoring in the amp. */
  AMP_SHOT(true, false),

  /** Arm moves to chain height, climber hooks are touching the chain */
  WAITING_CLIMBER_RAISED(true, false),

  /** On the ground, arm goes up */
  PREPARE_CLIMBER_RAISED(true, false),
  CLIMBER_RAISED(true, false),

  /** Actually climbing then hanging */
  PREPARE_CLIMBER_HANGING(true, false),
  CLIMBER_HANGING(true, false);

  public final boolean hasNote;
  public final boolean homed;
  public final boolean shootingMode;

  RobotState(boolean hasNote, boolean shootingMode) {
    this(hasNote, true, shootingMode);
  }

  RobotState(boolean hasNote, boolean homed, boolean shootingMode) {
    this.hasNote = hasNote;
    this.homed = homed;
    this.shootingMode = shootingMode;
  }
}
